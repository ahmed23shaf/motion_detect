#include "accel.h"

static struct gpio_callback int1_data;

#define ACCEL_THREAD_PRIO		 3
#define ACCEL_THREAD_STACK_SIZE 2048
struct k_thread ACCEL_thread;
K_THREAD_STACK_DEFINE(ACCEL_stack, ACCEL_THREAD_STACK_SIZE);

K_SEM_DEFINE(accel_fifo_sem, 0, 1);
K_SEM_DEFINE(accel_inactive_sem, 0, 1);

K_MSGQ_DEFINE(accel_msgq, SAMPLE_SETS*sizeof(accel_pkt), 1, 1);

static struct k_work accel_work;

static accel_pkt accel[SAMPLE_SETS];
static uint8_t   accel_raw_fifo[SAMPLE_FIFO_LEN]; // 6 bytes per each X,Y,Z set

// Shares interrupt lines of FIFO watermark, activity, and inactiviy detection
static void accel_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    MXC_LP_ClearWakeStatus();
    uint8_t status;
    i2c_read_reg(ADXL367_REG_STATUS, &status);

	if (status & ADXL367_STATUS_FIFO_WATERMARK)
		while (gpio_pin_get_dt(&int1))
			i2c_multi_read(ADXL367_REG_I2C_FIFO_DATA, accel_raw_fifo, SAMPLE_FIFO_LEN);

	if (status & ADXL367_STATUS_AWAKE)
	{
		gpio_pin_set_dt(&led, 1);

		i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_INACT_INT1 | ADXL367_INTMAP1_LOWER_FIFO_WATERMARK_INT1);
		k_sem_give(&accel_fifo_sem);
	}
	else
	{
		gpio_pin_set_dt(&led, 0);

		i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_ACT_INT1);
		k_sem_give(&accel_inactive_sem);
	}
    printk("In accel_isr\n");
}

static void accel_wake_handler(void)
{
    MXC_LP_ClearWakeStatus();
    uint8_t status;
    i2c_read_reg(ADXL367_REG_STATUS, &status);

	if (status & ADXL367_STATUS_FIFO_WATERMARK)
		while (gpio_pin_get_dt(&int1))
			i2c_multi_read(ADXL367_REG_I2C_FIFO_DATA, accel_raw_fifo, SAMPLE_FIFO_LEN);

	if (status & ADXL367_STATUS_AWAKE)
	{
		gpio_pin_set_dt(&led, 1);

		i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_INACT_INT1 | ADXL367_INTMAP1_LOWER_FIFO_WATERMARK_INT1);
		k_sem_give(&accel_fifo_sem);
	}
	else
	{
		gpio_pin_set_dt(&led, 0);

		i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_ACT_INT1);
		k_sem_give(&accel_inactive_sem);
	}
    printk("In accel_wake_handler\n");
}

static void accel_work_handler(struct k_work *work)
{
    k_msgq_put(&accel_msgq, &accel, K_NO_WAIT);
}

// Copies all 8 sets of acceleration values into 'dest'
void set_accel_val(accel_pkt *dest)
{
    memcpy(dest, accel, sizeof(accel));
}

static int twos_complement(unsigned int val, int num_bits)
{
    int value  = (int)val;

    if (value & (1 << (num_bits-1)))
       value -= 1 << num_bits;

    return value;
}

static int16_t decode_accel_axis(uint8_t msb, uint8_t lsb)
{
    uint16_t value = ((msb << 8) | lsb) & 0x3FFF;
    return twos_complement(value, 14); // 14-bit axis resolution
}

static void accel_main(void *p1, void *p2, void *p3)
{
    printk("In accel_main!\n");
    while (1)
    {
        k_sem_take(&accel_fifo_sem, K_FOREVER);
        k_work_submit(&accel_work);
    }
}

static int init_led(void)
{
	int err;

	if (!gpio_is_ready_dt(&led)) {
		return -1;
	}

	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		return err;
	}

	return 0;
}

// GPIO initialization used for FIFO watermark interrupts
static int int1_init(void)
{
    int err;

    IRQ_CONNECT(GPIOWAKE_IRQn, 2, accel_wake_handler, NULL, 0);
    irq_enable(GPIOWAKE_IRQn);

    err = gpio_is_ready_dt(&int1);
	if (!err) {
		printk("int1 not ready\n");
		return -1;
	}

	err = gpio_pin_configure_dt(&int1, GPIO_INPUT);
	if (err) {
		printk("int1 failed to configure\n");
		return err;
	}

	gpio_init_callback(&int1_data, accel_isr, BIT(int1.pin));

	err = gpio_add_callback(int1.port, &int1_data);
	if (err) {
		printk("Failed to add GPIO callback\n");
		return err;
	}

    // Set GPIO pin as global wake-up source
    mxc_gpio_cfg_t init1_cfg = {
        .port = MXC_GPIO0,
        .mask = MXC_GPIO_PIN_8
    };
    
    Wrap_MXC_LP_EnableGPIOWakeup(&init1_cfg);

    // err = gpio_pin_interrupt_configure_dt(&int1, GPIO_INT_EDGE_RISING); // enable IRQ line
	// if (err) {
	// 	printk("Failed to configure interrupt on %s pin %d\n", int1.port->name,
	// 		int1.pin);
	// 	return err;
	// }

	return 0;
}

static void fifo_init(uint8_t mode, uint16_t water_mark_lvl)
{
    // FIFO Mode: stream mode
    i2c_update_reg(ADXL367_REG_FIFO_CTL,
                   ADXL367_FIFO_CTL_FIFO_MODE(0x3),
                   ADXL367_FIFO_CTL_FIFO_MODE(mode));

    if (water_mark_lvl > 0xFF)
    {
        i2c_update_reg(ADXL367_REG_FIFO_CTL,
                       ADXL367_FIFO_CTL_FIFO_SAMPLE8,
                       ADXL367_FIFO_CTL_FIFO_SAMPLE8);
        
        i2c_write_reg(ADXL367_REG_FIFO_SAMPLES, (water_mark_lvl & 0xFF));
    }
    else
        i2c_write_reg(ADXL367_REG_FIFO_SAMPLES, water_mark_lvl);
}

// See the datasheet "Loop mode initialization" section for more detail
static void loop_mode_init(void)
{
    i2c_write_reg(ADXL367_REG_THRESH_ACT_H, 0x00); // Step 1a
    i2c_write_reg(ADXL367_REG_THRESH_ACT_L, 0x04); // Step 1b

    i2c_write_reg(ADXL367_REG_TIMER_ACT, 0x00);    // Step 2

    i2c_write_reg(ADXL367_REG_THRESH_INACT_H, 0x7F); // Step 3a
    i2c_write_reg(ADXL367_REG_THRESH_INACT_L, 0xFC); // Step 3b

    i2c_write_reg(ADXL367_REG_TIMER_INACT_H, 0x00); // Step 4a
    i2c_write_reg(ADXL367_REG_TIMER_INACT_L, 0x00); // Step 4b

    i2c_write_reg(ADXL367_REG_ACT_INACT_CTL, 0x3F); // Step 5
}

// Soft resets accelerometer and performs all needed register initializations with sensor,
// Adds a thread as well for this demo
int accel_init(void)
{
    init_led();

    i2c_write_reg(ADXL367_REG_SOFT_RESET, 'R');
    k_msleep(8);    // "A delay of 7.5 ms must be realized after software reset"

    loop_mode_init();

    // Measurement range: +- 2g, ODR: 50 Hz
    i2c_update_reg(ADXL367_REG_FILTER_CTL,
                   ADXL367_FILTER_CTL_RANGE(0x3) | ADXL367_FILTER_CTL_ODR(0x7),
                   ADXL367_FILTER_CTL_RANGE(ADXL367_RANGE_2G) | ADXL367_FILTER_CTL_ODR(ADXL367_ODR_50_HZ));
    fifo_init(ADXL367_FIFO_STREAM, SAMPLE_SETS*3);

    i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_ACT_INT1
                                           | ADXL367_INTMAP1_LOWER_INACT_INT1);

    //  Thread init
    k_thread_create(&ACCEL_thread,
					ACCEL_stack,
					ACCEL_THREAD_STACK_SIZE,
					accel_main,
					NULL, NULL, NULL,
					ACCEL_THREAD_PRIO,
					0,
					K_NO_WAIT);

	k_thread_name_set(&ACCEL_thread, "ACCEL");
    k_work_init(&accel_work, accel_work_handler);

    i2c_write_reg(ADXL367_REG_POWER_CTL, 0x07); // Step 7

    uint8_t status = 0xFF;
    do { i2c_read_reg(ADXL367_REG_STATUS, &status); } while (status & ADXL367_STATUS_AWAKE);
    do { i2c_read_reg(ADXL367_REG_STATUS, &status); } while (!(status & ADXL367_STATUS_AWAKE));

     // Linked mode (motion-detect) initialization
    i2c_write_reg(ADXL367_REG_THRESH_ACT_H,
                  THRESH_H(THRESH_LSB(THRESH_ACT_MG)));
    i2c_write_reg(ADXL367_REG_THRESH_ACT_L,
                  THRESH_L(THRESH_LSB(THRESH_ACT_MG)));
    
    i2c_write_reg(ADXL367_REG_TIMER_ACT, 0x02); // No timer for wakeup

    i2c_write_reg(ADXL367_REG_THRESH_INACT_H,
                  THRESH_H(THRESH_LSB(THRESH_INACT_MG)));
    i2c_write_reg(ADXL367_REG_THRESH_INACT_L,
                  THRESH_L(THRESH_LSB(THRESH_INACT_MG)));

    i2c_write_reg(ADXL367_REG_TIMER_INACT_H, 0x00); 
    i2c_write_reg(ADXL367_REG_TIMER_INACT_L, TIMER_INACT);

    i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_FIFO_WATERMARK_INT1);

    // Interrupt pin & begin accelerometer
    int1_init();

    return 0;
}