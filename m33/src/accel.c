#include "accel.h"

LOG_MODULE_REGISTER(accel, LOG_LEVEL_DBG);

static struct gpio_callback int1_data;

#define ACCEL_THREAD_PRIO		 4
#define ACCEL_THREAD_STACK_SIZE 2048
struct k_thread ACCEL_thread;
K_THREAD_STACK_DEFINE(ACCEL_stack, ACCEL_THREAD_STACK_SIZE);

K_SEM_DEFINE(accel_fifo_sem, 0, 1);
K_SEM_DEFINE(accel_active_sem, 0, 1);
K_SEM_DEFINE(accel_inactive_sem, 0, 1);

K_MSGQ_DEFINE(accel_msgq, SAMPLE_SETS*sizeof(accel_pkt), 1, 1);

static struct k_work accel_work;

static accel_pkt accel[SAMPLE_SETS];
static uint8_t   accel_raw_fifo[SAMPLE_FIFO_LEN]; // 6 bytes per each X,Y,Z set

static volatile accel_mode_t op_mode;
extern struct gpio_dt_spec led;

// Shares interrupt lines of FIFO watermark, activity, and inactiviy detection
static void accel_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    uint8_t status;
    i2c_read_reg(ADXL367_REG_STATUS, &status);
    
    if (status & ADXL367_STATUS_FIFO_WATERMARK)
    {
        accel_update();
        k_sem_give(&accel_fifo_sem);
    }

    if (status & ADXL367_STATUS_AWAKE)
    {
        gpio_pin_set_dt(&led, 1);
        LOG_WRN("REACHIN HERE: JUST BECAME AWAKE!");
        i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_ACT_INT1
                                           | ADXL367_INTMAP1_LOWER_INACT_INT1
                                           | ADXL367_INTMAP1_LOWER_FIFO_WATERMARK_INT1);
        // i2c_multi_read(ADXL367_REG_I2C_FIFO_DATA, accel_raw_fifo, SAMPLE_FIFO_LEN);
        while (gpio_pin_get_dt(&int1))
            i2c_multi_read(ADXL367_REG_I2C_FIFO_DATA, accel_raw_fifo, SAMPLE_FIFO_LEN);
    }
    
    if (!(status & ADXL367_STATUS_AWAKE))
    {
        gpio_pin_set_dt(&led, 0);
        LOG_WRN("REACHIN HERE: JUST GONNA GO SLEEP (I HML)!");
        i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_ACT_INT1);
        while (gpio_pin_get_dt(&int1))
            i2c_multi_read(ADXL367_REG_I2C_FIFO_DATA, accel_raw_fifo, SAMPLE_FIFO_LEN);
    }
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

accel_mode_t accel_get_mode(void)
{
    return op_mode;
}

// Modifies appropriate fields within power control register to set specified mode
int accel_set_mode(accel_mode_t new_mode)
{
    if (new_mode == op_mode)    return 0;

    int err = 0;

    switch (new_mode)
    {
        case ACCEL_MEASUREMENT:
            // Clear WAKEUP (bit 3), set MEASURE (bits [1:0]) = 0b10
            err = i2c_update_reg(ADXL367_REG_POWER_CTL,
                                 ADXL367_POWER_CTL_WAKEUP | ADXL367_POWER_CTL_MEASURE(0x3),
                                 ADXL367_POWER_CTL_MEASURE(ADXL367_MEASURE_ON));
            if (err)
                return err;
            k_msleep(100); // "a 100 ms wait time must be observed before reading acceleration data"
            op_mode = ACCEL_MEASUREMENT;
            break;
        case ACCEL_WAKEUP:
            // Set WAKEUP, set MEASURE = 0b10
            err = i2c_update_reg(ADXL367_REG_POWER_CTL,
                                 ADXL367_POWER_CTL_WAKEUP | ADXL367_POWER_CTL_MEASURE(0x3),
                                 ADXL367_POWER_CTL_WAKEUP | ADXL367_POWER_CTL_MEASURE(ADXL367_MEASURE_ON));
            if (err)
                return err;
            k_msleep(100);
            op_mode = ACCEL_WAKEUP;
            break;
        case ACCEL_STANDBY:
            // Clear WAKEUP and MEASURE[1:0]
            err = i2c_update_reg(ADXL367_REG_POWER_CTL,
                                 ADXL367_POWER_CTL_WAKEUP | ADXL367_POWER_CTL_MEASURE(0x3),
                                 ADXL367_POWER_CTL_MEASURE(ADXL367_MEASURE_STANDBY));
            if (err)
                return err;
            op_mode = ACCEL_STANDBY;
            break;
    }

    return 0;
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

// To be called after internal accelerometer FIFO data is populated
void accel_update(void)
{
    // // check to see if theres enough samples
    // uint8_t fifo_entries_l, fifo_entries_h;
    // i2c_read_reg(ADXL367_REG_FIFO_ENTRIES_L, &fifo_entries_l);
    // i2c_read_reg(ADXL367_REG_FIFO_ENTRIES_H, &fifo_entries_h);

    // uint16_t entries = ((fifo_entries_h & 0x03) << 8) | fifo_entries_l;

    // if (entries < SAMPLE_FIFO_LEN)
    // {
    //     LOG_WRN("Not enough samples! Only %d < %d", entries, SAMPLE_FIFO_LEN);
    //     return;
    // }

    i2c_multi_read(ADXL367_REG_I2C_FIFO_DATA, accel_raw_fifo, SAMPLE_FIFO_LEN);

    // format incoming data into accel_pkt
    for (int i = 0; i < SAMPLE_FIFO_LEN; i += 6)
    {
        int16_t x = decode_accel_axis(accel_raw_fifo[i],     accel_raw_fifo[i + 1]);
        int16_t y = decode_accel_axis(accel_raw_fifo[i + 2], accel_raw_fifo[i + 3]);
        int16_t z = decode_accel_axis(accel_raw_fifo[i + 4], accel_raw_fifo[i + 5]);
        
        accel[i / 6] = (accel_pkt){.x = x, .y = y, .z = z};
    }
}

static void accel_main(void *p1, void *p2, void *p3)
{
    LOG_WRN("In accel!");
    while (1)
    {
        // k_sem_take(&accel_active_sem, K_FOREVER);
        k_sem_take(&accel_fifo_sem, K_FOREVER);
        k_work_submit(&accel_work);
    }
}

// GPIO initialization used for FIFO watermark interrupts
static int int1_init(void)
{
    int err;

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

    err = gpio_pin_interrupt_configure_dt(&int1, GPIO_INT_EDGE_RISING); // enable IRQ line
	if (err) {
		printk("Failed to configure interrupt on %s pin %d\n", int1.port->name,
			int1.pin);
		return err;
	}

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
    i2c_write_reg(ADXL367_REG_SOFT_RESET, 'R');
    op_mode = ACCEL_STANDBY;

    k_msleep(8);    // "A delay of 7.5 ms must be realized after software reset"

    loop_mode_init();

    // Measurement range: +- 2g, ODR: 50 Hz
    i2c_update_reg(ADXL367_REG_FILTER_CTL,
                   ADXL367_FILTER_CTL_RANGE(0x3) | ADXL367_FILTER_CTL_ODR(0x7),
                   ADXL367_FILTER_CTL_RANGE(ADXL367_RANGE_2G) | ADXL367_FILTER_CTL_ODR(ADXL367_ODR_50_HZ));
    fifo_init(ADXL367_FIFO_STREAM, SAMPLE_SETS*3);
    // i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_FIFO_WATERMARK_INT1 | ADXL367_INTMAP1_LOWER_AWAKE_INT1);
    // i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_AWAKE_INT1);
    i2c_write_reg(ADXL367_REG_INTMAP1_LOWER, ADXL367_INTMAP1_LOWER_ACT_INT1
                                           | ADXL367_INTMAP1_LOWER_INACT_INT1
                                           | ADXL367_INTMAP1_LOWER_FIFO_WATERMARK_INT1);

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

    LOG_ERR("MAKING IT HERE!");

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

    // Interrupt pin & begin accelerometer
    int1_init();

    return 0;
}