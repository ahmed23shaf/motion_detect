#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "idle.h"
#include "accel.h"

//      Motion detection (power-saving features) of ADXL367 on ME30
// 

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static struct k_work main_work;

#define LED0_NODE DT_ALIAS(led0)
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

accel_pkt incoming[SAMPLE_SETS];

static void main_work_handler(struct k_work *work)
{
    for (int i = 0; i < SAMPLE_SETS * 6; i += 6)
    {
        // Print updated accelerometer readings
        LOG_INF("accel x: %f [mg]", (double)(incoming[i/6].x * ADXL367_SCALE_FACTOR_2G));
        LOG_INF("accel y: %f [mg]", (double)(incoming[i/6].y * ADXL367_SCALE_FACTOR_2G));
        LOG_INF("accel z: %f [mg]\n", (double)(incoming[i/6].z * ADXL367_SCALE_FACTOR_2G));
    }
}

static int init_led(void)
{
	int err;

	if (!gpio_is_ready_dt(&led)) {
		return -1;
	}

	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int init(void)
{
    int err;

    k_work_init(&main_work, main_work_handler);

    // idle_init();

    err = init_led();
    if (err)    return err;
    
    err = accel_init();
    if (err)    return err;

	return 0;
}

int main(void)
{
    // k_sleep(K_SECONDS(5));
    // LOG_INF("5 seconds have passed!");
    
    if (init()) { LOG_ERR("init() failed..."); return -1; }

    while (1)
    {
        k_msgq_get(&accel_msgq, &incoming, K_FOREVER);
        k_work_submit(&main_work);
        k_sleep(K_SECONDS(1));
    }
    

    return 0;
}
