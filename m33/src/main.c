#include <zephyr/kernel.h>

#include "idle.h"
#include "accel.h"

//      Motion detection (power-saving features) of ADXL367 on ME30
//

accel_pkt incoming[SAMPLE_SETS];

static int init(void)
{
    int err;

    idle_init();
    
    err = accel_init();
    if (err)    return err;

	return 0;
}

int main(void)
{
    k_sleep(K_SECONDS(5));
    
    printk("5 seconds have passed!\n");
    if (init()) { printk("init() failed..."); return -1; }

    k_sleep(K_FOREVER);

    return 0;
}
