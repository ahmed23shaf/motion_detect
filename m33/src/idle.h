#ifndef IDLE_H_
#define IDLE_H_

//      Idle thread that manually enters LPM on ME30, has a low priority
//

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <wrap_max32_lp.h>

#include "sem.h"

#define NOPs(count)                       \
    do {                                  \
        for (int i = 0; i < (count); i++) \
            __ASM volatile ("nop");       \
    } while (0)

typedef enum {
    PWR_ACTIVE,
	PWR_SLEEP,
	PWR_STANDBY,
	PWR_BACKUP
} pwr_mode_t;

void idle_init(void);

#endif // IDLE_H_
