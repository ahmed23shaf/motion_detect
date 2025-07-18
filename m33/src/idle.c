#include "idle.h"

LOG_MODULE_REGISTER(my_idle, LOG_LEVEL_DBG);

#define IDLE_THREAD_PRIO		4
#define IDLE_THREAD_STACK_SIZE  1024
struct k_thread IDLE_thread;
K_THREAD_STACK_DEFINE(IDLE_stack, IDLE_THREAD_STACK_SIZE);

static void set_pwr_mode(pwr_mode_t new_mode)
{
	switch (new_mode)
	{
		case PWR_SLEEP:
			LOG_INF("Entering SLEEP mode!");
            NOPs(10000);

			Wrap_MXC_LP_EnterMicroPowerMode();
			break;
		case PWR_STANDBY:
			LOG_INF("Entering STANDBY mode!");
			NOPs(10000);

			Wrap_MXC_LP_EnterStandbyMode();
			break;
		case PWR_BACKUP:
			LOG_INF("Entering BACKUP mode!");
			NOPs(10000);

			Wrap_MXC_LP_EnterBackupMode();
			break;
		default: break;
	}

    LOG_INF("In ACTIVE mode!");
}

static void idle_main(void *p1,void *p2,void *p3)
{
    while (1)
    {
        k_sem_take(&accel_inactive_sem, K_FOREVER);
		set_pwr_mode(PWR_STANDBY);
    }
}

void idle_init(void)
{
    // Thread initialization
    k_thread_create(&IDLE_thread,
					IDLE_stack,
					IDLE_THREAD_STACK_SIZE,
					idle_main,
					NULL, NULL, NULL,
					IDLE_THREAD_PRIO,
					0,
					K_NO_WAIT);

	k_thread_name_set(&IDLE_thread, "my_idle");
}

