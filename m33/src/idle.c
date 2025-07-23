#include "idle.h"

#define IDLE_THREAD_PRIO		3
#define IDLE_THREAD_STACK_SIZE  1024
struct k_thread IDLE_thread;
K_THREAD_STACK_DEFINE(IDLE_stack, IDLE_THREAD_STACK_SIZE);

static void set_pwr_mode(pwr_mode_t new_mode)
{
	// SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

	switch (new_mode)
	{
		case SLEEP:
			printk("Entering SLEEP mode!\n");
            NOPs(10000);

			Wrap_MXC_LP_EnterMicroPowerMode();
			break;
		case STANDBY:
			printk("Entering STANDBY mode!\n");
			NOPs(10000);

			Wrap_MXC_LP_EnterStandbyMode();
			break;
		case BACKUP:
			printk("Entering BACKUP mode!\n");
			NOPs(10000);

			Wrap_MXC_LP_EnterBackupMode();
			break;
		default: break;
	}

	// SysTick->CTRL |= ~SysTick_CTRL_TICKINT_Msk;
    printk("In ACTIVE mode!\n");
}

static void idle_main(void *p1,void *p2,void *p3)
{
	printk("In idle_main!\n");
    while (1)
    {
        k_sem_take(&accel_inactive_sem, K_FOREVER);
		set_pwr_mode(STANDBY);
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

