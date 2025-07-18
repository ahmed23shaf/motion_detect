#pragma once

#include <zephyr/kernel.h>

extern struct k_sem accel_fifo_sem;
extern struct k_sem accel_active_sem;
extern struct k_sem accel_inactive_sem;
