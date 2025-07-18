#ifndef ACCEL_H_
#define ACCEL_H_

//      Zephyr-based driver for ME30's ADXL367 accelerometer
//

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include <wrap_max32_wut.h>
#include <wrap_max32_sys.h>

#include "i2c.h"
#include "sem.h"

#define SAMPLE_SETS     8
#define SAMPLE_FIFO_LEN SAMPLE_SETS*6  // 2 bytes per each axis of X,Y,Z ==> 6x multiplier

#define THRESH_ACT_MG     50
#define THRESH_INACT_MG   400

#define TIMER_INACT       (uint8_t)20    // # of samples below threshold that causes inactivity event

// Macros for writing into THRESHOLD registers
#define THRESH_LSB(THRESH)  ((uint16_t)(THRESH / 0.25)) // 0.25 is the +/- 2g scale factor defined below
#define THRESH_H(LSB)       ((uint8_t)((LSB >> 6) & 0x7F))
#define THRESH_L(LSB)       ((uint8_t)((LSB & 0x3F) << 2))

/* GPIO INT1 pin */
#define INT1_NODE DT_ALIAS(p8)
static const struct gpio_dt_spec int1 = GPIO_DT_SPEC_GET_OR(INT1_NODE, gpios, {0});

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} accel_pkt;

typedef enum {
    ACCEL_MEASUREMENT,
    ACCEL_WAKEUP,
    ACCEL_STANDBY
} accel_mode_t;

extern struct k_msgq accel_msgq;

int          accel_init(void);
void         set_accel_val(accel_pkt *dest);
void         accel_update(void);

accel_mode_t accel_get_mode(void);
int          accel_set_mode(accel_mode_t new_mode);

/* Registers */
#define ADXL367_REG_DEVID_AD 0x00
#define ADXL367_REG_DEVID_MST 0x01
#define ADXL367_REG_PARTID 0x02
#define ADXL367_REG_REVID 0x03
#define ADXL367_REG_SERIAL_NUMBER_R3 0x04
#define ADXL367_REG_SERIAL_NUMBER_R2 0x05
#define ADXL367_REG_SERIAL_NUMBER_R1 0x06
#define ADXL367_REG_SERIAL_NUMBER_R0 0x07
#define ADXL367_REG_XDATA 0x08
#define ADXL367_REG_YDATA 0x09
#define ADXL367_REG_ZDATA 0x0A
#define ADXL367_REG_STATUS 0x0B
#define ADXL367_REG_FIFO_ENTRIES_L 0x0C
#define ADXL367_REG_FIFO_ENTRIES_H 0x0D
#define ADXL367_REG_XDATA_H 0x0E
#define ADXL367_REG_XDATA_L 0x0F
#define ADXL367_REG_YDATA_H 0x10
#define ADXL367_REG_YDATA_L 0x11
#define ADXL367_REG_ZDATA_H 0x12
#define ADXL367_REG_ZDATA_L 0x13
#define ADXL367_REG_TEMP_H 0x14
#define ADXL367_REG_TEMP_L 0x15
#define ADXL367_REG_EX_ADC_H 0x16
#define ADXL367_REG_EX_ADC_L 0x17
#define ADXL367_REG_I2C_FIFO_DATA 0x18
#define ADXL367_REG_SOFT_RESET 0x1F
#define ADXL367_REG_THRESH_ACT_H 0x20
#define ADXL367_REG_THRESH_ACT_L 0x21
#define ADXL367_REG_TIMER_ACT 0x22
#define ADXL367_REG_THRESH_INACT_H 0x23
#define ADXL367_REG_THRESH_INACT_L 0x24
#define ADXL367_REG_TIMER_INACT_H 0x25
#define ADXL367_REG_TIMER_INACT_L 0x26
#define ADXL367_REG_ACT_INACT_CTL 0x27
#define ADXL367_REG_FIFO_CTL 0x28
#define ADXL367_REG_FIFO_SAMPLES 0x29
#define ADXL367_REG_INTMAP1_LOWER 0x2A
#define ADXL367_REG_INTMAP2_LOWER 0x2B
#define ADXL367_REG_FILTER_CTL 0x2C
#define ADXL367_REG_POWER_CTL 0x2D
#define ADXL367_REG_SELF_TEST 0x2E
#define ADXL367_REG_TAP_THRESH 0x2F
#define ADXL367_REG_TAP_DUR 0x30
#define ADXL367_REG_TAP_LATENT 0x31
#define ADXL367_REG_TAP_WINDOW 0x32
#define ADXL367_REG_X_OFFSET 0x33
#define ADXL367_REG_Y_OFFSET 0x34
#define ADXL367_REG_Z_OFFSET 0x35
#define ADXL367_REG_X_SEN 0x36
#define ADXL367_REG_Y_SEN 0x37
#define ADXL367_REG_Z_SEN 0x38
#define ADXL367_REG_TIMER_CTL 0x39
#define ADXL367_REG_INTMAP1_UPPER 0x3A
#define ADXL367_REG_INTMAP2_UPPER 0x3B
#define ADXL367_REG_ADC_CTL 0x3C
#define ADXL367_REG_TEMP_CTL 0x3D
#define ADXL367_REG_TEMP_ADC_OVER_THRSH_H 0x3E
#define ADXL367_REG_TEMP_ADC_OVER_THRSH_L 0x3F
#define ADXL367_REG_TEMP_ADC_UNDER_THRSH_H 0x40
#define ADXL367_REG_TEMP_ADC_UNDER_THRSH_L 0x41
#define ADXL367_REG_TEMP_ADC_TIMER 0x42
#define ADXL367_REG_AXIS_MASK 0x43
#define ADXL367_REG_STATUS_COPY 0x44
#define ADXL367_REG_STATUS2 0x45

/* ADXL367_REG_POWER_CTL defintions */
#define ADXL367_POWER_CTL_EXT_CLK       (1 << 6)
#define ADXL367_POWER_CTL_LOW_NOISE(x)  (((x) & 0x3) << 4)
#define ADXL367_POWER_CTL_WAKEUP        (1 << 3)
#define ADXL367_POWER_CTL_AUTOSLEEP     (1 << 2)
#define ADXL367_POWER_CTL_MEASURE(x)    (((x) & 0x3) << 0)

/* ADXL367_POWER_CTL_MEASURE options */
#define ADXL367_MEASURE_STANDBY         0
#define ADXL367_MEASURE_ON              2

/* ADXL367_REG_FILTER_CTL definitions */
#define ADXL367_FILTER_CTL_RANGE(x)     (((x) & 0x3) << 6)
#define ADXL367_FILTER_CTL_I2C_HS       (1 << 5)
#define ADXL367_FILTER_CTL_EXT_SAMPLE   (1 << 3)
#define ADXL367_FILTER_CTL_ODR(x)       ((x) & 0x7)

/* ADXL367_FILTER_CTL_RANGE(x) options */
#define ADXL367_RANGE_2G                0
#define ADXL367_RANGE_4G                1
#define ADXL367_RANGE_8G                2

/* ADXL367_FILTER_CTL_ODR(x) options */
#define ADXL367_ODR_12_5_HZ             0
#define ADXL367_ODR_25_HZ               1
#define ADXL367_ODR_50_HZ               2
#define ADXL367_ODR_100_HZ              3
#define ADXL367_ODR_200_HZ              4
#define ADXL367_ODR_400_HZ              5

/* ADXL367_REG_FIFO_CTL */
#define ADXL367_FIFO_CTL_CHANNEL_SELECT(x) (((x) & 0x0F) << 3)
#define ADXL367_FIFO_CTL_FIFO_SAMPLE8      (1<<2)
#define ADXL367_FIFO_CTL_FIFO_MODE(x)      ((x) & 0x3)

/* ADXL367_FIFO_CTL_FIFO_MODE(x) options */
#define ADXL367_FIFO_DISABLE          0
#define ADXL367_FIFO_OLDEST_SAVED     1
#define ADXL367_FIFO_STREAM           2
#define ADXL367_FIFO_TRIGGERED        3

/* ADXL367_REG_INTMAP1_LOWER definitions*/
#define ADXL367_INTMAP1_LOWER_INT_LOW_INT1         (1 << 7)
#define ADXL367_INTMAP1_LOWER_AWAKE_INT1           (1 << 6)
#define ADXL367_INTMAP1_LOWER_INACT_INT1           (1 << 5)
#define ADXL367_INTMAP1_LOWER_ACT_INT1             (1 << 4)
#define ADXL367_INTMAP1_LOWER_FIFO_OVERRUN_INT1    (1 << 3)
#define ADXL367_INTMAP1_LOWER_FIFO_WATERMARK_INT1  (1 << 2)
#define ADXL367_INTMAP1_LOWER_FIFO_READY_INT1      (1 << 1)
#define ADXL367_INTMAP1_LOWER_DATA_READY_INT1      (1 << 0)

/* ADXL367_REG_STATUS definitions */
#define ADXL367_STATUS_ERR_USER_REGS    (1 << 7)
#define ADXL367_STATUS_AWAKE            (1 << 6)
#define ADXL367_STATUS_INACT            (1 << 5)
#define ADXL367_STATUS_ACT              (1 << 4)
#define ADXL367_STATUS_FIFO_OVERRUN     (1 << 3)
#define ADXL367_STATUS_FIFO_WATERMARK   (1 << 2)
#define ADXL367_STATUS_FIFO_RDY         (1 << 1)
#define ADXL367_STATUS_DATA_RDY         (1 << 0)

/* ADXL367_REG_ACT_INACT_CTL definitions */
#define ADXL367_ACT_INACT_CTL_LINKLOOP(x)   (((x) & 0x3) << 4)
#define ADXL367_ACT_INACT_CTL_INACT_REF     (0x3 << 2)
#define ADXL367_ACT_INACT_CTL_INACT_EN      (0x1 << 2)
#define ADXL367_ACT_INACT_CTL_ACT_REF       (0x3 << 0)
#define ADXL367_ACT_INACT_CTL_ACT_EN        (0x1 << 0)

/* ADXL367_ACT_INACT_CTL_LINKLOOP(x) options */
#define ADXL367_MODE_DEFAULT            0
#define ADXL367_MODE_LINK               1
#define ADXL367_MODE_LOOP               3

#define ADXL367_SCALE_FACTOR_2G 0.25f // in units of mg/LSB

#endif  // ACCEL_H_
