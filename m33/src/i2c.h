#ifndef I2C_H_
#define I2C_H_

//      I2C Zephyr-based driver for ME30's ADXL367 sensor communication
//

#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

static const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(DT_NODELABEL(adxl367));

// Single register read
static inline int i2c_read_reg(uint8_t reg, uint8_t *buf)
{
    return i2c_reg_read_byte_dt(&i2c_dev, reg, buf);
}

// Burst read starting at 'reg_start'
static inline int i2c_multi_read(uint8_t reg_start, uint8_t *buf, uint32_t bufsz)
{
    return i2c_burst_read_dt(&i2c_dev, reg_start, buf, bufsz);
}

// Single register write
static inline int i2c_write_reg(uint8_t reg, uint8_t val)
{
    return i2c_reg_write_byte_dt(&i2c_dev, reg, val);
}

// Update register bits using mask
static inline int i2c_update_reg(uint8_t reg, uint8_t mask, uint8_t val)
{
    return i2c_reg_update_byte_dt(&i2c_dev, reg, mask, val);
}

#endif // I2C_H_