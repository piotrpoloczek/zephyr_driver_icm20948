#define DT_DRV_COMPAT invensense_icm20948

#include "icm20948.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(icm20948, CONFIG_SENSOR_LOG_LEVEL);

static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct icm20948_data *data = dev->data;
    const struct icm20948_config *cfg = dev->config;

    uint8_t buf[6];
    if (i2c_burst_read_dt(&cfg->i2c, 0x3B, buf, 6) < 0) {
        LOG_ERR("Failed to read accel data");
        return -EIO;
    }

    data->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4] << 8) | buf[5]);

    return 0;
}

static int icm20948_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct icm20948_data *data = dev->data;

    switch (chan) {
        case SENSOR_CHAN_ACCEL_X:
            val->val1 = data->accel_x;
            break;
        case SENSOR_CHAN_ACCEL_Y:
            val->val1 = data->accel_y;
            break;
        case SENSOR_CHAN_ACCEL_Z:
            val->val1 = data->accel_z;
            break;
        default:
            return -ENOTSUP;
    }

    val->val2 = 0;
    return 0;
}

static int icm20948_init(const struct device *dev)
{
    const struct icm20948_config *cfg = dev->config;

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    // Switch to bank 0
    i2c_reg_write_byte_dt(&cfg->i2c, 0x7F, 0x00);

    // Reset the device
    i2c_reg_write_byte_dt(&cfg->i2c, 0x06, 0x80);  // PWR_MGMT_1 = RESET
    k_msleep(100);

    // Wake up and auto-select best clock
    i2c_reg_write_byte_dt(&cfg->i2c, 0x06, 0x01);  // PWR_MGMT_1 = Clock Auto
    k_msleep(10);

    // Switch to bank 2 to configure accelerometer
    i2c_reg_write_byte_dt(&cfg->i2c, 0x7F, 0x20);  // REG_BANK_SEL = 2 << 4

    // Set ACCEL_CONFIG: range = ±2g (default), DLPF enabled, DLPF = 6
    i2c_reg_write_byte_dt(&cfg->i2c, 0x14, 0x09);  // ACCEL_CONFIG

    // Set ACCEL_SMPLRT_DIV
    i2c_reg_write_byte_dt(&cfg->i2c, 0x10, 0x00);  // DIV_1
    i2c_reg_write_byte_dt(&cfg->i2c, 0x11, 0x01);  // DIV_2

    // Enable accelerometer and gyroscope in bank 0
    i2c_reg_write_byte_dt(&cfg->i2c, 0x7F, 0x00);  // REG_BANK_SEL = 0
    i2c_reg_write_byte_dt(&cfg->i2c, 0x07, 0x00);  // PWR_MGMT_2 = enable all

    // Check WHO_AM_I
    uint8_t whoami = 0;
    if (i2c_reg_read_byte_dt(&cfg->i2c, 0x00, &whoami) < 0) {
        LOG_ERR("Failed to read WHO_AM_I");
        return -EIO;
    }

    if (whoami != 0xEA) {
        LOG_ERR("Unexpected WHO_AM_I: 0x%02X", whoami);
        return -EINVAL;
    }

    LOG_INF("ICM20948 initialized successfully (WHO_AM_I = 0x%X)", whoami);
    return 0;
}


static const struct sensor_driver_api icm20948_api = {
    .sample_fetch = icm20948_sample_fetch,
    .channel_get = icm20948_channel_get,
};

#define ICM20948_INIT(inst)                                                   \
    static struct icm20948_data icm20948_data_##inst;                         \
    static const struct icm20948_config icm20948_config_##inst = {           \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                    \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(inst,                                               \
                          icm20948_init, NULL,                                \
                          &icm20948_data_##inst, &icm20948_config_##inst,     \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,           \
                          &icm20948_api);

DT_INST_FOREACH_STATUS_OKAY(ICM20948_INIT)
