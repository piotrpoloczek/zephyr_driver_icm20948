// icm20948.c
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

    // Switch to user bank 0 to access accel data registers
    i2c_reg_write_byte_dt(&cfg->i2c, 0x7F, 0x00);

    uint8_t buf[6];
    if (i2c_burst_read_dt(&cfg->i2c, 0x2D, buf, 6) < 0) {
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
    uint8_t whoami = 0;

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    i2c_reg_write_byte_dt(&cfg->i2c, 0x7F, 0x00); // Bank 0
    i2c_reg_write_byte_dt(&cfg->i2c, 0x06, 0x80); // Reset
    k_msleep(100);
    i2c_reg_write_byte_dt(&cfg->i2c, 0x06, 0x01); // Clock auto select
    k_msleep(10);
    i2c_reg_write_byte_dt(&cfg->i2c, 0x07, 0x00); // Enable all sensors

    i2c_reg_write_byte_dt(&cfg->i2c, 0x7F, 0x20); // Bank 2
    i2c_reg_write_byte_dt(&cfg->i2c, 0x14, 0x09); // ACCEL_CONFIG (2g, DLPF)
    i2c_reg_write_byte_dt(&cfg->i2c, 0x10, 0x00); // ACCEL_SMPLRT_DIV_1
    i2c_reg_write_byte_dt(&cfg->i2c, 0x11, 0x01); // ACCEL_SMPLRT_DIV_2
    i2c_reg_write_byte_dt(&cfg->i2c, 0x09, 0x01); // ODR_ALIGN_EN
    i2c_reg_write_byte_dt(&cfg->i2c, 0x7F, 0x00); // Back to Bank 0

    if (i2c_reg_read_byte_dt(&cfg->i2c, 0x00, &whoami) < 0) {
        LOG_ERR("Failed to read WHO_AM_I");
        return -EIO;
    }

    if (whoami != 0xEA) {
        LOG_ERR("Unexpected WHO_AM_I: 0x%02X", whoami);
        return -EINVAL;
    }

    LOG_INF("ICM20948 initialized (WHO_AM_I=0x%02X)", whoami);
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
    };                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                              \
                          icm20948_init, NULL,                               \
                          &icm20948_data_##inst, &icm20948_config_##inst,    \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,          \
                          &icm20948_api);

DT_INST_FOREACH_STATUS_OKAY(ICM20948_INIT)
