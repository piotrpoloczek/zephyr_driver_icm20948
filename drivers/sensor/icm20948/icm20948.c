#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"
#include "icm20948_reg.h"

LOG_MODULE_REGISTER(icm20948, CONFIG_SENSOR_LOG_LEVEL);

static int icm20948_switch_bank(const struct device *dev, uint8_t bank)
{
    const struct icm20948_config *cfg = dev->config;
    return i2c_reg_write_byte_dt(&cfg->i2c, ICM20948_REG_BANK_SEL, bank);
}

static int icm20948_write_register(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct icm20948_config *cfg = dev->config;
    return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

static int icm20948_read_register(const struct device *dev, uint8_t reg, uint8_t *val)
{
    const struct icm20948_config *cfg = dev->config;
    return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

static int icm20948_read_registers(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    const struct icm20948_config *cfg = dev->config;
    return i2c_burst_read_dt(&cfg->i2c, reg, buf, len);
}

static int icm20948_reset_chip(const struct device *dev)
{
    icm20948_switch_bank(dev, ICM20948_BANK0);
    icm20948_write_register(dev, ICM20948_REG_PWR_MGMT_1, ICM20948_DEVICE_RESET);
    k_msleep(100);

    // Wake from sleep
    return icm20948_write_register(dev, ICM20948_REG_PWR_MGMT_1, ICM20948_CLKSEL_AUTO);
}

static int icm20948_chip_check(const struct device *dev)
{
    uint8_t whoami = 0;
    icm20948_switch_bank(dev, ICM20948_BANK0);
    icm20948_read_register(dev, ICM20948_WHO_AM_I, &whoami);
    if (whoami != ICM20948_WHO_AM_I_ID) {
        LOG_ERR("Wrong WHO_AM_I: 0x%02X (expected 0x%02X)", whoami, ICM20948_WHO_AM_I_ID);
        return -ENODEV;
    }
    return 0;
}

static int icm20948_sensor_init(const struct device *dev)
{
    struct icm20948_data *data = dev->data;

    k_mutex_init(&data->lock);

    if (icm20948_reset_chip(dev) < 0) {
        LOG_ERR("Failed to reset ICM20948");
        return -EIO;
    }

    if (icm20948_chip_check(dev) < 0) {
        return -ENODEV;
    }

    // Disable sleep, enable accel/gyro
    icm20948_switch_bank(dev, ICM20948_BANK0);
    icm20948_write_register(dev, ICM20948_REG_PWR_MGMT_1, ICM20948_CLKSEL_AUTO);
    icm20948_write_register(dev, ICM20948_REG_PWR_MGMT_2, 0x00);

    // Disable I2C master mode for bypass
    icm20948_write_register(dev, ICM20948_REG_USER_CTRL, 0x00);
    icm20948_write_register(dev, ICM20948_REG_INT_PIN_CFG, ICM20948_BYPASS_EN);

    LOG_INF("ICM20948 initialized");

    return 0;
}

// Basic sensor API stubs
static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    ARG_UNUSED(chan);
    // Will be implemented later
    return 0;
}

static int icm20948_channel_get(const struct device *dev,
                                 enum sensor_channel chan,
                                 struct sensor_value *val)
{
    ARG_UNUSED(chan);
    ARG_UNUSED(val);
    // Will be implemented later
    return 0;
}

static int16_t bytes_to_int16(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8) | lsb);
}

static float get_accel_scale(uint8_t fs) {
    switch (fs) {
    case ICM20948_ACCEL_FS_2G:  return ICM20948_ACCEL_SCALE_2G;
    case ICM20948_ACCEL_FS_4G:  return ICM20948_ACCEL_SCALE_4G;
    case ICM20948_ACCEL_FS_8G:  return ICM20948_ACCEL_SCALE_8G;
    case ICM20948_ACCEL_FS_16G: return ICM20948_ACCEL_SCALE_16G;
    default: return ICM20948_ACCEL_SCALE_2G;
    }
}

static float get_gyro_scale(uint8_t fs) {
    switch (fs) {
    case ICM20948_GYRO_FS_250DPS:  return ICM20948_GYRO_SCALE_250DPS;
    case ICM20948_GYRO_FS_500DPS:  return ICM20948_GYRO_SCALE_500DPS;
    case ICM20948_GYRO_FS_1000DPS: return ICM20948_GYRO_SCALE_1000DPS;
    case ICM20948_GYRO_FS_2000DPS: return ICM20948_GYRO_SCALE_2000DPS;
    default: return ICM20948_GYRO_SCALE_250DPS;
    }
}

int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct icm20948_data *data = dev->data;
    uint8_t buf[14];

    k_mutex_lock(&data->lock, K_FOREVER);

    // Switch to bank 0
    icm20948_switch_bank(dev, ICM20948_BANK0);

    // Read accel, gyro, temp in one shot
    int ret = icm20948_read_registers(dev, ICM20948_REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (ret < 0) {
        k_mutex_unlock(&data->lock);
        LOG_ERR("Failed to read IMU data");
        return ret;
    }

    // Unpack and convert raw values
    float a_scale = get_accel_scale(data->accel_fs);
    float g_scale = get_gyro_scale(data->gyro_fs);

    data->accel[0] = bytes_to_int16(buf[0], buf[1]) * a_scale;
    data->accel[1] = bytes_to_int16(buf[2], buf[3]) * a_scale;
    data->accel[2] = bytes_to_int16(buf[4], buf[5]) * a_scale;

    data->gyro[0] = bytes_to_int16(buf[6], buf[7]) * g_scale;
    data->gyro[1] = bytes_to_int16(buf[8], buf[9]) * g_scale;
    data->gyro[2] = bytes_to_int16(buf[10], buf[11]) * g_scale;

    data->temperature = ((int16_t)((buf[12] << 8) | buf[13])) / 333.87 + 21.0;

        // Read 6 bytes magnetometer data via I2C master -> EXT_SLV_SENS_DATA_00
    icm20948_switch_bank(dev, ICM20948_BANK_0);

    uint8_t mag_buf[9];
    ret = icm20948_read_registers(dev, ICM20948_REG_EXT_SLV_SENS_DATA_00, mag_buf, 9);
    if (ret < 0) {
        k_mutex_unlock(&data->lock);
        LOG_ERR("Failed to read magnetometer data");
        return ret;
    }

    // Check DRDY and overflow
    if (!(mag_buf[0] & AK09916_ST1_DRDY)) {
        LOG_WRN("Magnetometer data not ready");
        k_mutex_unlock(&data->lock);
        return -EAGAIN;
    }

    if (mag_buf[8] & AK09916_ST2_HOFL) {
        LOG_WRN("Magnetometer overflow");
        k_mutex_unlock(&data->lock);
        return -EIO;
    }

    // Convert to signed values
    int16_t mx = (int16_t)(mag_buf[2] << 8 | mag_buf[1]);
    int16_t my = (int16_t)(mag_buf[4] << 8 | mag_buf[3]);
    int16_t mz = (int16_t)(mag_buf[6] << 8 | mag_buf[5]);

    data->mag[0] = mx * AK09916_MAG_SCALE;
    data->mag[1] = my * AK09916_MAG_SCALE;
    data->mag[2] = mz * AK09916_MAG_SCALE;


    k_mutex_unlock(&data->lock);
    return 0;
}

int icm20948_channel_get(const struct device *dev,
                         enum sensor_channel chan,
                         struct sensor_value *val)
{
    struct icm20948_data *data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);

    float *src = NULL;
    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
        val->val1 = (int32_t)data->accel[0];
        val->val2 = (data->accel[0] - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_ACCEL_Y:
        val->val1 = (int32_t)data->accel[1];
        val->val2 = (data->accel[1] - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_ACCEL_Z:
        val->val1 = (int32_t)data->accel[2];
        val->val2 = (data->accel[2] - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_GYRO_X:
        val->val1 = (int32_t)data->gyro[0];
        val->val2 = (data->gyro[0] - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_GYRO_Y:
        val->val1 = (int32_t)data->gyro[1];
        val->val2 = (data->gyro[1] - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_GYRO_Z:
        val->val1 = (int32_t)data->gyro[2];
        val->val2 = (data->gyro[2] - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_DIE_TEMP:
        val->val1 = (int32_t)data->temperature;
        val->val2 = (data->temperature - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_ACCEL_XYZ:
        case SENSOR_CHAN_MAGN_X:
        val->val1 = (int32_t)data->mag[0];
        val->val2 = (data->mag[0] - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_MAGN_Y:
        val->val1 = (int32_t)data->mag[1];
        val->val2 = (data->mag[1] - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_MAGN_Z:
        val->val1 = (int32_t)data->mag[2];
        val->val2 = (data->mag[2] - val->val1) * 1e6;
        break;
    case SENSOR_CHAN_GYRO_XYZ:
    default:
        k_mutex_unlock(&data->lock);
        return -ENOTSUP;
    }

    k_mutex_unlock(&data->lock);
    return 0;
}


bool icm20948_mag_init(const struct device *dev)
{
    bool init_success = false;
    uint8_t tries = 0;

    icm20948_switch_bank(dev, ICM20948_BANK_0);
    icm20948_write_register(dev, ICM20948_REG_USER_CTRL, ICM20948_I2C_MST_EN);
    k_msleep(10);

    icm20948_switch_bank(dev, ICM20948_BANK_3);
    icm20948_write_register(dev, ICM20948_REG_I2C_MST_CTRL, 0x07);  // 345.6 kHz
    k_msleep(10);

    while (!init_success && tries < 10) {
        // Read AK09916 WHO_AM_I
        icm20948_switch_bank(dev, ICM20948_BANK_3);
        icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // read
        icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_REG, AK09916_REG_WHO_AM_I);
        icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_CTRL, 0x81); // 1 byte

        k_msleep(20); // give time for I2C transaction

        icm20948_switch_bank(dev, ICM20948_BANK_0);
        uint8_t whoami = 0;
        icm20948_read_register(dev, ICM20948_REG_EXT_SLV_SENS_DATA_00, &whoami);

        if (whoami == AK09916_WHO_AM_I_1 || whoami == AK09916_WHO_AM_I_2) {
            init_success = true;
        } else {
            tries++;
            k_msleep(10);
        }
    }

    if (init_success) {
        icm20948_set_mag_mode(dev, AK09916_CONT_MODE_100HZ);
        LOG_INF("AK09916 magnetometer initialized.");
    } else {
        LOG_ERR("AK09916 WHO_AM_I failed");
    }

    return init_success;
}

int icm20948_set_mag_mode(const struct device *dev, enum ak09916_mode mode)
{
    icm20948_switch_bank(dev, ICM20948_BANK_3);

    // Write mode to CNTL2
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR); // write
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_REG, AK09916_REG_CNTL2);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_DO, mode);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_CTRL, 0x81); // 1 byte

    k_msleep(10);
    return 0;
}





static const struct sensor_driver_api icm20948_api_funcs = {
    .sample_fetch = icm20948_sample_fetch,
    .channel_get  = icm20948_channel_get,
};

#define ICM20948_DEFINE(inst)                                              \
    static struct icm20948_data icm20948_data_##inst;                      \
    static const struct icm20948_config icm20948_config_##inst = {        \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                 \
    };                                                                     \
    DEVICE_DT_INST_DEFINE(inst,                                           \
        icm20948_sensor_init,                                              \
        NULL,                                                              \
        &icm20948_data_##inst,                                             \
        &icm20948_config_##inst,                                           \
        POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                         \
        &icm20948_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(ICM20948_DEFINE)
