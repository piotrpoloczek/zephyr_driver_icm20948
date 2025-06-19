#define DT_DRV_COMPAT invensense_icm20948


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "icm20948.h"
#include "icm20948_reg.h"

LOG_MODULE_REGISTER(icm20948, CONFIG_SENSOR_LOG_LEVEL);

int icm20948_switch_bank(const struct device *dev, uint8_t bank)
{
    const struct icm20948_config *cfg = dev->config;
    return i2c_reg_write_byte_dt(&cfg->i2c, ICM20948_REG_BANK_SEL, bank);
}

int icm20948_write_register(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct icm20948_config *cfg = dev->config;
    return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

int icm20948_read_register(const struct device *dev, uint8_t reg, uint8_t *val)
{
    const struct icm20948_config *cfg = dev->config;
    return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

int icm20948_read_registers(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len)
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

    icm20948_switch_bank(dev, ICM20948_BANK_0);
    icm20948_read_register(dev, ICM20948_WHO_AM_I, &whoami);

    if (whoami != ICM20948_WHO_AM_I_ID) {
        LOG_ERR("Wrong WHO_AM_I: 0x%02X (expected 0x%02X)", whoami, ICM20948_WHO_AM_I_ID);
        return -ENODEV;
    }

    LOG_INF("ICM20948 detected (WHO_AM_I = 0x%02X)", whoami);
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

    // Wake up and enable sensors
    icm20948_switch_bank(dev, ICM20948_BANK_0);
    icm20948_write_register(dev, ICM20948_REG_PWR_MGMT_1, ICM20948_CLKSEL_AUTO);
    icm20948_write_register(dev, ICM20948_REG_PWR_MGMT_2, 0x00);

    // Enable I2C master mode (required for AK09916)
    icm20948_write_register(dev, ICM20948_REG_USER_CTRL, ICM20948_I2C_MST_EN);
    k_msleep(10);

    // Set default FS ranges in software (hardware is already default)
    data->accel_fs = ICM20948_ACCEL_FS_2G;
    data->gyro_fs  = ICM20948_GYRO_FS_250DPS;

    // Optional: configure bypass if external I2C devices are needed
    icm20948_write_register(dev, ICM20948_REG_INT_PIN_CFG, ICM20948_BYPASS_EN);

    // Initialize magnetometer (AK09916)
    if (!icm20948_mag_init(dev)) {
        LOG_WRN("Magnetometer not detected or not responding");
    }

    LOG_INF("ICM20948 initialized and ready");

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
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);

    // --- Read accelerometer, gyroscope, and temperature ---
    icm20948_switch_bank(dev, ICM20948_BANK_0);
    ret = icm20948_read_registers(dev, ICM20948_REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read accel/gyro/temp data");
        k_mutex_unlock(&data->lock);
        return ret;
    }

    float a_scale = get_accel_scale(data->accel_fs);
    float g_scale = get_gyro_scale(data->gyro_fs);

    data->accel[0] = bytes_to_int16(buf[0], buf[1]) * a_scale;
    data->accel[1] = bytes_to_int16(buf[2], buf[3]) * a_scale;
    data->accel[2] = bytes_to_int16(buf[4], buf[5]) * a_scale;

    data->gyro[0] = bytes_to_int16(buf[6], buf[7]) * g_scale;
    data->gyro[1] = bytes_to_int16(buf[8], buf[9]) * g_scale;
    data->gyro[2] = bytes_to_int16(buf[10], buf[11]) * g_scale;

    data->temperature = ((int16_t)((buf[12] << 8) | buf[13])) / 333.87 + 21.0;

    // --- Read magnetometer (AK09916) ---
    icm20948_switch_bank(dev, ICM20948_BANK_0);


    // uint8_t st1 = 0;
    // ret = icm20948_read_mag_register(dev, AK09916_REG_ST1, &st1);
    // if (ret < 0 || !(st1 & AK09916_ST1_DRDY)) {
    //     LOG_WRN("Magnetometer data not ready (manual)");
    //     k_mutex_unlock(&data->lock);
    //     return -EAGAIN;
    // }

    // uint8_t mag_raw[6];
    // for (int i = 0; i < 6; i++) {
    //     icm20948_read_mag_register(dev, AK09916_REG_HXL + i, &mag_raw[i]);
    // }

    // uint8_t st2 = 0;
    // icm20948_read_mag_register(dev, AK09916_REG_ST2, &st2);
    // if (st2 & AK09916_ST2_HOFL) {
    //     LOG_WRN("Magnetometer overflow");
    //     k_mutex_unlock(&data->lock);
    //     return -EIO;
    // }

    // int16_t mx = (int16_t)(mag_raw[1] << 8 | mag_raw[0]);
    // int16_t my = (int16_t)(mag_raw[3] << 8 | mag_raw[2]);
    // int16_t mz = (int16_t)(mag_raw[5] << 8 | mag_raw[4]);

    // data->mag[0] = mx * AK09916_MAG_SCALE;
    // data->mag[1] = my * AK09916_MAG_SCALE;
    // data->mag[2] = mz * AK09916_MAG_SCALE;


    uint8_t mag_buf[9];
    ret = icm20948_read_registers(dev, ICM20948_REG_EXT_SLV_SENS_DATA_00, mag_buf, sizeof(mag_buf));
    
    if (ret < 0) {
        LOG_ERR("Failed to read magnetometer registers");
        k_mutex_unlock(&data->lock);
        return ret;
    }

    LOG_DBG("AK09916 ST1 = 0x%02X ST2 = 0x%02X", mag_buf[0], mag_buf[8]);

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

    int16_t mx = (int16_t)(mag_buf[2] << 8 | mag_buf[1]);
    int16_t my = (int16_t)(mag_buf[4] << 8 | mag_buf[3]);
    int16_t mz = (int16_t)(mag_buf[6] << 8 | mag_buf[5]);

    data->mag[0] = mx * AK09916_MAG_SCALE;
    data->mag[1] = my * AK09916_MAG_SCALE;
    data->mag[2] = mz * AK09916_MAG_SCALE;

    LOG_DBG("Accel raw: %.2f %.2f %.2f", data->accel[0], data->accel[1], data->accel[2]);
    LOG_DBG("Gyro raw: %.2f %.2f %.2f", data->gyro[0], data->gyro[1], data->gyro[2]);
    LOG_DBG("Mag raw: %.2f %.2f %.2f", data->mag[0], data->mag[1], data->mag[2]);

    k_mutex_unlock(&data->lock);
    return 0;
}


int icm20948_channel_get(const struct device *dev,
                         enum sensor_channel chan,
                         struct sensor_value *val)
{
    struct icm20948_data *data = dev->data;

    k_mutex_lock(&data->lock, K_FOREVER);

    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
    case SENSOR_CHAN_ACCEL_Y:
    case SENSOR_CHAN_ACCEL_Z:
        val->val1 = (int32_t)data->accel[chan - SENSOR_CHAN_ACCEL_X];
        val->val2 = (data->accel[chan - SENSOR_CHAN_ACCEL_X] - val->val1) * 1e6;
        break;

    case SENSOR_CHAN_ACCEL_XYZ:
        for (int i = 0; i < 3; i++) {
            val[i].val1 = (int32_t)data->accel[i];
            val[i].val2 = (data->accel[i] - val[i].val1) * 1e6;
        }
        break;

    case SENSOR_CHAN_GYRO_X:
    case SENSOR_CHAN_GYRO_Y:
    case SENSOR_CHAN_GYRO_Z:
        val->val1 = (int32_t)data->gyro[chan - SENSOR_CHAN_GYRO_X];
        val->val2 = (data->gyro[chan - SENSOR_CHAN_GYRO_X] - val->val1) * 1e6;
        break;

    case SENSOR_CHAN_GYRO_XYZ:
        for (int i = 0; i < 3; i++) {
            val[i].val1 = (int32_t)data->gyro[i];
            val[i].val2 = (data->gyro[i] - val[i].val1) * 1e6;
        }
        break;

    case SENSOR_CHAN_MAGN_X:
    case SENSOR_CHAN_MAGN_Y:
    case SENSOR_CHAN_MAGN_Z:
        val->val1 = (int32_t)data->mag[chan - SENSOR_CHAN_MAGN_X];
        val->val2 = (data->mag[chan - SENSOR_CHAN_MAGN_X] - val->val1) * 1e6;
        break;

    case SENSOR_CHAN_MAGN_XYZ:
        for (int i = 0; i < 3; i++) {
            val[i].val1 = (int32_t)data->mag[i];
            val[i].val2 = (data->mag[i] - val[i].val1) * 1e6;
        }
        break;

    case SENSOR_CHAN_DIE_TEMP:
        val->val1 = (int32_t)data->temperature;
        val->val2 = (data->temperature - val->val1) * 1e6;
        break;

    default:
        k_mutex_unlock(&data->lock);
        return -ENOTSUP;
    }

    k_mutex_unlock(&data->lock);
    return 0;
}


int icm20948_write_mag_register(const struct device *dev, uint8_t reg, uint8_t val)
{
    icm20948_switch_bank(dev, ICM20948_BANK_3);

    icm20948_write_register(dev, ICM20948_REG_I2C_SLV4_ADDR, AK09916_I2C_ADDR);       // Write mode
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV4_REG, reg);                     // Target reg
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV4_DO, val);                      // Data
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV4_CTRL, 0x80);                   // Enable

    k_msleep(10); // Wait for transaction to complete

    return 0;
}

int icm20948_read_mag_register(const struct device *dev, uint8_t reg, uint8_t *val)
{
    icm20948_switch_bank(dev, ICM20948_BANK_3);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV4_ADDR, AK09916_I2C_ADDR | 0x80); // Read
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV4_REG, reg);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV4_CTRL, 0x81);  // 1 byte
    k_msleep(10);

    icm20948_switch_bank(dev, ICM20948_BANK_0);
    return icm20948_read_register(dev, ICM20948_REG_I2C_SLV4_DI, val);
}


bool icm20948_mag_init(const struct device *dev)
{
    bool init_success = false;

    ak09916_reset(dev);
    k_msleep(50);
    icm20948_set_mag_mode(dev, AK09916_CONT_MODE_100HZ);
    k_msleep(10);
    icm20948_setup_auto_read_mag(dev);

    // Enable I2C master mode
    icm20948_switch_bank(dev, ICM20948_BANK_0);
    icm20948_write_register(dev, ICM20948_REG_USER_CTRL, ICM20948_I2C_MST_EN);
    k_msleep(10);

    // Set I2C master clock
    icm20948_switch_bank(dev, ICM20948_BANK_3);
    icm20948_write_register(dev, ICM20948_REG_I2C_MST_CTRL, 0x07);  // ~345.6 kHz
    k_msleep(10);

    // 🟡 Add WHO_AM_I check here
    if (!icm20948_mag_check_whoami(dev)) {
        LOG_ERR("AK09916 WHO_AM_I failed");
        return false;
    }

    LOG_INF("AK09916 magnetometer detected");

    // Continue setup: continuous read of magnetometer data
    icm20948_set_mag_mode(dev, AK09916_CONT_MODE_100HZ);
    icm20948_switch_bank(dev, ICM20948_BANK_3);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // read
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_REG, AK09916_REG_ST1);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_CTRL, 0x89); // enable, 9 bytes

    return true;
}




int icm20948_set_mag_mode(const struct device *dev, enum ak09916_mode mode)
{
    icm20948_switch_bank(dev, ICM20948_BANK_3);

    // Write to CNTL2 register to set mode
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR); // Write
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_REG, AK09916_REG_CNTL2);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_DO, mode);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_CTRL, 0x81); // Enable, 1 byte write

    k_msleep(10);

    // Disable slave write slot after execution (optional cleanup)
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_CTRL, 0x00);

    return 0;
}


void ak09916_reset(const struct device *dev)
{
    icm20948_write_mag_register(dev, AK09916_REG_CNTL3, 0x01);
    k_msleep(100);
}



bool icm20948_mag_check_whoami(const struct device *dev)
{
    uint8_t whoami = 0;

    icm20948_switch_bank(dev, ICM20948_BANK_3);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // read
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_REG, AK09916_REG_WHO_AM_I);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_CTRL, 0x81); // enable, 1 byte

    k_msleep(20);

    icm20948_switch_bank(dev, ICM20948_BANK_0);
    icm20948_read_register(dev, ICM20948_REG_EXT_SLV_SENS_DATA_00, &whoami);

    if (whoami == AK09916_WHO_AM_I_1 || whoami == AK09916_WHO_AM_I_2) {
        return true;
    }

    LOG_ERR("AK09916 WHO_AM_I failed: 0x%02X", whoami);
    return false;
}



void icm20948_setup_auto_read_mag(const struct device *dev) {
    icm20948_switch_bank(dev, ICM20948_BANK_3);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80);  // Read mode
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_REG, AK09916_REG_ST1);
    icm20948_write_register(dev, ICM20948_REG_I2C_SLV0_CTRL, 0x89); // Enable, 9 bytes
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
