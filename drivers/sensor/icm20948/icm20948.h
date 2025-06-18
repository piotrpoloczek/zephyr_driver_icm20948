#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define ICM20948_I2C_ADDR      0x68  // Default I2C address when AD0 = low

// Sensor full-scale range settings
enum icm20948_accel_fs {
    ICM20948_ACCEL_FS_2G  = 0,
    ICM20948_ACCEL_FS_4G  = 1,
    ICM20948_ACCEL_FS_8G  = 2,
    ICM20948_ACCEL_FS_16G = 3,
};

enum icm20948_gyro_fs {
    ICM20948_GYRO_FS_250DPS  = 0,
    ICM20948_GYRO_FS_500DPS  = 1,
    ICM20948_GYRO_FS_1000DPS = 2,
    ICM20948_GYRO_FS_2000DPS = 3,
};

enum icm20948_bank {
    ICM20948_BANK_0 = 0x00,
    ICM20948_BANK_1 = 0x10,
    ICM20948_BANK_2 = 0x20,
    ICM20948_BANK_3 = 0x30,
};

// Magnetometer (AK09916) operation mode
enum ak09916_mode {
    AK09916_POWER_DOWN        = 0x00,
    AK09916_SINGLE_MEASURE   = 0x01,
    AK09916_CONT_MODE_10HZ   = 0x02,
    AK09916_CONT_MODE_20HZ   = 0x04,
    AK09916_CONT_MODE_50HZ   = 0x06,
    AK09916_CONT_MODE_100HZ  = 0x08,
    AK09916_SELF_TEST        = 0x10
};

// WHO_AM_I expected value for ICM20948
#define ICM20948_WHO_AM_I_ID 0xEA

// WHO_AM_I expected values for AK09916
#define AK09916_WHO_AM_I_1 0x09
#define AK09916_WHO_AM_I_2 0x0A

// Scale factors (raw -> SI units)
#define ICM20948_ACCEL_SCALE_2G     (9.80665 / 16384.0)
#define ICM20948_ACCEL_SCALE_4G     (9.80665 / 8192.0)
#define ICM20948_ACCEL_SCALE_8G     (9.80665 / 4096.0)
#define ICM20948_ACCEL_SCALE_16G    (9.80665 / 2048.0)

#define ICM20948_GYRO_SCALE_250DPS   (1.0 / 131.0)
#define ICM20948_GYRO_SCALE_500DPS   (1.0 / 65.5)
#define ICM20948_GYRO_SCALE_1000DPS  (1.0 / 32.8)
#define ICM20948_GYRO_SCALE_2000DPS  (1.0 / 16.4)

#define AK09916_MAG_SCALE            (0.15F)  // µT per LSB

struct icm20948_config {
    struct i2c_dt_spec i2c;
};

struct icm20948_data {
    float accel[3]; // in m/s^2
    float gyro[3];  // in degrees/sec
    float mag[3];   // in microtesla
    float temperature;

    uint8_t accel_fs;
    uint8_t gyro_fs;

    struct k_mutex lock;
};

// Bank and register handling
int icm20948_switch_bank(const struct device *dev, uint8_t bank);
int icm20948_read_register(const struct device *dev, uint8_t reg, uint8_t *val);
int icm20948_write_register(const struct device *dev, uint8_t reg, uint8_t val);
int icm20948_read_registers(const struct device *dev, uint8_t reg, uint8_t *buf, size_t len);

// High-level init
bool icm20948_init_chip(const struct device *dev);
bool icm20948_mag_init(const struct device *dev);

// Motion configuration
int icm20948_set_accel_fs(const struct device *dev, enum icm20948_accel_fs fs);
int icm20948_set_gyro_fs(const struct device *dev, enum icm20948_gyro_fs fs);
int icm20948_set_mag_mode(const struct device *dev, enum ak09916_mode mode);

// Zephyr Sensor API
int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan);
int icm20948_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val);

