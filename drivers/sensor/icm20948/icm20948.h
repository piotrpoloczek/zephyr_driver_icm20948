#pragma once
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

struct icm20948_config {
    struct i2c_dt_spec i2c;
};

struct icm20948_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
};
