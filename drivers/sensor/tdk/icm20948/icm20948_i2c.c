/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icm20948

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>

#include "icm20948.h"

LOG_MODULE_DECLARE(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

static int icm20948_i2c_read_data(const struct device *dev, uint8_t reg, uint8_t *data, size_t len)
{
	const struct icm20948_config *config = dev->config;
	
	return i2c_burst_read_dt(&config->i2c, reg, data, len);
}

static int icm20948_i2c_write_data(const struct device *dev, uint8_t reg, uint8_t *data, size_t len)
{
	const struct icm20948_config *config = dev->config;

	return i2c_burst_write_dt(&config->i2c, reg, data, len);
}

static int icm20948_i2c_write_reg(const struct device *dev, uint8_t reg, uint8_t data)
{
	const struct icm20948_config *config = dev->config;

	return i2c_reg_write_byte_dt(&config->i2c, reg, data);
}

static int icm20948_i2c_read_reg(const struct device *dev, uint8_t reg, uint8_t *data)
{
	const struct icm20948_config *config = dev->config;

	return i2c_reg_read_byte_dt(&config->i2c, reg, data);
}

static int icm20948_i2c_update_reg(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t value)
{
	const struct icm20948_config *config = dev->config;
	uint8_t reg_val;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, reg_addr, &reg_val);
	if (ret < 0) {
		return ret;
	}

	reg_val = (reg_val & ~mask) | (value & mask);

	return i2c_reg_write_byte_dt(&config->i2c, reg_addr, reg_val);
}

/* I2C transfer function structure */
static const struct icm20948_transfer_function icm20948_i2c_transfer_fn = {
	.read_reg = icm20948_i2c_read_data,
	.write_reg = icm20948_i2c_write_data,
};

int icm20948_i2c_init(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	const struct icm20948_config *config = dev->config;

	/* Check if I2C bus is ready */
	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	/* Set transfer functions */
	data->hw_tf = &icm20948_i2c_transfer_fn;

	LOG_DBG("ICM20948 I2C interface initialized");
	return 0;
}
