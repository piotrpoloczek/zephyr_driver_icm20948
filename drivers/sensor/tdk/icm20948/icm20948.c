/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_icm20948

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>

#include "icm20948.h"

LOG_MODULE_REGISTER(ICM20948, CONFIG_SENSOR_LOG_LEVEL);

/* Sensitivity values based on full-scale settings */
static const uint16_t accel_sensitivity_shift[] = { 16384, 8192, 4096, 2048 }; /* LSB/g */
static const uint16_t gyro_sensitivity_shift[] = { 131, 65, 33, 16 }; /* LSB/(deg/s) */




bool icm20948_is_connected(const struct device *dev)
{
    uint8_t who_am_i;
    int ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_WHO_AM_I, &who_am_i);
    if (ret < 0) {
        return false;
    }
    return (who_am_i == 0xEA);
}


uint8_t icm20948_get_who_am_i(const struct device *dev)
{
    uint8_t who_am_i = 0;
    int ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_WHO_AM_I, &who_am_i);
    if (ret < 0) {
        LOG_ERR("Failed to read WHO_AM_I register");
        return 0;
    }
    return who_am_i;
}

int icm20948_data_ready(const struct device *dev)
{
    uint8_t status;
    int ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_INT_STATUS, &status);
    if (ret < 0) {
        return ret;
    }
    return (status & BIT(0)) ? 1 : 0;
}


/* Power Management Functions */
int icm20948_sleep(const struct device *dev, bool enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_PWR_MGMT_1, 
                              BIT(6), enable ? BIT(6) : 0);
}

int icm20948_low_power(const struct device *dev, bool enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_PWR_MGMT_1, 
                              BIT(5), enable ? BIT(5) : 0);
}

int icm20948_set_clock_source(const struct device *dev, uint8_t clock_source)
{
    if (clock_source > 7) {
        return -EINVAL;
    }
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_PWR_MGMT_1, 
                              0x07, clock_source);
}

/* Interrupt Configuration Functions */
int icm20948_clear_interrupts(const struct device *dev)
{
    uint8_t int_status;
    uint8_t int_status_1;
    int ret;
    
    /* Read interrupt status registers to clear interrupts */
    ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_INT_STATUS, &int_status);
    if (ret < 0) {
        return ret;
    }
    
    ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_INT_STATUS_1, &int_status_1);
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}

int icm20948_cfg_int_active_low(const struct device *dev, bool active_low)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_PIN_CFG, 
                              BIT(7), active_low ? BIT(7) : 0);
}

int icm20948_cfg_int_open_drain(const struct device *dev, bool open_drain)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_PIN_CFG, 
                              BIT(6), open_drain ? BIT(6) : 0);
}

int icm20948_cfg_int_latch(const struct device *dev, bool latching)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_PIN_CFG, 
                              BIT(5), latching ? BIT(5) : 0);
}

int icm20948_cfg_int_any_read_to_clear(const struct device *dev, bool enabled)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_PIN_CFG, 
                              BIT(4), enabled ? BIT(4) : 0);
}

/* Individual Interrupt Enable Functions */
int icm20948_int_enable_i2c(const struct device *dev, bool enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_ENABLE, 
                              BIT(6), enable ? BIT(6) : 0);
}

int icm20948_int_enable_pll(const struct device *dev, bool enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_ENABLE, 
                              BIT(2), enable ? BIT(2) : 0);
}

int icm20948_int_enable_wom(const struct device *dev, bool enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_ENABLE, 
                              BIT(3), enable ? BIT(3) : 0);
}

int icm20948_int_enable_raw_data_ready(const struct device *dev, bool enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_ENABLE_1, 
                              BIT(0), enable ? BIT(0) : 0);
}

int icm20948_int_enable_fifo_overflow(const struct device *dev, uint8_t bm_enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_ENABLE_2, 
                              0x1F, bm_enable & 0x1F);
}

int icm20948_int_enable_fifo_watermark(const struct device *dev, uint8_t bm_enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_INT_ENABLE_3, 
                              0x1F, bm_enable & 0x1F);
}

/* Wake-on-Motion Functions */
int icm20948_wom_logic(const struct device *dev, uint8_t enable, uint8_t mode)
{
    int ret;
    uint8_t ctrl_val = 0;
    
    if (enable) {
        ctrl_val |= BIT(7); /* ACCEL_INTEL_EN */
    }
    if (mode) {
        ctrl_val |= BIT(6); /* ACCEL_INTEL_MODE_INT */
    }
    
    ret = icm20948_write_reg(dev, ICM20948_BANK_2, ICM20948_ACCEL_INTEL_CTRL, ctrl_val);
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}

int icm20948_wom_threshold(const struct device *dev, uint8_t threshold)
{
    return icm20948_write_reg(dev, ICM20948_BANK_2, ICM20948_ACCEL_WOM_THR, threshold);
}

/* FIFO Functions */
int icm20948_enable_fifo(const struct device *dev, bool enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_USER_CTRL, 
                              BIT(6), enable ? BIT(6) : 0);
}

int icm20948_reset_fifo(const struct device *dev)
{
    int ret;
    
    /* Reset FIFO */
    ret = icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_FIFO_RST, 0x1F, 0x1F);
    if (ret < 0) {
        return ret;
    }
    
    /* Wait for reset to complete */
    k_sleep(K_MSEC(1));
    
    /* Clear reset bits */
    ret = icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_FIFO_RST, 0x1F, 0x00);
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}

int icm20948_set_fifo_mode(const struct device *dev, bool snapshot)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_FIFO_MODE, 
                              BIT(0), snapshot ? BIT(0) : 0);
}

int icm20948_get_fifo_count(const struct device *dev, uint16_t *count)
{
    uint8_t fifo_count_h, fifo_count_l;
    int ret;
    
    ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_FIFO_COUNTH, &fifo_count_h);
    if (ret < 0) {
        return ret;
    }
    
    ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_FIFO_COUNTL, &fifo_count_l);
    if (ret < 0) {
        return ret;
    }
    
    *count = (fifo_count_h << 8) | fifo_count_l;
    return 0;
}

int icm20948_read_fifo(const struct device *dev, uint8_t *data, uint16_t len)
{
    return icm20948_read_data(dev, ICM20948_BANK_0, ICM20948_FIFO_R_W, data, len);
}

/* Calibration and Bias Functions */
int icm20948_set_accel_bias(const struct device *dev, int32_t bias_x, int32_t bias_y, int32_t bias_z)
{
    int ret;
    uint8_t bias_data[2];
    
    /* Convert bias values to 16-bit signed values and write to offset registers */
    /* X-axis bias */
    bias_data[0] = (int16_t)(bias_x >> 8) & 0xFF;
    bias_data[1] = (int16_t)bias_x & 0xFF;
    ret = icm20948_write_data(dev, ICM20948_BANK_1, ICM20948_XA_OFFS_H, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    
    /* Y-axis bias */
    bias_data[0] = (int16_t)(bias_y >> 8) & 0xFF;
    bias_data[1] = (int16_t)bias_y & 0xFF;
    ret = icm20948_write_data(dev, ICM20948_BANK_1, ICM20948_YA_OFFS_H, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    
    /* Z-axis bias */
    bias_data[0] = (int16_t)(bias_z >> 8) & 0xFF;
    bias_data[1] = (int16_t)bias_z & 0xFF;
    ret = icm20948_write_data(dev, ICM20948_BANK_1, ICM20948_ZA_OFFS_H, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}

int icm20948_get_accel_bias(const struct device *dev, int32_t *bias_x, int32_t *bias_y, int32_t *bias_z)
{
    int ret;
    uint8_t bias_data[2];
    
    /* Read X-axis bias */
    ret = icm20948_read_data(dev, ICM20948_BANK_1, ICM20948_XA_OFFS_H, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    *bias_x = (int32_t)((int16_t)((bias_data[0] << 8) | bias_data[1]));
    
    /* Read Y-axis bias */
    ret = icm20948_read_data(dev, ICM20948_BANK_1, ICM20948_YA_OFFS_H, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    *bias_y = (int32_t)((int16_t)((bias_data[0] << 8) | bias_data[1]));
    
    /* Read Z-axis bias */
    ret = icm20948_read_data(dev, ICM20948_BANK_1, ICM20948_ZA_OFFS_H, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    *bias_z = (int32_t)((int16_t)((bias_data[0] << 8) | bias_data[1]));
    
    return 0;
}

int icm20948_set_gyro_bias(const struct device *dev, int32_t bias_x, int32_t bias_y, int32_t bias_z)
{
    int ret;
    uint8_t bias_data[2];
    
    /* Convert bias values to 16-bit signed values and write to offset registers */
    /* X-axis bias */
    bias_data[0] = (int16_t)(bias_x >> 8) & 0xFF;
    bias_data[1] = (int16_t)bias_x & 0xFF;
    ret = icm20948_write_data(dev, ICM20948_BANK_2, ICM20948_XG_OFFS_USRH, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    
    /* Y-axis bias */
    bias_data[0] = (int16_t)(bias_y >> 8) & 0xFF;
    bias_data[1] = (int16_t)bias_y & 0xFF;
    ret = icm20948_write_data(dev, ICM20948_BANK_2, ICM20948_YG_OFFS_USRH, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    
    /* Z-axis bias */
    bias_data[0] = (int16_t)(bias_z >> 8) & 0xFF;
    bias_data[1] = (int16_t)bias_z & 0xFF;
    ret = icm20948_write_data(dev, ICM20948_BANK_2, ICM20948_ZG_OFFS_USRH, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}

int icm20948_get_gyro_bias(const struct device *dev, int32_t *bias_x, int32_t *bias_y, int32_t *bias_z)
{
    int ret;
    uint8_t bias_data[2];
    
    /* Read X-axis bias */
    ret = icm20948_read_data(dev, ICM20948_BANK_2, ICM20948_XG_OFFS_USRH, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    *bias_x = (int32_t)((int16_t)((bias_data[0] << 8) | bias_data[1]));
    
    /* Read Y-axis bias */
    ret = icm20948_read_data(dev, ICM20948_BANK_2, ICM20948_YG_OFFS_USRH, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    *bias_y = (int32_t)((int16_t)((bias_data[0] << 8) | bias_data[1]));
    
    /* Read Z-axis bias */
    ret = icm20948_read_data(dev, ICM20948_BANK_2, ICM20948_ZG_OFFS_USRH, bias_data, 2);
    if (ret < 0) {
        return ret;
    }
    *bias_z = (int32_t)((int16_t)((bias_data[0] << 8) | bias_data[1]));
    
    return 0;
}

/* DMP Functions */
int icm20948_enable_dmp(const struct device *dev, bool enable)
{
    return icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_USER_CTRL, 
                              BIT(7), enable ? BIT(7) : 0);
}

int icm20948_reset_dmp(const struct device *dev)
{
    int ret;
    
    /* Reset DMP */
    ret = icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_USER_CTRL, BIT(3), BIT(3));
    if (ret < 0) {
        return ret;
    }
    
    /* Wait for reset to complete */
    k_sleep(K_MSEC(1));
    
    /* Clear reset bit */
    ret = icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_USER_CTRL, BIT(3), 0);
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}

int icm20948_load_dmp_firmware(const struct device *dev)
{
    /* DMP firmware loading would require the actual firmware binary */
    /* This is a placeholder - actual implementation would need firmware data */
    LOG_WRN("DMP firmware loading not implemented - requires firmware binary");
    return -ENOTSUP;
}

int icm20948_dmp_set_start_address(const struct device *dev, uint16_t address)
{
    int ret;
    
    ret = icm20948_write_reg(dev, ICM20948_BANK_2, ICM20948_PRGM_STRT_ADDRH, (address >> 8) & 0xFF);
    if (ret < 0) {
        return ret;
    }

    ret = icm20948_write_reg(dev, ICM20948_BANK_2, ICM20948_PRGM_STRT_ADDRL, address & 0xFF);
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}


/* Bank selection helper functions */
static int icm20948_set_bank(const struct device *dev, uint8_t bank)
{
	struct icm20948_data *data = dev->data;
	uint8_t bank_val = bank & 0x03; /* Only 2 bits for bank selection */
	bank_val = bank_val << 4; /* Banks are in bits 4 and 5 */

	if (data->current_bank == bank_val) {
		return 0; /* Already on the correct bank */
	}

	int ret = data->hw_tf->write_reg(dev, ICM20948_REG_BANK_SEL, &bank_val, 1);
	if (ret == 0) {
		data->current_bank = bank_val;
	}

	return ret;
}

/* Low-level register access functions */
int icm20948_read_reg(const struct device *dev, uint8_t bank, uint8_t reg, uint8_t *val)
{
	int ret = icm20948_set_bank(dev, bank);
	if (ret < 0) {
		return ret;
	}

	struct icm20948_data *data = dev->data;
	return data->hw_tf->read_reg(dev, reg, val, 1);
}

int icm20948_write_reg(const struct device *dev, uint8_t bank, uint8_t reg, uint8_t val)
{
	int ret = icm20948_set_bank(dev, bank);
	if (ret < 0) {
		return ret;
	}

	struct icm20948_data *data = dev->data;
	return data->hw_tf->write_reg(dev, reg, &val, 1);
}

int icm20948_read_data(const struct device *dev, uint8_t bank, uint8_t reg, uint8_t *data, size_t len)
{
	int ret = icm20948_set_bank(dev, bank);
	if (ret < 0) {
		return ret;
	}

	struct icm20948_data *drv_data = dev->data;
	return drv_data->hw_tf->read_reg(dev, reg, data, len);
}

int icm20948_write_data(const struct device *dev, uint8_t bank, uint8_t reg, uint8_t *data, size_t len)
{
	int ret = icm20948_set_bank(dev, bank);
	if (ret < 0) {
		return ret;
	}

	struct icm20948_data *drv_data = dev->data;
	return drv_data->hw_tf->write_reg(dev, reg, data, len);
}

int icm20948_update_reg(const struct device *dev, uint8_t bank, uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t old_val;
	int ret = icm20948_read_reg(dev, bank, reg, &old_val);
	if (ret < 0) {
		return ret;
	}

	uint8_t new_val = (old_val & ~mask) | (value & mask);
	if (new_val == old_val) {
		return 0; /* No change needed */
	}

	return icm20948_write_reg(dev, bank, reg, new_val);
}

/* I2C Master helper functions for magnetometer access */
static int icm20948_i2c_master_read_reg(const struct device *dev, uint8_t addr, uint8_t reg, uint8_t *val)
{
	int ret;
	uint8_t status;

	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV4_ADDR, addr | 0x80);
	if (ret < 0) {
		return ret;
	}
	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV4_REG, reg);
	if (ret < 0) {
		return ret;
	}
	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLV4_CTRL_EN);
	if (ret < 0) {
		return ret;
	}

	/* Wait for the transaction to complete */
	k_sleep(K_MSEC(1));
	ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_I2C_MST_STATUS, &status);
	if (ret < 0) {
		return ret;
	}

	if (status & ICM20948_I2C_MST_STATUS_SLV4_NACK) {
		return -EIO;
	}

	return icm20948_read_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV4_DI, val);
}

static int icm20948_i2c_master_write_reg(const struct device *dev, uint8_t addr, uint8_t reg, uint8_t val)
{
	int ret;
	uint8_t status;

	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV4_ADDR, addr);
	if (ret < 0) {
		return ret;
	}
	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV4_REG, reg);
	if (ret < 0) {
		return ret;
	}
	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV4_DO, val);
	if (ret < 0) {
		return ret;
	}
	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLV4_CTRL_EN);
	if (ret < 0) {
		return ret;
	}

	/* Wait for the transaction to complete */
	k_sleep(K_MSEC(1));
	ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_I2C_MST_STATUS, &status);
	if (ret < 0) {
		return ret;
	}

	if (status & ICM20948_I2C_MST_STATUS_SLV4_NACK) {
		return -EIO;
	}

	return 0;
}

/* Device initialization and configuration */
static int icm20948_software_reset(const struct device *dev)
{
	/* Set the device reset bit in PWR_MGMT_1 */
	int ret = icm20948_write_reg(dev, ICM20948_BANK_0, ICM20948_PWR_MGMT_1, BIT(7));
	if (ret < 0) {
		return ret;
	}

	/* Wait for the reset to complete */
	k_sleep(K_MSEC(100));
	
	/* Clear the reset bit */
	uint8_t pwr_mgmt_1 = 0x00;
	ret = icm20948_write_reg(dev, ICM20948_BANK_0, ICM20948_PWR_MGMT_1, pwr_mgmt_1);
	if (ret < 0) {
		return ret;
	}

	/* Wait for the device to stabilize */
	k_sleep(K_MSEC(100));
	
	return 0;
}

static int icm20948_check_chip_id(const struct device *dev)
{
	uint8_t chip_id;
	int ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_WHO_AM_I, &chip_id);
	if (ret < 0) {
		return ret;
	}

	if (chip_id != 0xEA) {
		LOG_ERR("Invalid chip ID: 0x%02x, expected 0xEA", chip_id);
		//return -EIO;
	}

	return 0;
}

static int icm20948_setup_power_management(const struct device *dev)
{
	int ret;

	/* Clear sleep bit */
	ret = icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_PWR_MGMT_1, BIT(6), 0);
	if (ret < 0) {
		return ret;
	}
	k_sleep(K_MSEC(10));

	/* Set clock source to auto select best available */
	ret = icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_PWR_MGMT_1, 0x07, 0x01);
	if (ret < 0) {
		return ret;
	}
	k_sleep(K_MSEC(10));

	/* Enable all accelerometer and gyroscope axes */
	ret = icm20948_write_reg(dev, ICM20948_BANK_0, ICM20948_PWR_MGMT_2, 0x00);
	if (ret < 0) {
		return ret;
	}
	k_sleep(K_MSEC(10));

	return 0;
}

static int icm20948_set_accel_fs(const struct device *dev, uint8_t fs_sel)
{
	if (fs_sel > 3) {
		return -EINVAL;
	}

	struct icm20948_data *data = dev->data;
	int ret = icm20948_update_reg(dev, ICM20948_BANK_2, ICM20948_ACCEL_CONFIG, 0x06, fs_sel << 1);
	if (ret == 0) {
		data->accel_fs_sel = fs_sel;
	}
	return ret;
}

static int icm20948_set_gyro_fs(const struct device *dev, uint8_t fs_sel)
{
	if (fs_sel > 3) {
		return -EINVAL;
	}

	struct icm20948_data *data = dev->data;
	int ret = icm20948_update_reg(dev, ICM20948_BANK_2, ICM20948_GYRO_CONFIG_1, 0x06, fs_sel << 1);
	if (ret == 0) {
		data->gyro_fs_sel = fs_sel;
	}
	return ret;
}

static int icm20948_set_accel_sample_rate(const struct device *dev, uint16_t rate_div)
{
	/* Set accelerometer sample rate divider */
	uint8_t div_h = (rate_div >> 8) & 0x0F;
	uint8_t div_l = rate_div & 0xFF;
	
	int ret = icm20948_write_reg(dev, ICM20948_BANK_2, ICM20948_ACCEL_SMPLRT_DIV_1, div_h);
	if (ret < 0) {
		return ret;
	}
	
	return icm20948_write_reg(dev, ICM20948_BANK_2, ICM20948_ACCEL_SMPLRT_DIV_2, div_l);
}

static int icm20948_set_gyro_sample_rate(const struct device *dev, uint8_t rate_div)
{
	/* Set gyroscope sample rate divider */
	return icm20948_write_reg(dev, ICM20948_BANK_2, ICM20948_GYRO_SMPLRT_DIV, rate_div);
}

static int icm20948_enable_accel_dlpf(const struct device *dev, bool enable)
{
	struct icm20948_data *data = dev->data;
	int ret = icm20948_update_reg(dev, ICM20948_BANK_2, ICM20948_ACCEL_CONFIG, BIT(0), enable ? 0 : BIT(0));
	if (ret == 0) {
		data->accel_dlpf_enable = enable;
	}
	return ret;
}

static int icm20948_enable_gyro_dlpf(const struct device *dev, bool enable)
{
	struct icm20948_data *data = dev->data;
	int ret = icm20948_update_reg(dev, ICM20948_BANK_2, ICM20948_GYRO_CONFIG_1, BIT(0), enable ? 0 : BIT(0));
	if (ret == 0) {
		data->gyro_dlpf_enable = enable;
	}
	return ret;
}

static int icm20948_set_accel_dlpf_cfg(const struct device *dev, uint8_t cfg)
{
	struct icm20948_data *data = dev->data;
	int ret = icm20948_update_reg(dev, ICM20948_BANK_2, ICM20948_ACCEL_CONFIG, 0x38, cfg << 3);
	if (ret == 0) {
		data->accel_dlpf_cfg = cfg;
	}
	return ret;
}

static int icm20948_set_gyro_dlpf_cfg(const struct device *dev, uint8_t cfg)
{
	struct icm20948_data *data = dev->data;
	int ret = icm20948_update_reg(dev, ICM20948_BANK_2, ICM20948_GYRO_CONFIG_1, 0x38, cfg << 3);
	if (ret == 0) {
		data->gyro_dlpf_cfg = cfg;
	}
	return ret;
}

#ifdef CONFIG_ICM20948_ENABLE_MAGN
static int icm20948_setup_magnetometer(const struct device *dev)
{
	int ret;
	uint8_t mag_id;

	/* Enable I2C master mode */
	ret = icm20948_update_reg(dev, ICM20948_BANK_0, ICM20948_USER_CTRL, BIT(5), BIT(5));
	if (ret < 0) {
		return ret;
	}

	/* Configure I2C master clock frequency to 400kHz */
	ret = icm20948_update_reg(dev, ICM20948_BANK_3, ICM20948_I2C_MST_CTRL, 0x1F, 0x07);
	if (ret < 0) {
		return ret;
	}
	k_sleep(K_MSEC(10));

	/* Reset the magnetometer */
	ret = icm20948_i2c_master_write_reg(dev, AK09916_I2C_ADDR, AK09916_CNTL3, AK09916_CNTL3_SRST);
	if (ret < 0) {
		LOG_ERR("Failed to reset magnetometer");
		return ret;
	}
	k_sleep(K_MSEC(10));

	/* Check magnetometer ID */
	ret = icm20948_i2c_master_read_reg(dev, AK09916_I2C_ADDR, AK09916_WIA2, &mag_id);
	if (ret < 0) {
		LOG_ERR("Failed to read magnetometer ID");
		return ret;
	}
	if (mag_id != AK09916_CHIP_ID) {
		LOG_ERR("Invalid magnetometer ID: 0x%02x, expected 0x%02x", mag_id, AK09916_CHIP_ID);
		return -EIO;
	}

	LOG_INF("AK09916 magnetometer ID verified: 0x%02x", mag_id);

	/* Set magnetometer to continuous measurement mode 4 (100Hz) */
	ret = icm20948_i2c_master_write_reg(dev, AK09916_I2C_ADDR, AK09916_CNTL2, AK09916_MODE_CONT_4);
	if (ret < 0) {
		LOG_ERR("Failed to set magnetometer mode");
		return ret;
	}
	k_sleep(K_MSEC(10));

	/*
	 * The following section configures the I2C master interface to communicate
	 * with the AK09916 magnetometer.
	 *
	 * Slave 0 is set up to periodically read the magnetometer data, starting
	 * from the ST1 register.
	 */

	/* Configure I2C slave 0 to read from magnetometer */
	/* Set magnetometer I2C address with read bit */
	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80);
	if (ret < 0) {
		return ret;
	}

	/* Set register to start reading from (ST1) */
	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV0_REG, AK09916_ST1);
	if (ret < 0) {
		return ret;
	}

	/* Enable reading 9 bytes and set group order */
	ret = icm20948_update_reg(dev, ICM20948_BANK_3, ICM20948_I2C_SLV0_CTRL, 0xFF, BIT(7) | 0x09);
	if (ret < 0) {
		return ret;
	}

	/* Set I2C master ODR to match magnetometer rate (~100Hz -> ~1100/10 = 110Hz) */
	/* Sample rate is 1.1kHz/(1+odr) */
	ret = icm20948_write_reg(dev, ICM20948_BANK_3, ICM20948_I2C_MST_ODR_CONFIG, 9);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("AK09916 magnetometer configured successfully");
	return 0;
}
#endif

/* Data reading functions */
static int icm20948_read_accel(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	uint8_t buf[6];
	int ret;

	ret = icm20948_read_data(dev, ICM20948_BANK_0, ICM20948_ACCEL_XOUT_H, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	data->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
	data->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
	data->accel_z = (int16_t)((buf[4] << 8) | buf[5]);

	return 0;
}

static int icm20948_read_gyro(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	uint8_t buf[6];
	int ret;

	ret = icm20948_read_data(dev, ICM20948_BANK_0, ICM20948_GYRO_XOUT_H, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	data->gyro_x = (int16_t)((buf[0] << 8) | buf[1]);
	data->gyro_y = (int16_t)((buf[2] << 8) | buf[3]);
	data->gyro_z = (int16_t)((buf[4] << 8) | buf[5]);

	return 0;
}

#ifdef CONFIG_ICM20948_ENABLE_TEMP
static int icm20948_read_temp(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	uint8_t buf[2];
	int ret;

	ret = icm20948_read_data(dev, ICM20948_BANK_0, ICM20948_TEMP_OUT_H, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	data->temperature = (int16_t)((buf[0] << 8) | buf[1]);

	return 0;
}
#endif

#ifdef CONFIG_ICM20948_ENABLE_MAGN
static int icm20948_read_mag(const struct device *dev)
{
	struct icm20948_data *data = dev->data;
	uint8_t buf[9]; /* ST1 + 6 bytes of data + reserved + ST2 */
	int ret;

	/* Read magnetometer data from external sensor data registers */
	ret = icm20948_read_data(dev, ICM20948_BANK_0, ICM20948_EXT_SLV_SENS_DATA_00, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	/* Check if data is ready */
	if (!(buf[0] & AK09916_ST1_DRDY)) {
		return -EBUSY;
	}

	/* Check for overflow */
	if (buf[8] & AK09916_ST2_HOFL) {
		return -EOVERFLOW;
	}

	/* Magnetometer data is stored as little-endian values */
	data->mag_x = (int16_t)((buf[2] << 8) | buf[1]);
	data->mag_y = (int16_t)((buf[4] << 8) | buf[3]);
	data->mag_z = (int16_t)((buf[6] << 8) | buf[5]);

	return 0;
}
#endif

/* Value conversion functions */
static void icm20948_convert_accel(const struct device *dev, struct sensor_value *val, int16_t raw)
{
	struct icm20948_data *data = dev->data;
	int64_t conv_val;

	/* Convert to m/s^2 */
	conv_val = ((int64_t)raw * SENSOR_G) / accel_sensitivity_shift[data->accel_fs_sel];
	val->val1 = (int32_t)(conv_val / 1000000LL);
	val->val2 = (int32_t)(conv_val % 1000000LL);
}

static void icm20948_convert_gyro(const struct device *dev, struct sensor_value *val, int16_t raw)
{
	struct icm20948_data *data = dev->data;
	int64_t conv_val;

	/* Convert to rad/s */
	conv_val = ((int64_t)raw * SENSOR_PI) / (gyro_sensitivity_shift[data->gyro_fs_sel] * 180LL);
	val->val1 = (int32_t)(conv_val / 1000000LL);
	val->val2 = (int32_t)(conv_val % 1000000LL);
}

#ifdef CONFIG_ICM20948_ENABLE_TEMP
static void icm20948_convert_temp(struct sensor_value *val, int16_t raw)
{
	/* Formula from datasheet: Temperature in °C = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + 21°C
	 * RoomTemp_Offset = 0, Temp_Sensitivity = 333.87
	 * Simplified: Temp_C = (raw / 333.87) + 21
	 */
	int32_t temp_mc = (((int32_t)raw * 1000) / 334) + 21000;
	val->val1 = temp_mc / 1000;
	val->val2 = (temp_mc % 1000) * 1000;
}
#endif

#ifdef CONFIG_ICM20948_ENABLE_MAGN
static void icm20948_convert_mag(struct sensor_value *val, int16_t raw)
{
	/* Convert to micro Tesla (uT)
	 * AK09916 has 0.15 uT per LSB
	 */
	int32_t mag_micro_tesla = raw * 150;
	val->val1 = mag_micro_tesla / 1000000;
	val->val2 = mag_micro_tesla % 1000000;
}
#endif

/* Sensor API implementation */
static int icm20948_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret = 0;

	/* Read data based on requested channel */
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = icm20948_read_accel(dev);
		break;

	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		ret = icm20948_read_gyro(dev);
		break;

#ifdef CONFIG_ICM20948_ENABLE_TEMP
	case SENSOR_CHAN_DIE_TEMP:
		ret = icm20948_read_temp(dev);
		break;
#endif

#ifdef CONFIG_ICM20948_ENABLE_MAGN
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		ret = icm20948_read_mag(dev);
		break;
#endif

	case SENSOR_CHAN_ALL:
		ret = icm20948_read_accel(dev);
		if (ret < 0) {
			return ret;
		}
		ret = icm20948_read_gyro(dev);
		if (ret < 0) {
			return ret;
		}
#ifdef CONFIG_ICM20948_ENABLE_TEMP
		ret = icm20948_read_temp(dev);
		if (ret < 0) {
			return ret;
		}
#endif
#ifdef CONFIG_ICM20948_ENABLE_MAGN
		ret = icm20948_read_mag(dev);
		if (ret < 0) {
			return ret;
		}
#endif
		break;

	default:
		return -ENOTSUP;
	}

	return ret;
}

static int icm20948_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		icm20948_convert_accel(dev, val, data->accel_x);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20948_convert_accel(dev, val, data->accel_y);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20948_convert_accel(dev, val, data->accel_z);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20948_convert_accel(dev, &val[0], data->accel_x);
		icm20948_convert_accel(dev, &val[1], data->accel_y);
		icm20948_convert_accel(dev, &val[2], data->accel_z);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm20948_convert_gyro(dev, val, data->gyro_x);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20948_convert_gyro(dev, val, data->gyro_y);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20948_convert_gyro(dev, val, data->gyro_z);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20948_convert_gyro(dev, &val[0], data->gyro_x);
		icm20948_convert_gyro(dev, &val[1], data->gyro_y);
		icm20948_convert_gyro(dev, &val[2], data->gyro_z);
		break;
#ifdef CONFIG_ICM20948_ENABLE_TEMP
	case SENSOR_CHAN_DIE_TEMP:
		icm20948_convert_temp(val, data->temperature);
		break;
#endif
#ifdef CONFIG_ICM20948_ENABLE_MAGN
	case SENSOR_CHAN_MAGN_X:
		icm20948_convert_mag(val, data->mag_x);
		break;
	case SENSOR_CHAN_MAGN_Y:
		icm20948_convert_mag(val, data->mag_y);
		break;
	case SENSOR_CHAN_MAGN_Z:
		icm20948_convert_mag(val, data->mag_z);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		icm20948_convert_mag(&val[0], data->mag_x);
		icm20948_convert_mag(&val[1], data->mag_y);
		icm20948_convert_mag(&val[2], data->mag_z);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int icm20948_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;
	int ret;

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		if (chan == SENSOR_CHAN_ACCEL_X ||
		    chan == SENSOR_CHAN_ACCEL_Y ||
		    chan == SENSOR_CHAN_ACCEL_Z ||
		    chan == SENSOR_CHAN_ACCEL_XYZ) {
			/* Convert value to accelerometer full-scale selection */
			uint8_t fs_sel;
			if (val->val1 <= 2) {
				fs_sel = 0; /* ±2g */
			} else if (val->val1 <= 4) {
				fs_sel = 1; /* ±4g */
			} else if (val->val1 <= 8) {
				fs_sel = 2; /* ±8g */
			} else {
				fs_sel = 3; /* ±16g */
			}
			ret = icm20948_set_accel_fs(dev, fs_sel);
			if (ret < 0) {
				return ret;
			}
		} else if (chan == SENSOR_CHAN_GYRO_X ||
			   chan == SENSOR_CHAN_GYRO_Y ||
			   chan == SENSOR_CHAN_GYRO_Z ||
			   chan == SENSOR_CHAN_GYRO_XYZ) {
			/* Convert value to gyroscope full-scale selection */
			uint8_t fs_sel;
			if (val->val1 <= 250) {
				fs_sel = 0; /* ±250 dps */
			} else if (val->val1 <= 500) {
				fs_sel = 1; /* ±500 dps */
			} else if (val->val1 <= 1000) {
				fs_sel = 2; /* ±1000 dps */
			} else {
				fs_sel = 3; /* ±2000 dps */
			}
			ret = icm20948_set_gyro_fs(dev, fs_sel);
			if (ret < 0) {
				return ret;
			}
		} else {
			return -ENOTSUP;
		}
		break;

	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (chan == SENSOR_CHAN_ACCEL_X ||
		    chan == SENSOR_CHAN_ACCEL_Y ||
		    chan == SENSOR_CHAN_ACCEL_Z ||
		    chan == SENSOR_CHAN_ACCEL_XYZ) {
			/* Accelerometer sample rate: 1.125 kHz/(1+div) */
			uint16_t div = 1125 / val->val1 - 1;
			if (div > 4095) {
				div = 4095; /* Min rate ~0.275 Hz */
			}
			ret = icm20948_set_accel_sample_rate(dev, div);
			if (ret < 0) {
				return ret;
			}
		} else if (chan == SENSOR_CHAN_GYRO_X ||
			   chan == SENSOR_CHAN_GYRO_Y ||
			   chan == SENSOR_CHAN_GYRO_Z ||
			   chan == SENSOR_CHAN_GYRO_XYZ) {
			/* Gyroscope sample rate: 1.1 kHz/(1+div) */
			uint8_t div = 1100 / val->val1 - 1;
			if (div > 255) {
				div = 255; /* Min rate ~4.3 Hz */
			}
			ret = icm20948_set_gyro_sample_rate(dev, div);
			if (ret < 0) {
				return ret;
			}
		} else {
			return -ENOTSUP;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static int icm20948_attr_get(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, struct sensor_value *val)
{
	struct icm20948_data *data = dev->data;

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		if (chan == SENSOR_CHAN_ACCEL_X ||
		    chan == SENSOR_CHAN_ACCEL_Y ||
		    chan == SENSOR_CHAN_ACCEL_Z ||
		    chan == SENSOR_CHAN_ACCEL_XYZ) {
			/* Return current accelerometer full-scale setting */
			switch (data->accel_fs_sel) {
			case 0:
				val->val1 = 2; /* ±2g */
				break;
			case 1:
				val->val1 = 4; /* ±4g */
				break;
			case 2:
				val->val1 = 8; /* ±8g */
				break;
			case 3:
				val->val1 = 16; /* ±16g */
				break;
			}
			val->val2 = 0;
		} else if (chan == SENSOR_CHAN_GYRO_X ||
			   chan == SENSOR_CHAN_GYRO_Y ||
			   chan == SENSOR_CHAN_GYRO_Z ||
			   chan == SENSOR_CHAN_GYRO_XYZ) {
			/* Return current gyroscope full-scale setting */
			switch (data->gyro_fs_sel) {
			case 0:
				val->val1 = 250; /* ±250 dps */
				break;
			case 1:
				val->val1 = 500; /* ±500 dps */
				break;
			case 2:
				val->val1 = 1000; /* ±1000 dps */
				break;
			case 3:
				val->val1 = 2000; /* ±2000 dps */
				break;
			}
			val->val2 = 0;
		} else {
			return -ENOTSUP;
		}
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

#ifdef CONFIG_ICM20948_TRIGGER
/* All trigger logic is now in icm20948_trig.c */
#endif /* CONFIG_ICM20948_TRIGGER */

/* Main initialization function */
int icm20948_init(const struct device *dev)
{
	const struct icm20948_config *config = dev->config;
	struct icm20948_data *data = dev->data;
	int ret;

	/* Initialize bus interface */
	ret = config->bus_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize bus interface");
		return ret;
	}

	/* Software reset */
	ret = icm20948_software_reset(dev);
	if (ret < 0) {
		LOG_ERR("Failed to reset device");
		return ret;
	}

	/* Check chip ID */
	ret = icm20948_check_chip_id(dev);
	if (ret < 0) {
		LOG_ERR("Invalid chip ID");
		return ret;
	}

	/* Setup power management */
	ret = icm20948_setup_power_management(dev);
	if (ret < 0) {
		LOG_ERR("Failed to setup power management");
		return ret;
	}

	/* Set default full-scale ranges: ±4g for accel, ±500dps for gyro */
	ret = icm20948_set_accel_fs(dev, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set accelerometer full-scale");
		return ret;
	}

	ret = icm20948_set_gyro_fs(dev, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set gyroscope full-scale");
		return ret;
	}

	/* Set default sample rates: ~100Hz for both accel and gyro */
	ret = icm20948_set_accel_sample_rate(dev, 10);
	if (ret < 0) {
		LOG_ERR("Failed to set accelerometer sample rate");
		return ret;
	}

	ret = icm20948_set_gyro_sample_rate(dev, 10);
	if (ret < 0) {
		LOG_ERR("Failed to set gyroscope sample rate");
		return ret;
	}

	/* Enable DLPF for both sensors */
	ret = icm20948_enable_accel_dlpf(dev, true);
	if (ret < 0) {
		LOG_ERR("Failed to enable accelerometer DLPF");
		return ret;
	}

	ret = icm20948_enable_gyro_dlpf(dev, true);
	if (ret < 0) {
		LOG_ERR("Failed to enable gyroscope DLPF");
		return ret;
	}

	/* Set DLPF configuration: 246Hz for accel, 196Hz for gyro */
	ret = icm20948_set_accel_dlpf_cfg(dev, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set accelerometer DLPF config");
		return ret;
	}

	ret = icm20948_set_gyro_dlpf_cfg(dev, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set gyroscope DLPF config");
		return ret;
	}

#ifdef CONFIG_ICM20948_ENABLE_MAGN
	/* Setup magnetometer */
	ret = icm20948_setup_magnetometer(dev);
	if (ret < 0) {
		LOG_ERR("Failed to setup magnetometer");
		return ret;
	}
#endif

#ifdef CONFIG_ICM20948_TRIGGER
	/* Initialize interrupt handling */
	ret = icm20948_init_interrupt(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize interrupt");
		return ret;
	}
#endif

	LOG_INF("ICM-20948 initialized successfully");
	return 0;
}

/* Sensor driver API */
const struct sensor_driver_api icm20948_api = {
	.sample_fetch = icm20948_sample_fetch,
	.channel_get = icm20948_channel_get,
	.attr_set = icm20948_attr_set,
	.attr_get = icm20948_attr_get,
#ifdef CONFIG_ICM20948_TRIGGER
	.trigger_set = icm20948_trigger_set,
#endif
};

/* I2C device instantiation */
#define ICM20948_DEFINE_I2C(inst)						\
	static struct icm20948_data icm20948_data_##inst = {			\
		.accel_fs_sel = 1,						\
		.gyro_fs_sel = 1,						\
		.accel_dlpf_enable = true,					\
		.gyro_dlpf_enable = true,					\
		.accel_dlpf_cfg = 0,						\
		.gyro_dlpf_cfg = 0,						\
	};									\
	static const struct icm20948_config icm20948_config_##inst = {		\
		.bus_init = icm20948_i2c_init,					\
		.i2c = I2C_DT_SPEC_INST_GET(inst),				\
		IF_ENABLED(CONFIG_ICM20948_TRIGGER,				\
			(.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {}),)) \
	};									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,					\
				      icm20948_init,				\
				      NULL,					\
				      &icm20948_data_##inst,			\
				      &icm20948_config_##inst,			\
				      POST_KERNEL,				\
				      CONFIG_SENSOR_INIT_PRIORITY,		\
				      &icm20948_api);

/* Create device instances for all enabled ICM-20948 nodes */
DT_INST_FOREACH_STATUS_OKAY(ICM20948_DEFINE_I2C)