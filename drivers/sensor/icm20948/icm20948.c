#define DT_DRV_COMPAT invensense_icm20948

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include "icm20948.h"

LOG_MODULE_REGISTER(icm20948, CONFIG_SENSOR_LOG_LEVEL);

int icm20948_init(const struct device *dev)
{
    const struct icm20948_config *cfg = dev->config;
    struct icm20948_data *data = dev->data;

    if (!device_is_ready(cfg->i2c.bus)) {
        return false;
    }

    data->current_bank = 0;

    reset_icm20948(dev);

    uint8_t tries = 0;
    while ((icm20948_whoami(dev) != ICM20948_WHO_AM_I_CONTENT) && (tries < 10)) {
        reset_icm20948(dev);
        k_msleep(300);
        tries++;
    }
    if (tries == 10) {
        return false;
    }

    data->acc_offset = (struct icm20948_vec3f){0.0f, 0.0f, 0.0f};
    data->acc_corr_factor = (struct icm20948_vec3f){1.0f, 1.0f, 1.0f};
    data->acc_range_factor = 1.0f;

    data->gyr_offset = (struct icm20948_vec3f){0.0f, 0.0f, 0.0f};
    data->gyr_range_factor = 1.0f;

    data->fifo_type = ICM20948_FIFO_ACC;

    icm20948_sleep(dev, false);

    write_register8(dev, 2, ICM20948_ODR_ALIGN_EN, 1);

    return true;
}


void icm20948_auto_offsets(const struct device *dev)
{
    struct icm20948_data *data = dev->data;
    struct icm20948_vec3f acc, gyr;

    data->acc_offset = (struct icm20948_vec3f){0.0f, 0.0f, 0.0f};

    icm20948_set_gyr_dlpf(dev, ICM20948_DLPF_6);
    icm20948_set_gyr_range(dev, ICM20948_GYRO_RANGE_250);
    icm20948_set_acc_range(dev, ICM20948_ACC_RANGE_2G);
    icm20948_set_acc_dlpf(dev, ICM20948_DLPF_6);

    k_msleep(100);

    for (int i = 0; i < 10; i++) {
        icm20948_read_sensor(dev);
        k_msleep(10);
    }

    for (int i = 0; i < 50; i++) {
        icm20948_read_sensor(dev);
        icm20948_get_acc_raw(dev, &acc);
        data->acc_offset = vec3f_add(data->acc_offset, acc);
        k_msleep(10);
    }

    data->acc_offset = vec3f_div(data->acc_offset, 50.0f);
    data->acc_offset.z -= 16384.0f;

    data->gyr_offset = (struct icm20948_vec3f){0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 50; i++) {
        icm20948_read_sensor(dev);
        icm20948_get_gyr_raw(dev, &gyr);
        data->gyr_offset = vec3f_add(data->gyr_offset, gyr);
        k_msleep(1);
    }

    data->gyr_offset = vec3f_div(data->gyr_offset, 50.0f);
}


void icm20948_set_acc_offsets(const struct device *dev, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
    struct icm20948_data *data = dev->data;

    data->acc_offset.x = 0.5f * (x_max + x_min);
    data->acc_offset.y = 0.5f * (y_max + y_min);
    data->acc_offset.z = 0.5f * (z_max + z_min);

    data->acc_corr_factor.x = (x_max + fabsf(x_min)) / 32768.0f;
    data->acc_corr_factor.y = (y_max + fabsf(y_min)) / 32768.0f;
    data->acc_corr_factor.z = (z_max + fabsf(z_min)) / 32768.0f;
}


void icm20948_set_acc_offsets_vec(const struct device *dev, struct icm20948_vec3f offset)
{
    struct icm20948_data *data = dev->data;
    data->acc_offset = offset;
}


struct icm20948_vec3f icm20948_get_acc_offsets(const struct device *dev)
{
    struct icm20948_data *data = dev->data;
    return data->acc_offset;
}


void icm20948_set_gyr_offsets(const struct device *dev, float x, float y, float z)
{
    struct icm20948_data *data = dev->data;

    data->gyr_offset.x = x;
    data->gyr_offset.y = y;
    data->gyr_offset.z = z;
}

void icm20948_set_gyr_offsets_vec(const struct device *dev, struct icm20948_vec3f offset)
{
    struct icm20948_data *data = dev->data;
    data->gyr_offset = offset;
}

struct icm20948_vec3f icm20948_get_gyr_offsets(const struct device *dev)
{
    struct icm20948_data *data = dev->data;
    return data->gyr_offset;
}

uint8_t icm20948_who_am_i(const struct device *dev)
{
    return icm20948_read_reg(dev, 0, ICM20948_WHO_AM_I);
}


void icm20948_enable_acc(const struct device *dev, bool enable)
{
    uint8_t val = icm20948_read_reg(dev, 0, ICM20948_PWR_MGMT_2);

    if (enable) {
        val &= ~ICM20948_ACC_EN;
    } else {
        val |= ICM20948_ACC_EN;
    }

    icm20948_write_reg(dev, 0, ICM20948_PWR_MGMT_2, val);
}


void icm20948_set_acc_range(const struct device *dev, enum icm20948_acc_range range)
{
    struct icm20948_data *data = dev->data;
    uint8_t val = icm20948_read_reg(dev, 2, ICM20948_ACCEL_CONFIG);

    val &= ~0x06; // Clear range bits
    val |= (range << 1);

    icm20948_write_reg(dev, 2, ICM20948_ACCEL_CONFIG, val);

    data->acc_range_factor = 1 << range;
}

void icm20948_set_acc_dlpf(const struct device *dev, enum icm20948_dlpf mode)
{
    uint8_t val = icm20948_read_reg(dev, 2, ICM20948_ACCEL_CONFIG);

    if (mode == ICM20948_DLPF_OFF) {
        val &= ~0x01; // Disable DLPF
    } else {
        val |= 0x01;  // Enable DLPF
        val &= ~0x38; // Clear DLPF_CFG bits (bits 3-5)
        val |= (mode << 3);
    }

    icm20948_write_reg(dev, 2, ICM20948_ACCEL_CONFIG, val);
}


void icm20948_set_acc_sample_rate_divider(const struct device *dev, uint16_t div)
{
    icm20948_write_reg16(dev, 2, ICM20948_ACCEL_SMPLRT_DIV_1, div);
}


void icm20948_enable_gyr(const struct device *dev, bool enable)
{
    uint8_t val = icm20948_read_reg(dev, 0, ICM20948_PWR_MGMT_2);

    if (enable) {
        val &= ~ICM20948_GYR_EN;
    } else {
        val |= ICM20948_GYR_EN;
    }

    icm20948_write_reg(dev, 0, ICM20948_PWR_MGMT_2, val);
}

void icm20948_set_gyr_range(const struct device *dev, enum icm20948_gyro_range range)
{
    struct icm20948_data *data = dev->data;
    uint8_t val = icm20948_read_reg(dev, 2, ICM20948_GYRO_CONFIG_1);

    val &= ~0x06;             // Clear range bits
    val |= (range << 1);      // Set new range

    icm20948_write_reg(dev, 2, ICM20948_GYRO_CONFIG_1, val);

    data->gyr_range_factor = (1 << range);
}


void icm20948_set_gyr_dlpf(const struct device *dev, enum icm20948_dlpf mode)
{
    uint8_t val = icm20948_read_reg(dev, 2, ICM20948_GYRO_CONFIG_1);

    if (mode == ICM20948_DLPF_OFF) {
        val &= ~0x01;  // DLPF disabled
    } else {
        val |= 0x01;   // Enable DLPF
        val &= ~0x38;  // Clear DLPF_CFG bits
        val |= (mode << 3);
    }

    icm20948_write_reg(dev, 2, ICM20948_GYRO_CONFIG_1, val);
}


void icm20948_set_gyr_sample_rate_divider(const struct device *dev, uint8_t div)
{
    icm20948_write_reg(dev, 2, ICM20948_GYRO_SMPLRT_DIV, div);
}


void icm20948_set_temp_dlpf(const struct device *dev, enum icm20948_dlpf mode)
{
    icm20948_write_reg(dev, 2, ICM20948_TEMP_CONFIG, (uint8_t)mode);
}


void icm20948_set_i2c_master_sample_rate(const struct device *dev, uint8_t rate_exp)
{
    if (rate_exp < 16) {
        icm20948_write_reg(dev, 3, ICM20948_I2C_MST_ODR_CFG, rate_exp);
    }
}

void icm20948_read_sensor(const struct device *dev)
{
    struct icm20948_data *data = dev->data;
    icm20948_read_all_data(dev, data->buffer);  // Reads at least 14 bytes: acc, gyr, temp
}


void icm20948_get_acc_raw(const struct device *dev, struct icm20948_vec3f *acc)
{
    struct icm20948_data *data = dev->data;
    const uint8_t *buf = data->buffer;

    acc->x = (int16_t)((buf[0] << 8) | buf[1]);
    acc->y = (int16_t)((buf[2] << 8) | buf[3]);
    acc->z = (int16_t)((buf[4] << 8) | buf[5]);
}

void icm20948_get_corrected_acc_raw(const struct device *dev, struct icm20948_vec3f *acc)
{
    icm20948_get_acc_raw(dev, acc);
    icm20948_correct_acc_raw(dev, acc);  // Implement this function to subtract offset + scale
}

void icm20948_get_acc_g(const struct device *dev, struct icm20948_vec3f *g)
{
    struct icm20948_data *data = dev->data;

    icm20948_get_corrected_acc_raw(dev, g);

    g->x = g->x * data->acc_range_factor / 16384.0f;
    g->y = g->y * data->acc_range_factor / 16384.0f;
    g->z = g->z * data->acc_range_factor / 16384.0f;
}

void icm20948_get_acc_raw_from_fifo(const struct device *dev, struct icm20948_vec3f *acc)
{
    icm20948_read_xyz_from_fifo(dev, acc, ICM20948_SENSOR_ACC);
}


void icm20948_get_corrected_acc_raw_from_fifo(const struct device *dev, struct icm20948_vec3f *acc)
{
    icm20948_get_acc_raw_from_fifo(dev, acc);
    icm20948_correct_acc_raw(dev, acc);
}


void icm20948_get_g_from_fifo(const struct device *dev, struct icm20948_vec3f *g)
{
    struct icm20948_data *data = dev->data;
    icm20948_get_corrected_acc_raw_from_fifo(dev, g);

    g->x = g->x * data->acc_range_factor / 16384.0f;
    g->y = g->y * data->acc_range_factor / 16384.0f;
    g->z = g->z * data->acc_range_factor / 16384.0f;
}

float icm20948_get_resultant_g(const struct icm20948_vec3f *g)
{
    return sqrtf(g->x * g->x + g->y * g->y + g->z * g->z);
}

float icm20948_get_temperature(const struct device *dev)
{
    struct icm20948_data *data = dev->data;
    int16_t raw = (int16_t)((data->buffer[12] << 8) | data->buffer[13]);
    return ((float)raw - ICM20948_ROOM_TEMP_OFFSET) / ICM20948_T_SENSITIVITY + 21.0f;
}

void icm20948_get_gyr_raw(const struct device *dev, struct icm20948_vec3f *gyr)
{
    struct icm20948_data *data = dev->data;
    const uint8_t *buf = data->buffer;

    gyr->x = (int16_t)((buf[6] << 8) | buf[7]);
    gyr->y = (int16_t)((buf[8] << 8) | buf[9]);
    gyr->z = (int16_t)((buf[10] << 8) | buf[11]);
}

void icm20948_get_corrected_gyr_raw(const struct device *dev, struct icm20948_vec3f *gyr)
{
    icm20948_get_gyr_raw(dev, gyr);
    icm20948_correct_gyr_raw(dev, gyr);
}

void icm20948_get_gyr_values(const struct device *dev, struct icm20948_vec3f *gyr)
{
    struct icm20948_data *data = dev->data;
    icm20948_get_corrected_gyr_raw(dev, gyr);

    gyr->x = gyr->x * data->gyr_range_factor * 250.0f / 32768.0f;
    gyr->y = gyr->y * data->gyr_range_factor * 250.0f / 32768.0f;
    gyr->z = gyr->z * data->gyr_range_factor * 250.0f / 32768.0f;
}

void icm20948_get_gyr_from_fifo(const struct device *dev, struct icm20948_vec3f *gyr)
{
    struct icm20948_data *data = dev->data;
    icm20948_read_xyz_from_fifo(dev, gyr, ICM20948_SENSOR_GYR);

    icm20948_correct_gyr_raw(dev, gyr);
    gyr->x = gyr->x * data->gyr_range_factor * 250.0f / 32768.0f;
    gyr->y = gyr->y * data->gyr_range_factor * 250.0f / 32768.0f;
    gyr->z = gyr->z * data->gyr_range_factor * 250.0f / 32768.0f;
}

void icm20948_get_mag_values(const struct device *dev, struct icm20948_vec3f *mag)
{
    struct icm20948_data *data = dev->data;
    const uint8_t *buf = data->buffer;

    int16_t x = (int16_t)((buf[15] << 8) | buf[14]);
    int16_t y = (int16_t)((buf[17] << 8) | buf[16]);
    int16_t z = (int16_t)((buf[19] << 8) | buf[18]);

    mag->x = x * AK09916_MAG_LSB;
    mag->y = y * AK09916_MAG_LSB;
    mag->z = z * AK09916_MAG_LSB;
}

void icm20948_enable_cycle(const struct device *dev, enum icm20948_cycle cycle)
{
    uint8_t val = icm20948_read_u8(dev, 0, ICM20948_LP_CONFIG);
    val &= 0x0F;
    val |= (uint8_t)cycle;
    icm20948_write_u8(dev, 0, ICM20948_LP_CONFIG, val);
}

void icm20948_enable_cycle(const struct device *dev, enum icm20948_cycle cycle)
{
    uint8_t val = icm20948_read_u8(dev, 0, ICM20948_LP_CONFIG);
    val &= 0x0F;
    val |= (uint8_t)cycle;
    icm20948_write_u8(dev, 0, ICM20948_LP_CONFIG, val);
}

void icm20948_enable_low_power(const struct device *dev, bool enable)
{
    uint8_t val = icm20948_read_u8(dev, 0, ICM20948_PWR_MGMT_1);

    if (enable) {
        val |= ICM20948_LP_EN;
    } else {
        val &= ~ICM20948_LP_EN;
    }
    icm20948_write_u8(dev, 0, ICM20948_PWR_MGMT_1, val);
}

void icm20948_set_gyr_avg_cycle(const struct device *dev, enum icm20948_gyro_avg_lp avg)
{
    icm20948_write_u8(dev, 2, ICM20948_GYRO_CONFIG_2, (uint8_t)avg);
}


void icm20948_set_acc_avg_cycle(const struct device *dev, enum icm20948_acc_avg_lp avg)
{
    icm20948_write_u8(dev, 2, ICM20948_ACCEL_CONFIG_2, (uint8_t)avg);
}

void icm20948_set_sleep(const struct device *dev, bool enable)
{
    uint8_t val = icm20948_read_u8(dev, 0, ICM20948_PWR_MGMT_1);
    if (enable) {
        val |= ICM20948_SLEEP;
    } else {
        val &= ~ICM20948_SLEEP;
    }
    icm20948_write_u8(dev, 0, ICM20948_PWR_MGMT_1, val);
}

void icm20948_get_angles(const struct device *dev, struct icm20948_vec3f *angle)
{
    struct icm20948_vec3f g;
    icm20948_get_g_values(dev, &g);  // Already returns normalized Gs

    g.x = CLAMP(g.x, -1.0f, 1.0f);
    g.y = CLAMP(g.y, -1.0f, 1.0f);
    g.z = CLAMP(g.z, -1.0f, 1.0f);

    angle->x = asin(g.x) * 57.296f;
    angle->y = asin(g.y) * 57.296f;
    angle->z = asin(g.z) * 57.296f;
}

enum icm20948_orientation icm20948_get_orientation(const struct device *dev)
{
    struct icm20948_vec3f angle;
    icm20948_get_angles(dev, &angle);

    if (fabsf(angle.x) < 45.0f) {
        if (fabsf(angle.y) < 45.0f) {
            return (angle.z > 0) ? ICM20948_FLAT : ICM20948_FLAT_1;
        } else {
            return (angle.y > 0) ? ICM20948_XY : ICM20948_XY_1;
        }
    } else {
        return (angle.x > 0) ? ICM20948_YX : ICM20948_YX_1;
    }
}

const char *icm20948_orientation_to_str(enum icm20948_orientation o)
{
    switch (o) {
    case ICM20948_FLAT:   return "z up";
    case ICM20948_FLAT_1: return "z down";
    case ICM20948_XY:     return "y up";
    case ICM20948_XY_1:   return "y down";
    case ICM20948_YX:     return "x up";
    case ICM20948_YX_1:   return "x down";
    default:              return "unknown";
    }
}


const char *icm20948_get_orientation_as_str(const struct device *dev)
{
    enum icm20948_orientation o = icm20948_get_orientation(dev);
    return icm20948_orientation_to_str(o);
}

float icm20948_get_pitch(const struct device *dev)
{
    struct icm20948_vec3f a;
    icm20948_get_angles(dev, &a);
    return atan2f(-a.x, sqrtf(fabsf(a.y * a.y + a.z * a.z))) * 180.0f / (float)M_PI;
}

float icm20948_get_roll(const struct device *dev)
{
    struct icm20948_vec3f a;
    icm20948_get_angles(dev, &a);
    return atan2f(a.y, a.z) * 180.0f / (float)M_PI;
}

void icm20948_set_int_pin_polarity(const struct device *dev, bool active_low)
{
    uint8_t val = icm20948_read_u8(dev, 0, ICM20948_INT_PIN_CFG);

    if (active_low) {
        val |= ICM20948_INT1_ACTL;
    } else {
        val &= ~ICM20948_INT1_ACTL;
    }

    icm20948_write_u8(dev, 0, ICM20948_INT_PIN_CFG, val);
}

void icm20948_enable_int_latch(const struct device *dev, bool latch)
{
    uint8_t val = icm20948_read_u8(dev, 0, ICM20948_INT_PIN_CFG);

    if (latch) {
        val |= ICM20948_INT_1_LATCH_EN;
    } else {
        val &= ~ICM20948_INT_1_LATCH_EN;
    }

    icm20948_write_u8(dev, 0, ICM20948_INT_PIN_CFG, val);
}


void icm20948_enable_clear_int_by_any_read(const struct device *dev, bool enable)
{
    uint8_t val = icm20948_read_u8(dev, 0, ICM20948_INT_PIN_CFG);

    if (enable) {
        val |= ICM20948_INT_ANYRD_2CLEAR;
    } else {
        val &= ~ICM20948_INT_ANYRD_2CLEAR;
    }

    icm20948_write_u8(dev, 0, ICM20948_INT_PIN_CFG, val);
}

void icm20948_set_fsync_int_polarity(const struct device *dev, bool active_low)
{
    uint8_t val = icm20948_read_u8(dev, 0, ICM20948_INT_PIN_CFG);

    if (active_low) {
        val |= ICM20948_ACTL_FSYNC;
    } else {
        val &= ~ICM20948_ACTL_FSYNC;
    }

    icm20948_write_u8(dev, 0, ICM20948_INT_PIN_CFG, val);
}


void icm20948_enable_interrupt(const struct device *dev, enum icm20948_int_type int_type)
{
    uint8_t val;

    switch (int_type) {
    case ICM20948_FSYNC_INT:
        val = icm20948_read_u8(dev, 0, ICM20948_INT_PIN_CFG);
        val |= ICM20948_FSYNC_INT_MODE_EN;
        icm20948_write_u8(dev, 0, ICM20948_INT_PIN_CFG, val);

        val = icm20948_read_u8(dev, 0, ICM20948_INT_ENABLE);
        val |= 0x80;
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE, val);
        break;

    case ICM20948_WOM_INT:
        val = icm20948_read_u8(dev, 0, ICM20948_INT_ENABLE);
        val |= 0x08;
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE, val);

        val = icm20948_read_u8(dev, 2, ICM20948_ACCEL_INTEL_CTRL);
        val |= 0x02;
        icm20948_write_u8(dev, 2, ICM20948_ACCEL_INTEL_CTRL, val);
        break;

    case ICM20948_DMP_INT:
        val = icm20948_read_u8(dev, 0, ICM20948_INT_ENABLE);
        val |= 0x02;
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE, val);
        break;

    case ICM20948_DATA_READY_INT:
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE_1, 0x01);
        break;

    case ICM20948_FIFO_OVF_INT:
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE_2, 0x01);
        break;

    case ICM20948_FIFO_WM_INT:
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE_3, 0x01);
        break;
    }
}

void icm20948_disable_interrupt(const struct device *dev, enum icm20948_int_type int_type)
{
    uint8_t val;

    switch (int_type) {
    case ICM20948_FSYNC_INT:
        val = icm20948_read_u8(dev, 0, ICM20948_INT_PIN_CFG);
        val &= ~ICM20948_FSYNC_INT_MODE_EN;
        icm20948_write_u8(dev, 0, ICM20948_INT_PIN_CFG, val);

        val = icm20948_read_u8(dev, 0, ICM20948_INT_ENABLE);
        val &= ~(0x80);
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE, val);
        break;

    case ICM20948_WOM_INT:
        val = icm20948_read_u8(dev, 0, ICM20948_INT_ENABLE);
        val &= ~(0x08);
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE, val);

        val = icm20948_read_u8(dev, 2, ICM20948_ACCEL_INTEL_CTRL);
        val &= ~(0x02);
        icm20948_write_u8(dev, 2, ICM20948_ACCEL_INTEL_CTRL, val);
        break;

    case ICM20948_DMP_INT:
        val = icm20948_read_u8(dev, 0, ICM20948_INT_ENABLE);
        val &= ~(0x02);
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE, val);
        break;

    case ICM20948_DATA_READY_INT:
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE_1, 0x00);
        break;

    case ICM20948_FIFO_OVF_INT:
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE_2, 0x00);
        break;

    case ICM20948_FIFO_WM_INT:
        icm20948_write_u8(dev, 0, ICM20948_INT_ENABLE_3, 0x00);
        break;
    }
}

uint8_t icm20948_read_and_clear_interrupts(const struct device *dev)
{
    uint8_t int_source = 0;
    uint8_t val;

    val = icm20948_read_u8(dev, 0, ICM20948_I2C_MST_STATUS);
    if (val & 0x80) {
        int_source |= 0x01;
    }

    val = icm20948_read_u8(dev, 0, ICM20948_INT_STATUS);
    if (val & 0x08) {
        int_source |= 0x02;
    }
    if (val & 0x02) {
        int_source |= 0x04;
    }

    val = icm20948_read_u8(dev, 0, ICM20948_INT_STATUS_1);
    if (val & 0x01) {
        int_source |= 0x08;
    }

    val = icm20948_read_u8(dev, 0, ICM20948_INT_STATUS_2);
    if (val & 0x01) {
        int_source |= 0x10;
    }

    val = icm20948_read_u8(dev, 0, ICM20948_INT_STATUS_3);
    if (val & 0x01) {
        int_source |= 0x20;
    }

    return int_source;
}


bool icm20948_check_interrupt(uint8_t source, enum icm20948_int_type type)
{
    return (source & type) != 0;
}


void icm20948_set_wake_on_motion_threshold(const struct device *dev,
                                           uint8_t threshold,
                                           bool enable_comparison)
{
    uint8_t val = icm20948_read_u8(dev, 2, ICM20948_ACCEL_INTEL_CTRL);

    if (enable_comparison) {
        val |= 0x01;
    } else {
        val &= ~0x01;
    }

    icm20948_write_u8(dev, 2, ICM20948_ACCEL_INTEL_CTRL, val);
    icm20948_write_u8(dev, 2, ICM20948_ACCEL_WOM_THR, threshold);
}

void icm20948_set_wake_on_motion_threshold(const struct device *dev,
                                           uint8_t threshold,
                                           bool enable_comparison)
{
    uint8_t val = icm20948_read_u8(dev, 2, ICM20948_ACCEL_INTEL_CTRL);

    if (enable_comparison) {
        val |= 0x01;
    } else {
        val &= ~0x01;
    }

    icm20948_write_u8(dev, 2, ICM20948_ACCEL_INTEL_CTRL, val);
    icm20948_write_u8(dev, 2, ICM20948_ACCEL_WOM_THR, threshold);
}

void icm20948_enable_fifo(const struct device *dev, bool enable)
{
    uint8_t val = icm20948_read_u8(dev, 0, ICM20948_USER_CTRL);

    if (enable) {
        val |= ICM20948_FIFO_EN;
    } else {
        val &= ~ICM20948_FIFO_EN;
    }

    icm20948_write_u8(dev, 0, ICM20948_USER_CTRL, val);
}

void icm20948_set_fifo_mode(const struct device *dev, bool stream_mode)
{
    uint8_t val = stream_mode ? 0x01 : 0x00;
    icm20948_write_u8(dev, 0, ICM20948_FIFO_MODE, val);
}

void icm20948_start_fifo(const struct device *dev, uint8_t fifo_type)
{
    // fifo_type is a bitmask of what sensors to include
    icm20948_write_u8(dev, 0, ICM20948_FIFO_EN_2, fifo_type);
}

void icm20948_stop_fifo(const struct device *dev)
{
    icm20948_write_u8(dev, 0, ICM20948_FIFO_EN_2, 0);
}


void icm20948_reset_fifo(const struct device *dev)
{
    icm20948_write_u8(dev, 0, ICM20948_FIFO_RST, 0x01);
    icm20948_write_u8(dev, 0, ICM20948_FIFO_RST, 0x00);
}

int16_t icm20948_get_fifo_count(const struct device *dev)
{
    return (int16_t) icm20948_read_u16(dev, 0, ICM20948_FIFO_COUNT);
}



int16_t icm20948_get_fifo_data_set_count(const struct device *dev, enum icm20948_fifo_type type)
{
    int16_t count = icm20948_get_fifo_count(dev);

    switch (type) {
    case ICM20948_FIFO_ACC:
    case ICM20948_FIFO_GYR:
        return count / 6;
    case ICM20948_FIFO_ACC_GYR:
        return count / 12;
    default:
        return 0;
    }
}

void icm20948_find_fifo_start(const struct device *dev, enum icm20948_fifo_type type)
{
    uint16_t count = icm20948_get_fifo_count(dev);
    int16_t skip_bytes = 0;

    if (type == ICM20948_FIFO_ACC || type == ICM20948_FIFO_GYR) {
        skip_bytes = count % 6;
    } else if (type == ICM20948_FIFO_ACC_GYR) {
        skip_bytes = count % 12;
    }

    for (int i = 0; i < skip_bytes; i++) {
        icm20948_read_u8(dev, 0, ICM20948_FIFO_R_W);
    }
}

bool icm20948_init_magnetometer(const struct device *dev)
{
    enable_i2c_master(dev);
    icm20948_reset_mag(dev);
    icm20948_reset(dev);
    icm20948_sleep(dev, false);
    icm20948_write_u8(dev, 2, ICM20948_ODR_ALIGN_EN, 1);

    bool success = false;
    for (int i = 0; i < 10; i++) {
        k_sleep(K_MSEC(10));
        enable_i2c_master(dev);
        k_sleep(K_MSEC(10));

        uint16_t whoami = icm20948_mag_whoami(dev);
        if (whoami == AK09916_WHO_AM_I_1 || whoami == AK09916_WHO_AM_I_2) {
            success = true;
            break;
        } else {
            i2c_master_reset(dev);
        }
    }

    if (success) {
        icm20948_set_mag_op_mode(dev, AK09916_CONT_MODE_100HZ);
    }

    return success;
}

uint16_t icm20948_mag_whoami(const struct device *dev)
{
    uint8_t msb = read_ak09916_register_u8(dev, AK09916_WIA_1);
    uint8_t lsb = read_ak09916_register_u8(dev, AK09916_WIA_2);
    return ((uint16_t)msb << 8) | lsb;
}

void icm20948_set_mag_op_mode(const struct device *dev, enum ak09916_op_mode mode)
{
    write_ak09916_register_u8(dev, AK09916_CNTL_2, mode);
    k_sleep(K_MSEC(10));

    if (mode != AK09916_PWR_DOWN) {
        enable_mag_data_read(dev, AK09916_HXL, 0x08);
    }
}

void icm20948_reset_mag(const struct device *dev)
{
    write_ak09916_register_u8(dev, AK09916_CNTL_3, 0x01);
    k_sleep(K_MSEC(100));
}

void icm20948_set_clock_auto(const struct device *dev)
{
    uint8_t reg = icm20948_read_u8(dev, 0, ICM20948_PWR_MGMT_1);
    reg |= 0x01;
    icm20948_write_u8(dev, 0, ICM20948_PWR_MGMT_1, reg);
    k_sleep(K_MSEC(10));
}

void icm20948_correct_accel(struct icm20948_data *data, struct icm20948_vec3f *v)
{
    v->x = (v->x - (data->acc_offset.x / data->acc_range)) / data->acc_corr.x;
    v->y = (v->y - (data->acc_offset.y / data->acc_range)) / data->acc_corr.y;
    v->z = (v->z - (data->acc_offset.z / data->acc_range)) / data->acc_corr.z;
}

void icm20948_correct_gyro(struct icm20948_data *data, struct icm20948_vec3f *v)
{
    v->x -= (data->gyro_offset.x / data->gyro_range);
    v->y -= (data->gyro_offset.y / data->gyro_range);
    v->z -= (data->gyro_offset.z / data->gyro_range);
}


void icm20948_switch_bank(const struct device *dev, uint8_t new_bank)
{
    struct icm20948_data *data = dev->data;
    const struct icm20948_config *cfg = dev->config;

    if (new_bank != data->current_bank) {
        data->current_bank = new_bank;

        uint8_t bank_reg = new_bank << 4;
        i2c_reg_write_byte_dt(&cfg->i2c, ICM20948_REG_BANK_SEL, bank_reg);
    }
}


void icm20948_write_register8(const struct device *dev, uint8_t bank, uint8_t reg, uint8_t val)
{
    const struct icm20948_config *cfg = dev->config;

    icm20948_switch_bank(dev, bank);
    i2c_reg_write_byte(cfg->i2c, reg, val);
    k_busy_wait(5);  // delayMicroseconds(5)
}

void icm20948_write_register16(const struct device *dev, uint8_t bank, uint8_t reg, uint16_t val)
{
    const struct icm20948_config *cfg = dev->config;

    icm20948_switch_bank(dev, bank);

    uint8_t buf[2] = { (val >> 8) & 0xFF, val & 0xFF };
    i2c_burst_write(cfg->i2c, reg, buf, 2);
    k_busy_wait(5);
}

uint8_t icm20948_read_register8(const struct device *dev, uint8_t bank, uint8_t reg)
{
    const struct icm20948_config *cfg = dev->config;
    uint8_t value = 0;

    icm20948_switch_bank(dev, bank);

    int ret = i2c_reg_read_byte(cfg->i2c, reg, &value);
    if (ret < 0) {
        LOG_ERR("Failed to read register 0x%02X", reg);
        return 0;
    }

    return value;
}

int16_t icm20948_read_register16(const struct device *dev, uint8_t bank, uint8_t reg)
{
    const struct icm20948_config *cfg = dev->config;
    uint8_t buf[2] = {0};

    icm20948_switch_bank(dev, bank);

    int ret = i2c_burst_read(cfg->i2c.bus, cfg->i2c.addr, reg, buf, 2);
    if (ret < 0) {
        LOG_ERR("Failed to read 16-bit register at 0x%02X", reg);
        return 0;
    }

    return (int16_t)((buf[0] << 8) | buf[1]);
}

void icm20948_read_all_data(const struct device *dev, uint8_t *data)
{
    const struct icm20948_config *cfg = dev->config;

    icm20948_switch_bank(dev, 0);

    int ret = i2c_burst_read(cfg->i2c.bus, cfg->i2c.addr, ICM20948_ACCEL_OUT, data, 20);
    if (ret < 0) {
        LOG_ERR("Failed to read all sensor data");
    }
}


void icm20948_read_xyz_from_fifo(const struct device *dev, struct icm20948_vec3f *out)
{
    const struct icm20948_config *cfg = dev->config;
    uint8_t fifo_data[6];

    icm20948_switch_bank(dev, 0);

    int ret = i2c_burst_read(cfg->i2c.bus, cfg->i2c.addr, ICM20948_FIFO_R_W, fifo_data, 6);
    if (ret < 0) {
        LOG_ERR("Failed to read FIFO data");
        return;
    }

    out->x = (int16_t)((fifo_data[0] << 8) | fifo_data[1]);
    out->y = (int16_t)((fifo_data[2] << 8) | fifo_data[3]);
    out->z = (int16_t)((fifo_data[4] << 8) | fifo_data[5]);
}


void icm20948_write_ak09916_register_slv4(const struct device *dev, uint8_t reg, uint8_t val)
{
    // Set AK09916 write address and register
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_ADDR, AK09916_ADDRESS);
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_DO, val);
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_REG, reg);
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVX_EN);

    // Wait for transaction complete with timeout
    int64_t timeout = k_uptime_get() + 100;
    while ((icm20948_read_register8(dev, 3, ICM20948_I2C_SLV4_CTRL) & ICM20948_I2C_SLVX_EN)) {
        if (k_uptime_get() > timeout) {
            LOG_ERR("SLV4 write timeout");
            break;
        }
    }
}

uint8_t icm20948_read_ak09916_register8_slv4(const struct device *dev, uint8_t reg)
{
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_ADDR, AK09916_ADDRESS | AK09916_READ);
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_REG, reg);
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVX_EN);

    int64_t timeout = k_uptime_get() + 100;
    while ((icm20948_read_register8(dev, 3, ICM20948_I2C_SLV4_CTRL) & ICM20948_I2C_SLVX_EN)) {
        if (k_uptime_get() > timeout) {
            LOG_ERR("SLV4 read timeout");
            return 0;
        }
    }

    return icm20948_read_register8(dev, 3, ICM20948_I2C_SLV4_DI);
}

int16_t icm20948_read_ak09916_register16(const struct device *dev, uint8_t reg)
{
    icm20948_enable_mag_data_read(dev, reg, 2);
    int16_t val = icm20948_read_register16(dev, 0, ICM20948_EXT_SLV_SENS_DATA_00);
    icm20948_enable_mag_data_read(dev, AK09916_HXL, 0x08);  // restore default continuous read
    return val;
}

void icm20948_reset_chip(const struct device *dev)
{
    icm20948_write_register8(dev, 0, ICM20948_PWR_MGMT_1, ICM20948_RESET);
    k_sleep(K_MSEC(10));
}

void icm20948_enable_i2c_master(const struct device *dev)
{
    icm20948_write_register8(dev, 0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN);
    icm20948_write_register8(dev, 3, ICM20948_I2C_MST_CTRL, 0x07);  // 345.6 kHz
    k_sleep(K_MSEC(10));
}

void icm20948_i2c_master_reset(const struct device *dev)
{
    uint8_t val = icm20948_read_register8(dev, 0, ICM20948_USER_CTRL);
    val |= ICM20948_I2C_MST_RST;
    icm20948_write_register8(dev, 0, ICM20948_USER_CTRL, val);
    k_sleep(K_MSEC(10));
}

void icm20948_enable_mag_data_read(const struct device *dev, uint8_t reg, uint8_t bytes)
{
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | AK09916_READ);
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV0_REG, reg);
    icm20948_write_register8(dev, 3, ICM20948_I2C_SLV0_CTRL, ICM20948_I2C_SLVX_EN | bytes);
    k_sleep(K_MSEC(10));
}




static const struct icm20948_config icm20948_config_0 = {
	.i2c = I2C_DT_SPEC_INST_GET(0),
};

static struct icm20948_data icm20948_data_0;

DEVICE_DT_INST_DEFINE(0,
		    icm20948_init,
		    NULL,
		    &icm20948_data_0,
		    &icm20948_config_0,
		    POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY,
		    NULL);