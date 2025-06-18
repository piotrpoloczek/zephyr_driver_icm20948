#ifndef ZEPHYR_ICM20948_H_
#define ZEPHYR_ICM20948_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <stdint.h>

/* Enums for ICM20948 configuration */

enum icm20948_cycle {
    ICM20948_NO_CYCLE              = 0x00,
    ICM20948_GYR_CYCLE             = 0x10,
    ICM20948_ACC_CYCLE             = 0x20,
    ICM20948_ACC_GYR_CYCLE         = 0x30,
    ICM20948_ACC_GYR_I2C_MST_CYCLE = 0x70
};

enum icm20948_int_pin_pol {
    ICM20948_ACT_HIGH,
    ICM20948_ACT_LOW
};

enum icm20948_int_type {
    ICM20948_FSYNC_INT      = 0x01,
    ICM20948_WOM_INT        = 0x02,
    ICM20948_DMP_INT        = 0x04,
    ICM20948_DATA_READY_INT = 0x08,
    ICM20948_FIFO_OVF_INT   = 0x10,
    ICM20948_FIFO_WM_INT    = 0x20
};

enum icm20948_fifo_type {
    ICM20948_FIFO_ACC     = 0x10,
    ICM20948_FIFO_GYR     = 0x0E,
    ICM20948_FIFO_ACC_GYR = 0x1E
};

enum icm20948_fifo_mode {
    ICM20948_CONTINUOUS,
    ICM20948_STOP_WHEN_FULL
};

enum icm20948_gyro_range {
    ICM20948_GYRO_RANGE_250,
    ICM20948_GYRO_RANGE_500,
    ICM20948_GYRO_RANGE_1000,
    ICM20948_GYRO_RANGE_2000
};

enum icm20948_dlpf {
    ICM20948_DLPF_0,
    ICM20948_DLPF_1,
    ICM20948_DLPF_2,
    ICM20948_DLPF_3,
    ICM20948_DLPF_4,
    ICM20948_DLPF_5,
    ICM20948_DLPF_6,
    ICM20948_DLPF_7,
    ICM20948_DLPF_OFF
};

enum icm20948_gyro_avg_low_pwr {
    ICM20948_GYR_AVG_1,
    ICM20948_GYR_AVG_2,
    ICM20948_GYR_AVG_4,
    ICM20948_GYR_AVG_8,
    ICM20948_GYR_AVG_16,
    ICM20948_GYR_AVG_32,
    ICM20948_GYR_AVG_64,
    ICM20948_GYR_AVG_128
};

enum icm20948_acc_range {
    ICM20948_ACC_RANGE_2G,
    ICM20948_ACC_RANGE_4G,
    ICM20948_ACC_RANGE_8G,
    ICM20948_ACC_RANGE_16G
};

enum icm20948_acc_avg_low_pwr {
    ICM20948_ACC_AVG_4,
    ICM20948_ACC_AVG_8,
    ICM20948_ACC_AVG_16,
    ICM20948_ACC_AVG_32
};

enum icm20948_wom_comp_en {
    ICM20948_WOM_COMP_DISABLE,
    ICM20948_WOM_COMP_ENABLE
};

enum ak09916_op_mode {
    AK09916_PWR_DOWN        = 0x00,
    AK09916_TRIGGER_MODE    = 0x01,
    AK09916_CONT_MODE_10HZ  = 0x02,
    AK09916_CONT_MODE_20HZ  = 0x04,
    AK09916_CONT_MODE_50HZ  = 0x06,
    AK09916_CONT_MODE_100HZ = 0x08
};

enum icm20948_orientation {
    ICM20948_FLAT,
    ICM20948_FLAT_1,
    ICM20948_XY,
    ICM20948_XY_1,
    ICM20948_YX,
    ICM20948_YX_1
};


#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20948_REGS_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20948_REGS_H_

/* AK09916 Magnetometer I2C Address */
#define AK09916_ADDRESS              0x0C

/* ICM20948 WHO_AM_I response */
#define ICM20948_WHO_AM_I_CONTENT    0xEA

/* WHO_AM_I values for AK09916 */
#define AK09916_WHO_AM_I_1           0x4809
#define AK09916_WHO_AM_I_2           0x0948

/* Register Bank Select */
#define ICM20948_REG_BANK_SEL        0x7F

/* USER BANK 0 Registers */
#define ICM20948_WHO_AM_I            0x00
#define ICM20948_USER_CTRL           0x03
#define ICM20948_LP_CONFIG           0x05
#define ICM20948_PWR_MGMT_1          0x06
#define ICM20948_PWR_MGMT_2          0x07
#define ICM20948_INT_PIN_CFG         0x0F
#define ICM20948_INT_ENABLE          0x10
#define ICM20948_INT_ENABLE_1        0x11
#define ICM20948_INT_ENABLE_2        0x12
#define ICM20948_INT_ENABLE_3        0x13
#define ICM20948_I2C_MST_STATUS      0x17
#define ICM20948_INT_STATUS          0x19
#define ICM20948_INT_STATUS_1        0x1A
#define ICM20948_INT_STATUS_2        0x1B
#define ICM20948_INT_STATUS_3        0x1C
#define ICM20948_DELAY_TIME_H        0x28
#define ICM20948_DELAY_TIME_L        0x29
#define ICM20948_ACCEL_OUT           0x2D
#define ICM20948_GYRO_OUT            0x33
#define ICM20948_TEMP_OUT            0x39
#define ICM20948_EXT_SLV_SENS_DATA_00 0x3B
#define ICM20948_EXT_SLV_SENS_DATA_01 0x3C
#define ICM20948_FIFO_EN_1           0x66
#define ICM20948_FIFO_EN_2           0x67
#define ICM20948_FIFO_RST            0x68
#define ICM20948_FIFO_MODE           0x69
#define ICM20948_FIFO_COUNT          0x70
#define ICM20948_FIFO_R_W            0x72
#define ICM20948_DATA_RDY_STATUS     0x74
#define ICM20948_FIFO_CFG            0x76

/* USER BANK 1 Registers */
#define ICM20948_SELF_TEST_X_GYRO    0x02
#define ICM20948_SELF_TEST_Y_GYRO    0x03
#define ICM20948_SELF_TEST_Z_GYRO    0x04
#define ICM20948_SELF_TEST_X_ACCEL   0x0E
#define ICM20948_SELF_TEST_Y_ACCEL   0x0F
#define ICM20948_SELF_TEST_Z_ACCEL   0x10
#define ICM20948_XA_OFFS_H           0x14
#define ICM20948_XA_OFFS_L           0x15
#define ICM20948_YA_OFFS_H           0x17
#define ICM20948_YA_OFFS_L           0x18
#define ICM20948_ZA_OFFS_H           0x1A
#define ICM20948_ZA_OFFS_L           0x1B
#define ICM20948_TIMEBASE_CORR_PLL   0x28

/* USER BANK 2 Registers */
#define ICM20948_GYRO_SMPLRT_DIV     0x00
#define ICM20948_GYRO_CONFIG_1       0x01
#define ICM20948_GYRO_CONFIG_2       0x02
#define ICM20948_XG_OFFS_USRH        0x03
#define ICM20948_XG_OFFS_USRL        0x04
#define ICM20948_YG_OFFS_USRH        0x05
#define ICM20948_YG_OFFS_USRL        0x06
#define ICM20948_ZG_OFFS_USRH        0x07
#define ICM20948_ZG_OFFS_USRL        0x08
#define ICM20948_ODR_ALIGN_EN        0x09
#define ICM20948_ACCEL_SMPLRT_DIV_1  0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2  0x11
#define ICM20948_ACCEL_INTEL_CTRL    0x12
#define ICM20948_ACCEL_WOM_THR       0x13
#define ICM20948_ACCEL_CONFIG        0x14
#define ICM20948_ACCEL_CONFIG_2      0x15
#define ICM20948_FSYNC_CONFIG        0x52
#define ICM20948_TEMP_CONFIG         0x53
#define ICM20948_MOD_CTRL_USR        0x54

/* USER BANK 3 Registers */
#define ICM20948_I2C_MST_ODR_CFG     0x00
#define ICM20948_I2C_MST_CTRL        0x01
#define ICM20948_I2C_MST_DELAY_CTRL  0x02
#define ICM20948_I2C_SLV0_ADDR       0x03
#define ICM20948_I2C_SLV0_REG        0x04
#define ICM20948_I2C_SLV0_CTRL       0x05
#define ICM20948_I2C_SLV0_DO         0x06
#define ICM20948_I2C_SLV4_ADDR       0x13
#define ICM20948_I2C_SLV4_REG        0x14
#define ICM20948_I2C_SLV4_CTRL       0x15
#define ICM20948_I2C_SLV4_DO         0x16
#define ICM20948_I2C_SLV4_DI         0x17

/* AK09916 Registers */
#define AK09916_WIA_1    0x00
#define AK09916_WIA_2    0x01
#define AK09916_STATUS_1 0x10
#define AK09916_HXL      0x11
#define AK09916_HXH      0x12
#define AK09916_HYL      0x13
#define AK09916_HYH      0x14
#define AK09916_HZL      0x15
#define AK09916_HZH      0x16
#define AK09916_STATUS_2 0x18
#define AK09916_CNTL_2   0x31
#define AK09916_CNTL_3   0x32

/* Bit Masks */
#define ICM20948_RESET              0x80
#define ICM20948_I2C_MST_EN         0x20
#define ICM20948_SLEEP              0x40
#define ICM20948_LP_EN              0x20
#define ICM20948_BYPASS_EN          0x02
#define ICM20948_GYR_EN             0x07
#define ICM20948_ACC_EN             0x38
#define ICM20948_FIFO_EN            0x40
#define ICM20948_INT1_ACTL          0x80
#define ICM20948_INT_1_LATCH_EN     0x20
#define ICM20948_ACTL_FSYNC         0x08
#define ICM20948_INT_ANYRD_2CLEAR   0x10
#define ICM20948_FSYNC_INT_MODE_EN  0x06
#define ICM20948_I2C_SLVX_EN        0x80
#define AK09916_16_BIT              0x10
#define AK09916_OVF                 0x08
#define AK09916_READ                0x80

/* Conversion Factors */
#define ICM20948_ROOM_TEMP_OFFSET   0.0f
#define ICM20948_T_SENSITIVITY      333.87f
#define AK09916_MAG_LSB             0.1495f
#define ICM20948_I2C_MST_RST        0x02


struct icm20948_config {
	struct i2c_dt_spec i2c;
};

struct icm20948_data {
	uint8_t current_bank;
	struct icm20948_vec3f acc_offset;
	struct icm20948_vec3f acc_corr_factor;
	float acc_range_factor;
	struct icm20948_vec3f gyr_offset;
	float gyr_range_factor;
	uint8_t buffer[20];
	uint8_t reg_val;
	int16_t temperature;
	enum icm20948_fifo_type fifo_type;
};

int icm20948_init(const struct device *dev);
void icm20948_auto_offsets(const struct device *dev);
void icm20948_set_acc_offsets_minmax(const struct device *dev, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
void icm20948_set_acc_offsets(const struct device *dev, struct icm20948_vec3f offset);
struct icm20948_vec3f icm20948_get_acc_offsets(const struct device *dev);
void icm20948_set_gyr_offsets(const struct device *dev, struct icm20948_vec3f offset);
struct icm20948_vec3f icm20948_get_gyr_offsets(const struct device *dev);
uint8_t icm20948_whoami(const struct device *dev);
void icm20948_enable_acc(const struct device *dev, bool enable);
void icm20948_set_acc_range(const struct device *dev, enum icm20948_acc_range range);
void icm20948_set_acc_dlpf(const struct device *dev, enum icm20948_dlpf dlpf);
void icm20948_set_acc_sample_rate_div(const struct device *dev, uint16_t divider);
void icm20948_enable_gyr(const struct device *dev, bool enable);
void icm20948_set_gyr_range(const struct device *dev, enum icm20948_gyro_range range);
void icm20948_set_gyr_dlpf(const struct device *dev, enum icm20948_dlpf dlpf);
void icm20948_set_gyr_sample_rate_div(const struct device *dev, uint8_t divider);
void icm20948_set_temp_dlpf(const struct device *dev, enum icm20948_dlpf dlpf);
void icm20948_set_i2c_master_sample_rate(const struct device *dev, uint8_t rate);

void icm20948_read_sensor(const struct device *dev);
void icm20948_get_acc_raw(const struct device *dev, struct icm20948_vec3f *out);
void icm20948_get_acc_corrected(const struct device *dev, struct icm20948_vec3f *out);
void icm20948_get_acc_g(const struct device *dev, struct icm20948_vec3f *out);
void icm20948_get_gyr_raw(const struct device *dev, struct icm20948_vec3f *out);
void icm20948_get_gyr_corrected(const struct device *dev, struct icm20948_vec3f *out);
void icm20948_get_gyr_values(const struct device *dev, struct icm20948_vec3f *out);
float icm20948_get_temperature(const struct device *dev);

void icm20948_enable_cycle(const struct device *dev, enum icm20948_cycle cycle);
void icm20948_enable_low_power(const struct device *dev, bool enable);
void icm20948_set_gyr_avg_cycle_mode(const struct device *dev, enum icm20948_gyr_avg_low_pwr avg);
void icm20948_set_acc_avg_cycle_mode(const struct device *dev, enum icm20948_acc_avg_low_pwr avg);
void icm20948_sleep(const struct device *dev, bool enable);

void icm20948_set_int_polarity(const struct device *dev, enum icm20948_int_pin_pol pol);
void icm20948_enable_int_latch(const struct device *dev, bool latch);
void icm20948_clear_int_on_read(const struct device *dev, bool enable);
void icm20948_set_fsync_polarity(const struct device *dev, enum icm20948_int_pin_pol pol);
void icm20948_enable_interrupt(const struct device *dev, enum icm20948_int_type type);
void icm20948_disable_interrupt(const struct device *dev, enum icm20948_int_type type);
uint8_t icm20948_read_and_clear_ints(const struct device *dev);
bool icm20948_check_int(const struct device *dev, uint8_t src, enum icm20948_int_type type);
void icm20948_set_wom_threshold(const struct device *dev, uint8_t threshold, enum icm20948_wom_comp wom);

void icm20948_enable_fifo(const struct device *dev, bool enable);
void icm20948_set_fifo_mode(const struct device *dev, enum icm20948_fifo_mode mode);
void icm20948_start_fifo(const struct device *dev, enum icm20948_fifo_type type);
void icm20948_stop_fifo(const struct device *dev);
void icm20948_reset_fifo(const struct device *dev);
int16_t icm20948_get_fifo_count(const struct device *dev);
int16_t icm20948_get_fifo_dataset_count(const struct device *dev);
void icm20948_find_fifo_begin(const struct device *dev);

bool icm20948_mag_init(const struct device *dev);
uint16_t icm20948_mag_whoami(const struct device *dev);
void icm20948_mag_set_mode(const struct device *dev, enum ak09916_op_mode mode);
void icm20948_mag_reset(const struct device *dev);

#endif
