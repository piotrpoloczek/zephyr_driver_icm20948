#pragma once

//
// ICM-20948 Register Banks
//
#define ICM20948_REG_BANK_SEL      0x7F

#define ICM20948_BANK0             0x00
#define ICM20948_BANK1             0x10
#define ICM20948_BANK2             0x20
#define ICM20948_BANK3             0x30

//
// Bank 0 Registers
//
#define ICM20948_WHO_AM_I          0x00
#define ICM20948_REG_USER_CTRL     0x03
#define ICM20948_REG_LP_CONFIG     0x05
#define ICM20948_REG_PWR_MGMT_1    0x06
#define ICM20948_REG_PWR_MGMT_2    0x07
#define ICM20948_REG_INT_PIN_CFG   0x0F
#define ICM20948_REG_INT_ENABLE    0x10
#define ICM20948_REG_ACCEL_XOUT_H  0x2D
#define ICM20948_REG_GYRO_XOUT_H   0x33
#define ICM20948_REG_TEMP_OUT_H    0x39
#define ICM20948_REG_EXT_SLV_SENS_DATA_00 0x3B

//
// Bank 1 Registers
//
#define ICM20948_REG_SELF_TEST_X_GYRO 0x02
#define ICM20948_REG_SELF_TEST_Y_GYRO 0x03
#define ICM20948_REG_SELF_TEST_Z_GYRO 0x04
#define ICM20948_REG_XA_OFFS_H        0x14
#define ICM20948_REG_YA_OFFS_H        0x17
#define ICM20948_REG_ZA_OFFS_H        0x1A
#define ICM20948_REG_TIMEBASE_CORR_PL 0x28

//
// Bank 2 Registers
//
#define ICM20948_REG_GYRO_SMPLRT_DIV  0x00
#define ICM20948_REG_GYRO_CONFIG_1    0x01
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_REG_ACCEL_CONFIG     0x14
#define ICM20948_REG_ACCEL_CONFIG_2   0x15

//
// Bank 3 Registers
//
#define ICM20948_REG_I2C_MST_CTRL     0x01
#define ICM20948_REG_I2C_SLV0_ADDR    0x03
#define ICM20948_REG_I2C_SLV0_REG     0x04
#define ICM20948_REG_I2C_SLV0_CTRL    0x05
#define ICM20948_REG_I2C_SLV0_DO      0x06

//
// USER_CTRL bits
//
#define ICM20948_I2C_IF_DIS           0x10
#define ICM20948_I2C_MST_EN           0x20

//
// PWR_MGMT_1 bits
//
#define ICM20948_CLKSEL_AUTO          0x01
#define ICM20948_DEVICE_RESET         0x80
#define ICM20948_SLEEP                0x40

//
// PWR_MGMT_2 bits
//
#define ICM20948_DISABLE_ACCEL        0x38
#define ICM20948_DISABLE_GYRO         0x07

//
// INT_PIN_CFG bits
//
#define ICM20948_BYPASS_EN            0x02

//
// Gyro Config 1 bits
//
#define ICM20948_GYRO_FS_SEL_SHIFT    1
#define ICM20948_GYRO_FS_SEL_MASK     0x06

//
// Accel Config bits
//
#define ICM20948_ACCEL_FS_SEL_SHIFT   1
#define ICM20948_ACCEL_FS_SEL_MASK    0x06

//
// AK09916 Magnetometer (via I2C passthrough)
//
#define AK09916_I2C_ADDR              0x0C

#define AK09916_REG_WHO_AM_I         0x01
#define AK09916_REG_ST1              0x10
#define AK09916_REG_HXL              0x11
#define AK09916_REG_HXH              0x12
#define AK09916_REG_HYL              0x13
#define AK09916_REG_HYH              0x14
#define AK09916_REG_HZL              0x15
#define AK09916_REG_HZH              0x16
#define AK09916_REG_ST2              0x18
#define AK09916_REG_CNTL2            0x31
#define AK09916_REG_CNTL3            0x32

#define AK09916_RESET_CMD            0x01
#define AK09916_SOFT_RESET_CMD       0x01

//
// Magnetometer data-ready and overflow bits
//
#define AK09916_ST1_DRDY             0x01
#define AK09916_ST2_HOFL             0x08


// I2C SLV4 (direct write/read to AK09916 registers)
#define ICM20948_REG_I2C_SLV4_ADDR   0x07
#define ICM20948_REG_I2C_SLV4_REG    0x08
#define ICM20948_REG_I2C_SLV4_DO     0x09
#define ICM20948_REG_I2C_SLV4_CTRL   0x0A
#define ICM20948_REG_I2C_SLV4_DI     0x0B


// #define AK09916_CONT_MODE_100HZ 0x08
// #define AK09916_REG_CNTL2       0x31
// #define AK09916_REG_ST1         0x10
// #define AK09916_REG_ST2         0x18
// #define AK09916_MAG_SCALE       0.15f


#define AK09916_REG_CNTL2  0x31
#define AK09916_REG_CNTL3  0x32
#define AK09916_REG_WIA_1  0x00
#define AK09916_REG_WIA_2  0x01
#define AK09916_WHO_AM_I_1 0x4809
#define AK09916_WHO_AM_I_2 0x0948

#define ICM20948_WHO_AM_I          0x00  // WHO_AM_I register address
#define ICM20948_WHO_AM_I_ID       0xEA  // Expected response from sensor

#define AK09916_WHO_AM_I 0x09



