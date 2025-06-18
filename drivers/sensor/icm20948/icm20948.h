#pragma once

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Sensor operational modes */
enum icm20948_cycle {
    ICM20948_NO_CYCLE              = 0x00,
    ICM20948_GYR_CYCLE             = 0x10, 
    ICM20948_ACC_CYCLE             = 0x20,
    ICM20948_ACC_GYR_CYCLE         = 0x30,
    ICM20948_ACC_GYR_I2C_MST_CYCLE = 0x70
};

/* Interrupt polarity */
enum icm20948_int_pin_pol {
    ICM20948_INT_POL_ACTIVE_HIGH,
    ICM20948_INT_POL_ACTIVE_LOW
};

/* Interrupt sources */
enum icm20948_int_type {
    ICM20948_INT_FSYNC        = 0x01,
    ICM20948_INT_WOM          = 0x02,
    ICM20948_INT_DMP          = 0x04,
    ICM20948_INT_DATA_READY   = 0x08,
    ICM20948_INT_FIFO_OVF     = 0x10,
    ICM20948_INT_FIFO_WM      = 0x20
};

/* FIFO configuration */
enum icm20948_fifo_type {
    ICM20948_FIFO_ACC      = 0x10,
    ICM20948_FIFO_GYR      = 0x0E,
    ICM20948_FIFO_ACC_GYR  = 0x1E
};

enum icm20948_fifo_mode {
    ICM20948_FIFO_MODE_CONTINUOUS,
    ICM20948_FIFO_MODE_STOP_WHEN_FULL
};

/* Gyroscope configuration */
enum icm20948_gyro_range {
    ICM20948_GYRO_RANGE_250_DPS,
    ICM20948_GYRO_RANGE_500_DPS,
    ICM20948_GYRO_RANGE_1000_DPS,
    ICM20948_GYRO_RANGE_2000_DPS
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

enum icm20948_gyro_avg {
    ICM20948_GYR_AVG_1,
    ICM20948_GYR_AVG_2,
    ICM20948_GYR_AVG_4,
    ICM20948_GYR_AVG_8,
    ICM20948_GYR_AVG_16,
    ICM20948_GYR_AVG_32,
    ICM20948_GYR_AVG_64,
    ICM20948_GYR_AVG_128
};

/* Accelerometer configuration */
enum icm20948_acc_range {
    ICM20948_ACC_RANGE_2G,
    ICM20948_ACC_RANGE_4G,
    ICM20948_ACC_RANGE_8G,
    ICM20948_ACC_RANGE_16G
};

enum icm20948_acc_avg {
    ICM20948_ACC_AVG_4,
    ICM20948_ACC_AVG_8,
    ICM20948_ACC_AVG_16,
    ICM20948_ACC_AVG_32
};

/* Wake-on-motion comparator */
enum icm20948_wom_comp {
    ICM20948_WOM_COMP_DISABLED,
    ICM20948_WOM_COMP_ENABLED
};

/* Magnetometer (AK09916) operation mode */
enum ak09916_op_mode {
    AK09916_MODE_PWR_DOWN        = 0x00,
    AK09916_MODE_TRIGGER         = 0x01,
    AK09916_MODE_CONTINUOUS_10HZ = 0x02,
    AK09916_MODE_CONTINUOUS_20HZ = 0x04,
    AK09916_MODE_CONTINUOUS_50HZ = 0x06,
    AK09916_MODE_CONTINUOUS_100HZ= 0x08
};

/* Device orientation options */
enum icm20948_orientation {
    ICM20948_ORIENT_FLAT,
    ICM20948_ORIENT_FLAT_1,
    ICM20948_ORIENT_XY,
    ICM20948_ORIENT_XY_1,
    ICM20948_ORIENT_YX,
    ICM20948_ORIENT_YX_1
};

#ifdef __cplusplus
}
#endif
