/**
 * @file    bsp_imu660rb.h
 * @brief   SeekFree IMU660RB (ST LSM6DSR) logic with abstract SPI transport.
 */
#ifndef BSP_IMU660RB_H
#define BSP_IMU660RB_H

#include <stdbool.h>
#include <stdint.h>

#define IMU660RB_WHO_AM_I             0x0Fu
#define IMU660RB_CTRL1_XL              0x10u
#define IMU660RB_CTRL2_G               0x11u
#define IMU660RB_CTRL3_C               0x12u
#define IMU660RB_CTRL4_C               0x13u
#define IMU660RB_CTRL5_C               0x14u
#define IMU660RB_CTRL6_C               0x15u
#define IMU660RB_CTRL7_G               0x16u
#define IMU660RB_CTRL9_XL              0x18u
#define IMU660RB_OUT_TEMP_L            0x20u

#define IMU660RB_WHO_AM_I_VALUE        0x6Bu
#define IMU660RB_CTRL3_SW_RESET_MASK   0x01u
#define IMU660RB_CTRL1_XL_VALUE        0x68u
#define IMU660RB_CTRL2_G_VALUE         0x64u
#define IMU660RB_CTRL3_C_VALUE         0x44u
#define IMU660RB_CTRL4_C_VALUE         0x06u
#define IMU660RB_CTRL5_C_VALUE         0x00u
#define IMU660RB_CTRL6_C_VALUE         0x00u
#define IMU660RB_CTRL7_G_VALUE         0x00u
#define IMU660RB_CTRL9_XL_VALUE        0x02u

#define IMU660RB_RESET_POLL_ATTEMPTS   20u
#define IMU660RB_RESET_POLL_MS         2u
#define IMU660RB_TURN_ON_MS            35u

#define IMU660RB_ACCEL_G_PER_LSB            0.000122f
#define IMU660RB_GYRO_DPS_PER_LSB           0.0175f
#define IMU660RB_TEMP_C_PER_LSB             (1.0f / 256.0f)
#define IMU660RB_TEMP_OFFSET_C              25.0f
#define IMU660RB_FRAME_BYTES                14u

#define IMU660RB_CALIB_SAMPLES             200u
#define IMU660RB_CALIB_MAX_STD_DPS         0.8f
#define IMU660RB_CALIB_MAX_READ_FAILURES   20u
#define IMU660RB_CALIB_MAX_MOTION_REJECTIONS 200u
#define IMU660RB_CALIB_GYRO_MAX_DPS        3.0f
#define IMU660RB_STATIONARY_ACCEL_MIN_G2   0.81f
#define IMU660RB_STATIONARY_ACCEL_MAX_G2   1.21f

#define IMU660RB_GYRO_FILTER_ALPHA         0.35f
#define IMU660RB_GYRO_DEADZONE_MIN_DPS     0.05f
#define IMU660RB_GYRO_DEADZONE_MAX_DPS     0.50f
#define IMU660RB_GYRO_DEADZONE_NOISE_GAIN  3.0f
#define IMU660RB_BIAS_STATIONARY_RATE_DPS  0.8f
#define IMU660RB_BIAS_CONFIRM_SAMPLES      100u
#define IMU660RB_BIAS_ADAPT_ALPHA          0.0025f
#define IMU660RB_BIAS_MAX_STEP_DPS         0.001f
#define IMU660RB_GYRO_Z_SIGN              -1.0f

typedef enum {
    IMU_CAL_IDLE = 0,
    IMU_CAL_RUNNING,
    IMU_CAL_DONE,
    IMU_CAL_FAILED
} imu_cal_state_t;

typedef enum {
    IMU660RB_INIT_OK = 0,
    IMU660RB_INIT_INVALID_ARGUMENT,
    IMU660RB_INIT_WHO_READ_FAILED,
    IMU660RB_INIT_WHO_MISMATCH,
    IMU660RB_INIT_RESET_WRITE_FAILED,
    IMU660RB_INIT_RESET_TIMEOUT,
    IMU660RB_INIT_WHO_AFTER_RESET_READ_FAILED,
    IMU660RB_INIT_WHO_AFTER_RESET_MISMATCH,
    IMU660RB_INIT_CONFIG_WRITE_FAILED,
    IMU660RB_INIT_CONFIG_READ_FAILED,
    IMU660RB_INIT_CONFIG_MISMATCH
} imu660rb_init_result_t;

typedef struct {
    bool (*Spi_Transfer)(uint8_t tx, uint8_t *rx);
    void (*Cs_Write)(bool high);
    void (*Delay_ms)(uint32_t ms);

    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float gyro_z_raw;
    float temp;
    float gyro_z_offset;
    float gyro_z_noise;
    float gyro_z_deadzone;
    float gyro_z_filtered;
    float gyro_z_previous;
    float heading;

    bool initialized;
    bool data_valid;
    imu_cal_state_t calibration_state;
    imu660rb_init_result_t init_result;
    uint8_t init_who_am_i;
    uint8_t init_ctrl1_xl;
    uint8_t init_ctrl2_g;
    uint8_t init_ctrl3_c;
    uint8_t init_ctrl4_c;
    uint8_t init_ctrl9_xl;

    uint16_t calibration_sample_count;
    uint8_t calibration_read_failures;
    uint16_t calibration_motion_rejections;
    float calibration_mean;
    float calibration_m2;
    uint16_t stationary_sample_count;
    uint32_t bias_update_count;
    bool gyro_filter_initialized;
    bool stationary_confirmed;
} imu_sensor_t;

bool Bsp_Imu660rb_Init_Device(imu_sensor_t *imu);
const char *Bsp_Imu660rb_Init_Result_Name(imu660rb_init_result_t result);
bool Bsp_Imu660rb_Read_Data(imu_sensor_t *imu);
void Bsp_Imu660rb_Calibration_Start(imu_sensor_t *imu);
imu_cal_state_t Bsp_Imu660rb_Calibration_Update(imu_sensor_t *imu);
bool Bsp_Imu660rb_Update_Heading(imu_sensor_t *imu, float dt);
bool Bsp_Imu660rb_Update_Stationary(imu_sensor_t *imu, float dt);

#endif /* BSP_IMU660RB_H */
