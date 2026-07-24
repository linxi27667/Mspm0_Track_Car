/**
 * @file    bsp_imu660rb.c
 * @brief   IMU660RB register access, conversion, calibration and yaw integration.
 */
#include "bsp_imu660rb.h"

#include <math.h>
#include <stddef.h>

#define IMU660RB_ACCEL_G_PER_LSB   0.000122f
#define IMU660RB_GYRO_DPS_PER_LSB  0.0175f
#define IMU660RB_TEMP_C_PER_LSB    (1.0f / 256.0f)
#define IMU660RB_TEMP_OFFSET_C     25.0f
#define IMU660RB_FRAME_BYTES       14u

static void Bsp_Imu660rb_End_Transaction(imu_sensor_t *imu)
{
    imu->Cs_Write(true);
}

static bool Bsp_Imu660rb_Read_Registers(imu_sensor_t *imu, uint8_t reg,
                                       uint8_t *data, uint16_t len)
{
    uint8_t rx = 0u;
    uint16_t i;

    if (imu == NULL || data == NULL || len == 0u ||
        imu->Spi_Transfer == NULL || imu->Cs_Write == NULL) {
        return false;
    }

    imu->Cs_Write(false);
    if (!imu->Spi_Transfer((uint8_t)(reg | 0x80u), &rx)) {
        Bsp_Imu660rb_End_Transaction(imu);
        return false;
    }
    for (i = 0u; i < len; i++) {
        if (!imu->Spi_Transfer(0u, &data[i])) {
            Bsp_Imu660rb_End_Transaction(imu);
            return false;
        }
    }
    Bsp_Imu660rb_End_Transaction(imu);
    return true;
}

static bool Bsp_Imu660rb_Write_Register(imu_sensor_t *imu, uint8_t reg,
                                        uint8_t value)
{
    uint8_t rx = 0u;

    if (imu == NULL || imu->Spi_Transfer == NULL || imu->Cs_Write == NULL) {
        return false;
    }

    imu->Cs_Write(false);
    if (!imu->Spi_Transfer((uint8_t)(reg & 0x7Fu), &rx) ||
        !imu->Spi_Transfer(value, &rx)) {
        Bsp_Imu660rb_End_Transaction(imu);
        return false;
    }
    Bsp_Imu660rb_End_Transaction(imu);
    return true;
}

static bool Bsp_Imu660rb_Read_Register(imu_sensor_t *imu, uint8_t reg,
                                      uint8_t *value)
{
    return Bsp_Imu660rb_Read_Registers(imu, reg, value, 1u);
}

static int16_t Bsp_Imu660rb_Make_Int16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static bool Bsp_Imu660rb_Frame_Is_Flat(const uint8_t *raw, uint8_t value)
{
    uint16_t i;

    for (i = 0u; i < IMU660RB_FRAME_BYTES; i++) {
        if (raw[i] != value) {
            return false;
        }
    }
    return true;
}

const char *Bsp_Imu660rb_Init_Result_Name(imu660rb_init_result_t result)
{
    switch (result) {
    case IMU660RB_INIT_OK: return "ok";
    case IMU660RB_INIT_INVALID_ARGUMENT: return "invalid_argument";
    case IMU660RB_INIT_WHO_READ_FAILED: return "who_read_failed";
    case IMU660RB_INIT_WHO_MISMATCH: return "who_mismatch";
    case IMU660RB_INIT_RESET_WRITE_FAILED: return "reset_write_failed";
    case IMU660RB_INIT_RESET_TIMEOUT: return "reset_timeout";
    case IMU660RB_INIT_WHO_AFTER_RESET_READ_FAILED: return "who_after_reset_read_failed";
    case IMU660RB_INIT_WHO_AFTER_RESET_MISMATCH: return "who_after_reset_mismatch";
    case IMU660RB_INIT_CONFIG_WRITE_FAILED: return "config_write_failed";
    case IMU660RB_INIT_CONFIG_READ_FAILED: return "config_read_failed";
    case IMU660RB_INIT_CONFIG_MISMATCH: return "config_mismatch";
    default: return "unknown";
    }
}

bool Bsp_Imu660rb_Init_Device(imu_sensor_t *imu)
{
    uint8_t who = 0u;
    uint8_t ctrl3_c = 0u;
    uint8_t ctrl5_c = 0u;
    uint8_t ctrl6_c = 0u;
    uint8_t ctrl7_g = 0u;
    uint8_t attempt;
    bool reset_done = false;
    bool who_read = false;

    if (imu == NULL) {
        return false;
    }

    imu->initialized = false;
    imu->data_valid = false;
    imu->calibration_state = IMU_CAL_IDLE;
    imu->init_result = IMU660RB_INIT_INVALID_ARGUMENT;
    imu->init_who_am_i = 0u;
    imu->init_ctrl1_xl = 0u;
    imu->init_ctrl2_g = 0u;
    imu->init_ctrl3_c = 0u;
    imu->init_ctrl4_c = 0u;
    imu->init_ctrl9_xl = 0u;
    imu->gyro_z_offset = 0.0f;
    imu->heading = 0.0f;

    if (imu->Spi_Transfer == NULL || imu->Cs_Write == NULL ||
        imu->Delay_ms == NULL) {
        return false;
    }
    imu->Cs_Write(true);

    if (!Bsp_Imu660rb_Read_Register(imu, IMU660RB_WHO_AM_I, &who)) {
        imu->init_result = IMU660RB_INIT_WHO_READ_FAILED;
        return false;
    }
    imu->init_who_am_i = who;
    if (who != IMU660RB_WHO_AM_I_VALUE) {
        imu->init_result = IMU660RB_INIT_WHO_MISMATCH;
        return false;
    }

    if (!Bsp_Imu660rb_Write_Register(imu, IMU660RB_CTRL3_C,
                                     IMU660RB_CTRL3_SW_RESET_MASK)) {
        imu->init_result = IMU660RB_INIT_RESET_WRITE_FAILED;
        return false;
    }
    for (attempt = 0u; attempt < IMU660RB_RESET_POLL_ATTEMPTS; attempt++) {
        imu->Delay_ms(IMU660RB_RESET_POLL_MS);
        if (Bsp_Imu660rb_Read_Register(imu, IMU660RB_CTRL3_C, &ctrl3_c) &&
            (ctrl3_c & IMU660RB_CTRL3_SW_RESET_MASK) == 0u) {
            reset_done = true;
        }
        if (reset_done &&
            Bsp_Imu660rb_Read_Register(imu, IMU660RB_WHO_AM_I, &who)) {
            who_read = true;
            imu->init_who_am_i = who;
            if (who == IMU660RB_WHO_AM_I_VALUE) {
                break;
            }
        }
    }
    if (!reset_done) {
        imu->init_result = IMU660RB_INIT_RESET_TIMEOUT;
        return false;
    }
    if (!who_read) {
        imu->init_result = IMU660RB_INIT_WHO_AFTER_RESET_READ_FAILED;
        return false;
    }
    if (who != IMU660RB_WHO_AM_I_VALUE) {
        imu->init_result = IMU660RB_INIT_WHO_AFTER_RESET_MISMATCH;
        return false;
    }

    /*
     * Configure interfaces and filters while both sensors are powered down,
     * then enable 416 Hz, +/-4 g and +/-500 dps last.
     */
    if (!Bsp_Imu660rb_Write_Register(imu, IMU660RB_CTRL3_C,
                                     IMU660RB_CTRL3_C_VALUE) ||
        !Bsp_Imu660rb_Write_Register(imu, IMU660RB_CTRL4_C,
                                     IMU660RB_CTRL4_C_VALUE) ||
        !Bsp_Imu660rb_Write_Register(imu, IMU660RB_CTRL5_C,
                                     IMU660RB_CTRL5_C_VALUE) ||
        !Bsp_Imu660rb_Write_Register(imu, IMU660RB_CTRL6_C,
                                     IMU660RB_CTRL6_C_VALUE) ||
        !Bsp_Imu660rb_Write_Register(imu, IMU660RB_CTRL7_G,
                                     IMU660RB_CTRL7_G_VALUE) ||
        !Bsp_Imu660rb_Write_Register(imu, IMU660RB_CTRL9_XL,
                                     IMU660RB_CTRL9_XL_VALUE) ||
        !Bsp_Imu660rb_Write_Register(imu, IMU660RB_CTRL1_XL,
                                     IMU660RB_CTRL1_XL_VALUE) ||
        !Bsp_Imu660rb_Write_Register(imu, IMU660RB_CTRL2_G,
                                     IMU660RB_CTRL2_G_VALUE)) {
        imu->init_result = IMU660RB_INIT_CONFIG_WRITE_FAILED;
        return false;
    }
    imu->Delay_ms(IMU660RB_TURN_ON_MS);

    if (!Bsp_Imu660rb_Read_Register(imu, IMU660RB_CTRL1_XL,
                                    &imu->init_ctrl1_xl) ||
        !Bsp_Imu660rb_Read_Register(imu, IMU660RB_CTRL2_G,
                                    &imu->init_ctrl2_g) ||
        !Bsp_Imu660rb_Read_Register(imu, IMU660RB_CTRL3_C,
                                    &imu->init_ctrl3_c) ||
        !Bsp_Imu660rb_Read_Register(imu, IMU660RB_CTRL4_C,
                                    &imu->init_ctrl4_c) ||
        !Bsp_Imu660rb_Read_Register(imu, IMU660RB_CTRL5_C, &ctrl5_c) ||
        !Bsp_Imu660rb_Read_Register(imu, IMU660RB_CTRL6_C, &ctrl6_c) ||
        !Bsp_Imu660rb_Read_Register(imu, IMU660RB_CTRL7_G, &ctrl7_g) ||
        !Bsp_Imu660rb_Read_Register(imu, IMU660RB_CTRL9_XL,
                                    &imu->init_ctrl9_xl)) {
        imu->init_result = IMU660RB_INIT_CONFIG_READ_FAILED;
        return false;
    }
    if (imu->init_ctrl1_xl != IMU660RB_CTRL1_XL_VALUE ||
        imu->init_ctrl2_g != IMU660RB_CTRL2_G_VALUE ||
        imu->init_ctrl3_c != IMU660RB_CTRL3_C_VALUE ||
        imu->init_ctrl4_c != IMU660RB_CTRL4_C_VALUE ||
        ctrl5_c != IMU660RB_CTRL5_C_VALUE ||
        ctrl6_c != IMU660RB_CTRL6_C_VALUE ||
        ctrl7_g != IMU660RB_CTRL7_G_VALUE ||
        imu->init_ctrl9_xl != IMU660RB_CTRL9_XL_VALUE) {
        imu->init_result = IMU660RB_INIT_CONFIG_MISMATCH;
        return false;
    }

    imu->initialized = true;
    imu->init_result = IMU660RB_INIT_OK;
    return true;
}

bool Bsp_Imu660rb_Read_Data(imu_sensor_t *imu)
{
    uint8_t raw[IMU660RB_FRAME_BYTES];
    int16_t raw_temp;
    int16_t raw_ax;
    int16_t raw_ay;
    int16_t raw_az;
    int16_t raw_gx;
    int16_t raw_gy;
    int16_t raw_gz;

    if (imu == NULL || !imu->initialized ||
        !Bsp_Imu660rb_Read_Registers(imu, IMU660RB_OUT_TEMP_L,
                                    raw, sizeof(raw))) {
        if (imu != NULL) {
            imu->data_valid = false;
        }
        return false;
    }

    /*
     * A disconnected or reset SPI slave commonly returns a flat 0x00/0xFF
     * frame. Reject it so stale heading can never remain motion-authoritative.
     */
    if (Bsp_Imu660rb_Frame_Is_Flat(raw, 0x00u) ||
        Bsp_Imu660rb_Frame_Is_Flat(raw, 0xFFu)) {
        imu->data_valid = false;
        return false;
    }

    raw_temp = Bsp_Imu660rb_Make_Int16(raw[0], raw[1]);
    raw_gx = Bsp_Imu660rb_Make_Int16(raw[2], raw[3]);
    raw_gy = Bsp_Imu660rb_Make_Int16(raw[4], raw[5]);
    raw_gz = Bsp_Imu660rb_Make_Int16(raw[6], raw[7]);
    raw_ax = Bsp_Imu660rb_Make_Int16(raw[8], raw[9]);
    raw_ay = Bsp_Imu660rb_Make_Int16(raw[10], raw[11]);
    raw_az = Bsp_Imu660rb_Make_Int16(raw[12], raw[13]);

    imu->temp = (float)raw_temp * IMU660RB_TEMP_C_PER_LSB +
                IMU660RB_TEMP_OFFSET_C;
    imu->accel_x = (float)raw_ax * IMU660RB_ACCEL_G_PER_LSB;
    imu->accel_y = (float)raw_ay * IMU660RB_ACCEL_G_PER_LSB;
    imu->accel_z = (float)raw_az * IMU660RB_ACCEL_G_PER_LSB;
    imu->gyro_x = (float)raw_gx * IMU660RB_GYRO_DPS_PER_LSB;
    imu->gyro_y = (float)raw_gy * IMU660RB_GYRO_DPS_PER_LSB;
    imu->gyro_z = (float)raw_gz * IMU660RB_GYRO_DPS_PER_LSB *
                  IMU660RB_GYRO_Z_SIGN;
    imu->data_valid = isfinite(imu->temp) &&
                      isfinite(imu->accel_x) && isfinite(imu->accel_y) &&
                      isfinite(imu->accel_z) && isfinite(imu->gyro_x) &&
                      isfinite(imu->gyro_y) && isfinite(imu->gyro_z);
    return imu->data_valid;
}

void Bsp_Imu660rb_Calibration_Start(imu_sensor_t *imu)
{
    if (imu == NULL) {
        return;
    }
    imu->calibration_sample_count = 0u;
    imu->calibration_read_failures = 0u;
    imu->calibration_mean = 0.0f;
    imu->calibration_m2 = 0.0f;
    imu->gyro_z_offset = 0.0f;
    imu->calibration_state = imu->initialized ? IMU_CAL_RUNNING : IMU_CAL_FAILED;
}

imu_cal_state_t Bsp_Imu660rb_Calibration_Update(imu_sensor_t *imu)
{
    float delta;
    float delta2;
    float std_dev;

    if (imu == NULL) {
        return IMU_CAL_FAILED;
    }
    if (imu->calibration_state != IMU_CAL_RUNNING) {
        return imu->calibration_state;
    }
    if (!Bsp_Imu660rb_Read_Data(imu)) {
        if (imu->calibration_read_failures < UINT8_MAX) {
            imu->calibration_read_failures++;
        }
        if (imu->calibration_read_failures >=
            IMU660RB_CALIB_MAX_READ_FAILURES) {
            imu->calibration_state = IMU_CAL_FAILED;
        }
        return imu->calibration_state;
    }

    imu->calibration_read_failures = 0u;
    imu->calibration_sample_count++;
    delta = imu->gyro_z - imu->calibration_mean;
    imu->calibration_mean += delta / (float)imu->calibration_sample_count;
    delta2 = imu->gyro_z - imu->calibration_mean;
    imu->calibration_m2 += delta * delta2;

    if (imu->calibration_sample_count >= IMU660RB_CALIB_SAMPLES) {
        std_dev = sqrtf(imu->calibration_m2 /
                        (float)imu->calibration_sample_count);
        if (!isfinite(std_dev) || std_dev > IMU660RB_CALIB_MAX_STD_DPS) {
            imu->calibration_state = IMU_CAL_FAILED;
            imu->data_valid = false;
        } else {
            imu->gyro_z_offset = imu->calibration_mean;
            imu->calibration_state = IMU_CAL_DONE;
        }
    }
    return imu->calibration_state;
}

bool Bsp_Imu660rb_Update_Heading(imu_sensor_t *imu, float dt)
{
    float gyro_for_heading;

    if (imu == NULL || !imu->initialized ||
        imu->calibration_state != IMU_CAL_DONE || !isfinite(dt) || dt <= 0.0f ||
        !Bsp_Imu660rb_Read_Data(imu)) {
        if (imu != NULL) {
            imu->data_valid = false;
        }
        return false;
    }

    imu->gyro_z -= imu->gyro_z_offset;
    gyro_for_heading = imu->gyro_z;
    if (gyro_for_heading > -IMU660RB_GYRO_DEADZONE_DPS &&
        gyro_for_heading < IMU660RB_GYRO_DEADZONE_DPS) {
        gyro_for_heading = 0.0f;
    }
    imu->heading += gyro_for_heading * dt;
    if (!isfinite(imu->heading)) {
        imu->data_valid = false;
        return false;
    }
    return true;
}
