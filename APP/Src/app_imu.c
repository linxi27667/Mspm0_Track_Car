/**
 * @file    app_imu.c
 * @brief   MSPM0 SPI1 binding for the SeekFree IMU660RB.
 */
#include "app_imu.h"

#include "app_common.h"
#include "imu_recovery_logic.h"
#include "app_log.h"
#include "bsp_common.h"
#include "ti_msp_dl_config.h"

#define IMU_SPI_TRANSFER_TIMEOUT_MS 5u
#define IMU_POWER_STABILIZE_MS      20u
#define IMU_INIT_ATTEMPTS           3u
#define IMU_INIT_RETRY_DELAY_MS     20u

static uint32_t s_transfer_started_at;
static uint32_t s_recovery_scheduled_at;
static uint32_t s_recovery_delay_ms;
static uint8_t s_recovery_failures;
static bool s_recovery_pending;

static bool HW_Spi_Timed_Out(void)
{
    return (uint32_t)(Bsp_GetTick() - s_transfer_started_at) >=
           IMU_SPI_TRANSFER_TIMEOUT_MS;
}

static void HW_Spi_Flush_Rx(void)
{
    while (!DL_SPI_isRXFIFOEmpty(IMU_SPI_INST)) {
        (void)DL_SPI_receiveData8(IMU_SPI_INST);
    }
}

static void HW_Imu_Cs_Write(bool high)
{
    if (high) {
        DL_GPIO_setPins(IMU_CS_PORT, IMU_CS_CS_PIN);
    } else {
        HW_Spi_Flush_Rx();
        DL_GPIO_clearPins(IMU_CS_PORT, IMU_CS_CS_PIN);
    }
}

static bool HW_Imu_Spi_Transfer(uint8_t tx, uint8_t *rx)
{
    if (rx == NULL) {
        return false;
    }
    s_transfer_started_at = Bsp_GetTick();
    while (DL_SPI_isTXFIFOFull(IMU_SPI_INST)) {
        if (HW_Spi_Timed_Out()) {
            APP_IMU_LOG_E("IMU", "[IMU] SPI timeout: wait=tx_fifo tx=0x%02X",
                          (unsigned)tx);
            return false;
        }
    }
    DL_SPI_transmitData8(IMU_SPI_INST, tx);
    while (DL_SPI_isRXFIFOEmpty(IMU_SPI_INST)) {
        if (HW_Spi_Timed_Out()) {
            APP_IMU_LOG_E("IMU", "[IMU] SPI timeout: wait=rx_fifo tx=0x%02X",
                          (unsigned)tx);
            return false;
        }
    }
    *rx = DL_SPI_receiveData8(IMU_SPI_INST);
    while (DL_SPI_isBusy(IMU_SPI_INST)) {
        if (HW_Spi_Timed_Out()) {
            APP_IMU_LOG_E("IMU", "[IMU] SPI timeout: wait=busy tx=0x%02X",
                          (unsigned)tx);
            return false;
        }
    }
    return true;
}

static void HW_Imu_Delay_ms(uint32_t ms)
{
    Bsp_Delay_ms(ms);
}

imu_sensor_t Imu_Sensor = {
    .Spi_Transfer = HW_Imu_Spi_Transfer,
    .Cs_Write = HW_Imu_Cs_Write,
    .Delay_ms = HW_Imu_Delay_ms,
    .initialized = false,
    .data_valid = false,
    .calibration_state = IMU_CAL_IDLE,
};

bool App_Imu_Init(void)
{
    bool ok = false;
    uint8_t attempt;

    HW_Imu_Cs_Write(true);
    HW_Spi_Flush_Rx();
    Bsp_Delay_ms(IMU_POWER_STABILIZE_MS);
    for (attempt = 1u; attempt <= IMU_INIT_ATTEMPTS; attempt++) {
        ok = Bsp_Imu660rb_Init_Device(&Imu_Sensor);
        if (ok) break;

        APP_IMU_LOG_W("IMU", "[IMU] init attempt %u/%u failed: stage=%s who=0x%02X",
                      (unsigned)attempt, (unsigned)IMU_INIT_ATTEMPTS,
                      Bsp_Imu660rb_Init_Result_Name(Imu_Sensor.init_result),
                      (unsigned)Imu_Sensor.init_who_am_i);
        HW_Imu_Cs_Write(true);
        HW_Spi_Flush_Rx();
        if (attempt < IMU_INIT_ATTEMPTS) {
            Bsp_Delay_ms(IMU_INIT_RETRY_DELAY_MS);
        }
    }
    if (ok) {
        APP_IMU_LOG_I("IMU", "[IMU] IMU660RB init OK: attempt=%u SPI1 mode0 8MHz, accel=+-4g gyro=+-500dps ODR=416Hz",
                      (unsigned)attempt);
    } else {
        APP_IMU_LOG_E("IMU", "[IMU] init failed: stage=%s who=0x%02X expected=0x%02X ctrl1=0x%02X ctrl2=0x%02X ctrl3=0x%02X ctrl4=0x%02X ctrl9=0x%02X",
                      Bsp_Imu660rb_Init_Result_Name(Imu_Sensor.init_result),
                      (unsigned)Imu_Sensor.init_who_am_i,
                      (unsigned)IMU660RB_WHO_AM_I_VALUE,
                      (unsigned)Imu_Sensor.init_ctrl1_xl,
                      (unsigned)Imu_Sensor.init_ctrl2_g,
                      (unsigned)Imu_Sensor.init_ctrl3_c,
                      (unsigned)Imu_Sensor.init_ctrl4_c,
                      (unsigned)Imu_Sensor.init_ctrl9_xl);
    }
    return ok;
}

void App_Imu_Request_Recovery(void)
{
    if (s_recovery_pending) return;

    s_recovery_scheduled_at = Bsp_GetTick();
    s_recovery_delay_ms = Imu_Recovery_BackoffMs(s_recovery_failures);
    s_recovery_pending = true;
    APP_IMU_LOG_W("IMU", "[IMU] recovery scheduled in %lu ms",
                  (unsigned long)s_recovery_delay_ms);
}

void App_Imu_Recovery_Update(uint32_t now, bool motion_active)
{
    bool healthy_for_idle = Imu_Sensor.initialized && Imu_Sensor.data_valid &&
        Imu_Sensor.calibration_state != IMU_CAL_FAILED;

    if (motion_active) return;
    /* 校准器自身会容忍有上限的上电瞬态无效帧；运行期间禁止恢复器抢先复位。 */
    if (Imu_Sensor.calibration_state == IMU_CAL_RUNNING) return;
    if (healthy_for_idle) {
        s_recovery_pending = false;
        return;
    }
    if (!s_recovery_pending) {
        App_Imu_Request_Recovery();
        return;
    }
    if (!Imu_Recovery_Due(now, s_recovery_scheduled_at,
                          s_recovery_delay_ms)) {
        return;
    }

    s_recovery_pending = false;
    APP_IMU_LOG_W("IMU", "[IMU] auto reconnect %u: software reset + reinit",
                  (unsigned)(s_recovery_failures + 1u));
    if (App_Imu_Init()) {
        Imu_Calibration_Start();
        APP_IMU_LOG_I("IMU", "[IMU] auto reconnect OK; calibration restarted");
        return;
    }

    if (s_recovery_failures < UINT8_MAX) s_recovery_failures++;
    App_Imu_Request_Recovery();
}

bool Imu_Read_Data(void)
{
    return Bsp_Imu660rb_Read_Data(&Imu_Sensor);
}

void Imu_Calibration_Start(void)
{
    Bsp_Imu660rb_Calibration_Start(&Imu_Sensor);
    if (Imu_Sensor.calibration_state != IMU_CAL_RUNNING) {
        APP_IMU_LOG_E("IMU", "[IMU] gyro calibration skipped: initialization unavailable");
        return;
    }
    APP_IMU_LOG_I("IMU", "[IMU] gyro calibration start: keep car still, samples=%u",
                  IMU660RB_CALIB_SAMPLES);
}

imu_cal_state_t Imu_Calibration_Update(void)
{
    imu_cal_state_t previous = Imu_Sensor.calibration_state;
    imu_cal_state_t current =
        Bsp_Imu660rb_Calibration_Update(&Imu_Sensor);

    if (previous != current && current == IMU_CAL_DONE) {
        s_recovery_failures = 0u;
        s_recovery_pending = false;
        APP_IMU_LOG_I("IMU", "[IMU] gyro calibration done: bias=%.3f noise=%.3f deadzone=%.3f dps rejected=%u",
                      (double)Imu_Sensor.gyro_z_offset,
                      (double)Imu_Sensor.gyro_z_noise,
                      (double)Imu_Sensor.gyro_z_deadzone,
                      (unsigned)Imu_Sensor.calibration_motion_rejections);
    } else if (previous != current && current == IMU_CAL_FAILED) {
        if (s_recovery_failures < UINT8_MAX) s_recovery_failures++;
        APP_IMU_LOG_E("IMU", "[IMU] gyro calibration failed: samples=%u read_failures=%u motion_rejections=%u",
                      (unsigned)Imu_Sensor.calibration_sample_count,
                      (unsigned)Imu_Sensor.calibration_read_failures,
                      (unsigned)Imu_Sensor.calibration_motion_rejections);
    }
    return current;
}

bool Imu_Update_Heading(float dt)
{
    return Bsp_Imu660rb_Update_Heading(&Imu_Sensor, dt);
}

bool Imu_Update_Stationary(float dt)
{
    return Bsp_Imu660rb_Update_Stationary(&Imu_Sensor, dt);
}
