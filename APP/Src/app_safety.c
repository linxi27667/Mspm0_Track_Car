/**
 * @file    app_safety.c
 * @brief   安全状态机实现 — 唯一授权电机输出的模块。
 *
 *          设计原则:
 *            1. Track/PID 可计算期望速度，但电机输出仅当
 *               Safety_AllowsMotorOutput() 返回 true 时才执行。
 *            2. BOOT/CALIBRATING/READY/FINISHED 状态始终切断 PWM。
 *            3. 运行中仅IMU与灰度传感器异常失效安全停车。
 *            4. 通信与心跳仅用于诊断，不参与运动授权。
 *
 * @note    本模块不直接操作硬件寄存器，通过 BSP/APP API 切断电机:
 *            App_Motor_Enable_Speed_PID(false) — 切开环
 *            Bsp_Motor_Write_Pwm(&Motor_*, 0)  — 直写 PWM=0
 */
#include "app_safety.h"
#include "app_all.h"
#include "safety_logic.h"
#include <elog.h>

/* ============================================================
 * 内部状态
 * ============================================================ */

static safety_state_t s_state = SAFETY_STATE_BOOT;
static safety_fault_t s_fault = SAFETY_FAULT_NONE;
static uint32_t s_finished_at = 0u;

/* 校准完成标志 */
static bool     s_calibration_complete = false;
static safety_health_t s_health;

/* ============================================================
 * 内部工具
 * ============================================================ */

static void Safety_CutMotor(void)
{
    /* 切开环 + 直写 PWM=0，确保即使速度 PID 漏更新也能停车 */
    App_Motor_Enable_Speed_PID(false);
    Bsp_Motor_Write_Pwm(&Motor_Left, 0);
    Bsp_Motor_Write_Pwm(&Motor_Right, 0);
}

static bool Safety_GrayscaleCalibrationValid(void)
{
    return Grayscale_Calibration_Valid(USER_GRAYSCALE_MIN_CAL_CONTRAST);
}

static uint16_t Safety_GrayscaleContrast(uint8_t channel)
{
    uint16_t white = Grayscale_Sensor.white[channel];
    uint16_t black = Grayscale_Sensor.black[channel];
    return (white >= black) ? (uint16_t)(white - black)
                            : (uint16_t)(black - white);
}

static void Safety_LogCalibrationContrast(void)
{
    elog_w("SAFETY", "calibration rejected min=%u D=%u,%u,%u,%u,%u,%u,%u,%u",
           USER_GRAYSCALE_MIN_CAL_CONTRAST,
           Safety_GrayscaleContrast(0), Safety_GrayscaleContrast(1),
           Safety_GrayscaleContrast(2), Safety_GrayscaleContrast(3),
           Safety_GrayscaleContrast(4), Safety_GrayscaleContrast(5),
           Safety_GrayscaleContrast(6), Safety_GrayscaleContrast(7));
}

static const char *s_state_names[] = {
    "BOOT", "CALIBRATING", "READY", "RUNNING", "FINISHED", "FAULT"
};

static const char *s_fault_names[] = {
    "NONE",
    "GRAYSCALE_EXPIRED",
    "CALIBRATION_INCOMPLETE",
    "SENSOR_ABNORMAL",
    "MANUAL_STOP",
    "LINE_LOST_TIMEOUT"
};

static bool Safety_FaultIsUserVisible(safety_fault_t fault)
{
    return fault == SAFETY_FAULT_GRAYSCALE_EXPIRED ||
           fault == SAFETY_FAULT_CALIBRATION_INCOMPLETE ||
           fault == SAFETY_FAULT_SENSOR_ABNORMAL ||
           fault == SAFETY_FAULT_LINE_LOST_TIMEOUT;
}

/* ============================================================
 * 初始化与状态转换
 * ============================================================ */

void Safety_Init(void)
{
    s_state                = SAFETY_STATE_BOOT;
    s_fault                = SAFETY_FAULT_NONE;
    s_finished_at          = 0u;
    s_calibration_complete = false;
    s_health.grayscale_required = false;
    s_health.grayscale_seen = false;
    s_health.sensor_abnormal = false;
    s_health.last_grayscale_tick = 0u;

    Safety_CutMotor();
    elog_i("SAFETY", "init: state=BOOT, motor cut");
}

void Safety_TransitionToCalibrating(void)
{
    if (s_state == SAFETY_STATE_FAULT) return;  /* 故障态锁定 */
    if (s_state == SAFETY_STATE_CALIBRATING) return;

    s_state = SAFETY_STATE_CALIBRATING;
    Safety_CutMotor();
    elog_i("SAFETY", "-> CALIBRATING");
}

void Safety_TransitionToReady(void)
{
    if (s_state == SAFETY_STATE_FAULT) return;
    if (s_state == SAFETY_STATE_READY) return;

    /* 进入 READY 前必须校准完成 */
    if (!s_calibration_complete) {
        elog_w("SAFETY", "READY blocked: calibration incomplete");
        return;
    }

    s_state = SAFETY_STATE_READY;
    Safety_CutMotor();
    elog_i("SAFETY", "-> READY");
}

void Safety_TransitionToRunning(void)
{
    if (s_state == SAFETY_STATE_FAULT) return;
    if (s_state == SAFETY_STATE_RUNNING) return;

    /* 必须从 READY 进入，校准已完成 */
    if (s_state != SAFETY_STATE_READY) {
        elog_w("SAFETY", "RUNNING blocked: state=%s",
               Safety_StateName(s_state));
        return;
    }
    if (!s_calibration_complete) {
        elog_w("SAFETY", "RUNNING blocked: calibration incomplete");
        return;
    }

    s_state = SAFETY_STATE_RUNNING;
    s_health.grayscale_required = true;

    elog_i("SAFETY", "-> RUNNING (motor armed)");
}

bool Safety_TransitionToRotation(void)
{
    if (!Safety_Logic_CanEnterRotation((uint8_t)s_state)) {
        return false;
    }

    s_state = SAFETY_STATE_RUNNING;
    s_health.grayscale_required = false;
    elog_i("SAFETY", "-> RUNNING (rotation)");
    return true;
}

void Safety_TransitionAfterRotation(void)
{
    if (s_state == SAFETY_STATE_FAULT) return;

    Safety_CutMotor();
    if (s_calibration_complete) {
        s_state = SAFETY_STATE_READY;
        elog_i("SAFETY", "rotation ended -> READY");
    } else {
        s_state = SAFETY_STATE_BOOT;
        elog_i("SAFETY", "rotation ended -> BOOT (gray calibration pending)");
    }
    s_health.grayscale_required = false;
}

bool Safety_TransitionToBenchTest(void)
{
    if ((s_state != SAFETY_STATE_BOOT && s_state != SAFETY_STATE_READY) ||
        s_fault != SAFETY_FAULT_NONE) {
        return false;
    }

    s_state = SAFETY_STATE_RUNNING;
    s_health.grayscale_required = false;
    elog_i("SAFETY", "-> RUNNING (motor bench test)");
    return true;
}

void Safety_TransitionAfterBenchTest(void)
{
    if (s_state == SAFETY_STATE_FAULT) return;

    Safety_CutMotor();
    s_state = s_calibration_complete ? SAFETY_STATE_READY : SAFETY_STATE_BOOT;
    s_health.grayscale_required = false;
    elog_i("SAFETY", "motor bench test ended -> %s",
           Safety_StateName(s_state));
}

void Safety_TransitionToFinished(void)
{
    if (s_state == SAFETY_STATE_FAULT) return;
    s_state = SAFETY_STATE_FINISHED;
    s_finished_at = Bsp_GetTick();
    s_health.grayscale_required = false;
    Safety_CutMotor();
    elog_i("SAFETY", "-> FINISHED");
}

void Safety_EnterFault(safety_fault_t fault)
{
    if (s_state == SAFETY_STATE_FAULT && s_fault == fault) return;

    s_state = SAFETY_STATE_FAULT;
    s_fault = fault;
    Safety_CutMotor();
    if (Safety_FaultIsUserVisible(fault)) {
        elog_e("SAFETY", "-> FAULT: %s", Safety_FaultName(fault));
    } else {
        elog_w("MOTION", "motion stopped: %s (auto recovery)",
               Safety_FaultName(fault));
    }
}

void Safety_ClearFault(void)
{
    safety_fault_t recovered_fault;

    if (s_state != SAFETY_STATE_FAULT) return;

    recovered_fault = s_fault;
    s_fault = SAFETY_FAULT_NONE;
    s_state = s_calibration_complete ? SAFETY_STATE_READY : SAFETY_STATE_BOOT;
    s_health.grayscale_required = false;
    s_health.sensor_abnormal = false;
    if (Safety_FaultIsUserVisible(recovered_fault)) {
        elog_i("SAFETY", "auto recovered -> %s", Safety_StateName(s_state));
    } else {
        elog_i("MOTION", "auto recovered -> %s", Safety_StateName(s_state));
    }
}

/* ============================================================
 * 控制 tick — 每 5ms 由主循环调用，按状态授权电机输出。
 * ============================================================ */
bool Safety_ControlTick(void)
{
    /* --- BOOT 态: 电机禁止 --- */
    if (s_state == SAFETY_STATE_BOOT ||
        s_state == SAFETY_STATE_CALIBRATING) {
        Safety_CutMotor();
        return false;
    }

    /* --- FAULT: 锁定切断，必须由明确的故障恢复流程解除 --- */
    if (s_state == SAFETY_STATE_FAULT) {
        Safety_CutMotor();
        return false;
    }

    /* --- FINISHED: 停车保持1秒后自动回到READY，无需复位 --- */
    if (s_state == SAFETY_STATE_FINISHED) {
        Safety_CutMotor();
        if (Safety_Logic_FinishedHoldElapsed(Bsp_GetTick(), s_finished_at,
                                             APP_SAFETY_FINISHED_HOLD_MS)) {
            if (s_calibration_complete) {
                Safety_TransitionToReady();
            } else {
                s_state = SAFETY_STATE_BOOT;
                elog_i("SAFETY", "finished hold elapsed -> BOOT (calibration pending)");
            }
        }
        return false;
    }

    /* READY 状态: 通过所有检查但仍禁止电机 */
    if (s_state == SAFETY_STATE_READY) {
        return false;
    }

    if (s_state == SAFETY_STATE_RUNNING) {
        uint32_t now = Bsp_GetTick();
        safety_logic_result_t result = Safety_Logic_Check(
            now, &s_health, APP_GRAYSCALE_FRAME_TIMEOUT_MS);
        if (result != SAFETY_LOGIC_OK) {
            static const safety_fault_t fault_map[] = {
                SAFETY_FAULT_NONE,
                SAFETY_FAULT_GRAYSCALE_EXPIRED,
                SAFETY_FAULT_SENSOR_ABNORMAL,
            };
            Safety_EnterFault(fault_map[result]);
            return false;
        }
        return true;
    }
    return false;
}

/* ============================================================
 * 查询接口
 * ============================================================ */

safety_state_t Safety_GetState(void) { return s_state; }
safety_fault_t Safety_GetFault(void) { return s_fault; }

bool Safety_AllowsMotorOutput(void)
{
    /* 同步查询接口：仅 RUNNING 且无故障时允许
     * 注意: 这不替代 Safety_ControlTick 的活跃监测，
     * 电机输出路径仍应先调用 Safety_ControlTick */
    return (s_state == SAFETY_STATE_RUNNING) &&
           (s_fault == SAFETY_FAULT_NONE);
}

const char *Safety_StateName(safety_state_t state)
{
    if ((uint32_t)state >= sizeof(s_state_names)/sizeof(s_state_names[0])) {
        return "?";
    }
    return s_state_names[state];
}

const char *Safety_FaultName(safety_fault_t fault)
{
    if ((uint32_t)fault >= sizeof(s_fault_names)/sizeof(s_fault_names[0])) {
        return "?";
    }
    return s_fault_names[fault];
}

/* ============================================================
 * 传感器健康更新
 * ============================================================ */

void Safety_NotifyGrayscaleFrame(uint32_t tick)
{
    s_health.last_grayscale_tick = tick;
    s_health.grayscale_seen = true;
}

void Safety_NotifySensorAbnormal(void)
{
    s_health.sensor_abnormal = true;
    if (s_state == SAFETY_STATE_RUNNING) {
        Safety_EnterFault(SAFETY_FAULT_SENSOR_ABNORMAL);
    }
}

void Safety_NotifyCalibrationComplete(void)
{
    s_calibration_complete = Safety_GrayscaleCalibrationValid();
    if (!s_calibration_complete) Safety_LogCalibrationContrast();
}

void Safety_TryAdvanceAfterCalibration(void)
{
    /* 检查灰度白+黑校准都完成 */
    if (Grayscale_Sensor.calibrated && Grayscale_Sensor.black_calibrated &&
        Safety_GrayscaleCalibrationValid()) {
        if (!s_calibration_complete) {
            s_calibration_complete = true;
            elog_i("SAFETY", "calibration complete (white+black)");
        }
        if (s_state == SAFETY_STATE_BOOT || s_state == SAFETY_STATE_CALIBRATING) {
            Safety_TransitionToReady();
        }
    } else if (Grayscale_Sensor.calibrated &&
               Grayscale_Sensor.black_calibrated) {
        s_calibration_complete = false;
        Safety_LogCalibrationContrast();
    }
}

/* ============================================================
 * 强制停车
 * ============================================================ */

void Safety_ForceMotorStop(void)
{
    Safety_CutMotor();
    /* 不强制进入 FAULT，仅切电机；由调用方决定是否升级状态 */
}
