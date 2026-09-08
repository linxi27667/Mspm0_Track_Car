/**
 * @file    app_safety.h
 * @brief   安全状态机 — 唯一授权电机输出的模块。
 *
 *          状态: BOOT → READY → RUNNING → FINISHED。
 *          运行期间仅由IMU与鱼眼循迹传感器故障撤销运动授权。
 *
 *          核心原则: Track/PID 可计算期望速度，但电机输出
 *          仅当 Safety_AllowsMotorOutput() 返回 true 时才执行。
 */
#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include "app_common.h"
#include <stdint.h>
#include <stdbool.h>

#define APP_SAFETY_FINISHED_HOLD_MS 1000u

/* 安全状态 */
typedef enum {
    SAFETY_STATE_BOOT = 0,          /* 初始化中,电机禁止 */
    SAFETY_STATE_CALIBRATING,       /* 校准中,电机禁止 */
    SAFETY_STATE_READY,             /* 就绪,等待启动,电机禁止 */
    SAFETY_STATE_RUNNING,           /* 循迹运行中,电机允许 */
    SAFETY_STATE_FINISHED,          /* 正常完成,电机禁止 */
    SAFETY_STATE_FAULT,             /* 故障停车,电机禁止 */
} safety_state_t;

/* 故障类型 */
typedef enum {
    SAFETY_FAULT_NONE = 0,
    SAFETY_FAULT_PATH_FISH_EXPIRED,      /* 鱼眼完整帧过期 */
    SAFETY_FAULT_PATH_FISH_NOT_READY,    /* 未获得连续居中的健康线型 */
    SAFETY_FAULT_SENSOR_ABNORMAL,        /* 传感器抬起/异常 */
    SAFETY_FAULT_MANUAL_STOP,            /* 手动停止 */
    SAFETY_FAULT_LINE_LOST_TIMEOUT,      /* 丢线超时 */
} safety_fault_t;

/* ============================================================
 * 初始化与状态转换
 * ============================================================ */
void Safety_Init(void);

void Safety_TransitionToCalibrating(void);
void Safety_TransitionToReady(void);
void Safety_TransitionToRunning(void);
bool Safety_TransitionToRotation(void);  /* IMU 原地旋转：允许从 BOOT/READY 进入 */
void Safety_TransitionAfterRotation(void); /* 旋转结束后切断电机并返回 READY */
bool Safety_TransitionToBenchTest(void); /* BOOT/READY可进入，灰度健康检查关闭 */
void Safety_TransitionAfterBenchTest(void);
void Safety_TransitionToFinished(void);
void Safety_EnterFault(safety_fault_t fault);
void Safety_ClearFault(void);  /* 人工确认后清除故障，电机保持关闭 */

/* ============================================================
 * 控制tick — 每5ms由主循环调用，按状态授权电机输出。
 * ============================================================ */
bool Safety_ControlTick(void);

/* ============================================================
 * 查询接口
 * ============================================================ */
safety_state_t Safety_GetState(void);
safety_fault_t Safety_GetFault(void);
bool Safety_AllowsMotorOutput(void);
const char *Safety_StateName(safety_state_t state);
const char *Safety_FaultName(safety_fault_t fault);

/* ============================================================
 * 传感器健康更新 (由各模块调用)
 * ============================================================ */

/* 发布有效鱼眼完整帧，并累计启动所需的连续健康帧。 */
void Safety_NotifyPathFishFrame(uint32_t tick, bool complete_frame);
void Safety_ResetPathFishReadiness(void);
bool Safety_PathFishReady(void);

/* 报告不可继续运动的传感器异常。 */
void Safety_NotifySensorAbnormal(void);

void Safety_NotifyLineLostTimeout(void);

/* ============================================================
 * 强制停车 — 切断所有电机输出
 * ============================================================ */
void Safety_ForceMotorStop(void);

#endif /* APP_SAFETY_H */
