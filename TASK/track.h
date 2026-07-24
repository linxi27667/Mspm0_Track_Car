/**
 * @file    track.h
 * @brief   12路鱼眼数字循迹任务调度层（TASK层 — 纯业务逻辑）
 * @note    传感器布局: A1 ... A12（车辆左→右）
 *          偏差权重线性归一化为 -3.5 ... +3.5
 *          负偏差=黑线在左侧(需左转)  正偏差=黑线在右侧(需右转)
 *          turn_output>0=右转(左轮加速) <0=左转(右轮加速)
 *
 *          架构:
 *          BSP层: bsp_path_fish / bsp_pid / bsp_imu660rb / bsp_encoder_motor
 *          APP层: app_path_fish / app_imu / app_encoder_motor
 *          TASK层: 本文件 — 只写业务逻辑，不碰任何硬件寄存器
 */
#ifndef __TRACK_H__
#define __TRACK_H__

#include <stdint.h>
#include <stdbool.h>
#include "app_pid.h"
#include "app_path_fish.h"
#include "app_imu.h"
#include "app_encoder_motor.h"
#include "user_tuning.h"

/* ============================================================
 * 循迹调度宏定义
 *
 * 所有 USER_* 默认值唯一定义在 APP/Inc/user_tuning.h。
 * 本文件只保留 TRACK_* 别名，不再保留 #ifndef 回退默认值，
 * 消除三层重复定义导致的死代码与值不一致风险。
 * ============================================================ */

/* 循迹状态机更新周期(ms) */
#define TRACK_UPDATE_PERIOD_MS    5

/* --- 直角弯道检测 --- */
#define TRACK_RIGHT_ANGLE_THRESHOLD       USER_RIGHT_ANGLE_THRESHOLD
#define TRACK_RIGHT_ANGLE_CONFIRM_FRAMES  USER_RIGHT_ANGLE_CONFIRM_FRAMES
#define TRACK_RIGHT_ANGLE_ARM_DELAY       USER_RIGHT_ANGLE_ARM_DELAY
#define TRACK_RIGHT_ANGLE_TIMEOUT         USER_RIGHT_ANGLE_TIMEOUT
#define TRACK_RIGHT_ANGLE_TARGET_ANGLE    USER_RIGHT_ANGLE_TARGET_ANGLE
#define TRACK_RIGHT_ANGLE_APPROACH_SPEED    USER_RIGHT_ANGLE_APPROACH_SPEED
#define TRACK_RIGHT_ANGLE_APPROACH_FALLBACK_MS USER_RIGHT_ANGLE_APPROACH_FALLBACK_MS
#define TRACK_RIGHT_ANGLE_STOP_MS           USER_RIGHT_ANGLE_STOP_MS
#define TRACK_RIGHT_ANGLE_RETRIGGER_DELAY   USER_RIGHT_ANGLE_RETRIGGER_DELAY
#define TRACK_RIGHT_ANGLE_WHITE_FALLBACK_FRAMES USER_RIGHT_ANGLE_WHITE_FALLBACK_FRAMES
#define TRACK_RIGHT_ANGLE_WHITE_RECENT_MS USER_RIGHT_ANGLE_WHITE_RECENT_MS
#define TRACK_RIGHT_ANGLE_TURN_LIMIT        USER_RIGHT_ANGLE_TURN_LIMIT
#define TRACK_RIGHT_ANGLE_DONE_ERROR        USER_RIGHT_ANGLE_DONE_ERROR
#define TRACK_RIGHT_ANGLE_DONE_RATE         USER_RIGHT_ANGLE_DONE_RATE
#define TRACK_RIGHT_ANGLE_DONE_FRAMES       USER_RIGHT_ANGLE_DONE_FRAMES
#define TRACK_RIGHT_ANGLE_HYBRID_MIN_TURN_DEG USER_RIGHT_ANGLE_HYBRID_MIN_TURN_DEG
#define TRACK_RIGHT_ANGLE_HYBRID_HANDOFF_DEG USER_RIGHT_ANGLE_HYBRID_HANDOFF_DEG
#define TRACK_RIGHT_ANGLE_HYBRID_MAX_RATE USER_RIGHT_ANGLE_HYBRID_MAX_RATE
#define TRACK_RIGHT_ANGLE_HYBRID_MAX_ERROR USER_RIGHT_ANGLE_HYBRID_MAX_ERROR
#define TRACK_RIGHT_ANGLE_HYBRID_LINE_FRAMES USER_RIGHT_ANGLE_HYBRID_LINE_FRAMES
#define TRACK_RIGHT_ANGLE_CAPTURE_SPEED     USER_RIGHT_ANGLE_CAPTURE_SPEED
#define TRACK_RIGHT_ANGLE_CAPTURE_TURN_LIMIT USER_RIGHT_ANGLE_CAPTURE_TURN_LIMIT
#define TRACK_RIGHT_ANGLE_CAPTURE_LINE_LIMIT USER_RIGHT_ANGLE_CAPTURE_LINE_LIMIT
#define TRACK_RIGHT_ANGLE_CAPTURE_HEADING_KP USER_RIGHT_ANGLE_CAPTURE_HEADING_KP
#define TRACK_RIGHT_ANGLE_CAPTURE_HEADING_KD USER_RIGHT_ANGLE_CAPTURE_HEADING_KD
#define TRACK_RIGHT_ANGLE_CAPTURE_HEADING_LIMIT USER_RIGHT_ANGLE_CAPTURE_HEADING_LIMIT
#define TRACK_RIGHT_ANGLE_CAPTURE_CONFIRM_FRAMES USER_RIGHT_ANGLE_CAPTURE_CONFIRM_FRAMES
#define TRACK_RIGHT_ANGLE_CAPTURE_MAX_ERROR USER_RIGHT_ANGLE_CAPTURE_MAX_ERROR
#define TRACK_RIGHT_ANGLE_CAPTURE_REPIVOT_ERROR USER_RIGHT_ANGLE_CAPTURE_REPIVOT_ERROR

/* --- LINE_FOLLOW 陀螺仪角速度阻尼 (国一方案核心) ---
 * 直行时用陀螺仪角速度作阻尼项, 抑制灰度传感器噪声引起的左右抖动
 * 本车极性: turn_output = line_pid_output + GYRO_DAMP * gyro_z
 * 调参: 直道抖动→加大(0.15→0.3), 弯道跟踪迟钝→减小(0.15→0.08) */
#define TRACK_LINE_FOLLOW_GYRO_DAMP       USER_LINE_FOLLOW_GYRO_DAMP

/* --- 变速策略 --- */
#define TRACK_SPEED_STRAIGHT              USER_TRACK_SPEED_STRAIGHT
#define TRACK_SPEED_CURVE                 USER_TRACK_SPEED_CURVE
#define TRACK_SPEED_RAMP_STEP             USER_TRACK_SPEED_RAMP_STEP
/* 国一方案: 连续曲率变速 (替代二档跳变)
 * error=0 → SPEED_STRAIGHT, |error|>=CURVE_ERROR_MAX → SPEED_CURVE, 中间线性插值
 * 效果: 进弯平滑减速不甩出, 出弯平滑加速不慢 */
#define TRACK_CURVE_ERROR_MAX             USER_TRACK_CURVE_ERROR_MAX
/* --- 电机差速 --- */
#define TRACK_MOTOR_INNER_MIN             USER_TRACK_MOTOR_INNER_MIN
#define TRACK_MOTOR_SPEED_MAX             100
#define TRACK_MOTOR_SPEED_MIN             (-100)

/* --- 偏差计算 --- */
/* --- 默认参数 --- */
#define TRACK_DEFAULT_BASE_SPEED          USER_TRACK_BASE_SPEED
#define TRACK_DEFAULT_CIRCLES             USER_TRACK_CIRCLES

/* 循迹PID默认参数（需实测调优） */
#define TRACK_LINE_PID_KP        USER_LINE_PID_KP
#define TRACK_LINE_PID_KI        USER_LINE_PID_KI
#define TRACK_LINE_PID_KD        USER_LINE_PID_KD
#define TRACK_LINE_PID_ILIMIT    USER_LINE_PID_ILIMIT
#define TRACK_LINE_PID_OLIMIT    USER_LINE_PID_OLIMIT
#define TRACK_LINE_PID_DERIV_ALPHA USER_LINE_PID_DERIV_ALPHA

#define TRACK_ROTATE_DONE_ERR_DEG USER_ROTATE_DONE_ERR_DEG

/* --- 架空电机台架测试（固定流程，避免远程任意PWM） --- */
#define TRACK_MOTOR_TEST_STAGE_MS     1200u
#define TRACK_MOTOR_TEST_SETTLE_MS      400u
#define TRACK_MOTOR_TEST_TIMEOUT_MS   12600u
#define TRACK_MOTOR_TEST_PHASE_COUNT     10u
#define TRACK_MOTOR_TEST_OPEN_PHASES      3u
#define TRACK_MOTOR_TEST_CLOSED_PHASES    6u
#define TRACK_MOTOR_TEST_STOP_PHASE       9u
#define TRACK_MOTOR_TEST_TOLERANCE       0.15f
#define TRACK_MOTOR_TEST_WHEEL_MISMATCH  0.10f
#define TRACK_MOTOR_TEST_STOP_MAX_AVG    0.25f
#define TRACK_MOTOR_TEST_TARGET_LOW     20
#define TRACK_MOTOR_TEST_TARGET_MID     30
#define TRACK_MOTOR_TEST_TARGET_HIGH    40

#include "track_logic.h"

/* ============================================================
 * 循迹调度对象 — 只包含状态机和输出（纯业务数据）
 * ============================================================ */
typedef struct {
    /* --- 偏差计算 --- */
    float    error;                 /* 加权偏差 */
    float    last_error;            /* 上次偏差 */

    /* --- 状态机 --- */
    track_state_t state;
    float    right_angle_target;    /* 直角弯道目标角度 */
    float    right_angle_entry_heading; /* 进入直角弯时的航向 */
    int8_t   corner_turn_sign;      /* +1=左转(+yaw), -1=右转(-yaw) */
    uint32_t state_timer;           /* 当前状态进入时刻(ms) */
    uint32_t tracking_started_at;   /* 本次循迹启动时刻 */
    uint32_t last_corner_completed_at; /* 最近一次有效出弯时刻 */
    uint32_t corner_started_at;     /* 本次直角状态机总起点 */
    track_corner_phase_t corner_phase;
    track_line_type_t line_type;
    uint16_t line_mask;
    uint64_t line_mask_history;     /* 最近4帧12位mask */
    uint32_t line_lost_started_at;  /* 当前连续丢线/无效线型起点 */
    uint32_t last_centered_at;      /* 最近一次正常居中线型时刻 */
    uint32_t last_corner_direction_at; /* 最近明确单侧角点方向证据时刻 */
    int8_t   last_corner_direction; /* +1=左侧远端支路, -1=右侧远端支路 */

    /* --- 陀螺仪辅助 (国一方案) --- */
    float    gyro_rate;             /* 当前Z轴角速度(deg/s), LINE_FOLLOW阻尼用 */

    /* --- 远程原地旋转（相对航向闭环） --- */
    float    rotate_heading_baseline;
    float    rotate_target_deg;
    float    rotate_actual_deg;
    float    rotate_error_deg;
    uint32_t rotate_started_at;
    uint32_t rotate_timeout_ms;
    uint8_t  rotate_settle_frames;
    uint8_t  rotate_stall_frames;
    uint8_t  rotate_stall_kick_frames;
    bool     rotate_stall_kick_active;
    const char *rotate_result;

    /* --- Current-heading hold (absolute heading PID) --- */
    float    heading_hold_target;
    float    heading_hold_actual;
    float    heading_hold_error;
    const char *heading_hold_result;

    /* --- 架空电机台架测试 --- */
    uint32_t motor_test_started_at;
    uint32_t motor_test_stage_started_at;
    int32_t  motor_test_left_start;
    int32_t  motor_test_right_start;
    uint8_t  motor_test_phase;
    int32_t  motor_test_target;
    const char *motor_test_result;
    int32_t  motor_test_left_sum;
    int32_t  motor_test_right_sum;
    uint32_t motor_test_left_abs_sum;
    uint32_t motor_test_right_abs_sum;
    uint16_t motor_test_sample_count;
    float    motor_test_prev_left_avg;
    float    motor_test_prev_right_avg;
    float    motor_test_scale_left_num;
    float    motor_test_scale_right_num;
    float    motor_test_scale_den;
    bool     motor_test_failed;

    /* --- 直角弯道多帧确认 --- */
    uint8_t  right_angle_confirm;   /* 直角弯连续确认计数 */
    uint8_t  corner_heading_settle_confirm; /* 90度航向稳定确认计数 */
    uint8_t  line_lost_confirm;     /* 丢线连续确认计数 */
    uint8_t  invalid_line_confirm;  /* 无效线型连续确认计数 */
    uint8_t  centered_history;      /* 入角前正常居中连续帧 */
    bool     centered_evidence_valid; /* 已锁存连续2帧居中证据 */
    uint8_t  capture_confirm;       /* 转后正常线连续捕获帧 */

    /* --- 编码器距离前探 --- */
    int32_t  corner_left_start;
    int32_t  corner_right_start;

    /* --- 圈数 --- */
    uint8_t  target_circles;
    uint8_t  current_circle;
    uint16_t right_angle_count;  /* 已完成直角弯总数(正方形: 4个=1圈) */

    /* --- PID（使用BSP层 bsp_pid_t） --- */
    bsp_pid_t line_pid;             /* 循迹位置PID */
    /* --- 输出 --- */
    float    turn_output;           /* 转向量(正=右转,负=左转) */
    int32_t  left_speed;            /* 左电机速度 */
    int32_t  right_speed;           /* 右电机速度 */
    int32_t  base_speed;            /* 当前基础速度（渐变） */
    int32_t  target_speed;          /* 目标基础速度（由变速策略决定） */

    /* --- AI 调参运行时可调参数（原 #define 宏常量，改为字段以支持在线调整） --- */
    float    gyro_damp;             /* 循迹陀螺仪阻尼（原 USER_LINE_FOLLOW_GYRO_DAMP）*/
    int32_t  speed_straight;        /* 直道速度（原 USER_TRACK_SPEED_STRAIGHT）*/
    int32_t  speed_curve;           /* 弯道速度（原 USER_TRACK_SPEED_CURVE）*/
    float    right_angle_threshold; /* 直角弯检测误差阈值（原 USER_RIGHT_ANGLE_THRESHOLD）*/

    /* --- 配置 --- */
    bool     use_gyro;              /* 是否启用陀螺仪融合 */
    bool     use_motor;             /* 是否输出到电机 */
    bool     use_speed_pid;         /* 普通循迹是否启用左右轮速PI */
} track_t;

/* ============================================================
 * 全局对象
 * ============================================================ */
extern track_t Track;

/* ============================================================
 * 对外接口（主循环调度）
 * ============================================================ */
void Track_Init(void);                  /* 初始化所有硬件+状态 */
void Track_Update(void);                /* 主循环调用，非阻塞，内部5ms周期 */
bool Track_Start(void);                 /* 启动循迹，Safety 拒绝时返回 false */
const char *Track_Get_Start_Result(void); /* 最近一次启动结果，供ACK返回 */
void Track_Stop(void);                  /* 停止循迹 */
bool Track_Start_Rotation(float target_deg); /* READY状态开始相对角度原地旋转 */
bool Track_Is_Rotating(void);
void Track_Abort_Rotation(const char *result); /* Safety 故障时结束旋转并发布结果 */
float Track_Get_Rotation_Target(void);
float Track_Get_Rotation_Angle(void);
float Track_Get_Rotation_Error(void);
const char *Track_Get_Rotation_Result(void);
bool Track_Start_Heading_Hold(void);
bool Track_Stop_Heading_Hold(void);
bool Track_Is_Heading_Hold(void);
float Track_Get_Heading_Hold_Target(void);
float Track_Get_Heading_Hold_Actual(void);
float Track_Get_Heading_Hold_Error(void);
const char *Track_Get_Heading_Hold_Result(void);
bool Track_Start_Motor_Test(void);
bool Track_Is_Motor_Test(void);
void Track_Abort_Motor_Test(const char *result);
uint8_t Track_Get_Motor_Test_Phase(void);
int32_t Track_Get_Motor_Test_Target(void);
const char *Track_Get_Motor_Test_Result(void);
void Track_Check_RightAngle_Completion(void); /* 仅更新圈数遥测，不自动停车 */
void Track_Set_Circles(uint8_t n);      /* 设置目标圈数 */
void Track_Set_Speed(int32_t base_speed); /* 设置基础速度 */
void Track_Set_TrackPID(float kp, float ki, float kd); /* 调循迹PID */

/* AI 远程调参 setter（运行时修改原宏常量参数） */
void Track_Set_GyroDamp(float damp);                /* 陀螺仪阻尼 */
void Track_Set_SpeedStraight(int32_t spd);          /* 直道速度 */
void Track_Set_SpeedCurve(int32_t spd);             /* 弯道速度 */
void Track_Set_RightAngleThreshold(float thr);      /* 直角弯检测误差阈值 */
track_state_t Track_Get_State(void);    /* 获取当前状态 */
uint8_t Track_Get_Circle(void);         /* 获取当前圈数 */

#endif /* __TRACK_H__ */
