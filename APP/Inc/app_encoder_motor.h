/**
 * @file    app_encoder_motor.h
 * @brief   霍尔编码器电机应用层声明（硬件绑定 + 业务逻辑入口）
 */
#ifndef APP_ENCODER_MOTOR_H
#define APP_ENCODER_MOTOR_H

#include "bsp_encoder_motor.h"
#include "ti_msp_dl_config.h"
#include "user_tuning.h"

/* ============ AT8236左电机双PWM配置 ============ */
#define MOTOR_LEFT_AIN1_PWM_TIMER   Motor_Left_Timer_INST
#define MOTOR_LEFT_AIN1_PWM_CHANNEL GPIO_Motor_Left_Timer_C1_IDX
#define MOTOR_LEFT_AIN2_PWM_TIMER   Motor_Left_Timer_INST
#define MOTOR_LEFT_AIN2_PWM_CHANNEL GPIO_Motor_Left_Timer_C2_IDX

/* ============ AT8236右电机双PWM配置 ============ */
#define MOTOR_RIGHT_BIN1_PWM_TIMER  Motor_Right_BIN1_Timer_INST
#define MOTOR_RIGHT_BIN1_PWM_CHANNEL GPIO_Motor_Right_BIN1_Timer_C0_IDX
#define MOTOR_RIGHT_BIN2_PWM_TIMER  Motor_Right_BIN2_Timer_INST
#define MOTOR_RIGHT_BIN2_PWM_CHANNEL GPIO_Motor_Right_BIN2_Timer_C1_IDX

/* SysConfig edge-aligned PWM uses the requested duty as its CC value. */
#define MOTOR_PWM_PERIOD             100u

/* ============ 编码器硬件配置（统一GPIO 4x正交解码） ============
 * MSPM0G3507只有TIMG8支持硬件QEI，无法提供两路硬件QEI。
 * 左右编码器均由PHA/PHB双边沿中断驱动同一正交状态机。
 */
#define LEFT_ENC_PHA_PORT           Left_Encoder_PORT
#define LEFT_ENC_PHA_PIN            Left_Encoder_LEFT_PHA_PIN
#define LEFT_ENC_PHB_PORT           Left_Encoder_PORT
#define LEFT_ENC_PHB_PIN            Left_Encoder_LEFT_PHB_PIN
#define RIGHT_ENC_PHA_PORT          Right_Encoder_PORT
#define RIGHT_ENC_PHA_PIN           Right_Encoder_PHA_PIN
#define RIGHT_ENC_PHB_PORT          Right_Encoder_PORT
#define RIGHT_ENC_PHB_PIN           Right_Encoder_PHB_PIN

/* 归一化为“车辆前进=正计数”；左右电机为镜像安装。 */
#define LEFT_ENC_FORWARD_SIGN       (1)
#define RIGHT_ENC_FORWARD_SIGN      (-1)

/* ============ 速度PID参数 ============
 * 级联控制: 位置PID(循迹) → 目标速度 → 速度PID → PWM
 *
 * 所有 USER_MOTOR_SPEED_PID_* 默认值唯一定义在 user_tuning.h。
 * 此处只保留别名，不再保留 #ifndef 回退默认值。
 *
 * speed_scale: 编码器计数/周期 对应 1% PWM 的系数
 *   计算式: actual_speed = encoder_speed / speed_scale
 *   校准方法: 设 PWM=50, 测 encoder_speed, scale = encoder_speed / 50
 *   注意: 未校准时 scale=1.0, encoder_speed 被直接当作速度单位,
 *         闭环性能不可靠。需实车标定后调用 App_Motor_Calibrate_Scale()。
 *   左右均为GPIO 4x正交解码，但电机、磁环和安装差异仍要求独立scale。
 *
 * integral_sep: 积分分离阈值, 误差超过此值时不累加积分
 *   作用: 启动/急停时避免积分饱和导致超调
 */
#define MOTOR_SPEED_PID_KP          USER_MOTOR_SPEED_PID_KP
#define MOTOR_SPEED_PID_KI          USER_MOTOR_SPEED_PID_KI
#define MOTOR_SPEED_PID_KD          USER_MOTOR_SPEED_PID_KD
#define MOTOR_SPEED_PID_ILIMIT      USER_MOTOR_SPEED_PID_ILIMIT
#define MOTOR_SPEED_PID_OLIMIT      USER_MOTOR_SPEED_PID_OLIMIT
#define MOTOR_SPEED_PID_INTEGRAL_SEP USER_MOTOR_SPEED_PID_INTEGRAL_SEP
#define MOTOR_SPEED_FILTER_ALPHA     USER_MOTOR_SPEED_FILTER_ALPHA
#define MOTOR_SPEED_START_THRESHOLD  USER_MOTOR_SPEED_START_THRESHOLD
#define MOTOR_SPEED_MIN_DRIVE_PWM    USER_MOTOR_SPEED_MIN_DRIVE_PWM

/* 左右独立速度比例尺 (国一方案: 左右编码器分辨率不同必须分别标定)
 * 若只定义了 USER_MOTOR_SPEED_SCALE (兼容旧配置), 左右都用该值 */
#ifdef USER_MOTOR_SPEED_SCALE
#ifndef USER_MOTOR_SPEED_SCALE_LEFT
#define USER_MOTOR_SPEED_SCALE_LEFT   USER_MOTOR_SPEED_SCALE
#endif
#ifndef USER_MOTOR_SPEED_SCALE_RIGHT
#define USER_MOTOR_SPEED_SCALE_RIGHT  USER_MOTOR_SPEED_SCALE
#endif
#endif

#define MOTOR_SPEED_SCALE_LEFT_DEFAULT   USER_MOTOR_SPEED_SCALE_LEFT
#define MOTOR_SPEED_SCALE_RIGHT_DEFAULT  USER_MOTOR_SPEED_SCALE_RIGHT

/* ============ 业务接口声明 ============ */
extern bsp_motor_t Motor_Left;
extern bsp_motor_t Motor_Right;

/* 左右编码器计数器（由GPIO双相双边沿中断维护） */
extern volatile int32_t g_left_enc_count;
extern volatile int32_t g_right_enc_count;

void App_Motor_Init(void);
void Motor_Set_Speed(int32_t speed);
void Motor_Right_Set_Speed(int32_t speed);
void Motor_Forward(int32_t speed);
void Motor_Backward(int32_t speed);
void Motor_Stop(void);
void Motor_Update_Status(void);
void App_Motor_Speed_PID_Update(void);
void App_Motor_Enable_Speed_PID(bool enable);
void App_Motor_Set_Speed_PID(float kp, float ki, float kd);
float App_Motor_Get_Speed_PID_Kp(void);
float App_Motor_Get_Speed_PID_Ki(void);
float App_Motor_Get_Speed_PID_Kd(void);

/* 速度比例尺设置 (国一方案: 左右编码器分辨率不同必须分别标定)
 * - App_Motor_Set_Speed_Scale: 同时设置左右 (兼容旧调用)
 * - App_Motor_Set_Speed_Scale_Left/Right: 左右独立设置 */
void App_Motor_Set_Speed_Scale(float scale);
void App_Motor_Set_Speed_Scale_Left(float scale);
void App_Motor_Set_Speed_Scale_Right(float scale);
float App_Motor_Get_Speed_Scale_Left(void);
float App_Motor_Get_Speed_Scale_Right(void);

/* 速度比例尺自动标定 (国一方案核心: 速度环可信的前提)
 * 使用方法: 悬空车轮 → 调用此函数 → 自动以 PWM=50 跑1秒测编码器速度
 *          → 计算 scale = encoder_speed/50 → 写入左右独立 scale → RTT 打印结果
 * 标定后把打印的 Scale 值写入 main.c 的 USER_MOTOR_SPEED_SCALE_LEFT/RIGHT */
void App_Motor_Calibrate_Scale(void);

/* 编码器GPIO中断回调（在main.c的GROUP1 ISR中调用） */
void Encoder_GPIO_IRQHandler(uint32_t pending_pins);

#endif /* APP_ENCODER_MOTOR_H */
