/**
 * @file    bsp_encoder_motor.h
 * @brief   霍尔编码器电机驱动核心框架（BSP — 纯逻辑，跨平台）
 */
#ifndef __BSP_ENCODER_MOTOR_H__
#define __BSP_ENCODER_MOTOR_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* --- AT8236 单路输入 PWM 端点 --- */
typedef struct {
    void     *timer;
    uint32_t  channel;
} bsp_motor_pwm_t;

/* --- 电机运行方向 --- */
typedef enum {
    BSP_MOTOR_DIR_STOP    = 0,
    BSP_MOTOR_DIR_FORWARD = 1,
    BSP_MOTOR_DIR_REVERSE = 2
} bsp_motor_dir_t;

/* --- 霍尔编码器电机对象 --- */
typedef struct bsp_motor bsp_motor_t;

struct bsp_motor {
    /* --- 1. 物理配置 (Config) --- */
    bsp_motor_pwm_t input1_pwm;
    bsp_motor_pwm_t input2_pwm;
    void     *enc_timer;

    /* --- 2. 运行状态 (Status) --- */
    int32_t current_pwm;        /* 当前实际PWM输出值 */
    int32_t target_speed;       /* 目标速度(0-100)，由业务层设置 */
    int32_t encoder_speed;      /* 编码器速度(计数/周期) */
    int32_t total_position;     /* 编码器累计位置 */
    bool    use_speed_pid;      /* true=速度PID闭环, false=开环直写PWM */

    /* --- 3. 抽象硬件操作方法 (Function Pointers) --- */
    void    (*Init)(bsp_motor_t *motor);
    void    (*Pwm_Write)(void *timer, uint32_t channel, uint32_t duty);
    int32_t (*Enc_Read)(void *enc_timer);
};

/* --- 4. 对外核心API (跨平台通用) --- */
void Bsp_Motor_Init_Device(bsp_motor_t *motor);
void Bsp_Motor_Set_Speed(bsp_motor_t *motor, int32_t speed_val);
void Bsp_Motor_Write_Pwm(bsp_motor_t *motor, int32_t pwm_val);
void Bsp_Motor_Update_Status(bsp_motor_t *motor);

#endif /* __BSP_ENCODER_MOTOR_H__ */
