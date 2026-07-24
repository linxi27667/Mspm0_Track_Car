/**
 * @file    bsp_encoder_motor.c
 * @brief   霍尔编码器电机核心逻辑实现（BSP — 纯逻辑，跨平台）
 */
#include "bsp_encoder_motor.h"

/* ================= 1. 设备初始化 ================= */
void Bsp_Motor_Init_Device(bsp_motor_t *motor)
{
    if (motor == NULL) return;

    motor->current_pwm    = 0;
    motor->target_speed   = 0;
    motor->encoder_speed  = 0;
    motor->total_position = 0;

    if (motor->Init != NULL) {
        motor->Init(motor);
    }
}

/* ================= 2. AT8236双输入PWM写入 ================= */
void Bsp_Motor_Write_Pwm(bsp_motor_t *motor, int32_t pwm_val)
{
    if (motor == NULL) return;

    /* 修复: PWM 限幅 [-100, 100]，避免调用方传入越界值导致语义混乱
     * (定时器 period=100, CC>100 时输出恒高=100%占空) */
    if (pwm_val > 100)  pwm_val = 100;
    if (pwm_val < -100) pwm_val = -100;

    motor->current_pwm = pwm_val;

    if (pwm_val > 0) {
        if (motor->Pwm_Write) {
            /* 先关闭反向输入，再输出正向PWM，避免换向时两路同时有效。 */
            motor->Pwm_Write(motor->input2_pwm.timer,
                             motor->input2_pwm.channel, 0u);
            motor->Pwm_Write(motor->input1_pwm.timer,
                             motor->input1_pwm.channel, (uint32_t)pwm_val);
        }
    } else if (pwm_val < 0) {
        if (motor->Pwm_Write) {
            /* 先关闭正向输入，再输出反向PWM。 */
            motor->Pwm_Write(motor->input1_pwm.timer,
                             motor->input1_pwm.channel, 0u);
            motor->Pwm_Write(motor->input2_pwm.timer,
                             motor->input2_pwm.channel, (uint32_t)(-pwm_val));
        }
    } else {
        if (motor->Pwm_Write) {
            /* 双低为滑行停车，也是故障状态的无源安全输出。 */
            motor->Pwm_Write(motor->input1_pwm.timer,
                             motor->input1_pwm.channel, 0u);
            motor->Pwm_Write(motor->input2_pwm.timer,
                             motor->input2_pwm.channel, 0u);
        }
    }
}

/* ================= 3. 速度设定（开环/闭环统一入口） ================= */
void Bsp_Motor_Set_Speed(bsp_motor_t *motor, int32_t speed_val)
{
    if (motor == NULL) return;

    motor->target_speed = speed_val;

    if (!motor->use_speed_pid) {
        /* 开环模式：直接写PWM */
        Bsp_Motor_Write_Pwm(motor, speed_val);
    }
    /* 闭环模式：由速度PID更新函数负责写PWM */
}

/* ================= 4. 状态更新（编码器读取） ================= */
void Bsp_Motor_Update_Status(bsp_motor_t *motor)
{
    if (motor == NULL) return;

    if (motor->Enc_Read != NULL) {
        motor->encoder_speed = motor->Enc_Read(motor->enc_timer);
        motor->total_position += motor->encoder_speed;
    }
}
