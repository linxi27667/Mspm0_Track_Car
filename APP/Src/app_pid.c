/**
 * @file    app_pid.c
 * @brief   通用PID控制器实现（纯算法，零硬件依赖）
 */
#include "app_pid.h"
#include <stddef.h>
#include <math.h>

//=====PID一次业务函数=====
void Pid_Init(bsp_pid_t *pid, float kp, float ki, float kd,
               float integral_limit, float output_limit)
{
    if (pid == NULL) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit   = integral_limit;
    pid->output_limit     = output_limit;
    pid->integral_sep_threshold = 0;
    pid->deriv_alpha      = 0;

    pid->error           = 0;
    pid->last_error      = 0;
    pid->integral        = 0;
    pid->derivative      = 0;
    pid->last_derivative = 0;
    pid->output          = 0;
}

void Pid_Set_Params(bsp_pid_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd)) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float Pid_Compute(bsp_pid_t *pid, float target, float feedback)
{
    if (pid == NULL) return 0;
    if (!isfinite(target) || !isfinite(feedback)) {
        Pid_Reset(pid);
        return 0;
    }

    pid->error = target - feedback;

    // 积分分离
    if (pid->integral_sep_threshold > 0 &&
        (pid->error > pid->integral_sep_threshold ||
         pid->error < -pid->integral_sep_threshold))
    {
        // 不累加积分
    }
    else
    {
        pid->integral += pid->error;
        if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }

    // 微分（支持不完全微分）
    float raw_derivative = pid->error - pid->last_error;
    if (pid->deriv_alpha > 0.0f)
    {
        pid->derivative = pid->deriv_alpha * pid->last_derivative
                        + (1.0f - pid->deriv_alpha) * raw_derivative;
    }
    else
    {
        pid->derivative = raw_derivative;
    }

    // PID输出
    pid->output = pid->kp * pid->error
                + pid->ki * pid->integral
                + pid->kd * pid->derivative;

    // 输出限幅
    if (pid->output > pid->output_limit)  pid->output = pid->output_limit;
    if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;

    pid->last_error      = pid->error;
    pid->last_derivative = pid->derivative;

    return pid->output;
}

void Pid_Reset(bsp_pid_t *pid)
{
    if (pid == NULL) return;
    pid->error           = 0;
    pid->last_error      = 0;
    pid->integral        = 0;
    pid->derivative      = 0;
    pid->last_derivative = 0;
    pid->output          = 0;
}
