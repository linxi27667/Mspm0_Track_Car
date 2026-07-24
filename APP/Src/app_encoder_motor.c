/**
 * @file    app_encoder_motor.c
 * @brief   霍尔编码器电机硬件绑定实现（MSPM0G3507）
 * @note    左右编码器: GPIO双相双边沿4x正交解码
 *          速度PID: 前馈(目标速度) + PID修正，级联位置PID
 */
#include "app_encoder_motor.h"
#include "app_log.h"
#include "app_common.h"
#include "app_pid.h"
#include "app_safety.h"
#include "motor_speed_logic.h"
#include "encoder_quadrature_logic.h"
#include <elog.h>

/* ================= 编码器中断计数 ================= */
volatile int32_t g_left_enc_count  = 0;
volatile int32_t g_right_enc_count = 0;
static int32_t   s_left_enc_last   = 0;
static int32_t   s_right_enc_last  = 0;
static uint8_t   s_left_enc_state;
static uint8_t   s_right_enc_state;

/* ================= 速度PID ================= */
static bsp_pid_t s_left_speed_pid;
static bsp_pid_t s_right_speed_pid;
/* 左右电机和编码器安装差异仍需分别标定scale。 */
static float     s_speed_scale_left  = MOTOR_SPEED_SCALE_LEFT_DEFAULT;
static float     s_speed_scale_right = MOTOR_SPEED_SCALE_RIGHT_DEFAULT;
static float     s_left_speed_filtered;
static float     s_right_speed_filtered;

/* ================= 1. 硬件底层函数 (HW_ 前缀) ================= */

static void HW_Motor_Init(bsp_motor_t *motor)
{
    if (motor == NULL) return;

    if (motor->input1_pwm.timer != NULL) {
        DL_Timer_enableClock((GPTIMER_Regs *)motor->input1_pwm.timer);
        DL_Timer_startCounter((GPTIMER_Regs *)motor->input1_pwm.timer);
    }
    if (motor->input2_pwm.timer != NULL &&
        motor->input2_pwm.timer != motor->input1_pwm.timer) {
        DL_Timer_enableClock((GPTIMER_Regs *)motor->input2_pwm.timer);
        DL_Timer_startCounter((GPTIMER_Regs *)motor->input2_pwm.timer);
    }
}

static void HW_Pwm_Write(void *timer, uint32_t channel, uint32_t duty)
{
    if (timer == NULL) return;
    uint32_t compare = Motor_Pwm_DutyToCompare(duty, MOTOR_PWM_PERIOD);
    DL_Timer_setCaptureCompareValue((GPTIMER_Regs *)timer, compare,
                                    (DL_TIMER_CC_INDEX)channel);
}

/* 左电机编码器读取 — GPIO 4x正交累计值差分。 */
static int32_t HW_Enc_Read_Left(void *enc_timer)
{
    static bool first_read = true;
    (void)enc_timer;

    int32_t current = g_left_enc_count;
    if (first_read) {
        s_left_enc_last = current;
        first_read = false;
        return 0;
    }

    int32_t delta = current - s_left_enc_last;
    s_left_enc_last = current;
    return delta;
}

/* 右电机编码器读取 — GPIO 4x正交累计值差分。 */
static int32_t HW_Enc_Read_Right(void *enc_timer)
{
    (void)enc_timer;  /* 右编码器不使用定时器 */
    static bool first_read = true;

    int32_t current = g_right_enc_count;
    if (first_read) {
        s_right_enc_last = current;
        first_read = false;
        return 0;
    }

    int32_t delta = current - s_right_enc_last;
    s_right_enc_last = current;
    return delta;
}

/* ================= 2. 对象实例化与引脚拼装 ================= */

bsp_motor_t Motor_Left = {
    .input1_pwm = {MOTOR_LEFT_AIN1_PWM_TIMER, MOTOR_LEFT_AIN1_PWM_CHANNEL},
    .input2_pwm = {MOTOR_LEFT_AIN2_PWM_TIMER, MOTOR_LEFT_AIN2_PWM_CHANNEL},
    .enc_timer  = NULL,
    .use_speed_pid = false,

    .Init        = HW_Motor_Init,
    .Pwm_Write   = HW_Pwm_Write,
    .Enc_Read    = HW_Enc_Read_Left
};

bsp_motor_t Motor_Right = {
    .input1_pwm = {MOTOR_RIGHT_BIN1_PWM_TIMER, MOTOR_RIGHT_BIN1_PWM_CHANNEL},
    .input2_pwm = {MOTOR_RIGHT_BIN2_PWM_TIMER, MOTOR_RIGHT_BIN2_PWM_CHANNEL},
    .enc_timer  = NULL,
    .use_speed_pid = false,

    .Init        = HW_Motor_Init,
    .Pwm_Write   = HW_Pwm_Write,
    .Enc_Read    = HW_Enc_Read_Right
};

/* ================= 3. 编码器GPIO中断处理 ================= */

static uint8_t HW_Encoder_Read_State(GPIO_Regs *port, uint32_t pha_pin,
                                     uint32_t phb_pin)
{
    uint32_t din = port->DIN31_0;
    uint8_t pha = (din & pha_pin) ? 1u : 0u;
    uint8_t phb = (din & phb_pin) ? 1u : 0u;
    return (uint8_t)((pha << 1u) | phb);
}

void Encoder_GPIO_IRQHandler(uint32_t pending_pins)
{
    if ((pending_pins & (LEFT_ENC_PHA_PIN | LEFT_ENC_PHB_PIN)) != 0u) {
        uint8_t current = HW_Encoder_Read_State(LEFT_ENC_PHA_PORT,
                                                LEFT_ENC_PHA_PIN,
                                                LEFT_ENC_PHB_PIN);
        int8_t delta = Encoder_Quadrature_Delta(s_left_enc_state, current);
        g_left_enc_count += (int32_t)delta * LEFT_ENC_FORWARD_SIGN;
        s_left_enc_state = current;
    }

    if ((pending_pins & (RIGHT_ENC_PHA_PIN | RIGHT_ENC_PHB_PIN)) != 0u) {
        uint8_t current = HW_Encoder_Read_State(RIGHT_ENC_PHA_PORT,
                                                RIGHT_ENC_PHA_PIN,
                                                RIGHT_ENC_PHB_PIN);
        int8_t delta = Encoder_Quadrature_Delta(s_right_enc_state, current);
        g_right_enc_count += (int32_t)delta * RIGHT_ENC_FORWARD_SIGN;
        s_right_enc_state = current;
    }
}

/* ================= 4. 速度PID ================= */

/**
 * @brief   单电机速度PID更新（前馈 + PID修正）
 * @note    前馈: target_speed 直接作为基础PWM
 *          PID修正: 根据编码器反馈修正PWM，消除左右电机机械差异
 *          最终PWM = target_speed + PID_correction
 *          scale: 左右独立的速度比例尺（覆盖电机与安装差异）
 */
static void Speed_PID_Update_Single(bsp_motor_t *motor, bsp_pid_t *pid,
                                    float scale, float *filtered_encoder)
{
    if (motor == NULL || pid == NULL || filtered_encoder == NULL) return;
    /* Encoder monitoring remains active in direct-PWM tracking mode. */
    Bsp_Motor_Update_Status(motor);
    if (!motor->use_speed_pid) return;

    *filtered_encoder = Motor_Speed_LowPass(*filtered_encoder,
                                             (float)motor->encoder_speed,
                                             MOTOR_SPEED_FILTER_ALPHA);

    if (motor->target_speed == 0) {
        Pid_Reset(pid);
        Bsp_Motor_Write_Pwm(motor, 0);
        return;
    }

    /* 编码器速度转换为0-100速度单位 (左右独立 scale) */
    float actual_speed = 0;
    if (scale > 0.001f) {
        actual_speed = *filtered_encoder / scale;
    }

    /* PID计算修正量: setpoint=target_speed, feedback=actual_speed */
    float correction = Pid_Compute(pid, (float)motor->target_speed, actual_speed);

    /* 前馈 + 修正 */
    int32_t pwm = motor->target_speed + (int32_t)correction;

    float actual_abs = (actual_speed >= 0.0f) ? actual_speed : -actual_speed;
    int32_t minimum_drive = (actual_abs < MOTOR_SPEED_START_THRESHOLD)
                          ? MOTOR_SPEED_MIN_DRIVE_PWM : 0;
    pwm = Motor_Speed_ConstrainPwm(motor->target_speed, pwm, minimum_drive);

    /* 写入电机 */
    Bsp_Motor_Write_Pwm(motor, pwm);
}

void App_Motor_Speed_PID_Update(void)
{
    Speed_PID_Update_Single(&Motor_Left, &s_left_speed_pid,
                            s_speed_scale_left, &s_left_speed_filtered);
    Speed_PID_Update_Single(&Motor_Right, &s_right_speed_pid,
                            s_speed_scale_right, &s_right_speed_filtered);
}

void App_Motor_Enable_Speed_PID(bool enable)
{
    Motor_Left.use_speed_pid  = enable;
    Motor_Right.use_speed_pid = enable;

    if (enable) {
        /* 切换到闭环时，复位PID防止积分突跳 */
        Pid_Reset(&s_left_speed_pid);
        Pid_Reset(&s_right_speed_pid);
        s_left_speed_filtered = 0.0f;
        s_right_speed_filtered = 0.0f;
        /* 修复: 同步编码器基准，避免首次读取产生大 delta 导致 PID 误差尖峰。
         * 调用 Bsp_Motor_Update_Status 更新 last_count 为当前值，
         * 然后清零 encoder_speed 丢弃这次读取。 */
        Bsp_Motor_Update_Status(&Motor_Left);
        Bsp_Motor_Update_Status(&Motor_Right);
        Motor_Left.encoder_speed  = 0;
        Motor_Right.encoder_speed = 0;
    } else {
        /* 切换回开环时，同步current_pwm = target_speed */
        Bsp_Motor_Write_Pwm(&Motor_Left,  Motor_Left.target_speed);
        Bsp_Motor_Write_Pwm(&Motor_Right, Motor_Right.target_speed);
    }
}

void App_Motor_Set_Speed_PID(float kp, float ki, float kd)
{
    Pid_Set_Params(&s_left_speed_pid, kp, ki, kd);
    Pid_Set_Params(&s_right_speed_pid, kp, ki, kd);
    Pid_Reset(&s_left_speed_pid);
    Pid_Reset(&s_right_speed_pid);
    s_left_speed_filtered = 0.0f;
    s_right_speed_filtered = 0.0f;
}

float App_Motor_Get_Speed_PID_Kp(void) { return s_left_speed_pid.kp; }
float App_Motor_Get_Speed_PID_Ki(void) { return s_left_speed_pid.ki; }
float App_Motor_Get_Speed_PID_Kd(void) { return s_left_speed_pid.kd; }

void App_Motor_Set_Speed_Scale(float scale)
{
    /* 兼容旧调用: 同时设置左右 scale */
    if (scale > 0.001f) {
        s_speed_scale_left  = scale;
        s_speed_scale_right = scale;
    }
}

void App_Motor_Set_Speed_Scale_Left(float scale)
{
    if (scale > 0.001f) {
        s_speed_scale_left = scale;
    }
}

void App_Motor_Set_Speed_Scale_Right(float scale)
{
    if (scale > 0.001f) {
        s_speed_scale_right = scale;
    }
}

float App_Motor_Get_Speed_Scale_Left(void)
{
    return s_speed_scale_left;
}

float App_Motor_Get_Speed_Scale_Right(void)
{
    return s_speed_scale_right;
}

/* 速度比例尺自动标定 (国一方案核心)
 * 流程: 切开环 → PWM=50 跑1秒 → 测左右 encoder_speed → scale=enc/50 → RTT打印
 * 使用前必须悬空车轮! */
#define MOTOR_CALIB_PWM         50
#define MOTOR_CALIB_SETTLE_MS   200     /* 加速稳定时间 */
#define MOTOR_CALIB_SAMPLE_MS   5       /* 采样周期(与速度PID一致) */
#define MOTOR_CALIB_SAMPLES     200     /* 200*5ms=1s */
void App_Motor_Calibrate_Scale(void)
{
    APP_DIAG_LOG_I("CALIB", "==== Speed Scale Calibration ====");
    APP_DIAG_LOG_I("CALIB", "Lift wheels! Running PWM=%d for %dms...",
           MOTOR_CALIB_PWM, MOTOR_CALIB_SAMPLES * MOTOR_CALIB_SAMPLE_MS);

    /* 切开环, 直写 PWM (避免闭环干扰测量) */
    App_Motor_Enable_Speed_PID(false);
    Bsp_Motor_Set_Speed(&Motor_Left,  MOTOR_CALIB_PWM);
    Bsp_Motor_Set_Speed(&Motor_Right, MOTOR_CALIB_PWM);

    /* 加速稳定 */
    Bsp_Delay_ms(MOTOR_CALIB_SETTLE_MS);

    /* 同步编码器基准, 丢弃首次读取 */
    Bsp_Motor_Update_Status(&Motor_Left);
    Bsp_Motor_Update_Status(&Motor_Right);
    Motor_Left.encoder_speed  = 0;
    Motor_Right.encoder_speed = 0;

    /* 采样 1 秒 */
    int32_t left_sum = 0, right_sum = 0;
    for (uint16_t i = 0; i < MOTOR_CALIB_SAMPLES; i++) {
        Bsp_Motor_Update_Status(&Motor_Left);
        Bsp_Motor_Update_Status(&Motor_Right);
        left_sum  += Motor_Left.encoder_speed;
        right_sum += Motor_Right.encoder_speed;
        Bsp_Delay_ms(MOTOR_CALIB_SAMPLE_MS);
    }

    /* 停车 */
    Bsp_Motor_Set_Speed(&Motor_Left,  0);
    Bsp_Motor_Set_Speed(&Motor_Right, 0);
    Bsp_Delay_ms(100);

    /* 计算平均 encoder_speed 和 scale */
    float left_avg  = (float)left_sum  / MOTOR_CALIB_SAMPLES;
    float right_avg = (float)right_sum / MOTOR_CALIB_SAMPLES;

    if (left_avg  > 1.0f) s_speed_scale_left  = left_avg  / (float)MOTOR_CALIB_PWM;
    if (right_avg > 1.0f) s_speed_scale_right = right_avg / (float)MOTOR_CALIB_PWM;

    APP_DIAG_LOG_I("CALIB", "LeftEncAvg:%ld RightEncAvg:%ld", (long)left_avg, (long)right_avg);
    APP_DIAG_LOG_I("CALIB", "Scale Left:%ld/1000 Right:%ld/1000",
           (long)(s_speed_scale_left  * 1000),
           (long)(s_speed_scale_right * 1000));
    APP_DIAG_LOG_I("CALIB", "Write to app_common.h/user_tuning.h:");
    APP_DIAG_LOG_I("CALIB", "  #define USER_MOTOR_SPEED_SCALE_LEFT   %d/1000", (long)(s_speed_scale_left  * 1000));
    APP_DIAG_LOG_I("CALIB", "  #define USER_MOTOR_SPEED_SCALE_RIGHT  %d/1000", (long)(s_speed_scale_right * 1000));
    APP_DIAG_LOG_I("CALIB", "==== Calibration Done ====");
}

/* ================= 5. 对外业务切入点 ================= */

void App_Motor_Init(void)
{
    /* 修复: Bsp_Motor_Init_Device 内部已调用 motor->Init (即 HW_Motor_Init)，
     * 原代码重复调用导致时钟使能/启动计数器执行两次。删除显式调用。 */
    Bsp_Motor_Init_Device(&Motor_Left);
    Bsp_Motor_Init_Device(&Motor_Right);

    g_left_enc_count = 0;
    g_right_enc_count = 0;
    s_left_enc_last = 0;
    s_right_enc_last = 0;
    s_left_enc_state = HW_Encoder_Read_State(LEFT_ENC_PHA_PORT,
                                             LEFT_ENC_PHA_PIN,
                                             LEFT_ENC_PHB_PIN);
    s_right_enc_state = HW_Encoder_Read_State(RIGHT_ENC_PHA_PORT,
                                              RIGHT_ENC_PHA_PIN,
                                              RIGHT_ENC_PHB_PIN);

    /* SysConfig已配置四个双边沿输入，此处仅使能GPIOB中断。 */
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    /* 初始化速度PID */
    Pid_Init(&s_left_speed_pid,
             MOTOR_SPEED_PID_KP, MOTOR_SPEED_PID_KI, MOTOR_SPEED_PID_KD,
             MOTOR_SPEED_PID_ILIMIT, MOTOR_SPEED_PID_OLIMIT);
    Pid_Init(&s_right_speed_pid,
             MOTOR_SPEED_PID_KP, MOTOR_SPEED_PID_KI, MOTOR_SPEED_PID_KD,
             MOTOR_SPEED_PID_ILIMIT, MOTOR_SPEED_PID_OLIMIT);

    /* 修复: 启用积分分离，避免启动/急停时积分饱和导致超调
     * 误差超过阈值时不累加积分，PID 仍正常计算 P/D */
    s_left_speed_pid.integral_sep_threshold  = MOTOR_SPEED_PID_INTEGRAL_SEP;
    s_right_speed_pid.integral_sep_threshold = MOTOR_SPEED_PID_INTEGRAL_SEP;

    /* Keep both H-bridges inactive until Safety explicitly authorizes motion. */
    App_Motor_Enable_Speed_PID(false);
    Bsp_Motor_Write_Pwm(&Motor_Left, 0);
    Bsp_Motor_Write_Pwm(&Motor_Right, 0);
}

void Motor_Set_Speed(int32_t speed)
{
    Bsp_Motor_Set_Speed(&Motor_Left, speed);
}

void Motor_Forward(int32_t speed)
{
    Bsp_Motor_Set_Speed(&Motor_Left, speed);
}

void Motor_Backward(int32_t speed)
{
    Bsp_Motor_Set_Speed(&Motor_Left, -speed);
}

void Motor_Stop(void)
{
    Bsp_Motor_Set_Speed(&Motor_Left, 0);
}

void Motor_Update_Status(void)
{
    Bsp_Motor_Update_Status(&Motor_Left);
    Bsp_Motor_Update_Status(&Motor_Right);
}

void Motor_Right_Set_Speed(int32_t speed)
{
    Bsp_Motor_Set_Speed(&Motor_Right, speed);
}
