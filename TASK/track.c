/**
 * @file    track.c
 * @brief   灰度PID循迹任务调度实现（TASK层 — 纯业务逻辑）
 * @note    数据流:
 *          Grayscale_Update → data_ready → Track_Calc_Error → PID → 差速 → Motor
 *          Imu_Update_Heading → heading → 直角弯原地旋转
 *
 *          本文件不直接操作任何硬件寄存器，全部通过 BSP/APP 层 API 调用
 */
#include "track.h"
#include "app_log.h"
#include "bsp_common.h"
#include "app_safety.h"
#include "imu_health_logic.h"
#include <elog.h>
#include <math.h>

/* ============================================================
 * 全局对象
 * ============================================================ */

track_t Track = {0};
static const char *s_track_start_result = "idle";
static void Track_Motor_Stop_Immediate(void);

static bool Track_Imu_Healthy(void)
{
    return Imu_Health_AllowsMotion(Imu_Sensor.initialized,
                                   Imu_Sensor.data_valid,
                                   Imu_Sensor.calibration_state) &&
           isfinite(Imu_Sensor.heading) && isfinite(Imu_Sensor.gyro_z);
}

static uint8_t Track_GrayscaleMask(void)
{
    uint8_t mask = 0u;
    for (uint8_t i = 0; i < BSP_GRAY_CHANNEL_NUM; i++) {
        if (Grayscale_Sensor.digital[i]) mask |= (uint8_t)(1u << i);
    }
    return mask;
}

/* ============================================================
 * 内部函数：偏差计算（数字灰度固定权重）
 * ============================================================ */

static uint8_t Track_CornerMinBlack(void)
{
    uint8_t corner_min_black = (uint8_t)(Track.right_angle_threshold + 0.5f);
    if (corner_min_black < 4u) corner_min_black = 4u;
    if (corner_min_black > BSP_GRAY_CHANNEL_NUM) {
        corner_min_black = BSP_GRAY_CHANNEL_NUM;
    }
    return corner_min_black;
}

static track_line_observation_t Track_ObserveLine(void)
{
    return Track_ClassifyLine8(Track_GrayscaleMask(), Track.error,
                               Track_CornerMinBlack());
}

static void Track_Calc_Error(track_line_observation_t observation)
{
    if (observation.type != TRACK_LINE_NORMAL) return;

    Track.last_error = Track.error;
    Track.error = observation.error;
}

/* ============================================================
 * 内部函数：变速策略
 * ============================================================ */

static void Track_Update_Speed(void)
{
    /* 国一方案: 连续曲率变速 (基于误差绝对值线性插值)
     * |error|=0 → SPEED_STRAIGHT (直道最快)
     * |error|>=CURVE_ERROR_MAX → SPEED_CURVE (弯道最慢)
     * 中间线性过渡, 进弯/出弯平滑无突变
     * 相比二档跳变: 进弯不甩出, 出弯不慢, 平均速度更高 */
    float abs_err = (Track.error >= 0) ? Track.error : -Track.error;
    float ratio = abs_err / TRACK_CURVE_ERROR_MAX;
    if (ratio > 1.0f) ratio = 1.0f;
    Track.target_speed = (int32_t)(Track.speed_straight +
                        (float)(Track.speed_curve - Track.speed_straight) * ratio);

    /* 速度渐变（避免突变导致车身抖动） */
    if (Track.base_speed < Track.target_speed) {
        Track.base_speed += TRACK_SPEED_RAMP_STEP;
        if (Track.base_speed > Track.target_speed)
            Track.base_speed = Track.target_speed;
    } else if (Track.base_speed > Track.target_speed) {
        Track.base_speed -= TRACK_SPEED_RAMP_STEP;
        if (Track.base_speed < Track.target_speed)
            Track.base_speed = Track.target_speed;
    }
}

/* ============================================================
 * 内部函数：赛道元素检测
 * ============================================================ */

static bool Track_ApproachCalibrationValid(void)
{
    return USER_ENCODER_LEFT_COUNTS_PER_MM > 0.0f &&
           USER_ENCODER_RIGHT_COUNTS_PER_MM > 0.0f &&
           (USER_SENSOR_TO_AXLE_MM + USER_RIGHT_ANGLE_DETECT_COMP_MM) > 0.0f;
}

static void Track_BeginRightAngle(uint32_t now, uint8_t digital_mask,
                                  const char *trigger)
{
    bool encoder_approach = Track_ApproachCalibrationValid();

    Track.right_angle_entry_heading = Imu_Sensor.heading;
    Track.right_angle_target = Imu_Sensor.heading;
    App_Motor_Enable_Speed_PID(true);
    Track.corner_left_start = Motor_Left.total_position;
    Track.corner_right_start = Motor_Right.total_position;
    Track.state = TRACK_STATE_RIGHT_ANGLE;
    Track.corner_phase = TRACK_CORNER_APPROACH;
    Track.corner_started_at = now;
    Track.state_timer = now;
    Track.right_angle_confirm = 0;
    Track.corner_heading_settle_confirm = 0;
    Track.capture_confirm = 0;
    Track.line_lost_confirm = 0;
    Track.centered_history = 0;
    APP_TRACK_LOG_I("TRACK", "CORNER_ENTER trigger=%s mask=0x%02X hist=%08lX err=%d enc=%ld,%ld entry=%d approach=%s fallback_ms=%lu target_mm=%d",
                    trigger, digital_mask,
                    (unsigned long)Track.line_mask_history,
                    (int)(Track.error * 1000.0f),
                    (long)Track.corner_left_start,
                    (long)Track.corner_right_start,
                    (int)Track.right_angle_entry_heading,
                    encoder_approach ? "encoder" : "time",
                    (unsigned long)TRACK_RIGHT_ANGLE_APPROACH_FALLBACK_MS,
                    (int)((USER_SENSOR_TO_AXLE_MM +
                           USER_RIGHT_ANGLE_DETECT_COMP_MM) * 10.0f));
}

static void Track_Detect_Pattern(track_line_observation_t observation)
{
    uint8_t digital_mask = Track.line_mask;

    if (Track.state != TRACK_STATE_LINE_FOLLOW) {
        Track.right_angle_confirm = 0;
        Track.line_lost_confirm = 0;
        Track.invalid_line_confirm = 0;
        return;
    }

    if (observation.type == TRACK_LINE_NORMAL) {
        Track.centered_history = Track_UpdateConfirmFrames(
            Track.centered_history, Track_Abs_Float(observation.error) <= 0.75f);
        if (Track.centered_history > 0u) {
            Track.last_centered_at = Bsp_GetTick();
        }
        Track.right_angle_confirm = 0u;
        Track.line_lost_confirm = 0u;
        Track.invalid_line_confirm = 0u;
        return;
    }

    if (observation.type == TRACK_LINE_LEFT_CORNER) {
        Track.right_angle_confirm = Track_UpdateCornerConfirm(
            Track.right_angle_confirm, Track.centered_history, true);
        Track.line_lost_confirm = 0u;
        Track.invalid_line_confirm = 0u;

        {
        bool gyro_available = Track.use_gyro && Track_Imu_Healthy();
        uint32_t now = Bsp_GetTick();
        bool corner_armed = Track_CornerArmed(
            now, Track.tracking_started_at, Track.last_corner_completed_at,
            Track.right_angle_count, TRACK_RIGHT_ANGLE_ARM_DELAY,
            TRACK_RIGHT_ANGLE_RETRIGGER_DELAY);

        if (gyro_available && corner_armed &&
            Track.right_angle_confirm >= TRACK_RIGHT_ANGLE_CONFIRM_FRAMES) {
                Track_BeginRightAngle(now, digital_mask, "mask");
                return;
        }
        }
        return;
    }

    Track.right_angle_confirm = 0u;
    if (observation.type == TRACK_LINE_LOST) {
        uint32_t now = Bsp_GetTick();
        bool gyro_available = Track.use_gyro && Track_Imu_Healthy();
        bool corner_armed = Track_CornerArmed(
            now, Track.tracking_started_at, Track.last_corner_completed_at,
            Track.right_angle_count, TRACK_RIGHT_ANGLE_ARM_DELAY,
            TRACK_RIGHT_ANGLE_RETRIGGER_DELAY);

        Track.invalid_line_confirm = 0u;
        Track.line_lost_confirm = Track_UpdateConfirmFrames(
            Track.line_lost_confirm, true);
        if (gyro_available && corner_armed &&
            Track.line_lost_confirm >= TRACK_RIGHT_ANGLE_WHITE_FALLBACK_FRAMES &&
            Track_WhiteCornerFallbackReady(
                Track.centered_history, Track.line_lost_confirm, now,
                Track.last_centered_at, TRACK_RIGHT_ANGLE_WHITE_RECENT_MS)) {
            Track_BeginRightAngle(now, digital_mask, "white");
            return;
        }
    } else {
        Track.centered_history = 0u;
        Track.line_lost_confirm = 0u;
        Track.invalid_line_confirm = Track_UpdateConfirmFrames(
            Track.invalid_line_confirm, true);
    }
}

/* ============================================================
 * 内部函数：立即停车（切开环+直写PWM=0，避免闭环下电机持续运转）
 * ============================================================ */
static void Track_Motor_Stop_Immediate(void)
{
    Track.left_speed  = 0;
    Track.right_speed = 0;
    Track.turn_output = 0;
    if (Track.use_motor) {
        Motor_Set_Speed(0);
        Motor_Right_Set_Speed(0);
        App_Motor_Enable_Speed_PID(false);
        Bsp_Motor_Write_Pwm(&Motor_Left, 0);
        Bsp_Motor_Write_Pwm(&Motor_Right, 0);
    }
}

static void Track_Rotate_End(const char *result)
{
    APP_ROTATE_LOG_I("ROTATE", "end result=%s actual=%d error=%d",
                     result, (int)Track.rotate_actual_deg,
                     (int)Track.rotate_error_deg);
    Track.rotate_result = result;
    Track.rotate_settle_frames = 0;
    Track.state = TRACK_STATE_IDLE;
    Track.left_speed = 0;
    Track.right_speed = 0;
    Track.turn_output = 0;

    if (Track.use_motor) {
        Motor_Set_Speed(0);
        Motor_Right_Set_Speed(0);
        App_Motor_Enable_Speed_PID(false);
        Bsp_Motor_Write_Pwm(&Motor_Left, 0);
        Bsp_Motor_Write_Pwm(&Motor_Right, 0);
    }

    if (Safety_GetState() != SAFETY_STATE_FAULT) {
        Safety_TransitionAfterRotation();
    }
}

/* Angle P plus measured yaw-rate D. Do not use Pid_Compute here: its D term
 * is a per-control-tick angle difference, while MPU gyro_z is the physical
 * derivative required to brake the chassis before it reaches the target. */
static void Track_Apply_Rotate_PD(float angle_error_deg, float gyro_rate_deg_s,
                                  float max_speed, int32_t min_speed,
                                  float coast_output)
{
    float output = USER_ROTATE_PID_KP * angle_error_deg -
                   USER_ROTATE_PID_KD_RATE * gyro_rate_deg_s;
    int32_t speed;
    int32_t desired_left = 0;
    int32_t desired_right = 0;

    if (output > max_speed) {
        output = max_speed;
    } else if (output < -max_speed) {
        output = -max_speed;
    }
    Track.turn_output = output;

    /* A target speed of 3 is below the encoder's stable 5ms resolution and
     * makes the wheel PI pulse its minimum starting PWM. Coast while inertia
     * is already carrying the chassis toward the target. If it stops outside
     * tolerance, PD is allowed to nudge it again. */
    if (Track_Rotate_ShouldCoast(angle_error_deg, gyro_rate_deg_s,
                                 output, coast_output)) {
        Track.left_speed = 0;
        Track.right_speed = 0;
        return;
    } else {
        speed = (int32_t)Track_Abs_Float(output);
        if (speed < min_speed) speed = min_speed;
        Track_Rotate_Command(output, speed, &desired_left, &desired_right);
    }

    Track.left_speed = Track_Slew_Command(Track.left_speed, desired_left,
                                          USER_ROTATE_SPEED_SLEW_STEP);
    Track.right_speed = Track_Slew_Command(Track.right_speed, desired_right,
                                           USER_ROTATE_SPEED_SLEW_STEP);
}

static void Track_Apply_Rotate_Envelope(float angle_error_deg,
                                        float gyro_rate_deg_s)
{
    float error_abs = Track_Abs_Float(angle_error_deg);
    if (error_abs < USER_ROTATE_PID_BLEND_ZONE_DEG) {
        float speed_limit = Track_Rotate_SpeedLimit(
            error_abs,
            USER_ROTATE_PID_FINE_ZONE_DEG,
            USER_ROTATE_PID_BLEND_ZONE_DEG,
            USER_ROTATE_PID_FINE_MAX_SPEED,
            USER_ROTATE_PID_MAX_SPEED);
        Track_Apply_Rotate_PD(angle_error_deg, gyro_rate_deg_s,
                              speed_limit, USER_ROTATE_PID_FINE_MIN_SPEED,
                              USER_ROTATE_PID_COAST_OUTPUT);
    } else {
        Track_Apply_Rotate_PD(angle_error_deg, gyro_rate_deg_s,
                              USER_ROTATE_PID_MAX_SPEED,
                              USER_ROTATE_PID_MIN_SPEED,
                              USER_ROTATE_PID_COAST_OUTPUT);
    }
}

static void Track_Handle_Rotate(void)
{
    uint32_t now = Bsp_GetTick();

    if (!Track_Imu_Healthy()) {
        Track_Rotate_End("imu_unavailable");
        return;
    }
    if (!Safety_AllowsMotorOutput()) {
        Track_Rotate_End("safety_revoked");
        return;
    }

    Track.rotate_actual_deg = Imu_Sensor.heading - Track.rotate_heading_baseline;
    Track.rotate_error_deg = Track_Rotate_Error(Track.rotate_heading_baseline,
                                                 Imu_Sensor.heading,
                                                 Track.rotate_target_deg);

    if (Track_Rotate_Timeout(now, Track.rotate_started_at,
                             Track.rotate_timeout_ms)) {
        Track_Rotate_End("rotate_timeout");
        return;
    }

    bool angle_inside = Track_Rotate_AngleInside(Track.rotate_error_deg);
    bool inside_tolerance = Track_Rotate_Complete(
        Track.rotate_error_deg, Track_Abs_Float(Imu_Sensor.gyro_z));
    Track.rotate_settle_frames = Track_Rotate_UpdateSettleFrames(
        Track.rotate_settle_frames, inside_tolerance);
    /* Once the angle is inside the acceptance band, coast to remove residual
     * yaw energy. Active rate braking here causes a low-speed limit cycle. */
    if (angle_inside) {
        Track.left_speed = 0;
        Track.right_speed = 0;
        Track.turn_output = 0;
        Motor_Set_Speed(0);
        Motor_Right_Set_Speed(0);
        if (Track_Rotate_Settled(Track.rotate_settle_frames,
                                 USER_ROTATE_SETTLE_FRAMES)) {
            Track_Rotate_End("complete");
        }
        return;
    }

    Track_Apply_Rotate_Envelope(Track.rotate_error_deg, Imu_Sensor.gyro_z);
    Motor_Set_Speed(Track.left_speed);
    Motor_Right_Set_Speed(Track.right_speed);
}

static void Track_Heading_Hold_End(const char *result)
{
    Track.heading_hold_result = result;
    Track.state = TRACK_STATE_IDLE;
    Track.left_speed = 0;
    Track.right_speed = 0;
    Track.turn_output = 0;

    if (Track.use_motor) {
        Motor_Set_Speed(0);
        Motor_Right_Set_Speed(0);
        App_Motor_Enable_Speed_PID(false);
        Bsp_Motor_Write_Pwm(&Motor_Left, 0);
        Bsp_Motor_Write_Pwm(&Motor_Right, 0);
    }
    if (Safety_GetState() != SAFETY_STATE_FAULT) {
        Safety_TransitionAfterRotation();
    }
}

static void Track_Motor_Test_End(const char *result)
{
    APP_TRACK_LOG_I("MOTORTEST", "end result=%s phase=%u target=%d L=%ld R=%ld",
                    result, Track.motor_test_phase, Track.motor_test_target,
                    (long)Motor_Left.total_position,
                    (long)Motor_Right.total_position);
    Track.motor_test_result = result;
    Track.motor_test_target = 0;
    Track.state = TRACK_STATE_IDLE;
    Track_Motor_Stop_Immediate();
    if (Safety_GetState() != SAFETY_STATE_FAULT) {
        Safety_TransitionAfterBenchTest();
    }
}

static void Track_Motor_Test_ResetSamples(void)
{
    Track.motor_test_left_sum = 0;
    Track.motor_test_right_sum = 0;
    Track.motor_test_left_abs_sum = 0u;
    Track.motor_test_right_abs_sum = 0u;
    Track.motor_test_sample_count = 0u;
}

static float Track_Motor_Test_Abs(float value)
{
    return value >= 0.0f ? value : -value;
}

static bool Track_Motor_Test_WithinTolerance(float measured, float expected)
{
    float limit = Track_Motor_Test_Abs(expected) * TRACK_MOTOR_TEST_TOLERANCE;
    return measured >= (expected - limit) && measured <= (expected + limit);
}

static void Track_Motor_Test_FinishStage(void)
{
    if (Track.motor_test_sample_count == 0u) {
        Track.motor_test_failed = true;
        APP_TRACK_LOG_W("MOTORTEST", "phase=%u no stable samples",
                        Track.motor_test_phase);
        return;
    }

    float left_avg = (float)Track.motor_test_left_sum /
                     (float)Track.motor_test_sample_count;
    float right_avg = (float)Track.motor_test_right_sum /
                      (float)Track.motor_test_sample_count;
    float left_abs_avg = (float)Track.motor_test_left_abs_sum /
                         (float)Track.motor_test_sample_count;
    float right_abs_avg = (float)Track.motor_test_right_abs_sum /
                          (float)Track.motor_test_sample_count;

    if (Track.motor_test_phase < TRACK_MOTOR_TEST_OPEN_PHASES) {
        APP_TRACK_LOG_I("MOTORTEST", "open phase=%u pwm=%d Lavg=%ld Ravg=%ld (x1000) n=%u",
                        Track.motor_test_phase, Track.motor_test_target,
                        (long)(left_avg * 1000.0f),
                        (long)(right_avg * 1000.0f),
                        Track.motor_test_sample_count);
        if (left_avg <= 0.0f || right_avg <= 0.0f ||
            (Track.motor_test_phase > 0u &&
             (left_avg <= Track.motor_test_prev_left_avg ||
              right_avg <= Track.motor_test_prev_right_avg))) {
            Track.motor_test_failed = true;
            APP_TRACK_LOG_W("MOTORTEST", "open response invalid phase=%u L=%ld R=%ld prevL=%ld prevR=%ld (x1000)",
                            Track.motor_test_phase,
                            (long)(left_avg * 1000.0f),
                            (long)(right_avg * 1000.0f),
                            (long)(Track.motor_test_prev_left_avg * 1000.0f),
                            (long)(Track.motor_test_prev_right_avg * 1000.0f));
        }
        Track.motor_test_scale_left_num +=
            (float)Track.motor_test_target * left_avg;
        Track.motor_test_scale_right_num +=
            (float)Track.motor_test_target * right_avg;
        Track.motor_test_scale_den +=
            (float)Track.motor_test_target * (float)Track.motor_test_target;
    } else if (Track.motor_test_phase < TRACK_MOTOR_TEST_STOP_PHASE) {
        float left_scale = App_Motor_Get_Speed_Scale_Left();
        float right_scale = App_Motor_Get_Speed_Scale_Right();
        float expected_left = left_scale * (float)Track.motor_test_target;
        float expected_right = right_scale * (float)Track.motor_test_target;
        bool left_ok = Track_Motor_Test_WithinTolerance(left_avg, expected_left);
        bool right_ok = Track_Motor_Test_WithinTolerance(right_avg, expected_right);
        bool first_in_direction = Track.motor_test_phase == TRACK_MOTOR_TEST_OPEN_PHASES ||
                                  Track.motor_test_phase ==
                                      (TRACK_MOTOR_TEST_OPEN_PHASES +
                                       TRACK_MOTOR_TEST_CLOSED_PHASES / 2u);
        bool monotonic = first_in_direction ||
                         (left_abs_avg > Track_Motor_Test_Abs(Track.motor_test_prev_left_avg) &&
                          right_abs_avg > Track_Motor_Test_Abs(Track.motor_test_prev_right_avg));
        float left_normalized = left_avg / left_scale;
        float right_normalized = right_avg / right_scale;
        float target_abs = Track_Motor_Test_Abs((float)Track.motor_test_target);
        float wheel_delta = Track_Motor_Test_Abs(
            Track_Motor_Test_Abs(left_normalized) -
            Track_Motor_Test_Abs(right_normalized));
        bool wheels_match = wheel_delta <=
                            target_abs * TRACK_MOTOR_TEST_WHEEL_MISMATCH;
        APP_TRACK_LOG_I("MOTORTEST", "closed phase=%u target=%d Lavg=%ld Ravg=%ld expL=%ld expR=%ld mismatch=%ld (x1000) pass=%u",
                        Track.motor_test_phase, Track.motor_test_target,
                        (long)(left_avg * 1000.0f),
                        (long)(right_avg * 1000.0f),
                        (long)(expected_left * 1000.0f),
                        (long)(expected_right * 1000.0f),
                        (long)(wheel_delta * 1000.0f),
                        (unsigned)(left_ok && right_ok && monotonic && wheels_match));
        if (!left_ok || !right_ok || !monotonic || !wheels_match) {
            Track.motor_test_failed = true;
        }
    } else {
        bool stopped = left_abs_avg <= TRACK_MOTOR_TEST_STOP_MAX_AVG &&
                       right_abs_avg <= TRACK_MOTOR_TEST_STOP_MAX_AVG;
        APP_TRACK_LOG_I("MOTORTEST", "stop phase=%u Labs=%ld Rabs=%ld (x1000) pass=%u",
                        Track.motor_test_phase,
                        (long)(left_abs_avg * 1000.0f),
                        (long)(right_abs_avg * 1000.0f),
                        (unsigned)stopped);
        if (!stopped) {
            Track.motor_test_failed = true;
        }
    }

    Track.motor_test_prev_left_avg = left_avg;
    Track.motor_test_prev_right_avg = right_avg;
}

static void Track_Handle_Motor_Test(void)
{
    static const int32_t targets[] = {
        TRACK_MOTOR_TEST_TARGET_LOW,
        TRACK_MOTOR_TEST_TARGET_MID,
        TRACK_MOTOR_TEST_TARGET_HIGH,
        TRACK_MOTOR_TEST_TARGET_LOW,
        TRACK_MOTOR_TEST_TARGET_MID,
        TRACK_MOTOR_TEST_TARGET_HIGH,
        -TRACK_MOTOR_TEST_TARGET_LOW,
        -TRACK_MOTOR_TEST_TARGET_MID,
        -TRACK_MOTOR_TEST_TARGET_HIGH,
        0
    };
    uint32_t now = Bsp_GetTick();

    if (!Safety_AllowsMotorOutput()) {
        Track_Motor_Test_End("safety_revoked");
        return;
    }
    if (Track_Timeout_Reached(now, Track.motor_test_started_at,
                              TRACK_MOTOR_TEST_TIMEOUT_MS)) {
        Track_Motor_Test_End("timeout");
        return;
    }

    if (Track_Timeout_Reached(now, Track.motor_test_stage_started_at,
                              TRACK_MOTOR_TEST_SETTLE_MS)) {
        Track.motor_test_left_sum += Motor_Left.encoder_speed;
        Track.motor_test_right_sum += Motor_Right.encoder_speed;
        Track.motor_test_left_abs_sum += (uint32_t)(Motor_Left.encoder_speed >= 0 ?
                                                   Motor_Left.encoder_speed :
                                                   -Motor_Left.encoder_speed);
        Track.motor_test_right_abs_sum += (uint32_t)(Motor_Right.encoder_speed >= 0 ?
                                                    Motor_Right.encoder_speed :
                                                    -Motor_Right.encoder_speed);
        Track.motor_test_sample_count++;
    }

    if (Track_Timeout_Reached(now, Track.motor_test_stage_started_at,
                              TRACK_MOTOR_TEST_STAGE_MS)) {
        if (Track.motor_test_phase == 0u &&
            (Track.motor_test_left_sum <= 0 ||
             Track.motor_test_right_sum <= 0)) {
            APP_TRACK_LOG_W("MOTORTEST", "sign check failed L=%ld R=%ld",
                            (long)Track.motor_test_left_sum,
                            (long)Track.motor_test_right_sum);
            Track_Motor_Test_End("encoder_sign_or_stall");
            return;
        }
        Track_Motor_Test_FinishStage();
        if (Track.motor_test_phase + 1u >= TRACK_MOTOR_TEST_PHASE_COUNT) {
            Track_Motor_Test_End(Track.motor_test_failed ?
                                 "qualification_failed" : "complete");
            return;
        }

        Track.motor_test_phase++;
        Track.motor_test_stage_started_at = now;
        Track.motor_test_target = targets[Track.motor_test_phase];
        Track.left_speed = Track.motor_test_target;
        Track.right_speed = Track.motor_test_target;
        Track_Motor_Test_ResetSamples();

        if (Track.motor_test_phase == TRACK_MOTOR_TEST_OPEN_PHASES) {
            float recommended_left = Track.motor_test_scale_left_num /
                                     Track.motor_test_scale_den;
            float recommended_right = Track.motor_test_scale_right_num /
                                      Track.motor_test_scale_den;
            APP_TRACK_LOG_I("MOTORTEST", "scale configured L=%ld R=%ld recommended L=%ld R=%ld (x1000)",
                            (long)(App_Motor_Get_Speed_Scale_Left() * 1000.0f),
                            (long)(App_Motor_Get_Speed_Scale_Right() * 1000.0f),
                            (long)(recommended_left * 1000.0f),
                            (long)(recommended_right * 1000.0f));
            Track.motor_test_prev_left_avg = 0;
            Track.motor_test_prev_right_avg = 0;
            App_Motor_Enable_Speed_PID(true);
        }
        if (Track.motor_test_phase ==
            (TRACK_MOTOR_TEST_OPEN_PHASES +
             TRACK_MOTOR_TEST_CLOSED_PHASES / 2u)) {
            Track.motor_test_prev_left_avg = 0;
            Track.motor_test_prev_right_avg = 0;
        }

        Motor_Set_Speed(Track.motor_test_target);
        Motor_Right_Set_Speed(Track.motor_test_target);
        APP_TRACK_LOG_I("MOTORTEST", "phase=%u mode=%s target=%d",
                        Track.motor_test_phase,
                        Track.motor_test_phase < TRACK_MOTOR_TEST_OPEN_PHASES ?
                        "open" : (Track.motor_test_phase ==
                                  TRACK_MOTOR_TEST_STOP_PHASE ?
                                  "stop" : "closed"),
                        Track.motor_test_target);
    }
}

static void Track_Handle_Heading_Hold(void)
{
    if (!Track_Imu_Healthy()) {
        Track_Heading_Hold_End("imu_unavailable");
        return;
    }
    if (!Safety_AllowsMotorOutput()) {
        Track_Heading_Hold_End("safety_revoked");
        return;
    }

    Track.heading_hold_actual = Imu_Sensor.heading;
    Track.heading_hold_error = Track.heading_hold_target - Track.heading_hold_actual;
    if (Track_Abs_Float(Track.heading_hold_error) <= USER_HEADING_HOLD_DEADBAND_DEG &&
        Track_Abs_Float(Imu_Sensor.gyro_z) <= USER_HEADING_HOLD_SETTLE_RATE_DEG_S) {
        Track.left_speed = 0;
        Track.right_speed = 0;
        Motor_Set_Speed(0);
        Motor_Right_Set_Speed(0);
        return;
    }

    Track_Apply_Rotate_PD(Track.heading_hold_error, Imu_Sensor.gyro_z,
                          USER_HEADING_HOLD_MAX_SPEED, USER_HEADING_HOLD_MIN_SPEED,
                          USER_HEADING_HOLD_COAST_OUTPUT);
    Motor_Set_Speed(Track.left_speed);
    Motor_Right_Set_Speed(Track.right_speed);
}

/* ============================================================
 * 内部函数：状态处理
 * ============================================================ */

/* 直角弯道处理：陀螺仪精确旋转90度 */
static void Track_Handle_RightAngle(track_line_observation_t observation)
{
    uint32_t now = Bsp_GetTick();
    int32_t left_delta = Track_EncoderDelta(Motor_Left.total_position,
                                            Track.corner_left_start);
    int32_t right_delta = Track_EncoderDelta(Motor_Right.total_position,
                                             Track.corner_right_start);
    float target_mm = USER_SENSOR_TO_AXLE_MM +
                      USER_RIGHT_ANGLE_DETECT_COMP_MM;

    if (!Track.use_gyro || !Track_Imu_Healthy()) {
        Track.state = TRACK_STATE_FINISHED;
        Track_Motor_Stop_Immediate();
        Safety_NotifySensorAbnormal();
        APP_TRACK_LOG_W("TRACK", "RIGHT_ANGLE aborted: gyro unavailable");
        return;
    }

    bool encoder_approach = Track_ApproachCalibrationValid();
    bool approach_done = encoder_approach
        ? Track_EncoderDistanceReached(
              Motor_Left.total_position, Motor_Right.total_position,
              Track.corner_left_start, Track.corner_right_start,
              USER_ENCODER_LEFT_COUNTS_PER_MM,
              USER_ENCODER_RIGHT_COUNTS_PER_MM, target_mm)
        : Track_Timeout_Reached(now, Track.corner_started_at,
                                TRACK_RIGHT_ANGLE_APPROACH_FALLBACK_MS);

    if (Track.corner_phase == TRACK_CORNER_APPROACH && approach_done) {
        Track.corner_phase = TRACK_CORNER_STOP;
        Track.state_timer = now;
        Track.base_speed = 0;
        Track.target_speed = 0;
        Track.turn_output = 0.0f;
        APP_TRACK_LOG_I("TRACK", "CORNER_STOP mask=0x%02X enc_d=%ld,%ld entry=%d actual=%d approach=%s fallback_ms=%lu",
                        Track.line_mask, (long)left_delta, (long)right_delta,
                        (int)Track.right_angle_entry_heading,
                        (int)Imu_Sensor.heading,
                        encoder_approach ? "encoder" : "time",
                        (unsigned long)TRACK_RIGHT_ANGLE_APPROACH_FALLBACK_MS);
    }

    if (Track.corner_phase == TRACK_CORNER_STOP &&
        Track_Timeout_Reached(now, Track.state_timer,
                              TRACK_RIGHT_ANGLE_STOP_MS)) {
        Track.right_angle_entry_heading = Imu_Sensor.heading;
        Track.right_angle_target = Imu_Sensor.heading +
                                   TRACK_RIGHT_ANGLE_TARGET_ANGLE;
        Track.corner_phase = TRACK_CORNER_PIVOT;
        Track.state_timer = now;
        Track.corner_heading_settle_confirm = 0;
        APP_TRACK_LOG_I("TRACK", "CORNER_PIVOT mask=0x%02X enc_d=%ld,%ld entry=%d target=%d actual=%d",
                        Track.line_mask, (long)left_delta, (long)right_delta,
                        (int)Track.right_angle_entry_heading,
                        (int)Track.right_angle_target,
                        (int)Imu_Sensor.heading);
    }

    float angle_error = Track.right_angle_target - Imu_Sensor.heading;
    float turned_abs = Track_Abs_Float(Imu_Sensor.heading -
                                       Track.right_angle_entry_heading);
    if (Track.corner_phase == TRACK_CORNER_PIVOT) {
        bool line_visible_after_turn =
            turned_abs >= TRACK_RIGHT_ANGLE_HYBRID_MIN_TURN_DEG &&
            observation.type == TRACK_LINE_NORMAL;

        Track.corner_heading_settle_confirm = Track_UpdateConfirmFrames(
            Track.corner_heading_settle_confirm,
            Track_RightAngle_HeadingSettled(angle_error,
                                            Imu_Sensor.gyro_z));
        Track.capture_confirm = Track_UpdateConfirmFrames(
            Track.capture_confirm, line_visible_after_turn);
        if (Track.corner_heading_settle_confirm >=
                TRACK_RIGHT_ANGLE_DONE_FRAMES ||
            Track_RightAngle_HybridHandoff(
                turned_abs, Imu_Sensor.gyro_z, Track.capture_confirm,
                TRACK_RIGHT_ANGLE_HYBRID_MIN_TURN_DEG,
                TRACK_RIGHT_ANGLE_HYBRID_HANDOFF_DEG,
                TRACK_RIGHT_ANGLE_HYBRID_MAX_RATE,
                TRACK_RIGHT_ANGLE_HYBRID_LINE_FRAMES)) {
            const char *reason = Track.capture_confirm >=
                                 TRACK_RIGHT_ANGLE_HYBRID_LINE_FRAMES
                               ? "line" :
                                 (turned_abs >= TRACK_RIGHT_ANGLE_HYBRID_HANDOFF_DEG
                                  ? "angle" : "settled");
            Track.corner_phase = TRACK_CORNER_CAPTURE;
            Track.corner_started_at = now;
            Track.state_timer = now;
            Track.capture_confirm = 0u;
            APP_TRACK_LOG_I("TRACK", "CORNER_CAPTURE reason=%s mask=0x%02X enc_d=%ld,%ld target=%d actual=%d err=%d turn=%d rate=%d",
                            reason, Track.line_mask, (long)left_delta,
                            (long)right_delta, (int)Track.right_angle_target,
                            (int)Imu_Sensor.heading, (int)angle_error,
                            (int)turned_abs,
                            (int)Imu_Sensor.gyro_z);
        }
    }

    if (Track.corner_phase == TRACK_CORNER_APPROACH) {
        Track.target_speed = TRACK_RIGHT_ANGLE_APPROACH_SPEED;
        Track.base_speed = TRACK_RIGHT_ANGLE_APPROACH_SPEED;
        Track.turn_output = Track_ClampTurn(
            Track_RightAngle_PivotTurn(
                Track.right_angle_entry_heading - Imu_Sensor.heading,
                Imu_Sensor.gyro_z, USER_ROTATE_PID_KP,
                USER_ROTATE_PID_KD_RATE,
                TRACK_RIGHT_ANGLE_CAPTURE_TURN_LIMIT),
            TRACK_RIGHT_ANGLE_CAPTURE_TURN_LIMIT);
    } else if (Track.corner_phase == TRACK_CORNER_STOP) {
        Track.target_speed = 0;
        Track.base_speed = 0;
        Track.turn_output = 0.0f;
    } else if (Track.corner_phase == TRACK_CORNER_PIVOT) {
        Track.target_speed = 0;
        Track.base_speed = 0;
        if (Track_Abs_Float(angle_error) <= TRACK_RIGHT_ANGLE_DONE_ERROR) {
            Track.left_speed = 0;
            Track.right_speed = 0;
            Track.turn_output = 0.0f;
        } else {
            Track_Apply_Rotate_Envelope(angle_error, Imu_Sensor.gyro_z);
        }
    } else {
        float heading_turn = Track_RightAngle_PivotTurn(
            angle_error, Imu_Sensor.gyro_z,
            TRACK_RIGHT_ANGLE_CAPTURE_HEADING_KP,
            TRACK_RIGHT_ANGLE_CAPTURE_HEADING_KD,
            TRACK_RIGHT_ANGLE_CAPTURE_HEADING_LIMIT);
        Track.target_speed = TRACK_RIGHT_ANGLE_CAPTURE_SPEED;
        Track.base_speed = TRACK_RIGHT_ANGLE_CAPTURE_SPEED;
        Track.turn_output = Track_CaptureBlendTurn(
            heading_turn, observation.error,
            observation.type == TRACK_LINE_NORMAL,
            USER_LINE_PID_KP, TRACK_RIGHT_ANGLE_CAPTURE_TURN_LIMIT);
    }

    if (Track.corner_phase == TRACK_CORNER_CAPTURE) {
        Track.capture_confirm = Track_UpdateConfirmFrames(
            Track.capture_confirm,
            turned_abs >= TRACK_RIGHT_ANGLE_HYBRID_MIN_TURN_DEG &&
            Track_CaptureLineValid(observation, 1.0f));
    }
    if (Track.corner_phase == TRACK_CORNER_CAPTURE &&
        Track.capture_confirm >= TRACK_RIGHT_ANGLE_CAPTURE_CONFIRM_FRAMES) {
        Track.state = TRACK_STATE_LINE_FOLLOW;
        Track.error = observation.error;
        Track.last_error = observation.error;
        Pid_Reset(&Track.line_pid);
        Track.right_angle_confirm = 0;
        Track.corner_heading_settle_confirm = 0;
        Track.capture_confirm = 0;
        Track.invalid_line_confirm = 0;
        Track.line_lost_confirm = 0;
        Track.centered_history = 0;
        Track.last_corner_completed_at = now;
        if (Track.right_angle_count < UINT8_MAX) {
            Track.right_angle_count++;
        }
        App_Motor_Enable_Speed_PID(Track.use_speed_pid);
        Track_Check_RightAngle_Completion();
        APP_TRACK_LOG_I("TRACK", "CORNER_DONE mask=0x%02X enc_d=%ld,%ld target=%d actual=%d err=%d turn=%d ra=%d",
                        Track.line_mask, (long)left_delta, (long)right_delta,
                        (int)Track.right_angle_target,
                        (int)Imu_Sensor.heading, (int)angle_error,
                        (int)turned_abs, Track.right_angle_count);
        return;
    }

    if (Track_Timeout_Reached(now, Track.corner_started_at,
                              TRACK_RIGHT_ANGLE_TIMEOUT)) {
        if (Track.corner_phase == TRACK_CORNER_PIVOT) {
            Track.corner_started_at = now;
            Track.state_timer = now;
            Track.corner_heading_settle_confirm = 0;
            Track.capture_confirm = 0;
            APP_TRACK_LOG_W("TRACK", "CORNER_RETRY pivot_timeout keep_pivot mask=0x%02X target=%d actual=%d err=%d turn=%d rate=%d",
                            Track.line_mask, (int)Track.right_angle_target,
                            (int)Imu_Sensor.heading, (int)angle_error,
                            (int)turned_abs,
                            (int)Imu_Sensor.gyro_z);
            return;
        }
        Track.corner_started_at = now;
        Track.state_timer = now;
        Track.capture_confirm = 0;
        APP_TRACK_LOG_W("TRACK", "CORNER_RETRY capture_timeout keep_capture mask=0x%02X enc_d=%ld,%ld target=%d actual=%d err=%d turn=%d rate=%d ra=%d",
                        Track.line_mask,
                        (long)left_delta, (long)right_delta,
                        (int)Track.right_angle_target,
                        (int)Imu_Sensor.heading, (int)angle_error,
                        (int)turned_abs, (int)Imu_Sensor.gyro_z,
                        Track.right_angle_count);
    }
}

/* 正方形计圈仅用于遥测。达到设定圈数后继续循迹，只有显式 stop
 * 或传感器硬件故障可以结束运动。 */
void Track_Check_RightAngle_Completion(void)
{
    uint16_t target_total = (uint16_t)Track.target_circles * 4u;

    Track.current_circle = Track_CirclesFromCorners(
        Track.right_angle_count, Track.target_circles);
    if ((uint16_t)Track.right_angle_count == target_total) {
        APP_TRACK_LOG_I("TRACK", "LAP_TARGET_REACHED right_angles=%d/%d (circles=%d/%d) -> CONTINUE",
                        Track.right_angle_count, target_total,
                        Track.current_circle, Track.target_circles);
    }
}

/* 差速输出到电机（内侧电机最低速度限制） */
static void Track_Motor_Output(void)
{
    bool pivot_turn = Track.state == TRACK_STATE_RIGHT_ANGLE &&
                      Track.corner_phase == TRACK_CORNER_PIVOT;
    bool capture_turn = Track.state == TRACK_STATE_RIGHT_ANGLE &&
                        Track.corner_phase == TRACK_CORNER_CAPTURE;

    /* 双重保险: Track 状态机 + Safety 授权
     * Safety_AllowsMotorOutput() 是唯一权威授权 */
    if (!Track_State_Allows_Motor(Track.state) ||
        !Safety_AllowsMotorOutput()) {
        Track_Motor_Stop_Immediate();
        return;
    }

    if (!pivot_turn) {
        Track.left_speed  = Track.base_speed + (int32_t)Track.turn_output;
        Track.right_speed = Track.base_speed - (int32_t)Track.turn_output;
    }

    /* 内侧电机最低速度限制（不允许反转，避免急转抖动）
     * 修复: 原代码双 if 互斥条件在两轮都低于 MIN 时均不执行，
     * 导致内侧电机可反转。改为分别钳位。 */
    if (!pivot_turn && !capture_turn &&
        (Track.left_speed != 0 || Track.right_speed != 0)) {
        if (Track.left_speed  < TRACK_MOTOR_INNER_MIN) Track.left_speed  = TRACK_MOTOR_INNER_MIN;
        if (Track.right_speed < TRACK_MOTOR_INNER_MIN) Track.right_speed = TRACK_MOTOR_INNER_MIN;
    }

    /* 电机速度限幅 */
    if (Track.left_speed >  TRACK_MOTOR_SPEED_MAX) Track.left_speed  =  TRACK_MOTOR_SPEED_MAX;
    if (Track.left_speed <  TRACK_MOTOR_SPEED_MIN) Track.left_speed  =  TRACK_MOTOR_SPEED_MIN;
    if (Track.right_speed > TRACK_MOTOR_SPEED_MAX) Track.right_speed =  TRACK_MOTOR_SPEED_MAX;
    if (Track.right_speed < TRACK_MOTOR_SPEED_MIN) Track.right_speed =  TRACK_MOTOR_SPEED_MIN;

    if (Track.use_motor) {
        Motor_Set_Speed(Track.left_speed);
        Motor_Right_Set_Speed(Track.right_speed);
    }
}

/* ============================================================
 * 对外接口实现
 * ============================================================ */

void Track_Init(void)
{
    /* 状态机清零 */
    Track.error              = 0;
    Track.last_error         = 0;
    Track.state              = TRACK_STATE_IDLE;
    Track.right_angle_target = 0;
    Track.right_angle_entry_heading = 0;
    Track.state_timer        = 0;
    Track.tracking_started_at = 0;
    Track.last_corner_completed_at = 0;
    Track.corner_started_at = 0;
    Track.corner_phase       = TRACK_CORNER_APPROACH;
    Track.line_type          = TRACK_LINE_LOST;
    Track.line_mask          = 0u;
    Track.line_mask_history  = 0u;
    Track.last_centered_at   = 0u;
    Track.gyro_rate          = 0;
    Track.rotate_heading_baseline = 0;
    Track.rotate_target_deg  = 0;
    Track.rotate_actual_deg  = 0;
    Track.rotate_error_deg   = 0;
    Track.rotate_started_at  = 0;
    Track.rotate_timeout_ms  = 0;
    Track.rotate_settle_frames = 0;
    Track.rotate_result      = "idle";
    Track.heading_hold_target = 0;
    Track.heading_hold_actual = 0;
    Track.heading_hold_error = 0;
    Track.heading_hold_result = "idle";
    Track.motor_test_started_at = 0;
    Track.motor_test_stage_started_at = 0;
    Track.motor_test_left_start = 0;
    Track.motor_test_right_start = 0;
    Track.motor_test_phase = 0;
    Track.motor_test_target = 0;
    Track.motor_test_result = "idle";
    Track.motor_test_left_sum = 0;
    Track.motor_test_right_sum = 0;
    Track.motor_test_left_abs_sum = 0u;
    Track.motor_test_right_abs_sum = 0u;
    Track.motor_test_sample_count = 0u;
    Track.motor_test_prev_left_avg = 0;
    Track.motor_test_prev_right_avg = 0;
    Track.motor_test_scale_left_num = 0.0f;
    Track.motor_test_scale_right_num = 0.0f;
    Track.motor_test_scale_den = 0.0f;
    Track.motor_test_failed = false;
    Track.right_angle_confirm = 0;
    Track.corner_heading_settle_confirm = 0;
    Track.line_lost_confirm  = 0;
    Track.invalid_line_confirm = 0;
    Track.centered_history   = 0;
    Track.capture_confirm    = 0;
    Track.corner_left_start  = 0;
    Track.corner_right_start = 0;
    Track.target_circles     = TRACK_DEFAULT_CIRCLES;
    Track.current_circle     = 0;
    Track.right_angle_count  = 0;
    Track.turn_output        = 0;
    Track.left_speed         = 0;
    Track.right_speed        = 0;
    Track.base_speed         = TRACK_DEFAULT_BASE_SPEED;
    Track.target_speed       = TRACK_DEFAULT_BASE_SPEED;
    Track.use_gyro           = true;
    Track.use_motor          = true;
    Track.use_speed_pid      = USER_TRACK_SPEED_PID_ENABLE != 0;

    /* AI 调参运行时可调参数初始化（从宏默认值载入） */
    Track.gyro_damp             = TRACK_LINE_FOLLOW_GYRO_DAMP;
    Track.speed_straight        = TRACK_SPEED_STRAIGHT;
    Track.speed_curve           = TRACK_SPEED_CURVE;
    Track.right_angle_threshold = TRACK_RIGHT_ANGLE_THRESHOLD;

    /* PID 初始化（使用BSP层 bsp_pid_t） */
    Pid_Init(&Track.line_pid,
             TRACK_LINE_PID_KP, TRACK_LINE_PID_KI, TRACK_LINE_PID_KD,
             TRACK_LINE_PID_ILIMIT, TRACK_LINE_PID_OLIMIT);
    Track.line_pid.deriv_alpha = TRACK_LINE_PID_DERIV_ALPHA;  /* 不完全微分 */

    App_Grayscale_Init();
    if (App_Imu_Init()) {
        Imu_Calibration_Start();
    } else {
        App_Imu_Request_Recovery();
    }
    App_Motor_Init();

    /* ====== 灰度校准策略: 自动校准白色(一次) + 手动K4校准黑色 ======
     * 传感器硬件现状: 8通道极性不一致(部分反向)。
     * 方案:
     *   1. 开机时车放白地 → 自动校准每通道 white (只一次)
     *   2. black 初始化为 white 的一半 (假设黑线ADC约为白地一半)
     *      这样滞回区间大致在 [white/4, 3*white/4], 白地ADC>3*white/4判白
     *   3. 手动 K4 校准黑色 (车放黑线按K4, 直接覆盖black为真实值)
     *   4. 不做动态跟踪 (实测会把black拉低到0导致失效)
     * white 保持开机值不更新, black 保持初始值直到K4校准。
     * 滞回比较器通过 black/white 大小自适应极性。 */
    APP_TRACK_LOG_I("TRACK", "Gray auto-cal WHITE (place car on white surface):");
    /* 扫描3轮让ADC稳定 + 取每通道当前ADC作为white */
    for (uint8_t round = 0; round < 3; round++) {
        for (uint8_t i = 0; i < 8; i++) {
            Grayscale_Update();
            Bsp_Delay_ms(3);
        }
    }
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t adc = Grayscale_Sensor.analog[i];
        Grayscale_Sensor.white[i] = adc;
        /* black 初始化为 white/2 (合理猜测: 黑线ADC约为白地一半)
         * 用户按K4手动校准后会更新为真实黑线值 */
        Grayscale_Sensor.black[i] = adc / 2;
        Grayscale_Sensor.digital[i] = 0;
        APP_TRACK_LOG_I("TRACK", "  S%d: ADC=%u white=%u black=%u",
               i, adc, adc, Grayscale_Sensor.black[i]);
    }
    Grayscale_Sensor.calibrated = 1;

    /* 启动阶段保持电桥关闭；Track_Start 根据配置启用轮速PI。 */
    App_Motor_Enable_Speed_PID(false);
    Bsp_Motor_Write_Pwm(&Motor_Left, 0);
    Bsp_Motor_Write_Pwm(&Motor_Right, 0);
}

void Track_Update(void)
{
    track_line_observation_t observation;
    /* Safety_ControlTick() 已在外部 5ms 节拍调用并授权，
     * 本函数仅在 motor_allowed=true 时被调用。
     * 灰度扫描已由 main.c 顶层 Grayscale_Update() 推进。 */

    /* ① 真实 dt 计算 (Safety 节拍已保证 5ms 周期，此处测量实际间隔) */
    static uint32_t last_update_tick = 0;
    uint32_t now = Bsp_GetTick();
    uint32_t elapsed_ms = (last_update_tick == 0) ? TRACK_UPDATE_PERIOD_MS
                                                  : (uint32_t)(now - last_update_tick);
    if (elapsed_ms > 20u) elapsed_ms = 20u;
    float dt = elapsed_ms / 1000.0f;
    last_update_tick = now;

    /* 架空台架测试不依赖IMU或灰度，只使用左右编码器速度闭环。 */
    if (Track.state == TRACK_STATE_MOTOR_TEST) {
        Track_Handle_Motor_Test();
        return;
    }

    /* ② 读取 IMU 并更新航向。任何无效帧都在本控制节拍内撤销输出。 */
    if (Track.use_gyro) {
        if (!Imu_Update_Heading(dt) || !Track_Imu_Healthy()) {
            Track.gyro_rate = 0.0f;
            Track_Motor_Stop_Immediate();
            Safety_NotifySensorAbnormal();
            APP_TRACK_LOG_W("TRACK", "IMU invalid -> immediate motor stop");
            return;
        }
        /* 记录当前角速度 (已减零偏), 供 LINE_FOLLOW 阻尼和上位机遥测使用 */
        Track.gyro_rate = Imu_Sensor.gyro_z;
    }

    /* 原地旋转不能等待灰度帧，必须每个5ms控制周期闭环。 */
    if (Track.state == TRACK_STATE_ROTATE) {
        Track_Handle_Rotate();
        return;
    }
    if (Track.state == TRACK_STATE_HEADING_HOLD) {
        Track_Handle_Heading_Hold();
        return;
    }

    /* ③ 等待灰度传感器完成一轮完整扫描（数据同步） */
    if (!Grayscale_Sensor.data_ready) return;
    Grayscale_Sensor.data_ready = 0;  /* 消费标志 */

    /* ③.5 传感器健康度检测 → 通知 Safety 模块
     * 检测条件: 8路ADC全部低于基准一半(传感器抬起/悬空/被遮挡)
     * 处理: 通知 Safety，由 Safety 决定是否停车 (单一授权原则) */
    uint8_t abnormal_count = 0;
    for (uint8_t i = 0; i < BSP_GRAY_CHANNEL_NUM; i++) {
        uint16_t min_base = (Grayscale_Sensor.white[i] < Grayscale_Sensor.black[i])
                          ? Grayscale_Sensor.white[i] : Grayscale_Sensor.black[i];
        if (Grayscale_Sensor.analog[i] < (uint16_t)(min_base / 2)) {
            abnormal_count++;
        }
    }
    if (abnormal_count >= 6 && Track.state == TRACK_STATE_LINE_FOLLOW) {
        APP_TRACK_LOG_W("TRACK", "SENSOR_BAD n=%d A=%u,%u,%u,%u,%u,%u,%u,%u",
                        abnormal_count,
                        Grayscale_Sensor.analog[0], Grayscale_Sensor.analog[1],
                        Grayscale_Sensor.analog[2], Grayscale_Sensor.analog[3],
                        Grayscale_Sensor.analog[4], Grayscale_Sensor.analog[5],
                        Grayscale_Sensor.analog[6], Grayscale_Sensor.analog[7]);
        Safety_NotifySensorAbnormal();
        return;  /* Safety 下个 tick 会切断电机 */
    }

    observation = Track_ObserveLine();
    Track.line_mask = Track_GrayscaleMask();
    Track.line_mask_history = (Track.line_mask_history << 8) |
                              (uint32_t)Track.line_mask;
    Track.line_type = observation.type;

    /* 正常连续线才更新位置误差。无效线型仅保持上一有效误差1帧。 */
    Track_Calc_Error(observation);

    /* ⑥ 变速策略 */
    Track_Update_Speed();

    /* ⑦ 检测赛道元素 */
    Track_Detect_Pattern(observation);

    /* ⑧ 状态机处理 */
    switch (Track.state) {
    case TRACK_STATE_LINE_FOLLOW:
        /* 国一方案: LINE_FOLLOW 串级混合控制
         * 主控: 灰度 PID 输出位置转向量 (line_turn)
         * 阻尼: 陀螺仪角速度抑制灰度传感器噪声引起的左右抖动 (gyro_damp)
         * 极性: 实测左转 gyro_z>0，而左转差速输出<0，因此阻尼项取正号
         * 效果: 直行稳(抖动被阻尼), 弯道灵(角速度小阻尼小不影响跟踪)
         * IMU 失效时由本节拍入口立即停车，禁止使用陈旧航向。 */
        {
        float line_turn = Pid_Compute(&Track.line_pid, Track.error, 0.0f);
        float damping = (Track.use_gyro && Track_Imu_Healthy())
                      ? Track.gyro_damp : 0.0f;
        Track.turn_output = Track_ClampTurn(
            Track_LineTurn_WithGyroDamping(
                line_turn, Track.gyro_rate, damping),
            TRACK_RIGHT_ANGLE_TURN_LIMIT);
        }
        break;

    case TRACK_STATE_RIGHT_ANGLE:
        Track_Handle_RightAngle(observation);
        break;

    default:
        break;
    }

    if (!Track_State_Allows_Motor(Track.state)) {
        Track_Motor_Stop_Immediate();
        return;
    }

    /* ⑨ 差速输出到电机（BSP层API） */
    Track_Motor_Output();

    /* 速度PID闭环更新已移至 main.c 的 5ms 独立节拍，
     * 保证 IDLE/FINISHED 状态下也能更新(避免电机保持最后PWM)。 */
}

bool Track_Start(void)
{
    uint8_t start_mask = Track_GrayscaleMask();

    s_track_start_result = "rejected";
    if (!Track_Imu_Healthy()) {
        s_track_start_result = "imu_unavailable";
        APP_TRACK_LOG_W("TRACK", "Start rejected: imu_unavailable");
        return false;
    }

    if (!Grayscale_Sensor.black_calibrated) {
        s_track_start_result = "calibration_incomplete";
        APP_TRACK_LOG_W("TRACK", "Start rejected: calibration_incomplete");
        return false;
    }

    if (!Grayscale_Calibration_Valid(TRACK_GRAYSCALE_MIN_CAL_CONTRAST)) {
        s_track_start_result = "calibration_contrast_low";
        APP_TRACK_LOG_W("TRACK", "Start rejected: calibration_contrast_low min=%u",
                        TRACK_GRAYSCALE_MIN_CAL_CONTRAST);
        return false;
    }

    /* Safety 授权进入 RUNNING（校准未完成会被 Safety 拒绝） */
    Safety_TransitionToRunning();
    if (!Safety_AllowsMotorOutput()) {
        s_track_start_result = "safety_rejected";
        APP_TRACK_LOG_W("TRACK", "Start blocked by Safety (state=%s)",
               Safety_StateName(Safety_GetState()));
        return false;
    }

    Track.state = TRACK_STATE_LINE_FOLLOW;
    Track.tracking_started_at = Bsp_GetTick();
    Track.current_circle = 0;
    Track.right_angle_count = 0;
    Track.right_angle_confirm = 0;
    Track.corner_heading_settle_confirm = 0;
    Track.line_lost_confirm = 0;
    Track.invalid_line_confirm = 0;
    Track.centered_history = 0;
    Track.line_mask_history = 0u;
    Track.last_centered_at = Track.tracking_started_at;
    Track.capture_confirm = 0;
    Track.line_mask = start_mask;
    Track.line_type = TRACK_LINE_NORMAL;
    Track.corner_phase = TRACK_CORNER_APPROACH;
    Track.last_corner_completed_at = 0;
    Pid_Reset(&Track.line_pid);
    if (Track.use_motor) {
        Motor_Set_Speed(0);
        Motor_Right_Set_Speed(0);
        App_Motor_Enable_Speed_PID(Track.use_speed_pid);
    }
    APP_TRACK_LOG_I("TRACK", "Tracking started (circles=%d speed=%d black_cal=%d wheel_pi=%d)",
           Track.target_circles, Track.base_speed,
           Grayscale_Sensor.black_calibrated, Track.use_speed_pid ? 1 : 0);
    s_track_start_result = "started";
    return true;
}

const char *Track_Get_Start_Result(void)
{
    return s_track_start_result;
}

void Track_Stop(void)
{
    bool clear_fault = Safety_GetState() == SAFETY_STATE_FAULT;

    if (Track.state == TRACK_STATE_ROTATE) {
        Track_Rotate_End("stopped");
        if (clear_fault) Safety_ClearFault();
        return;
    }
    if (Track.state == TRACK_STATE_HEADING_HOLD) {
        Track_Heading_Hold_End("stopped");
        if (clear_fault) Safety_ClearFault();
        return;
    }
    if (Track.state == TRACK_STATE_MOTOR_TEST) {
        Track_Motor_Test_End("stopped");
        if (clear_fault) Safety_ClearFault();
        return;
    }

    Track.state = TRACK_STATE_FINISHED;
    Track.left_speed  = 0;
    Track.right_speed = 0;
    Track.turn_output = 0;
    if (Track.use_motor) {
        /* 修复: 闭环模式下 Motor_Set_Speed 只更新 target_speed，
         * 不写 PWM；而 Track_Update 在 FINISHED 状态直接 return，
         * 速度 PID 永远不会再执行 → 电机保持最后一次 PWM 持续运转。
         * 必须切开环 + 直写 PWM=0 才能真正停车。 */
        Motor_Set_Speed(0);
        Motor_Right_Set_Speed(0);
        App_Motor_Enable_Speed_PID(false);
        Bsp_Motor_Write_Pwm(&Motor_Left, 0);
        Bsp_Motor_Write_Pwm(&Motor_Right, 0);
    }
    if (clear_fault) {
        /* K2/stop 是人工确认：清除锁定故障，但保持电机关闭和 Track
         * FINISHED。再次运动仍必须重新通过 start 的全部校验。 */
        Safety_ClearFault();
    } else {
        Safety_TransitionToFinished();
        Safety_TransitionToReady();
    }
    APP_TRACK_LOG_I("TRACK", "Tracking stopped (state=FINISHED safety=%s)",
                    Safety_StateName(Safety_GetState()));
}

bool Track_Start_Rotation(float target_deg)
{
    uint32_t target_abs;

    Track.rotate_result = "rejected";
    if (!Track_Rotate_TargetValid(target_deg)) {
        Track.rotate_result = "invalid_angle";
        APP_ROTATE_LOG_W("ROTATE", "reject reason=invalid_angle target=%d", (int)target_deg);
        return false;
    }
    if (Track.state != TRACK_STATE_IDLE) {
        Track.rotate_result = "not_ready";
        APP_ROTATE_LOG_W("ROTATE", "reject reason=not_ready track_state=%d",
                         (int)Track.state);
        return false;
    }
    if (!Track_Imu_Healthy()) {
        Track.rotate_result = "imu_unavailable";
        APP_ROTATE_LOG_W("ROTATE", "reject reason=imu_unavailable");
        return false;
    }

    if (!Safety_TransitionToRotation() || !Safety_AllowsMotorOutput()) {
        Track.rotate_result = "safety_rejected";
        APP_ROTATE_LOG_W("ROTATE", "reject reason=safety_rejected");
        return false;
    }

    target_abs = (uint32_t)Track_Abs_Float(target_deg);
    Track.rotate_heading_baseline = Imu_Sensor.heading;
    Track.rotate_target_deg = target_deg;
    Track.rotate_actual_deg = 0;
    Track.rotate_error_deg = target_deg;
    Track.rotate_started_at = Bsp_GetTick();
    Track.rotate_timeout_ms = USER_ROTATE_TIMEOUT_BASE_MS +
                              target_abs * USER_ROTATE_TIMEOUT_PER_DEG_MS;
    Track.rotate_settle_frames = 0;
    Track.rotate_result = "running";
    Track.state = TRACK_STATE_ROTATE;
    Track.left_speed = 0;
    Track.right_speed = 0;
    if (Track.use_motor) {
        Motor_Set_Speed(0);
        Motor_Right_Set_Speed(0);
        App_Motor_Enable_Speed_PID(true);
    }
    APP_ROTATE_LOG_I("ROTATE", "start target=%d timeout=%lu motor=closed_loop",
                     (int)target_deg, (unsigned long)Track.rotate_timeout_ms);
    return true;
}

bool Track_Is_Rotating(void)
{
    return Track.state == TRACK_STATE_ROTATE;
}

void Track_Abort_Rotation(const char *result)
{
    if (Track.state == TRACK_STATE_ROTATE) {
        Track_Rotate_End(result != NULL ? result : "safety_fault");
    }
}

float Track_Get_Rotation_Target(void) { return Track.rotate_target_deg; }
float Track_Get_Rotation_Angle(void) { return Track.rotate_actual_deg; }
float Track_Get_Rotation_Error(void) { return Track.rotate_error_deg; }
const char *Track_Get_Rotation_Result(void) { return Track.rotate_result; }

bool Track_Start_Heading_Hold(void)
{
    Track.heading_hold_result = "rejected";
    if (Track.state != TRACK_STATE_IDLE) {
        Track.heading_hold_result = "not_ready";
        return false;
    }
    if (!Track_Imu_Healthy()) {
        Track.heading_hold_result = "imu_unavailable";
        return false;
    }
    if (!Safety_TransitionToRotation() || !Safety_AllowsMotorOutput()) {
        Track.heading_hold_result = "safety_rejected";
        return false;
    }

    Track.heading_hold_target = Imu_Sensor.heading;
    Track.heading_hold_actual = Imu_Sensor.heading;
    Track.heading_hold_error = 0;
    Track.heading_hold_result = "holding";
    Track.state = TRACK_STATE_HEADING_HOLD;
    Track.left_speed = 0;
    Track.right_speed = 0;
    if (Track.use_motor) {
        Motor_Set_Speed(0);
        Motor_Right_Set_Speed(0);
        App_Motor_Enable_Speed_PID(true);
    }
    APP_ROTATE_LOG_I("HOLD", "start target=%d motor=closed_loop",
                     (int)Track.heading_hold_target);
    return true;
}

bool Track_Stop_Heading_Hold(void)
{
    if (Track.state != TRACK_STATE_HEADING_HOLD) {
        return false;
    }
    Track_Heading_Hold_End("stopped");
    APP_ROTATE_LOG_I("HOLD", "stopped");
    return true;
}

bool Track_Is_Heading_Hold(void)
{
    return Track.state == TRACK_STATE_HEADING_HOLD;
}

float Track_Get_Heading_Hold_Target(void) { return Track.heading_hold_target; }
float Track_Get_Heading_Hold_Actual(void) { return Track.heading_hold_actual; }
float Track_Get_Heading_Hold_Error(void) { return Track.heading_hold_error; }
const char *Track_Get_Heading_Hold_Result(void) { return Track.heading_hold_result; }

bool Track_Start_Motor_Test(void)
{
    safety_state_t safety_state = Safety_GetState();

    if ((Track.state != TRACK_STATE_IDLE &&
         Track.state != TRACK_STATE_FINISHED) ||
        (safety_state != SAFETY_STATE_BOOT &&
         safety_state != SAFETY_STATE_READY)) {
        Track.motor_test_result = "not_ready";
        return false;
    }
    if (!Safety_TransitionToBenchTest()) {
        Track.motor_test_result = "safety_rejected";
        return false;
    }
    if (!Safety_AllowsMotorOutput()) {
        Safety_TransitionAfterBenchTest();
        Track.motor_test_result = "safety_rejected";
        return false;
    }

    Track.motor_test_started_at = Bsp_GetTick();
    Track.motor_test_stage_started_at = Track.motor_test_started_at;
    Track.motor_test_left_start = Motor_Left.total_position;
    Track.motor_test_right_start = Motor_Right.total_position;
    Track.motor_test_phase = 0u;
    Track.motor_test_target = TRACK_MOTOR_TEST_TARGET_LOW;
    Track.motor_test_result = "running";
    Track.left_speed = TRACK_MOTOR_TEST_TARGET_LOW;
    Track.right_speed = TRACK_MOTOR_TEST_TARGET_LOW;
    Track.turn_output = 0.0f;
    Track.state = TRACK_STATE_MOTOR_TEST;
    Track.motor_test_prev_left_avg = 0;
    Track.motor_test_prev_right_avg = 0;
    Track.motor_test_scale_left_num = 0.0f;
    Track.motor_test_scale_right_num = 0.0f;
    Track.motor_test_scale_den = 0.0f;
    Track.motor_test_failed = false;
    Track_Motor_Test_ResetSamples();

    App_Motor_Enable_Speed_PID(false);
    Motor_Set_Speed(TRACK_MOTOR_TEST_TARGET_LOW);
    Motor_Right_Set_Speed(TRACK_MOTOR_TEST_TARGET_LOW);
    APP_TRACK_LOG_I("MOTORTEST", "start phase=0 open_loop pwm=%d timeout=%lu",
                    TRACK_MOTOR_TEST_TARGET_LOW,
                    (unsigned long)TRACK_MOTOR_TEST_TIMEOUT_MS);
    return true;
}

bool Track_Is_Motor_Test(void)
{
    return Track.state == TRACK_STATE_MOTOR_TEST;
}

void Track_Abort_Motor_Test(const char *result)
{
    if (Track.state == TRACK_STATE_MOTOR_TEST) {
        Track_Motor_Test_End(result != NULL ? result : "safety_fault");
    }
}

uint8_t Track_Get_Motor_Test_Phase(void) { return Track.motor_test_phase; }
int32_t Track_Get_Motor_Test_Target(void) { return Track.motor_test_target; }
const char *Track_Get_Motor_Test_Result(void) { return Track.motor_test_result; }

void Track_Set_Circles(uint8_t n)
{
    if (n < 1u) n = 1u;
    if (n > 20u) n = 20u;
    Track.target_circles = n;
}

void Track_Set_Speed(int32_t base_speed)
{
    if (base_speed < 0) base_speed = 0;
    if (base_speed > 100) base_speed = 100;
    Track.base_speed   = base_speed;
    Track.target_speed = base_speed;
}

void Track_Set_TrackPID(float kp, float ki, float kd)
{
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd)) return;
    if (kp < 0.0f) kp = 0.0f;
    if (kp > 50.0f) kp = 50.0f;
    if (ki < 0.0f) ki = 0.0f;
    if (ki > 5.0f) ki = 5.0f;
    if (kd < 0.0f) kd = 0.0f;
    if (kd > 20.0f) kd = 20.0f;
    Pid_Set_Params(&Track.line_pid, kp, ki, kd);
}

void Track_Set_GyroDamp(float damp)
{
    if (!isfinite(damp)) return;
    if (damp < 0.0f) damp = 0.0f;
    if (damp > 1.0f) damp = 1.0f;   /* 钳位 0~1 */
    Track.gyro_damp = damp;
}

void Track_Set_SpeedStraight(int32_t spd)
{
    if (spd < 0)   spd = 0;
    if (spd > 100) spd = 100;        /* 钳位 0~100 */
    Track.speed_straight = spd;
}

void Track_Set_SpeedCurve(int32_t spd)
{
    if (spd < 0)   spd = 0;
    if (spd > 100) spd = 100;
    Track.speed_curve = spd;
}

void Track_Set_RightAngleThreshold(float thr)
{
    if (!isfinite(thr)) return;
    if (thr < 0.5f) thr = 0.5f;
    if (thr > 10.0f) thr = 10.0f;   /* 钳位 0.5~10 */
    Track.right_angle_threshold = thr;
}

void Track_Calibrate_White(void)
{
    Grayscale_Calibrate_White();
}

void Track_Calibrate_Black(void)
{
    Grayscale_Calibrate_Black();
}

track_state_t Track_Get_State(void)
{
    return Track.state;
}

uint8_t Track_Get_Circle(void)
{
    return Track.current_circle;
}
