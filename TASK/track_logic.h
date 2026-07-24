/**
 * @file    track_logic.h
 * @brief   Pure tracking decision helpers that can be tested on host.
 */
#ifndef TRACK_LOGIC_H
#define TRACK_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TRACK_STATE_IDLE = 0,
    TRACK_STATE_LINE_FOLLOW,
    TRACK_STATE_RIGHT_ANGLE,
    TRACK_STATE_LINE_LOST,
    TRACK_STATE_START_LINE,
    TRACK_STATE_CROSS,
    TRACK_STATE_ROTATE,
    TRACK_STATE_HEADING_HOLD,
    TRACK_STATE_MOTOR_TEST,
    TRACK_STATE_FINISHED
} track_state_t;

typedef enum {
    TRACK_CORNER_APPROACH = 0,
    TRACK_CORNER_STOP,
    TRACK_CORNER_PIVOT,
    TRACK_CORNER_CAPTURE
} track_corner_phase_t;

typedef enum {
    TRACK_LINE_NORMAL = 0,
    TRACK_LINE_LEFT_CORNER,
    TRACK_LINE_RIGHT_CORNER,
    TRACK_LINE_LOST,
    TRACK_LINE_INVALID
} track_line_type_t;

typedef struct {
    track_line_type_t type;
    float error;
    uint8_t black_count;
} track_line_observation_t;

#ifndef TRACK_RIGHT_ANGLE_TURN_LIMIT
#define TRACK_RIGHT_ANGLE_TURN_LIMIT      10.0f
#endif
#ifndef TRACK_RIGHT_ANGLE_DONE_ERROR
#define TRACK_RIGHT_ANGLE_DONE_ERROR       3.0f
#endif
#ifndef TRACK_RIGHT_ANGLE_DONE_RATE
#define TRACK_RIGHT_ANGLE_DONE_RATE       12.0f
#endif
#ifndef TRACK_RIGHT_ANGLE_DONE_FRAMES
#define TRACK_RIGHT_ANGLE_DONE_FRAMES      4u
#endif

static inline float Track_Abs_Float(float v)
{
    return (v >= 0.0f) ? v : -v;
}

#define TRACK_ROTATE_MIN_DEG       1.0f
#define TRACK_ROTATE_MAX_DEG       180.0f
#ifndef TRACK_ROTATE_DONE_ERR_DEG
#define TRACK_ROTATE_DONE_ERR_DEG  3.0f
#endif
#define TRACK_ROTATE_SETTLE_RATE   6.0f

static inline bool Track_Rotate_TargetValid(float target_deg)
{
    float magnitude;

    if (target_deg != target_deg) return false;
    magnitude = Track_Abs_Float(target_deg);
    return magnitude >= TRACK_ROTATE_MIN_DEG &&
           magnitude <= TRACK_ROTATE_MAX_DEG;
}

static inline float Track_Rotate_Error(float heading_baseline,
                                       float heading,
                                       float target_deg)
{
    return target_deg - (heading - heading_baseline);
}

static inline bool Track_Rotate_AngleInside(float angle_error_deg)
{
    return Track_Abs_Float(angle_error_deg) <= TRACK_ROTATE_DONE_ERR_DEG;
}

static inline bool Track_Rotate_Complete(float angle_error_deg,
                                          float gyro_abs_deg_s)
{
    return Track_Rotate_AngleInside(angle_error_deg) &&
           gyro_abs_deg_s <= TRACK_ROTATE_SETTLE_RATE;
}

static inline uint8_t Track_Rotate_UpdateSettleFrames(uint8_t current,
                                                       bool inside_tolerance)
{
    if (!inside_tolerance) return 0u;
    return (current < 255u) ? (uint8_t)(current + 1u) : current;
}

static inline bool Track_Rotate_Settled(uint8_t settle_frames,
                                          uint8_t required_frames)
{
    return required_frames > 0u && settle_frames >= required_frames;
}

static inline bool Track_Rotate_ShouldCoast(float angle_error_deg,
                                             float gyro_rate_deg_s,
                                             float output,
                                             float coast_output)
{
    bool moving_toward_target = angle_error_deg * gyro_rate_deg_s > 0.0f;
    return moving_toward_target &&
           Track_Abs_Float(output) < coast_output;
}

static inline bool Track_Rotate_StallCandidate(float angle_error_deg,
                                                float gyro_rate_deg_s,
                                                int32_t left_encoder_speed,
                                                int32_t right_encoder_speed,
                                                float min_error_deg,
                                                float max_error_deg,
                                                float stall_rate_deg_s,
                                                int32_t encoder_idle_max)
{
    float error_abs = Track_Abs_Float(angle_error_deg);
    return min_error_deg >= 0.0f && max_error_deg >= min_error_deg &&
           encoder_idle_max >= 0 && error_abs >= min_error_deg &&
           error_abs <= max_error_deg &&
           Track_Abs_Float(gyro_rate_deg_s) <= stall_rate_deg_s &&
           left_encoder_speed >= -encoder_idle_max &&
           left_encoder_speed <= encoder_idle_max &&
           right_encoder_speed >= -encoder_idle_max &&
           right_encoder_speed <= encoder_idle_max;
}

static inline bool Track_Rotate_RecoveryMoving(float gyro_rate_deg_s,
                                                int32_t left_encoder_speed,
                                                int32_t right_encoder_speed,
                                                float release_rate_deg_s,
                                                int32_t release_encoder_speed)
{
    return Track_Abs_Float(gyro_rate_deg_s) > release_rate_deg_s ||
           left_encoder_speed < -release_encoder_speed ||
           left_encoder_speed > release_encoder_speed ||
           right_encoder_speed < -release_encoder_speed ||
           right_encoder_speed > release_encoder_speed;
}

static inline float Track_Rotate_SpeedLimit(float error_abs_deg,
                                             float fine_zone_deg,
                                             float blend_zone_deg,
                                             float fine_max_speed,
                                             float max_speed)
{
    if (error_abs_deg <= fine_zone_deg) return fine_max_speed;
    if (blend_zone_deg <= fine_zone_deg || error_abs_deg >= blend_zone_deg) {
        return max_speed;
    }

    float ratio = (error_abs_deg - fine_zone_deg) /
                  (blend_zone_deg - fine_zone_deg);
    return fine_max_speed + ratio * (max_speed - fine_max_speed);
}

static inline void Track_Rotate_Command(float target_deg,
                                        int32_t speed,
                                        int32_t *left_speed,
                                        int32_t *right_speed)
{
    if (left_speed == 0 || right_speed == 0) return;

    if (target_deg > 0.0f) {
        *left_speed = -speed;
        *right_speed = speed;
    } else {
        *left_speed = speed;
        *right_speed = -speed;
    }
}

static inline int32_t Track_Slew_Command(int32_t current,
                                          int32_t target,
                                          int32_t step)
{
    /* Zero means coast. Do not walk through tiny nonzero targets because the
     * motor loop converts them to its minimum startup PWM. */
    if (target == 0) return 0;
    if (step <= 0) return target;
    if (current < target) {
        current += step;
        return (current > target) ? target : current;
    }
    if (current > target) {
        current -= step;
        return (current < target) ? target : current;
    }
    return current;
}

static inline bool Track_Rotate_Timeout(uint32_t now,
                                        uint32_t started_at,
                                        uint32_t timeout_ms)
{
    return (uint32_t)(now - started_at) >= timeout_ms;
}

static inline bool Track_State_Allows_Motor(track_state_t state)
{
    return state != TRACK_STATE_IDLE && state != TRACK_STATE_FINISHED;
}

static inline bool Track_State_Is_LineTracking(track_state_t state)
{
    return state >= TRACK_STATE_LINE_FOLLOW && state <= TRACK_STATE_CROSS;
}

static inline bool Track_Timeout_Reached(uint32_t now,
                                          uint32_t started_at,
                                          uint32_t timeout_ms)
{
    return (uint32_t)(now - started_at) >= timeout_ms;
}

static inline bool Track_CornerArmed(uint32_t now,
                                     uint32_t tracking_started_at,
                                     uint32_t last_corner_completed_at,
                                     uint16_t completed_corners,
                                     uint32_t arm_delay_ms,
                                     uint32_t retrigger_delay_ms)
{
    bool initial_delay_done = Track_Timeout_Reached(
        now, tracking_started_at, arm_delay_ms);
    bool retrigger_delay_done = completed_corners == 0u ||
        Track_Timeout_Reached(now, last_corner_completed_at,
                              retrigger_delay_ms);
    return initial_delay_done && retrigger_delay_done;
}

static inline float Track_LineTurn_WithGyroDamping(float line_turn,
                                                    float gyro_rate,
                                                    float damping)
{
    /* Wheel differential and gyro heading use opposite signs on this chassis. */
    return line_turn + gyro_rate * damping;
}

static inline uint8_t Track_CountBits8(uint8_t value)
{
    uint8_t count = 0;
    while (value != 0u) {
        count = (uint8_t)(count + (value & 1u));
        value >>= 1;
    }
    return count;
}

static inline float Track_ComputeDigitalLineError8(uint8_t digital_mask,
                                                    float fallback)
{
    float weighted_sum = 0.0f;
    uint8_t black_count = 0u;

    if (digital_mask == 0u) return fallback;

    for (uint8_t i = 0u; i < 8u; i++) {
        if ((digital_mask & (uint8_t)(1u << i)) == 0u) continue;

        weighted_sum += ((float)i * 2.0f - 7.0f) * 0.5f;
        black_count++;
    }

    return weighted_sum / (float)black_count;
}

static inline uint8_t Track_CountBits12(uint16_t value)
{
    uint8_t count = 0u;

    value &= 0x0FFFu;
    while (value != 0u) {
        count = (uint8_t)(count + (uint8_t)(value & 1u));
        value >>= 1;
    }
    return count;
}

static inline float Track_ComputeDigitalLineError12(uint16_t digital_mask,
                                                     float fallback)
{
    float weighted_sum = 0.0f;
    uint8_t black_count = 0u;

    digital_mask &= 0x0FFFu;
    if (digital_mask == 0u) return fallback;

    for (uint8_t i = 0u; i < 12u; i++) {
        if ((digital_mask & (uint16_t)(1u << i)) == 0u) continue;

        /* A1..A12 are vehicle left..right. Keep the legacy controller's
         * -3.5..+3.5 input scale while increasing spatial resolution. */
        weighted_sum += -3.5f + (float)i * (7.0f / 11.0f);
        black_count++;
    }

    return weighted_sum / (float)black_count;
}

static inline float Track_LimitRecoveryError(float error, float limit)
{
    float magnitude = Track_Abs_Float(limit);
    if (error > magnitude) return magnitude;
    if (error < -magnitude) return -magnitude;
    return error;
}

static inline bool Track_MaskIsContiguous(uint8_t digital_mask)
{
    if (digital_mask == 0u) return false;

    while ((digital_mask & 1u) == 0u) digital_mask >>= 1;
    while ((digital_mask & 1u) != 0u) digital_mask >>= 1;
    return digital_mask == 0u;
}

static inline bool Track_MaskIsContiguous12(uint16_t digital_mask)
{
    digital_mask &= 0x0FFFu;
    if (digital_mask == 0u) return false;

    while ((digital_mask & 1u) == 0u) digital_mask >>= 1;
    while ((digital_mask & 1u) != 0u) digital_mask >>= 1;
    return digital_mask == 0u;
}

static inline uint8_t Track_MirrorMask8(uint8_t digital_mask)
{
    digital_mask = (uint8_t)(((digital_mask & 0x55u) << 1) |
                             ((digital_mask & 0xAAu) >> 1));
    digital_mask = (uint8_t)(((digital_mask & 0x33u) << 2) |
                             ((digital_mask & 0xCCu) >> 2));
    return (uint8_t)((digital_mask << 4) | (digital_mask >> 4));
}

static inline bool Track_SquareLeftCorner(uint8_t digital_mask,
                                           uint8_t min_black)
{
    uint8_t black_count = Track_CountBits8(digital_mask);
    bool enough_black = black_count >= min_black;
    bool strict_measured_corner = digital_mask == 0x0Fu ||
                                  digital_mask == 0x1Fu;
    bool center_pair = (digital_mask & (1u << 3)) != 0u &&
                       (digital_mask & (1u << 4)) != 0u;
    bool left_chain = center_pair &&
                      (digital_mask & (1u << 2)) != 0u &&
                      (digital_mask & 0x03u) != 0u;
    bool right_chain = center_pair &&
                       (digital_mask & (1u << 5)) != 0u &&
                       (digital_mask & 0xC0u) != 0u;
    bool ideal_left_corner =
        digital_mask != 0xFFu && Track_MaskIsContiguous(digital_mask) &&
        left_chain && !right_chain;

    /* Real-car L-corner evidence: the far-left branch reaches S0 while the
     * incoming line still covers S3.  Four-or-more black channels are still
     * required by enough_black, so sparse 0x09/0x0D transition noise cannot
     * directly trigger a turn; measured 0x0F/0x1B remain valid. */
    bool split_left_corner =
        (digital_mask & 0x01u) != 0u &&
        (digital_mask & 0x08u) != 0u &&
        (digital_mask & 0xE0u) == 0u &&
        black_count <= 5u;

    return enough_black &&
           (strict_measured_corner || ideal_left_corner || split_left_corner);
}

static inline bool Track_SquareRightCorner(uint8_t digital_mask,
                                            uint8_t min_black)
{
    return Track_SquareLeftCorner(Track_MirrorMask8(digital_mask), min_black);
}

/* On the counter-clockwise square track, four-or-more black channels after a
 * recently centered line are sufficient corner evidence. Direction is fixed
 * by the course, so a noisy physical channel order (for example 0x87) must not
 * be allowed to reverse or suppress the turn. Full black remains invalid. */
static inline bool Track_SquareCornerEvidence(uint8_t digital_mask,
                                               uint8_t min_black)
{
    return digital_mask != 0u && digital_mask != 0xFFu &&
           Track_CountBits8(digital_mask) >= min_black;
}

static inline bool Track_SquareCornerEvidence12(uint16_t digital_mask,
                                                 uint8_t min_black)
{
    digital_mask &= 0x0FFFu;
    return digital_mask != 0u && digital_mask != 0x0FFFu &&
           Track_CountBits12(digital_mask) >= min_black;
}

static inline track_line_observation_t Track_ClassifyLine8(
    uint8_t digital_mask, float fallback, uint8_t corner_min_black)
{
    track_line_observation_t observation;

    observation.black_count = Track_CountBits8(digital_mask);
    observation.error = fallback;

    if (digital_mask == 0u) {
        observation.type = TRACK_LINE_LOST;
        return observation;
    }
    if (Track_SquareLeftCorner(digital_mask, corner_min_black)) {
        observation.type = TRACK_LINE_LEFT_CORNER;
        observation.error = Track_ComputeDigitalLineError8(digital_mask,
                                                            fallback);
        return observation;
    }
    if (Track_SquareRightCorner(digital_mask, corner_min_black)) {
        observation.type = TRACK_LINE_RIGHT_CORNER;
        observation.error = Track_ComputeDigitalLineError8(digital_mask,
                                                            fallback);
        return observation;
    }
    if (observation.black_count <= 3u &&
        Track_MaskIsContiguous(digital_mask)) {
        observation.type = TRACK_LINE_NORMAL;
        observation.error = Track_ComputeDigitalLineError8(digital_mask,
                                                            fallback);
        return observation;
    }

    observation.type = TRACK_LINE_INVALID;
    return observation;
}

static inline track_line_observation_t Track_ClassifyLine12(
    uint16_t digital_mask, float fallback, uint8_t corner_min_black)
{
    track_line_observation_t observation;

    digital_mask &= 0x0FFFu;
    observation.black_count = Track_CountBits12(digital_mask);
    observation.error = fallback;

    if (digital_mask == 0u) {
        observation.type = TRACK_LINE_LOST;
        return observation;
    }
    if (digital_mask == 0x0FFFu) {
        observation.type = TRACK_LINE_INVALID;
        return observation;
    }
    if (Track_SquareCornerEvidence12(digital_mask, corner_min_black)) {
        observation.type = TRACK_LINE_LEFT_CORNER;
        observation.error = Track_ComputeDigitalLineError12(digital_mask,
                                                             fallback);
        return observation;
    }
    if (observation.black_count <= 4u &&
        Track_MaskIsContiguous12(digital_mask)) {
        observation.type = TRACK_LINE_NORMAL;
        observation.error = Track_ComputeDigitalLineError12(digital_mask,
                                                             fallback);
        return observation;
    }

    observation.type = TRACK_LINE_INVALID;
    return observation;
}

static inline int8_t Track_CornerTurnSign(track_line_type_t type)
{
    if (type == TRACK_LINE_LEFT_CORNER) return 1;
    if (type == TRACK_LINE_RIGHT_CORNER) return -1;
    return 0;
}

static inline int8_t Track_CornerDirectionFromMask(uint8_t digital_mask)
{
    uint8_t black_count = Track_CountBits8(digital_mask);
    bool center_anchor = (digital_mask & 0x18u) != 0u;
    bool far_left = (digital_mask & 0x03u) != 0u;
    bool far_right = (digital_mask & 0xC0u) != 0u;

    /* S2/S5仍属于中心附近，不能仅凭0x1C/0x38的轻微偏线决定90度方向。
     * 全白兜底至少需要4个黑点、中心入口锚点和仅一侧远端支路；
     * 0x09/0x0D等稀疏噪声、全黑及双侧展开均不得留下方向证据。 */
    if (black_count < 4u || !center_anchor || far_left == far_right) return 0;
    if (far_left) return 1;
    if (far_right) return -1;
    return 0;
}

static inline int8_t Track_CornerDirectionFromMask12(uint16_t digital_mask)
{
    uint8_t black_count;
    bool center_anchor;
    bool far_left;
    bool far_right;

    digital_mask &= 0x0FFFu;
    black_count = Track_CountBits12(digital_mask);
    center_anchor = (digital_mask & 0x0060u) != 0u; /* A6/A7 */
    far_left = (digital_mask & 0x0007u) != 0u;      /* A1..A3 */
    far_right = (digital_mask & 0x0E00u) != 0u;     /* A10..A12 */

    if (black_count < 6u || !center_anchor || far_left == far_right) return 0;
    return far_left ? 1 : -1;
}

static inline bool Track_CornerEvidenceRecent(int8_t turn_sign,
                                               uint32_t now,
                                               uint32_t evidence_at,
                                               uint32_t recent_ms)
{
    return (turn_sign == 1 || turn_sign == -1) &&
           (uint32_t)(now - evidence_at) <= recent_ms;
}

static inline bool Track_CenteredEvidenceRecent(bool has_evidence,
                                                 uint32_t now,
                                                 uint32_t evidence_at,
                                                 uint32_t recent_ms)
{
    return has_evidence &&
           (uint32_t)(now - evidence_at) <= recent_ms;
}

static inline uint8_t Track_UpdateCornerConfirm(uint8_t current,
                                                 uint8_t centered_history,
                                                 bool corner_candidate)
{
    if (!corner_candidate || centered_history < 2u) return 0u;
    return (current < 255u) ? (uint8_t)(current + 1u) : current;
}

static inline bool Track_CornerCandidateReady(bool gyro_available,
                                               bool corner_armed,
                                               bool centered_recent,
                                               uint8_t confirm_frames,
                                               uint8_t required_frames)
{
    return gyro_available && corner_armed && centered_recent &&
           required_frames > 0u && confirm_frames >= required_frames;
}

static inline uint8_t Track_UpdateCornerGate(uint8_t current,
                                              bool corner_candidate,
                                              bool gyro_available,
                                              bool corner_armed,
                                              bool centered_recent)
{
    bool gate_open = corner_candidate && gyro_available && corner_armed &&
                     centered_recent;
    return Track_UpdateCornerConfirm(current, gate_open ? 2u : 0u, gate_open);
}

static inline bool Track_WhiteCornerFallbackReady(uint8_t centered_history,
                                                   uint8_t white_frames,
                                                   uint8_t required_white_frames,
                                                   uint32_t now,
                                                   uint32_t last_centered_at,
                                                   uint32_t recent_ms)
{
    return required_white_frames > 0u && centered_history >= 2u &&
           white_frames >= required_white_frames &&
           (uint32_t)(now - last_centered_at) <= recent_ms;
}

static inline float Track_CornerTargetHeading(float entry_heading,
                                               int8_t turn_sign,
                                               float angle_magnitude)
{
    float magnitude = Track_Abs_Float(angle_magnitude);
    if (turn_sign > 0) return entry_heading + magnitude;
    if (turn_sign < 0) return entry_heading - magnitude;
    return entry_heading;
}

static inline float Track_CornerTurnProgress(float entry_heading,
                                              float actual_heading,
                                              int8_t turn_sign)
{
    if (turn_sign == 0) return 0.0f;
    return (actual_heading - entry_heading) * (float)turn_sign;
}

static inline bool Track_CornerCapturePoseValid(float turn_progress_deg,
                                                 float angle_error_deg,
                                                 float min_progress_deg,
                                                 float max_error_deg)
{
    return turn_progress_deg >= min_progress_deg &&
           Track_Abs_Float(angle_error_deg) <= max_error_deg;
}

static inline bool Track_CaptureLineValid(track_line_observation_t observation,
                                           float max_error)
{
    return observation.type == TRACK_LINE_NORMAL &&
           Track_Abs_Float(observation.error) <= max_error;
}

static inline int32_t Track_EncoderDelta(int32_t current, int32_t start)
{
    return (int32_t)((uint32_t)current - (uint32_t)start);
}

static inline bool Track_EncoderDistanceReached(int32_t left_current,
                                                 int32_t right_current,
                                                 int32_t left_start,
                                                 int32_t right_start,
                                                 float left_counts_per_mm,
                                                 float right_counts_per_mm,
                                                 float target_mm)
{
    int32_t left_delta;
    int32_t right_delta;
    float average_mm;

    if (left_counts_per_mm <= 0.0f || right_counts_per_mm <= 0.0f ||
        target_mm <= 0.0f) return false;

    left_delta = Track_EncoderDelta(left_current, left_start);
    right_delta = Track_EncoderDelta(right_current, right_start);
    if (left_delta < 0 || right_delta < 0) return false;

    average_mm = ((float)left_delta / left_counts_per_mm +
                  (float)right_delta / right_counts_per_mm) * 0.5f;
    return average_mm >= target_mm;
}

static inline uint8_t Track_UpdateConfirmFrames(uint8_t current,
                                                 bool confirmed)
{
    if (!confirmed) return 0u;
    return (current < 255u) ? (uint8_t)(current + 1u) : current;
}

static inline float Track_ClampTurn(float turn_output, float limit)
{
    if (turn_output > limit) return limit;
    if (turn_output < -limit) return -limit;
    return turn_output;
}

static inline float Track_RightAngle_PivotTurn(float angle_error_deg,
                                               float gyro_rate_deg_s,
                                               float kp,
                                               float kd_rate,
                                               float limit)
{
    float heading_output = kp * angle_error_deg - kd_rate * gyro_rate_deg_s;
    return Track_ClampTurn(-heading_output, limit);
}

static inline float Track_CaptureBlendTurn(float heading_turn,
                                            float line_error,
                                            bool line_visible,
                                            float line_kp,
                                            float line_limit,
                                            float limit)
{
    float line_turn = line_visible
                    ? Track_ClampTurn(line_kp * line_error, line_limit)
                    : 0.0f;
    return Track_ClampTurn(heading_turn + line_turn, limit);
}

static inline float Track_CaptureHeadingPriority(float angle_error_deg,
                                                  float heading_turn,
                                                  float blended_turn,
                                                  float heading_deadband_deg)
{
    if (Track_Abs_Float(angle_error_deg) <= heading_deadband_deg) {
        return blended_turn;
    }
    /* turn>0=右转，正角度误差需要左转(turn<0)，负误差反之。
     * 灰度可以减弱航向修正，但不能在尚未到位时把方向反过来。 */
    if ((angle_error_deg > 0.0f && blended_turn >= 0.0f) ||
        (angle_error_deg < 0.0f && blended_turn <= 0.0f)) {
        return heading_turn;
    }
    return blended_turn;
}

static inline bool Track_CornersComplete(uint16_t completed_corners,
                                          uint8_t target_circles)
{
    return completed_corners >= (uint16_t)target_circles * 4u;
}

static inline uint8_t Track_CirclesFromCorners(uint16_t completed_corners,
                                                uint8_t target_circles)
{
    uint16_t circles = completed_corners / 4u;
    return (circles > target_circles) ? target_circles : (uint8_t)circles;
}

static inline bool Track_RightAngle_HeadingSettled(float angle_error_deg,
                                                   float gyro_rate_deg_s)
{
    return Track_Abs_Float(angle_error_deg) <= TRACK_RIGHT_ANGLE_DONE_ERROR &&
           Track_Abs_Float(gyro_rate_deg_s) <= TRACK_RIGHT_ANGLE_DONE_RATE;
}

static inline bool Track_RightAngle_HybridHandoff(float turn_progress_deg,
                                                   float angle_error_deg,
                                                   float gyro_rate_deg_s,
                                                   uint8_t line_frames,
                                                   float min_line_turn_deg,
                                                   float handoff_turn_deg,
                                                   float max_line_error_deg,
                                                   float max_angle_error_deg,
                                                   float max_rate_deg_s,
                                                   uint8_t required_line_frames)
{
    bool rate_ready = Track_Abs_Float(gyro_rate_deg_s) <= max_rate_deg_s;
    bool line_ready = turn_progress_deg >= min_line_turn_deg &&
                      Track_Abs_Float(angle_error_deg) <= max_line_error_deg &&
                      line_frames >= required_line_frames;
    bool angle_ready = turn_progress_deg >= handoff_turn_deg &&
                       Track_Abs_Float(angle_error_deg) <= max_angle_error_deg;
    return rate_ready && (line_ready || angle_ready);
}

#endif /* TRACK_LOGIC_H */
