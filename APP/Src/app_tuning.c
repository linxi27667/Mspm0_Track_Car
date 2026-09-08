/**
 * @file    app_tuning.c
 * @brief   TC1 远程调参协议层实现
 *
 *          使用 app_tc1 的零堆分配字节流解帧器处理入站 TC1 帧。
 *          完整帧解码后按消息类型派发: auth/set/cmd/ping。
 *          遥测以 10Hz TC1 帧上报, 带序列号/时间戳/鱼眼帧龄/错误/速度/参数版本。
 *          set/cmd 带单调 id, MCU 原子应用并回传完整参数快照 (ack)。
 *
 * @note    RACE 模式 (APP_TUNING_MODE=0) 编译禁用入站命令处理。
 */
#include "app_tuning.h"
#include "app_tc1.h"
#include "app_esp01s.h"
#include "app_safety.h"
#include "app_common.h"
#include "app_log.h"
#include "track.h"
#include "imu_health_logic.h"
#include "bsp_common.h"
#include "bsp_path_fish.h"
#include "bsp_encoder_motor.h"
#include "user_tuning.h"
#include "elog.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#undef LOG_TAG
#define LOG_TAG  "tuning"

/* ============================================================
 * 内部状态
 * ============================================================ */
static tc1_parser_t s_parser;           /* TC1 字节流解帧器 */
static uint32_t     s_last_telem_tick = 0;
static uint32_t     s_telem_seq       = 0;
static bool         s_authenticated   = false;

/* 静态缓冲区 (避免栈分配) */
static char s_telem_json[TC1_MAX_JSON_BYTES + 1]; /* 遥测 JSON 组包缓冲 */
static char s_ack_json[384];            /* ACK JSON 组包缓冲 */
static char s_cached_ack_json[384];     /* 最近 SET/CMD ACK，供同 ID 重试重放 */
static uint16_t s_cached_ack_len = 0;
static uint32_t s_cached_ack_id = 0;
static bool s_cached_ack_valid = false;
static tc1_msg_type_t s_cached_ack_type = TC1_MSG_UNKNOWN;
static uint32_t s_cached_request_crc = 0;
static tc1_msg_type_t s_dispatch_type = TC1_MSG_UNKNOWN;
static uint32_t s_dispatch_request_crc = 0;
static char s_auth_session[33];
static bool s_auth_session_valid = false;
static uint8_t s_tx_frame[TC1_MAX_FRAME_BYTES + 1]; /* TC1 编码输出缓冲 */

/* ============================================================
 * 工具函数
 * ============================================================ */

/* 状态枚举 → 2 字母缩写 */
static const char *state_abbr(track_state_t st)
{
    switch (st) {
    case TRACK_STATE_IDLE:        return "ID";
    case TRACK_STATE_LINE_FOLLOW: return "LF";
    case TRACK_STATE_RIGHT_ANGLE: return "RA";
    case TRACK_STATE_LINE_LOST:   return "LL";
    case TRACK_STATE_START_LINE:  return "SL";
    case TRACK_STATE_CROSS:       return "CR";
    case TRACK_STATE_ROTATE:      return "RT";
    case TRACK_STATE_HEADING_HOLD:return "HH";
    case TRACK_STATE_MOTOR_TEST:  return "MT";
    case TRACK_STATE_FINISHED:    return "FN";
    default:                      return "??";
    }
}

/* 固件侧参数钳位 (与 host/param_schema.py 范围一致) */
static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t clamp_i(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 在 JSON 中查找 "key":数字 模式 (作用于 TC1 已校验的 payload) */
static bool find_num(const char *json, const char *key, float *out)
{
    char pattern[16];
    (void)snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t') p++;
    if (*p != ':') return false;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    *out = (float)atof(p);
    return true;
}

/* 在 JSON 中查找 "key":"value" 字符串值 */
static bool find_str(const char *json, const char *key, char *out, uint8_t out_size)
{
    char pattern[16];
    (void)snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = json;

    /* The value of another field can equal key (for example
     * {"t":"cmd","cmd":"rotate"}). Skip it and keep looking for a
     * matching JSON key followed by ':'. */
    while ((p = strstr(p, pattern)) != NULL) {
        const char *value = p + strlen(pattern);
        while (*value == ' ' || *value == '\t') value++;
        if (*value != ':') {
            p = value;
            continue;
        }
        value++;
        while (*value == ' ' || *value == '\t') value++;
        if (*value != '"') return false;
        value++;
        uint8_t i = 0;
        while (*value && *value != '"' && i < (out_size - 1)) {
            out[i++] = *value++;
        }
        out[i] = '\0';
        return (*value == '"');
    }
    return false;
}

static bool find_bool(const char *json, const char *key, bool *out)
{
    char pattern[16];
    const char *p;

    (void)snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    p = strstr(json, pattern);
    if (p == NULL) return false;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t') p++;
    if (*p != ':') return false;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    if (strncmp(p, "true", 4u) == 0) {
        *out = true;
        return true;
    }
    if (strncmp(p, "false", 5u) == 0) {
        *out = false;
        return true;
    }
    return false;
}

/* 发送 TC1 帧 (JSON → TC1 编码 → ESP01S 发送) */
static void send_tc1_json(const char *json, uint16_t json_len)
{
    uint16_t frame_len = App_TC1_EncodeFrame(json, json_len,
                                             s_tx_frame, sizeof(s_tx_frame));
    if (frame_len == 0) {
        APP_TUNING_LOG_W(LOG_TAG, "encode failed, json_len=%u", json_len);
        return;
    }
    (void)App_ESP01S_SendJSON(s_tx_frame, frame_len);
}

/* ============================================================
 * ACK 组包与发送
 * ============================================================ */

/* 构建线环、速度策略和电机环的完整 applied 快照。 */
static uint16_t build_applied_snapshot(char *buf, uint16_t size)
{
    int n = snprintf(buf, size,
        "{\"KP\":%.3f,\"KI\":%.3f,\"KD\":%.3f,\"GDAMP\":%.3f,"
        "\"VBASE\":%d,\"VSTR\":%d,\"VCUR\":%d,\"RATHR\":%.3f,"
        "\"MKP\":%.3f,\"MKI\":%.3f,\"MKD\":%.3f,"
        "\"MSL\":%.3f,\"MSR\":%.3f}",
        (double)Track.line_pid.kp,
        (double)Track.line_pid.ki,
        (double)Track.line_pid.kd,
        (double)Track.gyro_damp,
        (int)Track.base_speed,
        (int)Track.speed_straight,
        (int)Track.speed_curve, (double)Track.right_angle_threshold,
        (double)App_Motor_Get_Speed_PID_Kp(),
        (double)App_Motor_Get_Speed_PID_Ki(),
        (double)App_Motor_Get_Speed_PID_Kd(),
        (double)App_Motor_Get_Speed_Scale_Left(),
        (double)App_Motor_Get_Speed_Scale_Right());
    if (n < 0 || n >= (int)size) return 0;
    return (uint16_t)n;
}

/* 发送 ACK 帧 */
static void send_ack(uint32_t msg_id, bool has_id, bool ok,
                     const char *kind, const char *error)
{
    char applied[256];
    uint16_t applied_len = 0;
    if (ok && strcmp(kind, "set") == 0) {
        applied_len = build_applied_snapshot(applied, sizeof(applied));
    }

    int n;
    if (ok && applied_len > 0) {
        if (has_id) {
            n = snprintf(s_ack_json, sizeof(s_ack_json),
                "{\"v\":1,\"t\":\"ack\",\"id\":%lu,\"ok\":true,\"kind\":\"%s\",\"applied\":%s}",
                (unsigned long)msg_id, kind, applied);
        } else {
            n = snprintf(s_ack_json, sizeof(s_ack_json),
                "{\"v\":1,\"t\":\"ack\",\"ok\":true,\"kind\":\"%s\",\"applied\":%s}",
                kind, applied);
        }
    } else if (ok) {
        if (has_id) {
            n = snprintf(s_ack_json, sizeof(s_ack_json),
                "{\"v\":1,\"t\":\"ack\",\"id\":%lu,\"ok\":true,\"kind\":\"%s\"}",
                (unsigned long)msg_id, kind);
        } else {
            n = snprintf(s_ack_json, sizeof(s_ack_json),
                "{\"v\":1,\"t\":\"ack\",\"ok\":true,\"kind\":\"%s\"}", kind);
        }
    } else {
        if (has_id) {
            n = snprintf(s_ack_json, sizeof(s_ack_json),
                "{\"v\":1,\"t\":\"ack\",\"id\":%lu,\"ok\":false,\"kind\":\"%s\",\"error\":\"%s\"}",
                (unsigned long)msg_id, kind, error ? error : "unknown");
        } else {
            n = snprintf(s_ack_json, sizeof(s_ack_json),
                "{\"v\":1,\"t\":\"ack\",\"ok\":false,\"kind\":\"%s\",\"error\":\"%s\"}",
                kind, error ? error : "unknown");
        }
    }

    if (n > 0 && n < (int)sizeof(s_ack_json)) {
        send_tc1_json(s_ack_json, (uint16_t)n);
        if (has_id) {
            memcpy(s_cached_ack_json, s_ack_json, (size_t)n + 1u);
            s_cached_ack_len = (uint16_t)n;
            s_cached_ack_id = msg_id;
            s_cached_ack_type = s_dispatch_type;
            s_cached_request_crc = s_dispatch_request_crc;
            s_cached_ack_valid = true;
        }
    }
}

/* ============================================================
 * 参数应用
 * ============================================================ */

static bool motor_tuning_allowed(void)
{
    safety_state_t safety = Safety_GetState();
    bool stationary = Track.state == TRACK_STATE_IDLE ||
                      Track.state == TRACK_STATE_FINISHED;
    return Track_Is_Motor_Test() ||
           (stationary &&
            (safety == SAFETY_STATE_BOOT || safety == SAFETY_STATE_READY));
}

/* 解析 set 消息中的参数并应用。电机参数整组先检查静止门控。 */
static bool apply_set_params(const char *json, const char **error)
{
    float v;
    float mkp;
    float mki;
    float mkd;
    float msl;
    float msr;
    bool has_mkp = find_num(json, "MKP", &mkp);
    bool has_mki = find_num(json, "MKI", &mki);
    bool has_mkd = find_num(json, "MKD", &mkd);
    bool has_msl = find_num(json, "MSL", &msl);
    bool has_msr = find_num(json, "MSR", &msr);
    bool has_motor = has_mkp || has_mki || has_mkd || has_msl || has_msr;

    if (has_motor && !motor_tuning_allowed()) {
        *error = "motor_tuning_requires_idle";
        return false;
    }

    if (find_num(json, "KP", &v)) {
        v = clamp_f(v, USER_PARAM_KP_MIN, USER_PARAM_KP_MAX);
        Track_Set_TrackPID(v, Track.line_pid.ki, Track.line_pid.kd);
        APP_TUNING_LOG_I(LOG_TAG, "set KP=%.3f", (double)v);
    }
    if (find_num(json, "KI", &v)) {
        v = clamp_f(v, USER_PARAM_KI_MIN, USER_PARAM_KI_MAX);
        Track_Set_TrackPID(Track.line_pid.kp, v, Track.line_pid.kd);
        APP_TUNING_LOG_I(LOG_TAG, "set KI=%.3f", (double)v);
    }
    if (find_num(json, "KD", &v)) {
        v = clamp_f(v, USER_PARAM_KD_MIN, USER_PARAM_KD_MAX);
        Track_Set_TrackPID(Track.line_pid.kp, Track.line_pid.ki, v);
        APP_TUNING_LOG_I(LOG_TAG, "set KD=%.3f", (double)v);
    }
    if (find_num(json, "GDAMP", &v)) {
        v = clamp_f(v, USER_PARAM_GDAMP_MIN, USER_PARAM_GDAMP_MAX);
        Track_Set_GyroDamp(v);
        APP_TUNING_LOG_I(LOG_TAG, "set GDAMP=%.3f", (double)v);
    }
    if (find_num(json, "VBASE", &v)) {
        int32_t iv = clamp_i((int32_t)v, USER_PARAM_VBASE_MIN, USER_PARAM_VBASE_MAX);
        Track_Set_Speed(iv);
        APP_TUNING_LOG_I(LOG_TAG, "set VBASE=%d", (int)iv);
    }
    if (find_num(json, "VSTR", &v)) {
        int32_t iv = clamp_i((int32_t)v, USER_PARAM_VSTR_MIN, USER_PARAM_VSTR_MAX);
        Track_Set_SpeedStraight(iv);
        APP_TUNING_LOG_I(LOG_TAG, "set VSTR=%d", (int)iv);
    }
    if (find_num(json, "VCUR", &v)) {
        int32_t iv = clamp_i((int32_t)v, USER_PARAM_VCUR_MIN, USER_PARAM_VCUR_MAX);
        Track_Set_SpeedCurve(iv);
        APP_TUNING_LOG_I(LOG_TAG, "set VCUR=%d", (int)iv);
    }
    if (find_num(json, "RATHR", &v)) {
        v = clamp_f(v, USER_PARAM_RATHR_MIN, USER_PARAM_RATHR_MAX);
        Track_Set_RightAngleThreshold(v);
        APP_TUNING_LOG_I(LOG_TAG, "set RATHR=%.3f", (double)v);
    }

    if (has_motor) {
        if (!has_mkp) mkp = App_Motor_Get_Speed_PID_Kp();
        if (!has_mki) mki = App_Motor_Get_Speed_PID_Ki();
        if (!has_mkd) mkd = App_Motor_Get_Speed_PID_Kd();
        if (!has_msl) msl = App_Motor_Get_Speed_Scale_Left();
        if (!has_msr) msr = App_Motor_Get_Speed_Scale_Right();

        mkp = clamp_f(mkp, USER_PARAM_MKP_MIN, USER_PARAM_MKP_MAX);
        mki = clamp_f(mki, USER_PARAM_MKI_MIN, USER_PARAM_MKI_MAX);
        mkd = clamp_f(mkd, USER_PARAM_MKD_MIN, USER_PARAM_MKD_MAX);
        msl = clamp_f(msl, USER_PARAM_MSL_MIN, USER_PARAM_MSL_MAX);
        msr = clamp_f(msr, USER_PARAM_MSR_MIN, USER_PARAM_MSR_MAX);
        App_Motor_Set_Speed_PID(mkp, mki, mkd);
        App_Motor_Set_Speed_Scale_Left(msl);
        App_Motor_Set_Speed_Scale_Right(msr);
        APP_TUNING_LOG_I(LOG_TAG,
                         "motor set P=%.3f I=%.3f D=%.3f SL=%.3f SR=%.3f",
                         (double)mkp, (double)mki, (double)mkd,
                         (double)msl, (double)msr);
    }
    return true;
}

/* 解析 cmd 消息并派发 */
static bool apply_cmd(const char *json, const char **error)
{
    char cmd[24];
    float degrees;
    bool lifted;

    *error = "unknown cmd";
    if (!find_str(json, "cmd", cmd, sizeof(cmd))) {
        *error = "invalid cmd";
        return false;
    }
    if (strcmp(cmd, "start") == 0) {
        APP_TUNING_LOG_I(LOG_TAG, "cmd start");
        if (Safety_GetState() != SAFETY_STATE_READY) {
            *error = (Safety_GetState() == SAFETY_STATE_BOOT ||
                      Safety_GetState() == SAFETY_STATE_CALIBRATING)
                   ? "path_fish_not_ready" : "motion_busy";
            return false;
        }
        if (!Track_Start()) {
            *error = Track_Get_Start_Result();
            return false;
        }
        return true;
    } else if (strcmp(cmd, "stop") == 0) {
        APP_TUNING_LOG_I(LOG_TAG, "cmd stop");
        Track_Stop();
        return true;
    } else if (strcmp(cmd, "motor_test") == 0) {
        if (!find_bool(json, "lifted", &lifted) || !lifted) {
            *error = "lifted_confirmation_required";
            return false;
        }
        if (Safety_GetState() != SAFETY_STATE_BOOT &&
            Safety_GetState() != SAFETY_STATE_READY) {
            *error = "motion_busy";
            return false;
        }
        if (!Track_Start_Motor_Test()) {
            *error = Track_Get_Motor_Test_Result();
            return false;
        }
        APP_TUNING_LOG_I(LOG_TAG, "cmd motor_test lifted=1");
        return true;
    } else if (strcmp(cmd, "rotate") == 0) {
        if (!find_num(json, "deg", &degrees) || !isfinite(degrees)) {
            *error = "invalid_angle";
            return false;
        }
#if APP_TUNING_ROTATE_LINK_TEST_MODE
        /* Communication test only: prove command parsing and ACK delivery
         * while keeping the motor path completely disarmed. */
        APP_TUNING_LOG_I(LOG_TAG, "cmd rotate link-test accepted=%d", (int)degrees);
        return true;
#else
        if (Safety_GetState() != SAFETY_STATE_BOOT &&
            Safety_GetState() != SAFETY_STATE_READY) {
            *error = "motion_busy";
            return false;
        }
        if (!Track_Start_Rotation(degrees)) {
            *error = Track_Get_Rotation_Result();
            return false;
        }
        APP_TUNING_LOG_I(LOG_TAG, "cmd rotate=%d", (int)degrees);
        return true;
#endif
    } else if (strcmp(cmd, "heading_hold_on") == 0) {
        if (Safety_GetState() != SAFETY_STATE_BOOT &&
            Safety_GetState() != SAFETY_STATE_READY) {
            *error = "motion_busy";
            return false;
        }
        if (!Track_Start_Heading_Hold()) {
            *error = Track_Get_Heading_Hold_Result();
            return false;
        }
        APP_TUNING_LOG_I(LOG_TAG, "cmd heading_hold_on");
        return true;
    } else if (strcmp(cmd, "heading_hold_off") == 0) {
        if (!Track_Stop_Heading_Hold()) {
            *error = "not_holding";
            return false;
        }
        APP_TUNING_LOG_I(LOG_TAG, "cmd heading_hold_off");
        return true;
    }
    APP_TUNING_LOG_W(LOG_TAG, "unknown cmd: %s", cmd);
    return false;
}

/* ============================================================
 * TC1 帧派发
 * ============================================================ */

static void dispatch_frame(const tc1_frame_t *frame)
{
#if !APP_TUNING_MODE
    /* RACE 模式: 禁用所有入站命令处理 */
    (void)frame;
    return;
#else
    if ((frame->type == TC1_MSG_SET || frame->type == TC1_MSG_CMD) &&
        frame->duplicate_id && s_cached_ack_valid &&
        frame->msg_id == s_cached_ack_id &&
        frame->type == s_cached_ack_type &&
        App_TC1_CRC32((const uint8_t *)frame->json, frame->json_len) ==
            s_cached_request_crc) {
        send_tc1_json(s_cached_ack_json, s_cached_ack_len);
        APP_TUNING_LOG_I(LOG_TAG, "replayed ACK id=%lu", (unsigned long)frame->msg_id);
        return;
    }
    switch (frame->type) {
    case TC1_MSG_AUTH: {
        /* 认证: 检查 token. 用 nack/log 响应 (ack kind 仅限 set/cmd) */
        char token[64];
        char session[33];
        if (find_str(frame->json, "token", token, sizeof(token)) &&
            strcmp(token, APP_TUNING_AUTH_TOKEN) == 0) {
            bool has_session = find_str(frame->json, "session", session, sizeof(session));
            if (!has_session || !s_auth_session_valid ||
                strcmp(session, s_auth_session) != 0) {
                /* 新 TCP/host 会话清理旧 ACK；同会话 AUTH 重发不影响幂等缓存。 */
                s_cached_ack_valid = false;
                if (has_session) {
                    memcpy(s_auth_session, session, strlen(session) + 1u);
                    s_auth_session_valid = true;
                } else {
                    s_auth_session_valid = false;
                }
            }
            s_authenticated = true;
            APP_TUNING_LOG_I(LOG_TAG, "auth ok");
            /* 发送 log 帧告知主机认证成功 */
            int n = snprintf(s_ack_json, sizeof(s_ack_json),
                "{\"v\":1,\"t\":\"log\",\"msg\":\"auth ok\"}");
            if (n > 0 && n < (int)sizeof(s_ack_json)) {
                send_tc1_json(s_ack_json, (uint16_t)n);
            }
        } else {
            s_authenticated = false;
            APP_TUNING_LOG_W(LOG_TAG, "auth failed");
            /* 发送 nack 帧告知主机认证失败 */
            int n = snprintf(s_ack_json, sizeof(s_ack_json),
                "{\"v\":1,\"t\":\"nack\",\"id\":%lu,\"error\":\"auth failed\"}",
                (unsigned long)frame->msg_id);
            if (n > 0 && n < (int)sizeof(s_ack_json)) {
                send_tc1_json(s_ack_json, (uint16_t)n);
            }
        }
        break;
    }
    case TC1_MSG_SET: {
        const char *error = NULL;
        s_dispatch_type = frame->type;
        s_dispatch_request_crc = App_TC1_CRC32(
            (const uint8_t *)frame->json, frame->json_len);
        if (!s_authenticated) {
            APP_TUNING_LOG_W(LOG_TAG, "set rejected: not authenticated");
            send_ack(frame->msg_id, frame->has_id, false, "set", "not authenticated");
            break;
        }
        if (!apply_set_params(frame->json, &error)) {
            send_ack(frame->msg_id, frame->has_id, false, "set", error);
            break;
        }
        send_ack(frame->msg_id, frame->has_id, true, "set", NULL);
        break;
    }
    case TC1_MSG_CMD: {
        const char *error;
        s_dispatch_type = frame->type;
        s_dispatch_request_crc = App_TC1_CRC32(
            (const uint8_t *)frame->json, frame->json_len);
        if (!s_authenticated) {
            APP_TUNING_LOG_W(LOG_TAG, "cmd rejected: not authenticated");
            send_ack(frame->msg_id, frame->has_id, false, "cmd", "not authenticated");
            break;
        }
        bool ok = apply_cmd(frame->json, &error);
        send_ack(frame->msg_id, frame->has_id, ok, "cmd",
                 ok ? NULL : error);
        break;
    }
    case TC1_MSG_PING: {
        /* 心跳仅用于链路诊断；回复 pong (携带 MCU 当前 tick)。 */
        int n = snprintf(s_ack_json, sizeof(s_ack_json),
            "{\"v\":1,\"t\":\"pong\",\"ms\":%lu}", (unsigned long)Bsp_GetTick());
        if (n > 0 && n < (int)sizeof(s_ack_json)) {
            send_tc1_json(s_ack_json, (uint16_t)n);
        }
        break;
    }
    default:
        /* pong/ack/nack/tele/log 是 MCU 发出的, 入站时忽略 */
        APP_TUNING_LOG_D(LOG_TAG, "ignore inbound msg type=%s",
               App_TC1_MsgTypeName(frame->type));
        break;
    }
#endif /* APP_TUNING_MODE */
}

/* ============================================================
 * 对外 API
 * ============================================================ */

void App_Tuning_OnRx(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return;

    /* 逐字节喂入 TC1 解帧器 */
    for (uint16_t i = 0; i < len; i++) {
        tc1_frame_t frame = {0};
        tc1_decode_result_t result = App_TC1_FeedByte(&s_parser, data[i], &frame);
        if (result == TC1_DECODE_OK) {
            dispatch_frame(&frame);
        } else if (result != TC1_DECODE_NEED_MORE) {
            /* 错误统计 (非 NEED_MORE 的结果表示一帧处理完毕) */
            APP_TUNING_LOG_D(LOG_TAG, "decode %s", App_TC1_DecodeResultName(result));
        }
    }
}

void App_Tuning_Init(void)
{
    App_TC1_Init(&s_parser);
    s_last_telem_tick = Bsp_GetTick();
    s_telem_seq = 0;
    s_authenticated = false;
    s_cached_ack_valid = false;
    s_cached_ack_len = 0u;
    s_auth_session_valid = false;
    App_ESP01S_RegisterRx(App_Tuning_OnRx);
    APP_TUNING_LOG_I(LOG_TAG, "TC1 tuning layer ready, telemetry %ums/frame, auth=%s",
           APP_TUNING_TELEM_INTERVAL_MS,
#if APP_TUNING_MODE
           "on"
#else
           "off (RACE mode)"
#endif
          );
}

void App_Tuning_Poll(void)
{
    /* 仅在 TCP 连接时上报, 避免 CIPSEND 失败 */
    if (!App_ESP01S_IsTCPConnected()) return;

    uint32_t now = Bsp_GetTick();
    if ((now - s_last_telem_tick) < APP_TUNING_TELEM_INTERVAL_MS) return;
    s_last_telem_tick = now;
    s_telem_seq++;

    /* 组包遥测 JSON (10Hz)
     * line/lm/bc/sa/se/bp 是鱼眼完整快照与健康信息。
     */
    char motion_json[128] = "";
    if (Track_Is_Motor_Test()) {
        int test_n = snprintf(motion_json, sizeof(motion_json),
            ",\"mt\":true,\"mph\":%u,\"mtg\":%d,\"mlp\":%ld,\"mrp\":%ld,\"mrs\":\"%s\"",
            Track_Get_Motor_Test_Phase(),
            Track_Get_Motor_Test_Target(),
            (long)Motor_Left.current_pwm,
            (long)Motor_Right.current_pwm,
            Track_Get_Motor_Test_Result());
        if (test_n < 0 || test_n >= (int)sizeof(motion_json)) {
            motion_json[0] = '\0';
        }
    } else if (Track_Is_Heading_Hold()) {
        int hold_n = snprintf(motion_json, sizeof(motion_json),
            ",\"hh\":true,\"ht\":%.2f,\"ha\":%.2f,\"he\":%.2f,\"hr\":\"%s\"",
            (double)Track_Get_Heading_Hold_Target(),
            (double)Track_Get_Heading_Hold_Actual(),
            (double)Track_Get_Heading_Hold_Error(),
            Track_Get_Heading_Hold_Result());
        if (hold_n < 0 || hold_n >= (int)sizeof(motion_json)) {
            motion_json[0] = '\0';
        }
    } else if (Track_Rotate_TargetValid(Track_Get_Rotation_Target())) {
        int rotate_n = snprintf(motion_json, sizeof(motion_json),
            ",\"tg\":%.2f,\"ta\":%.2f,\"te\":%.2f,\"rr\":\"%s\"",
            (double)Track_Get_Rotation_Target(),
            (double)Track_Get_Rotation_Angle(),
            (double)Track_Get_Rotation_Error(),
            Track_Get_Rotation_Result());
        if (rotate_n < 0 || rotate_n >= (int)sizeof(motion_json)) {
            motion_json[0] = '\0';
        }
    }

    int n = snprintf(s_telem_json, sizeof(s_telem_json),
        "{\"v\":1,\"t\":\"tele\","
        "\"seq\":%lu,\"ms\":%lu,"
        "\"line\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],"
        "\"lm\":%u,\"bc\":%u,\"sa\":%lu,\"se\":%lu,\"bp\":%u,"
        "\"gz\":%.2f,\"el\":%d,\"er\":%d,\"lp\":%ld,\"rp\":%ld,"
        "\"le\":%.2f,\"po\":%.2f,"
        "\"st\":\"%s\","
        "\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"gd\":%.3f,"
        "\"vb\":%d,\"vs\":%d,\"vc\":%d,\"rt\":%.3f,"
        "\"drop\":%lu%s}",
        (unsigned long)s_telem_seq, (unsigned long)now,
        Path_Fish_Sensor.digital[0], Path_Fish_Sensor.digital[1],
        Path_Fish_Sensor.digital[2], Path_Fish_Sensor.digital[3],
        Path_Fish_Sensor.digital[4], Path_Fish_Sensor.digital[5],
        Path_Fish_Sensor.digital[6], Path_Fish_Sensor.digital[7],
        Path_Fish_Sensor.digital[8], Path_Fish_Sensor.digital[9],
        Path_Fish_Sensor.digital[10], Path_Fish_Sensor.digital[11],
        Path_Fish_Sensor.mask, Path_Fish_Sensor.black_count,
        (unsigned long)Path_Fish_FrameAge(now),
        (unsigned long)Path_Fish_Sensor.error_count,
        Path_Fish_Sensor.black_is_one ? 1u : 0u,
        (double)Track.gyro_rate,
        (int)Motor_Left.encoder_speed, (int)Motor_Right.encoder_speed,
        (long)Motor_Left.current_pwm, (long)Motor_Right.current_pwm,
        (double)Track.error, (double)Track.turn_output,
        state_abbr(Track.state),
        (double)Track.line_pid.kp, (double)Track.line_pid.ki,
        (double)Track.line_pid.kd, (double)Track.gyro_damp,
        (int)Track.base_speed, (int)Track.speed_straight,
        (int)Track.speed_curve, (double)Track.right_angle_threshold,
        (unsigned long)App_ESP01S_GetTxDropCount(), motion_json);

    if (n > 0 && n < (int)sizeof(s_telem_json)) {
        send_tc1_json(s_telem_json, (uint16_t)n);
    }
}

uint32_t App_Tuning_GetTelemetrySeq(void)
{
    return s_telem_seq;
}

uint32_t App_Tuning_GetFramesOk(void)
{
    return s_parser.frames_ok;
}

uint32_t App_Tuning_GetFramesBadCrc(void)
{
    return s_parser.frames_bad_crc;
}

uint32_t App_Tuning_GetFramesBadSchema(void)
{
    return s_parser.frames_bad_schema;
}
