/**
 * @file    app_tc1.h
 * @brief   TC1 协议解帧器 — 零堆分配字节流解析器。
 *
 *          帧格式: TC1 <len_dec> <CRC32_8hex> <UTF-8_JSON>\n
 *          JSON 最大 384 字节，整帧最大 402 字节。
 *
 *          特性:
 *            - 字节流解析，自动处理拆包/粘包
 *            - CRC32 校验（与 Python binascii.crc32 一致）
 *            - 超长帧拒绝
 *            - 重复 ID 检测（单调递增）
 *            - 零堆分配（静态缓冲区）
 *
 *          消息类型: auth/set/cmd/ping/pong/ack/nack/tele/log
 */
#ifndef APP_TC1_H
#define APP_TC1_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================
 * 协议常量
 * ============================================================ */
#define TC1_FRAME_MAGIC       "TC1"
#define TC1_FRAME_MAGIC_LEN   3u
#define TC1_MAX_JSON_BYTES    384u
#define TC1_MAX_FRAME_BYTES   402u
#define TC1_CRC_HEX_LEN       8u
#define TC1_MAX_LEN_DIGITS    3u   /* 384 最大 3 位十进制 */

/* 消息类型 */
typedef enum {
    TC1_MSG_UNKNOWN = 0,
    TC1_MSG_AUTH,       /* 认证请求 */
    TC1_MSG_SET,        /* 参数设置 */
    TC1_MSG_CMD,        /* 命令 */
    TC1_MSG_PING,       /* 心跳请求 */
    TC1_MSG_PONG,       /* 心跳响应 */
    TC1_MSG_ACK,        /* 确认 */
    TC1_MSG_NACK,       /* 拒绝 */
    TC1_MSG_TELE,       /* 遥测 */
    TC1_MSG_LOG,        /* 日志 */
} tc1_msg_type_t;

/* 解帧结果 */
typedef enum {
    TC1_DECODE_OK = 0,
    TC1_DECODE_NEED_MORE,       /* 需要更多字节 */
    TC1_DECODE_BAD_MAGIC,       /* 帧头错误 */
    TC1_DECODE_BAD_LENGTH,      /* 长度字段错误 */
    TC1_DECODE_BAD_CRC,         /* CRC 校验失败 */
    TC1_DECODE_OVERSIZED,       /* 超长 */
    TC1_DECODE_BAD_JSON,        /* JSON 解析失败 */
    TC1_DECODE_BAD_SCHEMA,      /* schema 校验失败 */
    TC1_DECODE_DUPLICATE_ID,    /* 重复 ID */
} tc1_decode_result_t;

/* ============================================================
 * 解帧器状态
 * ============================================================ */
typedef enum {
    TC1_STATE_WAIT_MAGIC = 0,   /* 等待 "TC1" 前缀 */
    TC1_STATE_READ_LEN,         /* 读取长度(十进制,到空格) */
    TC1_STATE_READ_CRC,         /* 读取 CRC(8位十六进制,到空格) */
    TC1_STATE_READ_PAYLOAD,     /* 读取 payload(len 字节) */
    TC1_STATE_WAIT_NEWLINE,     /* 等待 \n 结束 */
} tc1_parser_state_t;

/* ============================================================
 * 解帧器对象
 * ============================================================ */
typedef struct {
    tc1_parser_state_t state;
    uint8_t  magic_idx;                 /* 帧头匹配位置 */
    char     len_buf[TC1_MAX_LEN_DIGITS + 1];
    uint8_t  len_idx;
    uint16_t declared_len;              /* 声明的 payload 长度 */
    char     crc_buf[TC1_CRC_HEX_LEN + 1];
    uint8_t  crc_idx;
    uint32_t declared_crc;              /* 声明的 CRC32 (8 hex 字符) */
    uint8_t  payload[TC1_MAX_JSON_BYTES + 1];  /* +1 for NUL */
    uint16_t payload_idx;
    /* 统计 */
    uint32_t frames_ok;
    uint32_t frames_bad_crc;
    uint32_t frames_oversized;
    uint32_t frames_bad_json;
    uint32_t frames_bad_schema;
    uint32_t frames_duplicate_id;
    /* 重复 ID 检测 */
    uint32_t last_msg_id;
    bool     last_msg_id_valid;
} tc1_parser_t;

/* ============================================================
 * 解出的帧（零拷贝，指向 parser 内部缓冲区）
 * ============================================================ */
typedef struct {
    tc1_msg_type_t type;
    const char    *json;        /* NUL 结尾的 JSON 字符串 */
    uint16_t       json_len;
    uint32_t       msg_id;      /* 消息 ID (0=无 ID) */
    bool           has_id;
    bool           duplicate_id; /* 与上一条带 ID 消息相同，供 ACK 重放 */
} tc1_frame_t;

/* ============================================================
 * 接口
 * ============================================================ */

/* 初始化解帧器 */
void App_TC1_Init(tc1_parser_t *p);

/* 喂入一个字节，返回是否有完整帧可消费 */
tc1_decode_result_t App_TC1_FeedByte(tc1_parser_t *p, uint8_t byte,
                                     tc1_frame_t *out_frame);

/* 批量喂入字节流，返回消费的字节数 */
uint16_t App_TC1_FeedBytes(tc1_parser_t *p, const uint8_t *data, uint16_t len,
                           tc1_frame_t *out_frame);

/* 编码帧到 out_buf，返回整帧长度（含 \n），0=失败 */
uint16_t App_TC1_EncodeFrame(const char *json, uint16_t json_len,
                             uint8_t *out_buf, uint16_t out_size);

/* CRC32 计算（与 Python binascii.crc32 一致，多项式 0xEDB88320） */
uint32_t App_TC1_CRC32(const uint8_t *data, uint16_t len);

/* 消息类型名称 */
const char *App_TC1_MsgTypeName(tc1_msg_type_t type);

/* 解码结果名称 */
const char *App_TC1_DecodeResultName(tc1_decode_result_t result);

/* 重置解帧器（不清统计） */
void App_TC1_Reset(tc1_parser_t *p);

#endif /* APP_TC1_H */
