/**
 * @file    app_tc1.c
 * @brief   TC1 协议解帧器实现 — 零堆分配字节流解析器。
 *
 *          帧格式: TC1 <len_dec> <CRC32_8hex> <UTF-8_JSON>\n
 *
 *          解析状态机:
 *            WAIT_MAGIC → READ_LEN → READ_CRC → READ_PAYLOAD → WAIT_NEWLINE
 *
 *          CRC32: 标准 CRC32（与 Python binascii.crc32 一致）
 *                 多项式 0xEDB88320，初始值 0xFFFFFFFF，终值异或 0xFFFFFFFF
 *
 * @note    零堆分配：所有缓冲区静态分配在 tc1_parser_t 结构体内。
 *          out_frame.json 指向 parser 内部 payload 缓冲区，
 *          调用方在下一次 FeedByte 前必须消费完。
 */
#include "app_tc1.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

/* ============================================================
 * CRC32 查表法（与 Python binascii.crc32 一致）
 * 多项式 0xEDB88320（反射形式）
 * ============================================================ */

static uint32_t s_crc_table[256];
static bool s_crc_table_init = false;

static void CRC32_InitTable(void)
{
    if (s_crc_table_init) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 1u) {
                crc = (crc >> 1) ^ 0xEDB88320u;
            } else {
                crc >>= 1;
            }
        }
        s_crc_table[i] = crc;
    }
    s_crc_table_init = true;
}

uint32_t App_TC1_CRC32(const uint8_t *data, uint16_t len)
{
    if (!s_crc_table_init) CRC32_InitTable();
    uint32_t crc = 0xFFFFFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ s_crc_table[(crc ^ data[i]) & 0xFFu];
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ============================================================
 * 内部工具
 * ============================================================ */

static const char s_magic[] = "TC1";

static uint16_t parse_dec_uint16(const char *s, uint8_t len)
{
    uint16_t val = 0;
    for (uint8_t i = 0; i < len; i++) {
        if (s[i] < '0' || s[i] > '9') return 0xFFFFu;  /* 非法 */
        val = val * 10u + (uint16_t)(s[i] - '0');
        if (val > TC1_MAX_JSON_BYTES) return 0xFFFFu;   /* 超长 */
    }
    return (len == 0) ? 0xFFFFu : val;
}

static uint32_t parse_hex_uint32(const char *s, uint8_t len)
{
    if (len != TC1_CRC_HEX_LEN) return 0xFFFFFFFFu;
    uint32_t val = 0;
    for (uint8_t i = 0; i < len; i++) {
        char c = s[i];
        uint32_t digit;
        if (c >= '0' && c <= '9') {
            digit = (uint32_t)(c - '0');
        } else if (c >= 'A' && c <= 'F') {
            digit = (uint32_t)(c - 'A' + 10);
        } else if (c >= 'a' && c <= 'f') {
            digit = (uint32_t)(c - 'a' + 10);
        } else {
            return 0xFFFFFFFFu;  /* 非法字符 */
        }
        val = (val << 4) | digit;
    }
    return val;
}

/* ============================================================
 * JSON 轻量解析（不依赖 cJSON，仅提取 "t" 和 "id" 字段）
 * ============================================================ */

/* 查找 JSON 中的字符串值: "key":"value"
 * 返回 value 的起始指针和长度，未找到返回 NULL */
static const char *json_find_string(const char *json, const char *key,
                                    uint8_t *out_len)
{
    char pattern[16];
    uint8_t plen = (uint8_t)snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (p == NULL) return NULL;
    p += plen;
    /* 跳过空白和冒号 */
    while (*p && (*p == ' ' || *p == '\t')) p++;
    if (*p != ':') return NULL;
    p++;
    while (*p && (*p == ' ' || *p == '\t')) p++;
    if (*p != '"') return NULL;
    p++;  /* 跳过开引号 */
    const char *start = p;
    /* 找到闭引号（不处理转义，协议消息类型不含特殊字符） */
    while (*p && *p != '"') p++;
    if (*p != '"') return NULL;
    *out_len = (uint8_t)(p - start);
    return start;
}

/* 查找 JSON 中的整数值: "key":number
 * 返回 true 并设置 out_val，未找到返回 false */
static bool json_find_uint(const char *json, const char *key, uint32_t *out_val)
{
    char pattern[16];
    uint8_t plen = (uint8_t)snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (p == NULL) return false;
    p += plen;
    while (*p && (*p == ' ' || *p == '\t')) p++;
    if (*p != ':') return false;
    p++;
    while (*p && (*p == ' ' || *p == '\t')) p++;
    if (*p < '0' || *p > '9') return false;
    uint32_t val = 0;
    while (*p >= '0' && *p <= '9') {
        val = val * 10u + (uint32_t)(*p - '0');
        p++;
    }
    *out_val = val;
    return true;
}

/* 根据 "t" 字段判断消息类型 */
static tc1_msg_type_t detect_msg_type(const char *json)
{
    uint8_t tlen = 0;
    const char *tval = json_find_string(json, "t", &tlen);
    if (tval == NULL || tlen == 0) return TC1_MSG_UNKNOWN;

    /* 精确匹配消息类型字符串 */
    static const struct {
        const char *name;
        uint8_t len;
        tc1_msg_type_t type;
    } types[] = {
        {"auth", 4, TC1_MSG_AUTH},
        {"set",  3, TC1_MSG_SET},
        {"cmd",  3, TC1_MSG_CMD},
        {"ping", 4, TC1_MSG_PING},
        {"pong", 4, TC1_MSG_PONG},
        {"ack",  3, TC1_MSG_ACK},
        {"nack", 4, TC1_MSG_NACK},
        {"tele", 4, TC1_MSG_TELE},
        {"log",  3, TC1_MSG_LOG},
    };

    for (uint8_t i = 0; i < sizeof(types)/sizeof(types[0]); i++) {
        if (tlen == types[i].len &&
            strncmp(tval, types[i].name, tlen) == 0) {
            return types[i].type;
        }
    }
    return TC1_MSG_UNKNOWN;
}

/* ============================================================
 * 解帧器实现
 * ============================================================ */

void App_TC1_Init(tc1_parser_t *p)
{
    if (p == NULL) return;
    memset(p, 0, sizeof(*p));
    p->state = TC1_STATE_WAIT_MAGIC;
    CRC32_InitTable();
}

void App_TC1_Reset(tc1_parser_t *p)
{
    if (p == NULL) return;
    p->state = TC1_STATE_WAIT_MAGIC;
    p->magic_idx = 0;
    p->len_idx = 0;
    p->crc_idx = 0;
    p->payload_idx = 0;
    p->declared_len = 0;
    p->declared_crc = 0;
    /* 不清统计和 last_msg_id */
}

static tc1_decode_result_t finalize_frame(tc1_parser_t *p, tc1_frame_t *out)
{
    /* NUL 终止 payload */
    p->payload[p->payload_idx] = '\0';

    /* CRC 校验 */
    uint32_t actual_crc = App_TC1_CRC32(p->payload, p->payload_idx);
    if (actual_crc != p->declared_crc) {
        p->frames_bad_crc++;
        return TC1_DECODE_BAD_CRC;
    }

    /* 基本 JSON 合法性检查（必须以 { 开头 } 结尾） */
    if (p->payload_idx < 2 ||
        p->payload[0] != '{' ||
        p->payload[p->payload_idx - 1] != '}') {
        p->frames_bad_json++;
        return TC1_DECODE_BAD_JSON;
    }

    /* 消息类型检测 */
    tc1_msg_type_t mtype = detect_msg_type((const char *)p->payload);
    if (mtype == TC1_MSG_UNKNOWN) {
        p->frames_bad_schema++;
        return TC1_DECODE_BAD_SCHEMA;
    }

    /* ID 提取与重复检测（仅对带 id 的消息） */
    uint32_t msg_id = 0;
    bool has_id = json_find_uint((const char *)p->payload, "id", &msg_id);
    bool duplicate_id = false;
    if (has_id) {
        if (p->last_msg_id_valid && msg_id == p->last_msg_id) {
            p->frames_duplicate_id++;
            duplicate_id = true;
        }
        p->last_msg_id = msg_id;
        p->last_msg_id_valid = true;
    }

    /* 填充输出帧（零拷贝，指向 parser 内部缓冲区） */
    if (out) {
        out->type = mtype;
        out->json = (const char *)p->payload;
        out->json_len = p->payload_idx;
        out->msg_id = msg_id;
        out->has_id = has_id;
        out->duplicate_id = duplicate_id;
    }

    p->frames_ok++;
    return TC1_DECODE_OK;
}

tc1_decode_result_t App_TC1_FeedByte(tc1_parser_t *p, uint8_t byte,
                                     tc1_frame_t *out_frame)
{
    if (p == NULL) return TC1_DECODE_BAD_JSON;

    switch (p->state) {
    case TC1_STATE_WAIT_MAGIC:
        /* 逐字节匹配 "TC1" 前缀，匹配完成后等待空格进入 READ_LEN */
        if (p->magic_idx < TC1_FRAME_MAGIC_LEN) {
            if (byte == (uint8_t)s_magic[p->magic_idx]) {
                p->magic_idx++;
            } else {
                /* 匹配失败，重置。若当前字节是 'T'，可能是新帧开头 */
                p->magic_idx = (byte == 'T') ? 1u : 0u;
            }
        } else {
            /* "TC1" 已完整匹配，等待分隔空格 */
            if (byte == ' ') {
                p->state = TC1_STATE_READ_LEN;
                p->len_idx = 0;
            } else {
                /* "TC1" 后非空格，重置。若当前字节是 'T'，可能是新帧开头 */
                p->magic_idx = (byte == 'T') ? 1u : 0u;
            }
        }
        break;

    case TC1_STATE_READ_LEN:
        if (byte == ' ') {
            /* 长度字段结束 */
            p->len_buf[p->len_idx] = '\0';
            p->declared_len = parse_dec_uint16(p->len_buf, p->len_idx);
            if (p->declared_len == 0xFFFFu || p->declared_len < 2 ||
                p->declared_len > TC1_MAX_JSON_BYTES) {
                if (p->declared_len > TC1_MAX_JSON_BYTES) {
                    p->frames_oversized++;
                    return TC1_DECODE_OVERSIZED;
                }
                App_TC1_Reset(p);
                return TC1_DECODE_BAD_LENGTH;
            }
            p->state = TC1_STATE_READ_CRC;
            p->crc_idx = 0;
        } else if (byte >= '0' && byte <= '9') {
            if (p->len_idx >= TC1_MAX_LEN_DIGITS) {
                App_TC1_Reset(p);
                return TC1_DECODE_BAD_LENGTH;
            }
            p->len_buf[p->len_idx++] = (char)byte;
        } else {
            /* 非法字符 */
            App_TC1_Reset(p);
            return TC1_DECODE_BAD_LENGTH;
        }
        break;

    case TC1_STATE_READ_CRC:
        if (byte == ' ') {
            /* CRC 字段结束，必须正好 8 位十六进制 */
            if (p->crc_idx != TC1_CRC_HEX_LEN) {
                App_TC1_Reset(p);
                return TC1_DECODE_BAD_CRC;
            }
            p->crc_buf[p->crc_idx] = '\0';
            p->declared_crc = parse_hex_uint32(p->crc_buf, p->crc_idx);
            p->state = TC1_STATE_READ_PAYLOAD;
            p->payload_idx = 0;
        } else if (isxdigit(byte)) {
            if (p->crc_idx >= TC1_CRC_HEX_LEN) {
                App_TC1_Reset(p);
                return TC1_DECODE_BAD_CRC;
            }
            p->crc_buf[p->crc_idx++] = (char)byte;
        } else {
            App_TC1_Reset(p);
            return TC1_DECODE_BAD_CRC;
        }
        break;

    case TC1_STATE_READ_PAYLOAD:
        if (p->payload_idx < p->declared_len) {
            p->payload[p->payload_idx++] = byte;
            if (p->payload_idx >= p->declared_len) {
                p->state = TC1_STATE_WAIT_NEWLINE;
            }
        } else {
            /* 不应该到这里 */
            p->state = TC1_STATE_WAIT_NEWLINE;
        }
        break;

    case TC1_STATE_WAIT_NEWLINE:
        if (byte == '\n') {
            tc1_decode_result_t result = finalize_frame(p, out_frame);
            App_TC1_Reset(p);
            return result;
        } else if (byte == '\r') {
            /* 允许 \r\n */
            break;
        } else {
            /* 帧后非换行，重置 */
            App_TC1_Reset(p);
            return TC1_DECODE_BAD_JSON;
        }
        break;

    default:
        App_TC1_Reset(p);
        break;
    }

    return TC1_DECODE_NEED_MORE;
}

uint16_t App_TC1_FeedBytes(tc1_parser_t *p, const uint8_t *data, uint16_t len,
                           tc1_frame_t *out_frame)
{
    if (p == NULL || data == NULL || len == 0) return 0;
    for (uint16_t i = 0; i < len; i++) {
        tc1_decode_result_t r = App_TC1_FeedByte(p, data[i], out_frame);
        if (r != TC1_DECODE_NEED_MORE) {
            return i + 1;  /* 消费到当前位置（含） */
        }
    }
    return len;  /* 全部消费但未出帧 */
}

uint16_t App_TC1_EncodeFrame(const char *json, uint16_t json_len,
                             uint8_t *out_buf, uint16_t out_size)
{
    if (json == NULL || out_buf == NULL || json_len == 0) return 0;
    if (json_len > TC1_MAX_JSON_BYTES) return 0;

    /* 计算所需空间: "TC1 " + len(3) + " " + CRC(8) + " " + json + "\n" */
    uint16_t needed = 4 + 3 + 1 + 8 + 1 + json_len + 1;
    if (needed > out_size) return 0;
    if (needed > TC1_MAX_FRAME_BYTES) return 0;

    uint32_t crc = App_TC1_CRC32((const uint8_t *)json, json_len);

    /* 格式: TC1 <len> <CRC32_8hex> <json>\n */
    uint16_t pos = 0;
    out_buf[pos++] = 'T';
    out_buf[pos++] = 'C';
    out_buf[pos++] = '1';
    out_buf[pos++] = ' ';
    /* len 十进制 */
    pos += (uint16_t)snprintf((char *)(out_buf + pos), 4, "%u", json_len);
    out_buf[pos++] = ' ';
    /* CRC 8 位十六进制大写 */
    pos += (uint16_t)snprintf((char *)(out_buf + pos), 9, "%08lX",
                              (unsigned long)crc);
    out_buf[pos++] = ' ';
    /* JSON payload */
    memcpy(out_buf + pos, json, json_len);
    pos += json_len;
    out_buf[pos++] = '\n';

    return pos;
}

/* ============================================================
 * 名称查询
 * ============================================================ */

static const char *s_msg_type_names[] = {
    "UNKNOWN", "auth", "set", "cmd", "ping", "pong", "ack", "nack", "tele", "log"
};

const char *App_TC1_MsgTypeName(tc1_msg_type_t type)
{
    if ((uint32_t)type >= sizeof(s_msg_type_names)/sizeof(s_msg_type_names[0])) {
        return "?";
    }
    return s_msg_type_names[type];
}

static const char *s_decode_result_names[] = {
    "OK", "NEED_MORE", "BAD_MAGIC", "BAD_LENGTH", "BAD_CRC",
    "OVERSIZED", "BAD_JSON", "BAD_SCHEMA", "DUPLICATE_ID"
};

const char *App_TC1_DecodeResultName(tc1_decode_result_t result)
{
    if ((uint32_t)result >= sizeof(s_decode_result_names)/sizeof(s_decode_result_names[0])) {
        return "?";
    }
    return s_decode_result_names[result];
}
