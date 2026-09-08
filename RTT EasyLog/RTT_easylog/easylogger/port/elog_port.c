/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */
 
#include <elog.h>

#include "bsp_common.h"
#include "SEGGER_RTT.h"
#include "app_esp01s.h"          /* ESP-01S 透传后端 */
#include "app_esp01s_cfg.h"
#include "app_tc1.h"
#include "ti_msp_dl_config.h"
#include <cmsis_compiler.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 * ESP-01S 日志透传后端
 *
 * 设计：elog_port_output 同步写 RTT + 非阻塞写入环形缓冲；
 *       主循环调 elog_port_esp_flush() 把缓冲通过 ESP CIPSEND 发往 PC。
 *
 * 协议：elog 日志封装为 {"t":"log","msg":"..."}\n JSON 帧，
 *       与遥测 {"t":"tele",...}\n 统一为同一 TCP 通道的行分隔协议。
 *       PC 端按 "t" 字段区分遥测和日志，避免裸文本误判。
 *
 * 特性：
 *   - 零阻塞：elog_port_output 仅内存拷贝，CIPSEND 推迟到主循环
 *   - 容错：TCP 断开时自动丢弃缓冲，连接恢复后续传新日志
 *   - 可关：s_elog_esp_enabled 默认 false，ESP 未初始化时零开销
 *   - JSON 转义：msg 内的 " 和 \ 自动转义，保证 JSON 合法
 * ============================================================ */

#define ELOG_ESP_RING_SIZE   2048   /* 环形缓冲大小（字节） */
#define ELOG_ESP_FLUSH_MS    200u   /* 远程日志发送节流周期 */

static uint8_t  s_elog_esp_buf[ELOG_ESP_RING_SIZE];
static volatile uint16_t s_elog_esp_head = 0;
static volatile uint16_t s_elog_esp_tail = 0;
static bool     s_elog_esp_enabled = false;

static uint16_t elog_esp_free_space(void)
{
    uint16_t head = s_elog_esp_head;
    uint16_t tail = s_elog_esp_tail;
    return (uint16_t)((ELOG_ESP_RING_SIZE + tail - head - 1) % ELOG_ESP_RING_SIZE);
}

/* 缓冲不足时丢弃最旧的完整日志行，禁止从一行中间开始发送。 */
static void elog_esp_drop_oldest_line(void)
{
    uint16_t tail = s_elog_esp_tail;
    while (tail != s_elog_esp_head) {
        char c = (char)s_elog_esp_buf[tail];
        tail = (uint16_t)((tail + 1u) % ELOG_ESP_RING_SIZE);
        if (c == '\n') break;
    }
    s_elog_esp_tail = tail;
}

#if APP_UART1_DIAG_CONSOLE_ENABLE
static void elog_uart1_diag_write(const char *log, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        char c = log[i];
        if (c == '\n') {
            while (DL_UART_Main_isTXFIFOFull(UART_ESP01S_INST)) {}
            DL_UART_Main_transmitData(UART_ESP01S_INST, '\r');
        }
        while (DL_UART_Main_isTXFIFOFull(UART_ESP01S_INST)) {}
        DL_UART_Main_transmitData(UART_ESP01S_INST, (uint8_t)c);
    }
}
#endif

void elog_port_esp_enable(bool en)
{
    s_elog_esp_enabled = en;
    if (!en) {
        /* 禁用时清空缓冲，避免下次启用时发送陈旧日志 */
        s_elog_esp_head = 0;
        s_elog_esp_tail = 0;
    }
}

/* 从环形缓冲读取一行（以 \n 分隔），返回行长度，0=无完整行 */
static uint16_t elog_esp_readline(char *out, uint16_t max_len)
{
    uint16_t tail = s_elog_esp_tail;
    uint16_t len = 0;
    while (tail != s_elog_esp_head && len < max_len - 1) {
        char c = (char)s_elog_esp_buf[tail];
        tail = (uint16_t)((tail + 1) % ELOG_ESP_RING_SIZE);
        if (c == '\n') {
            out[len] = '\0';
            s_elog_esp_tail = tail;
            return len;
        }
        /* 跳过 \r */
        if (c != '\r') {
            out[len++] = c;
        }
    }
    /* 无完整行，不消费缓冲 */
    return 0;
}

/* 把单行 log 封装为 {"t":"log","msg":"..."}\n 并发送 */
static void elog_esp_send_line(const char *line, uint16_t len)
{
    static char json_buf[512];
    static uint8_t tc1_buf[TC1_MAX_FRAME_BYTES];
    /* JSON 封装：{"t":"log","msg":"..."}\n
     * msg 内 " 转义为 \"，\ 转义为 \\ */
    uint16_t pos = 0;
    const char *prefix = "{\"v\":1,\"t\":\"log\",\"msg\":\"";
    const char *suffix = "\"}";
    uint16_t prefix_len = (uint16_t)strlen(prefix);
    uint16_t suffix_len = (uint16_t)strlen(suffix);

    /* 检查容量（最坏情况：每个字符都需转义，×2 + prefix + suffix + NUL） */
    if (prefix_len + len * 2 + suffix_len > TC1_MAX_JSON_BYTES) {
        /* Keep the encoded JSON within the shared TC1 protocol limit. */
        len = (uint16_t)((TC1_MAX_JSON_BYTES - prefix_len - suffix_len) / 2);
    }

    memcpy(json_buf, prefix, prefix_len);
    pos = prefix_len;
    for (uint16_t i = 0; i < len; i++) {
        char c = line[i];
        if ((unsigned char)c == 0x1B && (i + 1u) < len && line[i + 1u] == '[') {
            i += 2u;
            while (i < len) {
                char seq = line[i];
                if ((seq >= 'A' && seq <= 'Z') || (seq >= 'a' && seq <= 'z')) {
                    break;
                }
                i++;
            }
            continue;
        } else if (c == '"') {
            json_buf[pos++] = '\\';
            json_buf[pos++] = '"';
        } else if (c == '\\') {
            json_buf[pos++] = '\\';
            json_buf[pos++] = '\\';
        } else if ((unsigned char)c < 0x20) {
            /* 控制字符跳过（elog 颜色码等） */
            continue;
        } else {
            json_buf[pos++] = c;
        }
    }
    memcpy(json_buf + pos, suffix, suffix_len);
    pos += suffix_len;

    uint16_t frame_len = App_TC1_EncodeFrame(json_buf, pos,
                                             tc1_buf, sizeof(tc1_buf));
    if (frame_len > 0) {
        (void)App_ESP01S_SendJSON(tc1_buf, frame_len);
    }
}

void elog_port_esp_flush(void)
{
    static uint32_t s_last_flush_tick = 0;

    if (!s_elog_esp_enabled) return;
    if (!App_ESP01S_IsTCPConnected()) {
        /* TCP 断开：丢弃缓冲避免累积爆满 */
        s_elog_esp_head = 0;
        s_elog_esp_tail = 0;
        return;
    }

    if ((systick_ms - s_last_flush_tick) < ELOG_ESP_FLUSH_MS) {
        return;
    }
    s_last_flush_tick = systick_ms;

    /* ESP-01S AT+CIPSEND 需要节流。每次最多发送 1 行远程日志。 */
    char line[256];
    uint16_t len = elog_esp_readline(line, sizeof(line));
    if (len > 0) {
        elog_esp_send_line(line, len);
    }
}

/**
 * EasyLogger port initialize
 *
 * @return result
 */
ElogErrCode elog_port_init(void) {
    ElogErrCode result = ELOG_NO_ERR;

    /* add your code here */
    
    return result;
}

/**
 * EasyLogger port deinitialize
 *
 */
void elog_port_deinit(void) {

    /* add your code here */

}

/**
 * output log port interface
 *
 * @param log output of log
 * @param size log size
 *
 * @note 同时输出到两个后端：
 *         1. RTT 通道 0（同步，保留原行为，JLink 实时查看）
 *         2. ESP-01S 环形缓冲（非阻塞，主循环 flush 时通过 TCP 发往 PC）
 *       一条 elog_i/elog_w/... 语句即同时打印 RTT 日志和 ESP 日志。
 */
void elog_port_output(const char *log, size_t size) {

    /* 后端 1：RTT 同步输出（保留原行为） */
    SEGGER_RTT_Write(0, log, size);

#if APP_UART1_DIAG_CONSOLE_ENABLE
    /* 诊断模式：PA8/PA9 作为外接 USB-TTL 控制台，直接镜像日志。 */
    elog_uart1_diag_write(log, size);
#endif

    /* 后端 2：ESP-01S 透传环形缓冲（非阻塞，满则丢弃整行） */
    if (s_elog_esp_enabled && size < ELOG_ESP_RING_SIZE) {
        uint16_t head = s_elog_esp_head;
        while (elog_esp_free_space() < size) {
            elog_esp_drop_oldest_line();
        }
        for (size_t i = 0; i < size; i++) {
            s_elog_esp_buf[head] = (uint8_t)log[i];
            head = (uint16_t)((head + 1) % ELOG_ESP_RING_SIZE);
        }
        s_elog_esp_head = head;
    }
}

/**
 * output lock
 */
void elog_port_output_lock(void) {
    __disable_irq();
}

void elog_port_output_unlock(void) {
    __enable_irq();
}

/**
 * get current time interface
 *
 * @return current time
 */
const char *elog_port_get_time(void) {
    static char cur_system_time[16] = { 0 };
    snprintf(cur_system_time, sizeof(cur_system_time), "%lu", (unsigned long)systick_ms);
    return cur_system_time;
}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void) {
    
    /* add your code here */
    return "";
}

/**
 * get current thread name interface
 *
 * @return current thread name
 */
const char *elog_port_get_t_info(void) {
    
    /* add your code here */
    return "";
}
