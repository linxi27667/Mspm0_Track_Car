/**
 * @file    bsp_esp_at.h
 * @brief   ESP AT 指令引擎（BSP — 纯逻辑，跨平台可复用）
 * @note    架构定位：
 *            - 仅依赖 bsp_uart_t（字节流 TX/RX）+ bsp_common（tick 时标）
 *            - 零硬件头文件，可在任意 Cortex-M 项目复用
 *            - 实现：AT 指令收发 + 响应匹配 + +IPD 数据重组状态机
 *
 *          设计参考：排队叫号系统 esp8266.c / esp8266_mqtt.c 的 IPD 处理逻辑
 *          改进点：
 *            - 复用 bsp_uart_t 的 DMA+IDLE+FIFO，不再单字节中断
 *            - 状态机跨帧重组 +IPD，不丢数据
 *            - 非阻塞轮询，适配主循环架构
 *
 *          适用模组：ESP-01S / ESP8266 / ESP-12F（AT 固件，CIPMUX=0 单连接透传）
 */
#ifndef __BSP_ESP_AT_H__
#define __BSP_ESP_AT_H__

#include "bsp_uart.h"
#include "bsp_common.h"

/* ============ 状态机 ============ */
typedef enum {
    BSP_ESP_AT_STATE_LINE = 0,    /* 累积响应字节流（按 \r\n 自然分行，strstr 匹配） */
    BSP_ESP_AT_STATE_IPD_DATA,    /* 已解析 +IPD,<len>: 头，按长度收集数据体 */
} bsp_esp_at_state_t;

typedef enum {
    BSP_ESP_AT_RESPONSE_PENDING = 0,
    BSP_ESP_AT_RESPONSE_MATCHED,
    BSP_ESP_AT_RESPONSE_FAILED,
} bsp_esp_at_response_t;

/* ============ 引擎上下文 ============ */
typedef struct {
    bsp_uart_t         *uart;          /* 绑定的 UART 设备（TX/RX 通道） */
    bsp_esp_at_state_t  state;         /* 当前解析状态 */

    /* ---- 行/响应流缓冲（STATE_LINE 用） ---- */
    uint8_t            *line_buf;      /* 响应流缓冲（NUL 结尾，供 strstr 匹配） */
    uint16_t            line_size;     /* 缓冲容量 */
    uint16_t            line_len;      /* 已写入字节数 */

    /* ---- +IPD 数据重组（STATE_IPD_DATA 用） ---- */
    uint8_t            *ipd_buf;       /* +IPD 数据输出缓冲 */
    uint16_t            ipd_size;      /* IPD 缓冲容量 */
    uint16_t            ipd_len;       /* 已"存储"字节数（不含溢出丢弃） */
    uint16_t            ipd_recv;      /* 已"接收"字节数（含丢弃，用于判断完成） */
    uint16_t            ipd_expected;  /* +IPD 声明的数据长度 */

    /* ---- 诊断计数 ---- */
    uint32_t            at_fail_count;       /* AT 指令超时/ERROR/FAIL 次数 */
    uint32_t            ipd_overflow_count;  /* +IPD 数据超出缓冲的丢弃次数 */
    uint32_t            line_overflow_count; /* 响应流缓冲溢出（半缓冲滑窗）次数 */
} bsp_esp_at_ctx_t;

/* ============ API ============ */

/* 初始化上下文（绑定 UART + 提供缓冲） */
void Bsp_ESP_AT_Init(bsp_esp_at_ctx_t *ctx, bsp_uart_t *uart,
                     uint8_t *line_buf, uint16_t line_size,
                     uint8_t *ipd_buf, uint16_t ipd_size);

/**
 * 开始一条 AT 指令，不等待响应。
 * @return true = 指令已进入 UART DMA 队列；false = UART 尚忙或参数无效
 * @note 调用方后续必须周期性调用 Bsp_ESP_AT_PollResponse，并自行管理超时。
 */
bool Bsp_ESP_AT_BeginCommand(bsp_esp_at_ctx_t *ctx, const char *cmd);

/**
 * 非阻塞地解析当前 AT 指令的响应。
 * @return PENDING = 尚未完成；MATCHED = 收到 expect；FAILED = ERROR/FAIL/CLOSED
 */
bsp_esp_at_response_t Bsp_ESP_AT_PollResponse(bsp_esp_at_ctx_t *ctx,
                                               const char *expect);

/**
 * 发送 AT 指令，阻塞等待 expect 或 ERROR/FAIL/timeout
 * @param  cmd       AT 指令字符串（含 \r\n，由调用方拼接）
 * @param  expect    期望响应子串（如 "OK"、"GOT IP"、">"）；NULL 表示不匹配只等超时
 * @param  timeout_ms 超时（ms）
 * @return true = 匹配到 expect；false = 超时或收到 ERROR/FAIL
 * @note   阻塞兼容接口；主循环状态机应使用 BeginCommand + PollResponse。
 */
bool Bsp_ESP_AT_SendCmd(bsp_esp_at_ctx_t *ctx, const char *cmd,
                        const char *expect, uint32_t timeout_ms);

/* 带重试的 SendCmd（每次失败间隔 retry_delay_ms） */
bool Bsp_ESP_AT_SendCmdRetry(bsp_esp_at_ctx_t *ctx, const char *cmd,
                             const char *expect, uint32_t timeout_ms,
                             uint8_t retries, uint32_t retry_delay_ms);

/**
 * 等待响应（不发送指令，不清空 RX 残留）
 * @note 用于 AT+CIPSEND 流程：先 SendCmd(cmd, ">", ...) 拿到提示符，
 *       再 SendRaw(data, len) 发数据，最后 WaitResponse("SEND OK", ...) 确认。
 *       不清空 RX 是为了保留 SendRaw 期间已到达的响应字节。
 */
bool Bsp_ESP_AT_WaitResponse(bsp_esp_at_ctx_t *ctx, const char *expect,
                             uint32_t timeout_ms);

/* 发送原始字节（AT+CIPSEND 之后的数据体，透传给 TCP 对端） */
void Bsp_ESP_AT_SendRaw(bsp_esp_at_ctx_t *ctx, const uint8_t *data, uint16_t len);

/* 发送字符串 */
void Bsp_ESP_AT_SendString(bsp_esp_at_ctx_t *ctx, const char *s);

/**
 * 主循环轮询：排空 UART → 状态机推进
 * @param  out_len  输出：本次收齐的 +IPD 数据长度（仅在返回 true 时有效）
 * @return true = 收到一帧完整 +IPD 数据（数据在 ctx->ipd_buf，长度见 out_len）
 * @note   每次 Poll 最多消费 BSP_ESP_AT_POLL_BUDGET 字节，避免霸占主循环
 *         收到 true 后，ipd_buf 内容在下次 Poll 前有效，调用方应立即处理或拷走
 */
bool Bsp_ESP_AT_Poll(bsp_esp_at_ctx_t *ctx, uint16_t *out_len);

/* 获取响应流缓冲（调试用，NUL 结尾）。可用于上层扫描 CLOSED/WIFI DISCONNECT 等异步通知 */
const char *Bsp_ESP_AT_GetLine(const bsp_esp_at_ctx_t *ctx);

/* 清空响应流缓冲（处理完异步通知后调用，避免重复匹配） */
void Bsp_ESP_AT_ClearLine(bsp_esp_at_ctx_t *ctx);

/* 单次 Poll 最多消费的字节数（防止一帧大数据饿死主循环） */
#define BSP_ESP_AT_POLL_BUDGET   96

#endif /* __BSP_ESP_AT_H__ */
