/**
 * @file    bsp_esp_at.c
 * @brief   ESP AT 指令引擎实现（BSP — 纯逻辑）
 * @note    状态机说明：
 *            STATE_LINE  : 累积响应字节流。每字节都做两件事：
 *                          (a) strstr(line_buf, "+IPD,") — 检测 +IPD 头
 *                              找到且含 ':' 则解析长度 → 切 STATE_IPD_DATA
 *                          (b) 由 SendCmd 调用方 strstr(line_buf, expect)
 *                              匹配 OK/ERROR/>/GOT IP 等
 *                          缓冲满时做半缓冲滑窗（保留后一半），防止丢失
 *                          跨帧的 "+IPD," 头（罕见但需防御）。
 *            STATE_IPD_DATA : 按声明的 ipd_expected 长度逐字节收集。
 *                          存入 ipd_buf（超容量则丢弃并计数）。
 *                          ipd_recv == ipd_expected 时一帧完整 → 切回 STATE_LINE。
 *
 *          +IPD 帧格式（CIPMUX=0 单连接）：
 *              +IPD,<length>:<data...>
 *          头部无 \r\n，data 可为任意字节（含 \r\n），故必须长度计数，
 *          不能按行分割。本状态机正是为此设计。
 */
#include "bsp_esp_at.h"
#include <string.h>

/* ============================================================
 * 内部辅助
 * ============================================================ */

static void line_reset(bsp_esp_at_ctx_t *ctx)
{
    ctx->line_len = 0;
    if (ctx->line_size > 0) ctx->line_buf[0] = '\0';
}

/* 追加一字节到响应流缓冲；满则半缓冲滑窗 */
static void line_append(bsp_esp_at_ctx_t *ctx, uint8_t b)
{
    if (ctx->line_len + 1 < ctx->line_size) {
        ctx->line_buf[ctx->line_len++] = (char)b;
        ctx->line_buf[ctx->line_len]   = '\0';
        return;
    }
    /* 缓冲满：保留后一半（可能含未解析完的 +IPD, 头），丢弃前一半 */
    uint16_t keep = ctx->line_size / 2;
    if (keep > 0) {
        memmove(ctx->line_buf, ctx->line_buf + (ctx->line_len - keep), keep);
        ctx->line_len = keep;
        ctx->line_buf[ctx->line_len++] = (char)b;
        ctx->line_buf[ctx->line_len]   = '\0';
    }
    ctx->line_overflow_count++;
}

/* 在 line_buf 中查找 "+IPD," 并尝试解析头。
 * 找到完整头（含 ':'）则切到 STATE_IPD_DATA，返回 true。 */
static bool try_enter_ipd(bsp_esp_at_ctx_t *ctx)
{
    if (ctx->line_len < 6) return false;  /* "+IPD,:" 最少 6 字节 */

    const char *ipd = strstr((const char *)ctx->line_buf, "+IPD,");
    if (ipd == NULL) return false;

    /* 在 +IPD, 之后查找 ':' */
    uint16_t offset = (uint16_t)(ipd - (const char *)ctx->line_buf);
    char *colon = memchr(ipd + 5, ':', ctx->line_len - offset - 5);
    if (colon == NULL) return false;  /* ':' 还没到，继续累积 */

    /* 解析 ':' 之前的数字（长度字段） */
    uint16_t n = 0;
    bool ok = false;
    for (const char *p = ipd + 5; p < colon; p++) {
        if (*p < '0' || *p > '9') { ok = false; break; }
        n = (uint16_t)(n * 10 + (uint16_t)(*p - '0'));
        ok = true;
    }
    if (!ok || n == 0) {
        /* 解析失败：丢弃这个伪 +IPD,，继续 LINE 态 */
        line_reset(ctx);
        return false;
    }

    ctx->ipd_expected = n;
    ctx->ipd_len      = 0;
    ctx->ipd_recv     = 0;
    ctx->state        = BSP_ESP_AT_STATE_IPD_DATA;
    /* Preserve any AT response that preceded this unsolicited +IPD header.
     * A CIPSEND prompt/SEND OK and inbound TCP data can share one UART burst;
     * clearing the whole line here loses the TX response and causes a false
     * timeout even though the payload was sent successfully. */
    ctx->line_len = offset;
    ctx->line_buf[ctx->line_len] = '\0';
    return true;
}

/* 喂一个字节给状态机。
 * 返回 true = 一帧 +IPD 数据完整收齐（调用方应取出 ipd_buf） */
static bool feed_byte(bsp_esp_at_ctx_t *ctx, uint8_t b)
{
    bool ipd_complete = false;

    switch (ctx->state) {
    case BSP_ESP_AT_STATE_LINE:
        line_append(ctx, b);
        (void)try_enter_ipd(ctx);
        break;

    case BSP_ESP_AT_STATE_IPD_DATA:
        if (ctx->ipd_len < ctx->ipd_size) {
            ctx->ipd_buf[ctx->ipd_len++] = b;
        } else {
            ctx->ipd_overflow_count++;
        }
        ctx->ipd_recv++;
        if (ctx->ipd_recv >= ctx->ipd_expected) {
            ipd_complete = true;
            ctx->state = BSP_ESP_AT_STATE_LINE;
            /* Keep the response prefix retained by try_enter_ipd().  The
             * command owner clears it after consuming > / SEND OK. */
        }
        break;

    default:
        ctx->state = BSP_ESP_AT_STATE_LINE;
        line_reset(ctx);
        break;
    }
    return ipd_complete;
}

/* 从 UART 排空并喂入状态机，返回首个完整 +IPD 帧 */
static bool drain_uart(bsp_esp_at_ctx_t *ctx, uint16_t budget, uint16_t *out_len)
{
    while (budget-- > 0) {
        uint8_t b;
        Bsp_UART_PumpRx(ctx->uart);
        if (!Bsp_UART_ReadByte(ctx->uart, &b)) break;
        if (feed_byte(ctx, b)) {
            if (out_len) *out_len = ctx->ipd_len;
            return true;
        }
    }
    return false;
}

/* ============================================================
 * 对外 API
 * ============================================================ */

void Bsp_ESP_AT_Init(bsp_esp_at_ctx_t *ctx, bsp_uart_t *uart,
                     uint8_t *line_buf, uint16_t line_size,
                     uint8_t *ipd_buf, uint16_t ipd_size)
{
    if (ctx == NULL) return;
    ctx->uart      = uart;
    ctx->state     = BSP_ESP_AT_STATE_LINE;
    ctx->line_buf  = line_buf;
    ctx->line_size = line_size;
    ctx->line_len  = 0;
    ctx->ipd_buf   = ipd_buf;
    ctx->ipd_size  = ipd_size;
    ctx->ipd_len   = 0;
    ctx->ipd_recv  = 0;
    ctx->ipd_expected    = 0;
    ctx->at_fail_count   = 0;
    ctx->ipd_overflow_count = 0;
    ctx->line_overflow_count = 0;
    if (line_size > 0) line_buf[0] = '\0';
}

bool Bsp_ESP_AT_BeginCommand(bsp_esp_at_ctx_t *ctx, const char *cmd)
{
    if (ctx == NULL || ctx->uart == NULL || cmd == NULL || cmd[0] == '\0') {
        return false;
    }
    if (ctx->uart->tx_busy || Bsp_UART_TxPending(ctx->uart) != 0u) {
        return false;
    }

    /* Discard bytes belonging to the previous transaction before the new
     * command enters DMA, so an old ERROR cannot fail this command. */
    line_reset(ctx);
    ctx->state = BSP_ESP_AT_STATE_LINE;
    {
        uint8_t b;
        Bsp_UART_PumpRx(ctx->uart);
        while (Bsp_UART_ReadByte(ctx->uart, &b)) {}
    }

    return Bsp_UART_TrySendFrame(ctx->uart, (const uint8_t *)cmd,
                                 (uint16_t)strlen(cmd));
}

bsp_esp_at_response_t Bsp_ESP_AT_PollResponse(bsp_esp_at_ctx_t *ctx,
                                               const char *expect)
{
    if (ctx == NULL || ctx->uart == NULL || expect == NULL) {
        return BSP_ESP_AT_RESPONSE_FAILED;
    }

    uint16_t budget = BSP_ESP_AT_POLL_BUDGET;
    while (budget-- > 0u) {
        uint8_t b;
        Bsp_UART_PumpRx(ctx->uart);
        if (!Bsp_UART_ReadByte(ctx->uart, &b)) {
            break;
        }
        (void)feed_byte(ctx, b);
        if (ctx->state != BSP_ESP_AT_STATE_LINE) {
            continue;
        }
        if (strstr((const char *)ctx->line_buf, expect) != NULL) {
            return BSP_ESP_AT_RESPONSE_MATCHED;
        }
        if (strstr((const char *)ctx->line_buf, "ERROR") != NULL ||
            strstr((const char *)ctx->line_buf, "FAIL") != NULL ||
            strstr((const char *)ctx->line_buf, "CLOSED") != NULL) {
            ctx->at_fail_count++;
            return BSP_ESP_AT_RESPONSE_FAILED;
        }
    }
    return BSP_ESP_AT_RESPONSE_PENDING;
}

bool Bsp_ESP_AT_SendCmd(bsp_esp_at_ctx_t *ctx, const char *cmd,
                        const char *expect, uint32_t timeout_ms)
{
    if (ctx == NULL || ctx->uart == NULL) return false;

    /* 起始：清空响应流缓冲与状态机 */
    line_reset(ctx);
    ctx->state = BSP_ESP_AT_STATE_LINE;

    /* Publish in-flight DMA bytes before discarding the previous command's
     * tail; otherwise stale ERROR text can race the new command response. */
    {
        uint8_t b;
        Bsp_UART_PumpRx(ctx->uart);
        while (Bsp_UART_ReadByte(ctx->uart, &b)) {}
    }

    /* 发送指令 */
    if (cmd != NULL) {
        if (ctx->uart->SendCmdBlocking != NULL) {
            ctx->uart->SendCmdBlocking((const uint8_t *)cmd, (uint16_t)strlen(cmd));
        } else {
            Bsp_UART_Send_Block(ctx->uart, (const uint8_t *)cmd, (uint16_t)strlen(cmd));
        }
    }

    /* 阻塞等待响应 */
    uint32_t start = Bsp_GetTick();
    while ((Bsp_GetTick() - start) < timeout_ms) {
        uint8_t b;
        Bsp_UART_PumpRx(ctx->uart);
        while (Bsp_UART_ReadByte(ctx->uart, &b)) {
            (void)feed_byte(ctx, b);
            /* 仅在 LINE 态做匹配（IPD_DATA 态说明正在收数据，不应在指令期出现） */
            if (ctx->state == BSP_ESP_AT_STATE_LINE && expect != NULL) {
                if (strstr((const char *)ctx->line_buf, expect) != NULL) {
                    return true;
                }
                if (strstr((const char *)ctx->line_buf, "ERROR") != NULL ||
                    strstr((const char *)ctx->line_buf, "FAIL")  != NULL) {
                    ctx->at_fail_count++;
                    return false;
                }
            }
        }
        Bsp_Delay_ms(1);
    }
    ctx->at_fail_count++;
    return false;
}

bool Bsp_ESP_AT_SendCmdRetry(bsp_esp_at_ctx_t *ctx, const char *cmd,
                             const char *expect, uint32_t timeout_ms,
                             uint8_t retries, uint32_t retry_delay_ms)
{
    for (uint8_t i = 0; i < retries; i++) {
        if (Bsp_ESP_AT_SendCmd(ctx, cmd, expect, timeout_ms)) {
            return true;
        }
        if (retry_delay_ms > 0) Bsp_Delay_ms(retry_delay_ms);
    }
    return false;
}

bool Bsp_ESP_AT_WaitResponse(bsp_esp_at_ctx_t *ctx, const char *expect,
                             uint32_t timeout_ms)
{
    if (ctx == NULL || ctx->uart == NULL || expect == NULL) return false;

    uint32_t start = Bsp_GetTick();
    while ((Bsp_GetTick() - start) < timeout_ms) {
        uint8_t b;
        Bsp_UART_PumpRx(ctx->uart);
        while (Bsp_UART_ReadByte(ctx->uart, &b)) {
            (void)feed_byte(ctx, b);
            if (ctx->state == BSP_ESP_AT_STATE_LINE) {
                if (strstr((const char *)ctx->line_buf, expect) != NULL) {
                    return true;
                }
                if (strstr((const char *)ctx->line_buf, "ERROR") != NULL ||
                    strstr((const char *)ctx->line_buf, "FAIL")  != NULL ||
                    strstr((const char *)ctx->line_buf, "SEND FAIL") != NULL) {
                    ctx->at_fail_count++;
                    return false;
                }
                /* link closed during send */
                if (strstr((const char *)ctx->line_buf, "CLOSED") != NULL) {
                    ctx->at_fail_count++;
                    return false;
                }
            }
        }
        Bsp_Delay_ms(1);
    }
    ctx->at_fail_count++;
    return false;
}

void Bsp_ESP_AT_SendRaw(bsp_esp_at_ctx_t *ctx, const uint8_t *data, uint16_t len)
{
    if (ctx == NULL || ctx->uart == NULL || data == NULL || len == 0) return;
    Bsp_UART_Send_Block(ctx->uart, data, len);
}

void Bsp_ESP_AT_SendString(bsp_esp_at_ctx_t *ctx, const char *s)
{
    if (ctx == NULL || s == NULL) return;
    Bsp_UART_SendString(ctx->uart, s);
}

bool Bsp_ESP_AT_Poll(bsp_esp_at_ctx_t *ctx, uint16_t *out_len)
{
    if (ctx == NULL || ctx->uart == NULL) return false;
    return drain_uart(ctx, BSP_ESP_AT_POLL_BUDGET, out_len);
}

const char *Bsp_ESP_AT_GetLine(const bsp_esp_at_ctx_t *ctx)
{
    if (ctx == NULL) return NULL;
    return (const char *)ctx->line_buf;
}

void Bsp_ESP_AT_ClearLine(bsp_esp_at_ctx_t *ctx)
{
    if (ctx == NULL) return;
    line_reset(ctx);
}
