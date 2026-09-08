/**
 * @file    app_esp01s.c
 * @brief   ESP-01S 应用层实现（非阻塞连接状态机 + 3 级优先级 TX 队列 + JSON 收发）
 * @note    分层与 app_uart.c 完全对称：
 *            1) 缓冲区定义（DMA RX 暂存 / RX FIFO / TX FIFO）
 *            2) HW 钩子实现（Init / StartTxChunk / StopTx / RestartRx）—— 绑 UART1+DMA CH2/CH3
 *            3) 对象实例化（ESP_Uart_Dev: bsp_uart_t / g_esp_at: bsp_esp_at_ctx_t）
 *            4) 连接状态机（BOOT→AT_INIT→WIFI→TCP→ONLINE→BACKOFF，退避 1/2/4/8/30s）
 *            5) TX 队列（HIGH>TELEMETRY>LOG 三级优先级）
 *            6) 业务切入（Init / Poll / SendJSON入队 / RegisterRx）
 *            7) 中断回调（供 main.c ISR 调用）
 *
 *          连接状态机每轮 Poll 推进一步：
 *            BOOT → AT_INIT(AT/ATE0/CWMODE/CWDHCP) → WIFI(CWJAP) → TCP(CIPCLOSE/CIPMUX/CIPMODE/CIPSTART) → ONLINE
 *                                                                                                                  ↓
 *                                                                                               BACKOFF(1/2/4/8/30s) → AT_INIT
 *
 *          TX 优先级（固定）：
 *            HIGH(ack/nack) > telemetry/log fair-share > background pong
 */
#include "app_esp01s.h"
#include "app_common.h"
#include "app_log.h"
#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdio.h>
#include "elog.h"

#undef LOG_TAG
#define LOG_TAG  "esp01s"

/* TX 队列配置 */
#define ESP01S_TX_HIGH_SLOTS        3u   /* HIGH(ack/nack/pong) 槽位：永不丢弃 */
#define ESP01S_TX_LOG_SLOTS         2u   /* LOG 槽位：满时丢弃 */
#define ESP01S_TX_MAX_NONLOG_STREAK 1u   /* Prevent telemetry from starving logs. */
#define ESP01S_TX_FRAME_MAX         ESP01S_IPD_BUF_SIZE
#define ESP01S_TX_PROMPT_TIMEOUT    1000u
#define ESP01S_TX_SEND_TIMEOUT      ESP01S_AT_TIMEOUT_SEND
#define ESP01S_TX_RECOVERY_TIMEOUT  3000u
#define ESP01S_TX_GAP_MS            100u
#define ESP01S_TX_CHUNK_BYTES         64u
#define ESP01S_TX_CHUNK_GAP_MS        25u
#define ESP01S_TCP_SETTLE_MS         750u
#define ESP01S_RX_TO_TX_QUIET_MS      25u
#define ESP01S_ESCAPE_FLUSH_MS       100u
#define ESP01S_BAUD_115200          115200u

/* 连接状态机退避序列 (ms) */
static const uint32_t s_backoff_ms[] = {1000u, 2000u, 4000u, 8000u, 30000u};
#define ESP01S_BACKOFF_TABLE_SIZE  (sizeof(s_backoff_ms) / sizeof(s_backoff_ms[0]))

/* ============================================================
 * 1. 缓冲区定义
 * ============================================================ */

/* DMA 接收暂存（供 main.c ISR 访问，非 static） */
uint8_t g_esp_rx_dma_buf[ESP01S_DMA_RX_BUF_SIZE];
static uint32_t s_uart_at_tx_bytes = 0;
/* Number of bytes in the current DMA buffer already published to the FIFO.
 * The main-loop pump and the DMA-complete ISR share this cursor so a byte is
 * never delivered twice when a response crosses the DMA completion boundary. */
static volatile uint16_t s_esp_rx_dma_published = 0;

static bool bytes_contains(const uint8_t *data, uint16_t len,
                           const char *needle, uint16_t needle_len)
{
    if (data == NULL || needle == NULL || needle_len == 0 || len < needle_len) {
        return false;
    }
    for (uint16_t i = 0; i <= (uint16_t)(len - needle_len); i++) {
        if (memcmp(&data[i], needle, needle_len) == 0) {
            return true;
        }
    }
    return false;
}

/* RX 软件 FIFO（环形） */
static uint8_t s_esp_rx_fifo_buf[ESP01S_RX_FIFO_SIZE];

/* TX 软件 FIFO（环形）—— DMA 直接读此处，零拷贝 */
static uint8_t s_esp_tx_fifo_buf[ESP01S_TX_FIFO_SIZE];

/* AT 引擎缓冲 */
static uint8_t s_esp_line_buf[ESP01S_LINE_BUF_SIZE];
static uint8_t s_esp_ipd_buf[ESP01S_IPD_BUF_SIZE];
static uint16_t s_transparent_rx_len = 0;
static bool s_transparent_mode = false;

extern bsp_uart_t ESP_Uart_Dev;
static void HW_ESP_UART_SendCmdBlocking(const uint8_t *data, uint16_t len);
static void HW_ESP_UART_PumpRx(void);

/* ============================================================
 * 2. HW 钩子实现（绑 UART1 + DMA CH2 RX / CH3 TX）
 *    与 app_uart.c 中 UART0 的钩子完全对称，仅替换外设实例与 DMA 通道
 * ============================================================ */

static void HW_ESP_UART_Init(void)
{
    /* SysConfig 已完成 UART1 GPIO/波特率/FIFO/RX Timeout/DMA 通道配置
     * 此处仅补充 DMA 缓冲区地址、传输长度，并启动 DMA 接收 */
    DL_DMA_setSrcAddr(DMA, ESP01S_DMA_CH_RX, (uint32_t)&ESP01S_UART_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, ESP01S_DMA_CH_RX, (uint32_t)g_esp_rx_dma_buf);
    DL_DMA_setTransferSize(DMA, ESP01S_DMA_CH_RX, ESP01S_DMA_RX_BUF_SIZE);
    s_esp_rx_dma_published = 0;
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL2);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL2);
    DL_DMA_enableChannel(DMA, ESP01S_DMA_CH_RX);

    /* 显式使能 UART1/DMA NVIC 中断，确保 RX Timeout 与 TX DMA done 都能进 ISR。 */
    NVIC_SetPriority(ESP01S_UART_INT_IRQn, 5);
    NVIC_EnableIRQ(ESP01S_UART_INT_IRQn);
    NVIC_SetPriority(DMA_INT_IRQn, 4);
    NVIC_EnableIRQ(DMA_INT_IRQn);
}

static void HW_ESP_UART_RestartRx(void)
{
    DL_DMA_disableChannel(DMA, ESP01S_DMA_CH_RX);
    DL_DMA_setDestAddr(DMA, ESP01S_DMA_CH_RX, (uint32_t)g_esp_rx_dma_buf);
    DL_DMA_setTransferSize(DMA, ESP01S_DMA_CH_RX, ESP01S_DMA_RX_BUF_SIZE);
    s_esp_rx_dma_published = 0;
    DL_DMA_enableChannel(DMA, ESP01S_DMA_CH_RX);
}

/* DMA completion only occurs after a full buffer. Poll the in-flight transfer
 * for short AT responses, atomically publishing the newly written tail. */
static void HW_ESP_UART_PumpRx(void)
{
    uint16_t received = ESP01S_DMA_RX_BUF_SIZE -
                        DL_DMA_getTransferSize(DMA, ESP01S_DMA_CH_RX);
    if ((received == 0 || received <= s_esp_rx_dma_published) &&
        DL_UART_isRXFIFOEmpty(ESP01S_UART_INST)) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    DL_DMA_disableChannel(DMA, ESP01S_DMA_CH_RX);
    received = ESP01S_DMA_RX_BUF_SIZE -
               DL_DMA_getTransferSize(DMA, ESP01S_DMA_CH_RX);

    if (received > s_esp_rx_dma_published &&
        received < ESP01S_DMA_RX_BUF_SIZE) {
        Bsp_UART_Rx_PushStreamBytes(&ESP_Uart_Dev,
                                    &g_esp_rx_dma_buf[s_esp_rx_dma_published],
                                    (uint16_t)(received - s_esp_rx_dma_published));
        s_esp_rx_dma_published = received;
    }

    /* A CIPSEND prompt can be only one byte.  With the UART DMA trigger set
     * above one byte, that prompt (or the tail of an OK response) may remain
     * in the peripheral FIFO and never change DMASZ.  Publish those bytes
     * while DMA is stopped so command responses cannot be lost or shifted
     * into the following AT transaction. */
    while (!DL_UART_isRXFIFOEmpty(ESP01S_UART_INST)) {
        uint8_t byte = (uint8_t)DL_UART_receiveData(ESP01S_UART_INST);
        Bsp_UART_Rx_PushStreamBytes(&ESP_Uart_Dev, &byte, 1u);
    }
    if (received < ESP01S_DMA_RX_BUF_SIZE) {
        DL_DMA_enableChannel(DMA, ESP01S_DMA_CH_RX);
    }
    /* A full transfer is left for the DMA ISR, which publishes its remaining
     * tail and starts the next buffer. */
    __set_PRIMASK(primask);
}

static void HW_ESP_UART_StartTxChunk(const uint8_t *src, uint16_t len)
{
    if (len == 0) return;
    DL_DMA_setSrcAddr(DMA, ESP01S_DMA_CH_TX, (uint32_t)src);
    DL_DMA_setDestAddr(DMA, ESP01S_DMA_CH_TX, (uint32_t)&ESP01S_UART_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, ESP01S_DMA_CH_TX, len);
    DL_UART_enableDMATransmitEvent(ESP01S_UART_INST);
    DL_DMA_enableChannel(DMA, ESP01S_DMA_CH_TX);
}

static void HW_ESP_UART_StopTx(void)
{
    DL_UART_disableDMATransmitEvent(ESP01S_UART_INST);
}

/* ============================================================
 * 3. 对象实例化
 * ============================================================ */

bsp_uart_t ESP_Uart_Dev = {
    .Init         = HW_ESP_UART_Init,
    .StartTxChunk = HW_ESP_UART_StartTxChunk,
    .StopTx       = HW_ESP_UART_StopTx,
    .RestartRx    = HW_ESP_UART_RestartRx,
    .PumpRx       = HW_ESP_UART_PumpRx,
    .SendCmdBlocking = HW_ESP_UART_SendCmdBlocking,

    .rx_fifo_buf  = s_esp_rx_fifo_buf,
    .rx_fifo_size = ESP01S_RX_FIFO_SIZE,
    .rx_fifo_head = 0,
    .rx_fifo_tail = 0,

    .rx_frame_head = 0,
    .rx_frame_tail = 0,

    .tx_fifo_buf  = s_esp_tx_fifo_buf,
    .tx_fifo_size = ESP01S_TX_FIFO_SIZE,
    .tx_fifo_head = 0,
    .tx_fifo_tail = 0,
    .tx_chunk_len = 0,
    .tx_busy      = false,

    .tx_overflow       = false,
    .rx_overflow       = false,
    .rx_frame_overflow = false,
};

static void HW_ESP_UART_SendCmdBlocking(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return;

    for (uint16_t i = 0; i < len; i++) {
        uint32_t start = Bsp_GetTick();
        while (DL_UART_Main_isTXFIFOFull(ESP01S_UART_INST)) {
            if ((Bsp_GetTick() - start) > 20u) {
                APP_ESP_LOG_W(LOG_TAG, "AT TX timeout at byte %u", (unsigned)i);
                return;
            }
        }
        DL_UART_Main_transmitData(ESP01S_UART_INST, data[i]);
        s_uart_at_tx_bytes++;
    }
}

static bsp_esp_at_ctx_t g_esp_at;

/* ============================================================
 * 4. 连接状态机（BOOT → AT_INIT → WIFI → TCP → ONLINE → BACKOFF）
 * ============================================================ */

typedef enum {
    ESP_CONN_BOOT = 0,      /* 上电延时等待（非阻塞） */
    ESP_CONN_ESCAPE,        /* Attempt to escape an ESP left in transparent mode */
    ESP_CONN_AT_INIT,       /* AT + ATE0 + CWMODE + CWDHCP（逐步，每轮一条 AT） */
    ESP_CONN_WIFI,          /* CWJAP 连 AP */
    ESP_CONN_TCP,           /* CIPCLOSE + CIPMUX + CIPMODE + CIPSTART */
    ESP_CONN_ONLINE,        /* 已连线：非阻塞 IPD 轮询 + TX 状态机 */
    ESP_CONN_BACKOFF,       /* 退避等待 1/2/4/8/30s */
} esp_conn_state_t;

static esp_conn_state_t s_conn_state = ESP_CONN_BOOT;
static uint8_t  s_conn_step = 0;       /* 当前子步骤索引 */
static uint32_t s_conn_deadline = 0;   /* BOOT/BACKOFF 等待截止时刻 */
static uint8_t  s_backoff_idx = 0;     /* 退避序列索引 */

static volatile bool s_wifi_connected = false;
static volatile bool s_tcp_connected  = false;
static App_ESP01S_RxCallback s_rx_callback = NULL;
static App_ESP01S_ProgressCb s_progress_cb = NULL;

/* 诊断计数 */
static uint32_t s_tx_drop_count = 0;
static uint32_t s_tx_telemetry_overwrite_count = 0;
static uint32_t s_ipd_dispatch_count = 0;
static uint32_t s_uart_baud = ESP01S_BAUD_115200;
static uint32_t s_conn_fail_count = 0;

static bool s_conn_at_waiting = false;
static uint32_t s_conn_at_deadline = 0;

static void tx_queue_clear(void);

typedef enum {
    CONN_AT_PENDING = 0,
    CONN_AT_OK,
    CONN_AT_FAILED,
} conn_at_result_t;

static void conn_at_cancel(void)
{
    s_conn_at_waiting = false;
}

static bool conn_at_start(const char *cmd, uint32_t timeout_ms)
{
    if (!Bsp_ESP_AT_BeginCommand(&g_esp_at, cmd)) {
        return false;
    }
    s_conn_at_waiting = true;
    s_conn_at_deadline = Bsp_GetTick() + timeout_ms;
    return true;
}

static conn_at_result_t conn_at_poll(const char *expect)
{
    bsp_esp_at_response_t result;

    if (!s_conn_at_waiting) {
        return CONN_AT_FAILED;
    }
    result = Bsp_ESP_AT_PollResponse(&g_esp_at, expect);
    if (result == BSP_ESP_AT_RESPONSE_MATCHED) {
        s_conn_at_waiting = false;
        return CONN_AT_OK;
    }
    if (result == BSP_ESP_AT_RESPONSE_FAILED ||
        (int32_t)(Bsp_GetTick() - s_conn_at_deadline) >= 0) {
        if (result == BSP_ESP_AT_RESPONSE_PENDING) {
            g_esp_at.at_fail_count++;
        }
        s_conn_at_waiting = false;
        return CONN_AT_FAILED;
    }
    return CONN_AT_PENDING;
}

static void esp_log_at_diag(const char *stage)
{
    APP_ESP_LOG_W(LOG_TAG,
           "%s diag: baud=%lu at_fail=%lu at_tx=%lu fifo_rx=%lu rx_avail=%u rx_frames=%u tx_pending=%u tx_busy=%u tx_over=%u rx_over=%u rx_frame_over=%u line='%s'",
           stage,
           (unsigned long)s_uart_baud,
           (unsigned long)g_esp_at.at_fail_count,
           (unsigned long)s_uart_at_tx_bytes,
           0ul,
           (unsigned)Bsp_UART_Available(&ESP_Uart_Dev),
           (unsigned)Bsp_UART_Frame_Count(&ESP_Uart_Dev),
           (unsigned)Bsp_UART_TxPending(&ESP_Uart_Dev),
           ESP_Uart_Dev.tx_busy ? 1u : 0u,
           ESP_Uart_Dev.tx_overflow ? 1u : 0u,
           ESP_Uart_Dev.rx_overflow ? 1u : 0u,
           ESP_Uart_Dev.rx_frame_overflow ? 1u : 0u,
           Bsp_ESP_AT_GetLine(&g_esp_at));
    APP_ESP_LOG_W(LOG_TAG,
           "UART1 pins: PA8_TX PINCM=0x%08lX PA9_RX PINCM=0x%08lX GPIOA DIN=0x%08lX DOUT=0x%08lX DOE=0x%08lX",
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_UART_ESP01S_IOMUX_TX],
           (unsigned long)IOMUX->SECCFG.PINCM[GPIO_UART_ESP01S_IOMUX_RX],
           (unsigned long)GPIOA->DIN31_0,
           (unsigned long)GPIOA->DOUT31_0,
           (unsigned long)GPIOA->DOE31_0);
}

static void conn_notify_progress(uint8_t step, const char *msg)
{
    if (s_progress_cb != NULL) {
        s_progress_cb(step, msg);
    }
}

static void conn_enter_backoff(void)
{
    conn_at_cancel();
    s_conn_fail_count++;
    s_wifi_connected = false;
    s_tcp_connected  = false;
    s_conn_state = ESP_CONN_BACKOFF;
    s_conn_deadline = Bsp_GetTick() + s_backoff_ms[s_backoff_idx];
    APP_ESP_LOG_W(LOG_TAG, "enter backoff %lus (idx=%u fails=%lu)",
           (unsigned long)(s_backoff_ms[s_backoff_idx] / 1000u),
           (unsigned)s_backoff_idx,
           (unsigned long)s_conn_fail_count);
    conn_notify_progress(0xFF, "backoff");
}

static void conn_enter_escape(const char *reason)
{
    conn_at_cancel();
    tx_queue_clear();
    s_transparent_mode = false;
    s_transparent_rx_len = 0;
    s_wifi_connected = false;
    s_tcp_connected  = false;
    s_conn_state = ESP_CONN_ESCAPE;
    s_conn_step = 0;
    s_conn_deadline = Bsp_GetTick() + ESP01S_ESCAPE_GUARD_MS;
    if (reason != NULL) {
        APP_ESP_LOG_I(LOG_TAG, "%s, guard before escape", reason);
    }
    conn_notify_progress(1, "ESP recovery");
}

/* Reaching this point proves the AT parser responded after the escape. */
static void conn_enter_at_init_after_escape(void)
{
    s_conn_state = ESP_CONN_AT_INIT;
    s_conn_step = 1;  /* The escape state has already completed the AT probe. */
    s_wifi_connected = false;
    s_tcp_connected = false;
    conn_notify_progress(1, "AT init");
}

static void conn_enter_at_init(void)
{
    /* Every new AT recovery attempt begins by leaving a possible passthrough
     * session. A reset of the MSPM0 does not reset the ESP-01S itself. */
    conn_enter_escape("AT recovery");
}

/* ============================================================
 * 5. TX 队列（三级优先级：HIGH > TELEMETRY > LOG）
 * ============================================================ */

typedef enum {
    TX_PRIO_HIGH = 0,       /* ack/nack/pong — 永不丢弃 */
    TX_PRIO_TELEMETRY,      /* tele — 仅保留最新 */
    TX_PRIO_LOG,            /* log — 队列满时丢弃 */
    TX_PRIO_HEARTBEAT,      /* pong — no safety dependency on delivery */
} tx_prio_t;

typedef struct {
    uint8_t  data[ESP01S_TX_FRAME_MAX];
    uint16_t len;
} esp_tx_frame_t;

/* HIGH 优先级环形队列（FIFO） */
static esp_tx_frame_t s_tx_high[ESP01S_TX_HIGH_SLOTS];
static uint8_t s_tx_high_head = 0;
static uint8_t s_tx_high_tail = 0;
static uint8_t s_tx_high_count = 0;

/* TELEMETRY 优先级：单槽，最新覆盖 */
static esp_tx_frame_t s_tx_tele;
static bool s_tx_tele_valid = false;

/* LOG 优先级环形队列（FIFO） */
static esp_tx_frame_t s_tx_log[ESP01S_TX_LOG_SLOTS];
static uint8_t s_tx_log_head = 0;
static uint8_t s_tx_log_tail = 0;
static uint8_t s_tx_log_count = 0;

/* Pongs are observable but never allowed to displace data-bearing uplink. */
static esp_tx_frame_t s_tx_heartbeat;
static bool s_tx_heartbeat_valid = false;
static uint8_t s_tx_nonlog_streak = 0;

/* TX 发送状态机 */
typedef enum {
    ESP_TX_IDLE = 0,
    ESP_TX_WAIT_PROMPT,
    ESP_TX_WAIT_SEND_OK
} esp_tx_state_t;

static uint8_t        s_tx_active[ESP01S_TX_FRAME_MAX];
static uint16_t       s_tx_active_len = 0;
static uint16_t       s_tx_active_offset = 0;
static esp_tx_state_t s_tx_state = ESP_TX_IDLE;
static uint32_t       s_tx_deadline = 0;
static uint32_t       s_tx_next_allowed = 0;
static uint8_t        s_tx_consecutive_failures = 0;
static bool           s_tx_prompt_recovery = false;

/* 优先级检测：扫描 JSON 内容判断优先级 */
static tx_prio_t detect_priority(const uint8_t *data, uint16_t len)
{
    /* 匹配 "t":"xxx" 中的 xxx */
    static const char needle[] = "\"t\":\"";
    if (data == NULL || len < (uint16_t)(sizeof(needle) - 1 + 3)) {
        return TX_PRIO_LOG;
    }
    for (uint16_t i = 0; i <= (uint16_t)(len - (sizeof(needle) - 1 + 3)); i++) {
        if (memcmp(&data[i], needle, sizeof(needle) - 1) == 0) {
            const char *tag = (const char *)&data[i + (sizeof(needle) - 1)];
            if (memcmp(tag, "ack", 3) == 0 || memcmp(tag, "nac", 3) == 0) {
                return TX_PRIO_HIGH;
            }
            if (memcmp(tag, "tel", 3) == 0) {
                return TX_PRIO_TELEMETRY;
            }
            if (memcmp(tag, "pon", 3) == 0) {
                return TX_PRIO_HEARTBEAT;
            }
            return TX_PRIO_LOG;
        }
    }
    return TX_PRIO_LOG;
}

static void tx_queue_clear(void)
{
    s_tx_high_head = 0;
    s_tx_high_tail = 0;
    s_tx_high_count = 0;
    s_tx_tele_valid = false;
    s_tx_log_head = 0;
    s_tx_log_tail = 0;
    s_tx_log_count = 0;
    s_tx_heartbeat_valid = false;
    s_tx_nonlog_streak = 0;
    s_tx_active_len = 0;
    s_tx_active_offset = 0;
    s_tx_state = ESP_TX_IDLE;
    s_tx_next_allowed = 0;
    s_tx_consecutive_failures = 0;
    s_tx_prompt_recovery = false;
}

static bool tx_queue_push(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len > ESP01S_TX_FRAME_MAX) {
        s_tx_drop_count++;
        return false;
    }

    tx_prio_t prio = detect_priority(data, len);

    switch (prio) {
    case TX_PRIO_HIGH:
        /* HIGH 永不丢弃；满时覆盖最旧 */
        if (s_tx_high_count >= ESP01S_TX_HIGH_SLOTS) {
            memcpy(s_tx_high[s_tx_high_head].data, data, len);
            s_tx_high[s_tx_high_head].len = len;
            s_tx_high_head = (uint8_t)((s_tx_high_head + 1) % ESP01S_TX_HIGH_SLOTS);
            s_tx_high_tail = s_tx_high_head;  /* 覆盖最旧 */
        } else {
            memcpy(s_tx_high[s_tx_high_tail].data, data, len);
            s_tx_high[s_tx_high_tail].len = len;
            s_tx_high_tail = (uint8_t)((s_tx_high_tail + 1) % ESP01S_TX_HIGH_SLOTS);
            s_tx_high_count++;
        }
        return true;

    case TX_PRIO_TELEMETRY:
        /* TELEMETRY 仅保留最新 */
        memcpy(s_tx_tele.data, data, len);
        s_tx_tele.len = len;
        s_tx_tele_valid = true;
        s_tx_telemetry_overwrite_count++;
        return true;

    case TX_PRIO_LOG:
        /* LOG 满时丢弃 */
        if (s_tx_log_count >= ESP01S_TX_LOG_SLOTS) {
            s_tx_drop_count++;
            return false;
        }
        memcpy(s_tx_log[s_tx_log_tail].data, data, len);
        s_tx_log[s_tx_log_tail].len = len;
        s_tx_log_tail = (uint8_t)((s_tx_log_tail + 1) % ESP01S_TX_LOG_SLOTS);
        s_tx_log_count++;
        return true;

    case TX_PRIO_HEARTBEAT:
        memcpy(s_tx_heartbeat.data, data, len);
        s_tx_heartbeat.len = len;
        s_tx_heartbeat_valid = true;
        return true;

    default:
        s_tx_drop_count++;
        return false;
    }
}

/* 按优先级弹出：HIGH → TELEMETRY → LOG */
static bool tx_queue_pop(uint8_t *out, uint16_t *out_len)
{
    if (out == NULL || out_len == NULL) return false;

    /* HIGH 优先 */
    if (s_tx_high_count > 0) {
        *out_len = s_tx_high[s_tx_high_head].len;
        memcpy(out, s_tx_high[s_tx_high_head].data, *out_len);
        s_tx_high_head = (uint8_t)((s_tx_high_head + 1) % ESP01S_TX_HIGH_SLOTS);
        s_tx_high_count--;
        return true;
    }

    /* Logs have a bounded latency even while 10 Hz telemetry is active. */
    if (s_tx_log_count > 0 &&
        s_tx_nonlog_streak >= ESP01S_TX_MAX_NONLOG_STREAK) {
        *out_len = s_tx_log[s_tx_log_head].len;
        memcpy(out, s_tx_log[s_tx_log_head].data, *out_len);
        s_tx_log_head = (uint8_t)((s_tx_log_head + 1) % ESP01S_TX_LOG_SLOTS);
        s_tx_log_count--;
        s_tx_nonlog_streak = 0;
        return true;
    }

    /* TELEMETRY 次之 */
    if (s_tx_tele_valid) {
        *out_len = s_tx_tele.len;
        memcpy(out, s_tx_tele.data, *out_len);
        s_tx_tele_valid = false;
        s_tx_nonlog_streak++;
        return true;
    }

    /* LOG 最后 */
    if (s_tx_log_count > 0) {
        *out_len = s_tx_log[s_tx_log_head].len;
        memcpy(out, s_tx_log[s_tx_log_head].data, *out_len);
        s_tx_log_head = (uint8_t)((s_tx_log_head + 1) % ESP01S_TX_LOG_SLOTS);
        s_tx_log_count--;
        s_tx_nonlog_streak = 0;
        return true;
    }

    if (s_tx_heartbeat_valid) {
        *out_len = s_tx_heartbeat.len;
        memcpy(out, s_tx_heartbeat.data, *out_len);
        s_tx_heartbeat_valid = false;
        return true;
    }

    return false;
}

static void tx_state_fail(const char *reason)
{
    bool prompt_recovery_failed = s_tx_prompt_recovery;

    s_tx_drop_count++;
    if (s_tx_consecutive_failures < UINT8_MAX) {
        s_tx_consecutive_failures++;
    }
    s_tx_state = ESP_TX_IDLE;
    s_tx_active_len = 0;
    s_tx_active_offset = 0;
    s_tx_prompt_recovery = false;
    s_tx_next_allowed = Bsp_GetTick() + ESP01S_TX_SEND_TIMEOUT;
    if (reason != NULL) {
        esp_log_at_diag(reason);
        APP_ESP_LOG_W(LOG_TAG, "TX async fail: %s", reason);
    }
    bool link_closed = (Bsp_ESP_AT_GetLine(&g_esp_at) != NULL &&
                        strstr(Bsp_ESP_AT_GetLine(&g_esp_at), "CLOSED") != NULL);
    Bsp_ESP_AT_ClearLine(&g_esp_at);
    /* Prompt loss gets one payload-completion attempt before reaching here.
     * Any remaining TX error means the AT/TCP session is no longer trustworthy,
     * so recover immediately instead of repeating CIPSEND against stale state. */
    APP_ESP_LOG_W(LOG_TAG,
           "TX recovery: escape then reconnect (closed=%u prompt_recovery=%u fails=%u)",
           link_closed ? 1u : 0u, prompt_recovery_failed ? 1u : 0u,
           (unsigned)s_tx_consecutive_failures);
    conn_enter_at_init();
}

static void tx_state_poll(void)
{
    if (!s_tcp_connected) {
        tx_queue_clear();
        return;
    }

    const char *line = Bsp_ESP_AT_GetLine(&g_esp_at);
    uint32_t now = Bsp_GetTick();

    if (s_transparent_mode) {
        if (s_tx_state == ESP_TX_IDLE &&
            (int32_t)(now - s_tx_next_allowed) >= 0) {
            /* Online frames can be hundreds of bytes.  Polling them into the
             * UART FIFO at 115200 baud stalls the bare-metal scheduler for
             * longer than the 25 ms control safety limit. Keep a popped frame
             * active until the UART FIFO has room for the entire frame;
             * Bsp_UART_Send is byte-oriented and would otherwise enqueue a
             * truncated TC1 line when the FIFO becomes full. */
            if (s_tx_active_len == 0u) {
                if (tx_queue_pop(s_tx_active, &s_tx_active_len)) {
                    s_tx_active_offset = 0u;
                }
            }
            if (s_tx_active_len > 0u) {
                uint16_t remaining = (uint16_t)(s_tx_active_len -
                                                s_tx_active_offset);
                uint16_t chunk = remaining > ESP01S_TX_CHUNK_BYTES ?
                                 ESP01S_TX_CHUNK_BYTES : remaining;
                if (Bsp_UART_TrySendFrame(&ESP_Uart_Dev,
                                           &s_tx_active[s_tx_active_offset],
                                           chunk)) {
                    s_tx_active_offset = (uint16_t)(s_tx_active_offset + chunk);
                    if (s_tx_active_offset >= s_tx_active_len) {
                        s_tx_active_len = 0u;
                        s_tx_active_offset = 0u;
                        s_tx_next_allowed = now + ESP01S_TX_GAP_MS;
                    } else {
                        s_tx_next_allowed = now + ESP01S_TX_CHUNK_GAP_MS;
                    }
                }
                s_tx_consecutive_failures = 0;
            }
        }
        return;
    }

    switch (s_tx_state) {
    case ESP_TX_IDLE:
        if ((int32_t)(now - s_tx_next_allowed) < 0) {
            break;
        }
        if (tx_queue_pop(s_tx_active, &s_tx_active_len)) {
            char cmd[24];
            snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u\r\n",
                     (unsigned)s_tx_active_len);
            Bsp_ESP_AT_ClearLine(&g_esp_at);
            HW_ESP_UART_SendCmdBlocking((const uint8_t *)cmd,
                                        (uint16_t)strlen(cmd));
            s_tx_deadline = now + ESP01S_TX_PROMPT_TIMEOUT;
            s_tx_prompt_recovery = false;
            s_tx_state = ESP_TX_WAIT_PROMPT;
        }
        break;

    case ESP_TX_WAIT_PROMPT:
        if (line != NULL && strstr(line, ">") != NULL) {
            Bsp_ESP_AT_ClearLine(&g_esp_at);
            HW_ESP_UART_SendCmdBlocking(s_tx_active, s_tx_active_len);
            s_tx_deadline = now + ESP01S_TX_SEND_TIMEOUT;
            s_tx_state = ESP_TX_WAIT_SEND_OK;
        } else if (line != NULL &&
                   (strstr(line, "ERROR") != NULL ||
                    strstr(line, "FAIL") != NULL ||
                    strstr(line, "CLOSED") != NULL)) {
            tx_state_fail("prompt error");
        } else if ((int32_t)(now - s_tx_deadline) >= 0) {
            /* The ESP may have accepted CIPSEND while the one-byte prompt was
             * lost on UART. Complete the declared fixed-length payload first;
             * this also makes a later +++ escape safe for either CIPMODE. */
            APP_ESP_LOG_W(LOG_TAG, "prompt timeout: send payload before escape recovery");
            Bsp_ESP_AT_ClearLine(&g_esp_at);
            HW_ESP_UART_SendCmdBlocking(s_tx_active, s_tx_active_len);
            s_tx_deadline = now + ESP01S_TX_RECOVERY_TIMEOUT;
            s_tx_prompt_recovery = true;
            s_tx_state = ESP_TX_WAIT_SEND_OK;
        }
        break;

    case ESP_TX_WAIT_SEND_OK:
        if (line != NULL &&
            (strstr(line, "SEND OK") != NULL || strstr(line, "Recv ") != NULL)) {
            Bsp_ESP_AT_ClearLine(&g_esp_at);
            s_tx_active_len = 0;
            s_tx_state = ESP_TX_IDLE;
            s_tx_next_allowed = now + ESP01S_TX_GAP_MS;
            s_tx_consecutive_failures = 0;
            s_tx_prompt_recovery = false;
        } else if (line != NULL &&
                   (strstr(line, "ERROR") != NULL ||
                    strstr(line, "FAIL") != NULL ||
                    strstr(line, "SEND FAIL") != NULL ||
                    strstr(line, "CLOSED") != NULL)) {
            tx_state_fail("send error");
        } else if ((int32_t)(now - s_tx_deadline) >= 0) {
            tx_state_fail("send timeout");
        }
        break;

    default:
        s_tx_state = ESP_TX_IDLE;
        break;
    }
}

/* ============================================================
 * 6. 连接状态机逐步推进（每轮 Poll 调用一次）
 * ============================================================ */

static void conn_step_boot(void)
{
    uint32_t now = Bsp_GetTick();
    if ((int32_t)(now - s_conn_deadline) >= 0) {
        conn_enter_escape("boot delay done");
    }
}

static void conn_step_escape(void)
{
    uint32_t now = Bsp_GetTick();

    if ((int32_t)(now - s_conn_deadline) < 0) {
        return;
    }

    if (s_conn_step == 0) {
        /* In normal mode this is harmless stale TCP data. In transparent
         * mode the surrounding silent guards return the ESP to AT mode. */
        Bsp_ESP_AT_ClearLine(&g_esp_at);
        HW_ESP_UART_SendCmdBlocking((const uint8_t *)"+++", 3u);
        s_conn_step = 1;
        s_conn_deadline = now + ESP01S_ESCAPE_GUARD_MS;
        APP_ESP_LOG_I(LOG_TAG, "sent transparent-mode escape guard");
        return;
    }

    if (s_conn_step == 1) {
        /* In transparent mode +++ has now escaped. In normal AT mode this CRLF
         * terminates a possible buffered +++ token so it cannot prefix AT. */
        HW_ESP_UART_SendCmdBlocking((const uint8_t *)"\r\n", 2u);
        s_conn_step = 2;
        s_conn_deadline = now + ESP01S_ESCAPE_FLUSH_MS;
        return;
    }

    if (!s_conn_at_waiting) {
        (void)conn_at_start("AT\r\n", ESP01S_AT_TIMEOUT_SHORT);
        return;
    }

    if (conn_at_poll("OK") == CONN_AT_OK) {
        APP_ESP_LOG_I(LOG_TAG, "escape probe ready, start AT init");
        conn_enter_at_init_after_escape();
    } else if (!s_conn_at_waiting) {
        APP_ESP_LOG_W(LOG_TAG, "escape probe no response, enter backoff");
        conn_enter_backoff();
    }
}

static void conn_step_at_init(void)
{
    conn_at_result_t result;
    switch (s_conn_step) {
    case 0:
        /* Do not issue a naked AT probe: this state is only entered after
         * conn_step_escape has verified the parser is back in AT mode. */
        conn_enter_escape("AT probe retry");
        break;

    case 1:
        /* 关回显（非关键，失败跳过） */
        if (!s_conn_at_waiting) {
            (void)conn_at_start("ATE0\r\n", ESP01S_AT_TIMEOUT_SHORT);
            break;
        }
        result = conn_at_poll("OK");
        if (result != CONN_AT_PENDING) {
            s_conn_step = 2;
        }
        break;

    case 2:
        if (!s_conn_at_waiting) {
            (void)conn_at_start("AT+CWMODE=1\r\n", ESP01S_AT_TIMEOUT_SHORT);
            break;
        }
        result = conn_at_poll("OK");
        if (result == CONN_AT_OK) {
            s_conn_step = 3;
        } else if (result == CONN_AT_FAILED) {
            APP_ESP_LOG_W(LOG_TAG, "CWMODE fail, escape before retry");
            conn_enter_backoff();
        }
        break;

    case 3:
        if (!s_conn_at_waiting) {
            (void)conn_at_start("AT+CWDHCP=1,1\r\n", ESP01S_AT_TIMEOUT_SHORT);
            break;
        }
        result = conn_at_poll("OK");
        if (result == CONN_AT_OK) {
            APP_ESP_LOG_I(LOG_TAG, "AT init done, connecting WiFi");
            s_conn_state = ESP_CONN_WIFI;
            s_conn_step = 0;
            conn_notify_progress(2, "WiFi connecting");
        } else if (result == CONN_AT_FAILED) {
            APP_ESP_LOG_W(LOG_TAG, "CWDHCP fail, escape before retry");
            conn_enter_backoff();
        }
        break;

    default:
        s_conn_step = 0;
        break;
    }
}

static void conn_step_wifi(void)
{
    char cmd[96];
    conn_at_result_t result;
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n",
             ESP01S_WIFI_SSID, ESP01S_WIFI_PASS);

    /* ESP emits "WIFI GOT IP" before its final "OK". Waiting for the
     * terminal response prevents the next AT command from colliding with it. */
    if (!s_conn_at_waiting) {
        (void)conn_at_start(cmd, ESP01S_AT_TIMEOUT_LONG);
        return;
    }
    result = conn_at_poll("OK");
    if (result == CONN_AT_OK) {
        s_wifi_connected = true;
        s_backoff_idx = 0;  /* WiFi 成功，重置退避 */
        APP_ESP_LOG_I(LOG_TAG, "WiFi connected, SSID=%s", ESP01S_WIFI_SSID);
        s_conn_state = ESP_CONN_TCP;
        s_conn_step = 0;
        conn_notify_progress(3, "TCP connecting");
    } else if (result == CONN_AT_FAILED) {
        APP_ESP_LOG_W(LOG_TAG, "CWJAP fail, escape before retry, SSID=%s",
               ESP01S_WIFI_SSID);
        conn_enter_backoff();
    }
}

static void conn_step_tcp(void)
{
    conn_at_result_t result;
    switch (s_conn_step) {
    case 0: {
        const char *line;
        /* A known CLOSED/ERROR reply means there is no old socket to close.
         * A timeout is different: recover through +++ before issuing CIPMUX. */
        if (!s_conn_at_waiting) {
            (void)conn_at_start("AT+CIPCLOSE\r\n", ESP01S_AT_TIMEOUT_SHORT);
            break;
        }
        result = conn_at_poll("OK");
        if (result == CONN_AT_PENDING) {
            break;
        }
        line = Bsp_ESP_AT_GetLine(&g_esp_at);
        if (result != CONN_AT_OK && (line == NULL ||
                    (strstr(line, "CLOSED") == NULL && strstr(line, "ERROR") == NULL))) {
            APP_ESP_LOG_W(LOG_TAG, "CIPCLOSE no response, escape before retry");
            conn_enter_backoff();
            break;
        }
        s_conn_step = 1;
        break;
    }

    case 1:
        if (!s_conn_at_waiting) {
            (void)conn_at_start("AT+CIPMUX=0\r\n", ESP01S_AT_TIMEOUT_SHORT);
            break;
        }
        result = conn_at_poll("OK");
        if (result == CONN_AT_OK) {
            s_conn_step = 2;
        } else if (result == CONN_AT_FAILED) {
            APP_ESP_LOG_W(LOG_TAG, "CIPMUX fail, escape before retry");
            conn_enter_backoff();
        }
        break;

    case 2:
        if (!s_conn_at_waiting) {
            (void)conn_at_start("AT+CIPMODE=1\r\n", ESP01S_AT_TIMEOUT_SHORT);
            break;
        }
        result = conn_at_poll("OK");
        if (result == CONN_AT_OK) {
            s_conn_step = 3;
        } else if (result == CONN_AT_FAILED) {
            APP_ESP_LOG_W(LOG_TAG, "CIPMODE fail, escape before retry");
            conn_enter_backoff();
        }
        break;

    case 3: {
        char cmd[80];
        snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n",
                 ESP01S_SERVER_IP, ESP01S_SERVER_PORT);
        /* "CONNECT" is an intermediate notification; wait for command OK
         * before allowing CIPSEND or host traffic on the new socket. */
        if (!s_conn_at_waiting) {
            (void)conn_at_start(cmd, ESP01S_AT_TIMEOUT_CONNECT);
            break;
        }
        result = conn_at_poll("OK");
        if (result == CONN_AT_OK) {
            s_conn_step = 4;
        } else if (result == CONN_AT_FAILED) {
            APP_ESP_LOG_W(LOG_TAG, "CIPSTART fail, escape before retry, server=%s:%d",
                   ESP01S_SERVER_IP, ESP01S_SERVER_PORT);
            conn_enter_backoff();
        }
        break;
    }

    case 4:
        if (!s_conn_at_waiting) {
            (void)conn_at_start("AT+CIPSEND\r\n", ESP01S_AT_TIMEOUT_SHORT);
            break;
        }
        result = conn_at_poll(">");
        if (result == CONN_AT_OK) {
            s_transparent_mode = true;
            s_transparent_rx_len = 0;
            s_tcp_connected = true;
            s_backoff_idx = 0;  /* TCP 成功，重置退避 */
            s_conn_state = ESP_CONN_ONLINE;
            /* Some ESP-01S AT builds report CIPSTART OK before their socket
             * task is ready to accept CIPSEND.  Keep receiving host traffic,
             * but defer the first queued TX without blocking the control loop. */
            s_tx_next_allowed = Bsp_GetTick() + ESP01S_TCP_SETTLE_MS;
            APP_ESP_LOG_I(LOG_TAG, "TCP connected, server=%s:%d — ONLINE",
                   ESP01S_SERVER_IP, ESP01S_SERVER_PORT);
            conn_notify_progress(4, "ESP01S ready");
        } else if (result == CONN_AT_FAILED) {
            APP_ESP_LOG_W(LOG_TAG, "transparent CIPSEND fail, escape before retry");
            conn_enter_backoff();
        }
        break;

    default:
        s_conn_step = 0;
        break;
    }
}

static void conn_step_online(void)
{
    uint16_t len;
    bool received_ipd = false;

    /* 1. 非阻塞 IPD 轮询，收到完整帧立即派发给回调 */
    if (s_transparent_mode) {
        uint8_t byte;
        uint16_t budget = ESP01S_IPD_BUF_SIZE;
        Bsp_UART_PumpRx(&ESP_Uart_Dev);
        while (budget-- > 0 && Bsp_UART_ReadByte(&ESP_Uart_Dev, &byte)) {
            if (s_transparent_rx_len < ESP01S_IPD_BUF_SIZE) {
                s_esp_ipd_buf[s_transparent_rx_len++] = byte;
            } else {
                s_transparent_rx_len = 0;
                g_esp_at.ipd_overflow_count++;
            }
            if (byte == '\n') {
                if (bytes_contains(s_esp_ipd_buf, s_transparent_rx_len,
                                   "CLOSED", 6u) ||
                    bytes_contains(s_esp_ipd_buf, s_transparent_rx_len,
                                   "WIFI DISCONNECT", 15u)) {
                    s_transparent_rx_len = 0;
                    s_transparent_mode = false;
                    s_tcp_connected = false;
                    conn_enter_at_init();
                    return;
                }
                if (s_rx_callback != NULL && s_transparent_rx_len > 0) {
                    s_ipd_dispatch_count++;
                    s_rx_callback(s_esp_ipd_buf, s_transparent_rx_len);
                }
                s_transparent_rx_len = 0;
                received_ipd = true;
            }
        }
    } else while (Bsp_ESP_AT_Poll(&g_esp_at, &len)) {
        received_ipd = true;
        if (s_rx_callback != NULL && len > 0) {
            s_ipd_dispatch_count++;
            s_rx_callback(g_esp_at.ipd_buf, len);
        }
    }

    /* Do not issue CIPSEND in the same scheduler pass that completed +IPD.
     * The ESP8266 AT task can still be finishing its unsolicited UART output;
     * a short quiet window prevents a valid CIPSEND from being silently
     * discarded while keeping the 5 ms control scheduler non-blocking. */
    if (received_ipd) {
        uint32_t quiet_until = Bsp_GetTick() + ESP01S_RX_TO_TX_QUIET_MS;
        if ((int32_t)(quiet_until - s_tx_next_allowed) > 0) {
            s_tx_next_allowed = quiet_until;
        }
    }

    /* 2. 检测异步断连通知（CLOSED / WIFI DISCONNECT） */
    {
        const char *line = Bsp_ESP_AT_GetLine(&g_esp_at);
        if (line != NULL && line[0] != '\0') {
            bool tcp_drop  = (strstr(line, "CLOSED") != NULL);
            bool wifi_drop = (strstr(line, "WIFI DISCONNECT") != NULL);
            if (tcp_drop || wifi_drop) {
                Bsp_ESP_AT_ClearLine(&g_esp_at);
                tx_queue_clear();
                s_tcp_connected = false;
                if (wifi_drop) {
                    s_wifi_connected = false;
                    APP_ESP_LOG_W(LOG_TAG, "WiFi disconnected — escape then recover");
                    conn_enter_at_init();
                } else {
                    APP_ESP_LOG_W(LOG_TAG, "TCP closed by remote — escape then reconnect");
                    conn_enter_at_init();
                }
                return;
            }
        }
    }

    /* 3. 驱动 TX 状态机 */
    tx_state_poll();
}

static void conn_step_backoff(void)
{
    uint32_t now = Bsp_GetTick();
    if ((int32_t)(now - s_conn_deadline) >= 0) {
        /* 退避结束后递增索引（封顶在表末尾） */
        if (s_backoff_idx < (uint8_t)(ESP01S_BACKOFF_TABLE_SIZE - 1)) {
            s_backoff_idx++;
        }
        APP_ESP_LOG_I(LOG_TAG, "backoff done, retry AT init (idx=%u)", (unsigned)s_backoff_idx);
        conn_enter_at_init();
    }
}

/* ============================================================
 * 7. 业务切入点
 * ============================================================ */

bool App_ESP01S_Init(App_ESP01S_ProgressCb progress)
{
    s_wifi_connected = false;
    s_tcp_connected  = false;
    s_rx_callback    = NULL;
    s_progress_cb    = progress;
    s_backoff_idx    = 0;
    s_conn_fail_count = 0;
    s_transparent_mode = false;
    s_transparent_rx_len = 0;
    tx_queue_clear();
    conn_at_cancel();

    /* 初始化 ESP UART（UART1 + DMA） */
    Bsp_UART_Init(&ESP_Uart_Dev);

    /* 初始化 AT 引擎 */
    Bsp_ESP_AT_Init(&g_esp_at, &ESP_Uart_Dev,
                    s_esp_line_buf, ESP01S_LINE_BUF_SIZE,
                    s_esp_ipd_buf,  ESP01S_IPD_BUF_SIZE);

    /* 启动 BOOT 延时状态（非阻塞，由 Poll 推进） */
    s_conn_state = ESP_CONN_BOOT;
    s_conn_deadline = Bsp_GetTick() + ESP01S_BOOT_DELAY_MS;
    conn_notify_progress(1, "ESP01S booting");
    APP_ESP_LOG_I(LOG_TAG, "init done, boot delay %dms (non-blocking)", ESP01S_BOOT_DELAY_MS);

    return true;  /* HW 初始化成功，连接由 Poll 驱动 */
}

void App_ESP01S_RegisterRx(App_ESP01S_RxCallback cb)
{
    s_rx_callback = cb;
}

void App_ESP01S_Poll(void)
{
    switch (s_conn_state) {
    case ESP_CONN_BOOT:    conn_step_boot();    break;
    case ESP_CONN_ESCAPE:  conn_step_escape();  break;
    case ESP_CONN_AT_INIT: conn_step_at_init(); break;
    case ESP_CONN_WIFI:    conn_step_wifi();    break;
    case ESP_CONN_TCP:     conn_step_tcp();     break;
    case ESP_CONN_ONLINE:  conn_step_online();  break;
    case ESP_CONN_BACKOFF: conn_step_backoff(); break;
    default:
        s_conn_state = ESP_CONN_BOOT;
        s_conn_deadline = Bsp_GetTick() + ESP01S_BOOT_DELAY_MS;
        break;
    }
}

bool App_ESP01S_SendJSON(const uint8_t *data, uint16_t len)
{
    if (!s_tcp_connected) return false;
    if (data == NULL || len == 0) return false;
    return tx_queue_push(data, len);
}

bool App_ESP01S_SendJSONString(const char *s)
{
    if (s == NULL) return false;
    return App_ESP01S_SendJSON((const uint8_t *)s, (uint16_t)strlen(s));
}

/* ============================================================
 * 8. 状态查询
 * ============================================================ */
bool     App_ESP01S_IsWiFiConnected(void)   { return s_wifi_connected; }
bool     App_ESP01S_IsTCPConnected(void)    { return s_tcp_connected; }
uint32_t App_ESP01S_GetATFailCount(void)    { return g_esp_at.at_fail_count; }
uint32_t App_ESP01S_GetIPDOverflowCount(void) { return g_esp_at.ipd_overflow_count; }
uint32_t App_ESP01S_GetTxDropCount(void)    { return s_tx_drop_count; }
uint32_t App_ESP01S_GetTelemetryOverwriteCount(void) { return s_tx_telemetry_overwrite_count; }
uint32_t App_ESP01S_GetIPDDispatchCount(void) { return s_ipd_dispatch_count; }

const char *App_ESP01S_GetStateName(void)
{
    switch (s_conn_state) {
    case ESP_CONN_BOOT:    return "BOOT";
    case ESP_CONN_ESCAPE:  return "ESCAPE";
    case ESP_CONN_AT_INIT: return "AT_INIT";
    case ESP_CONN_WIFI:    return "WIFI";
    case ESP_CONN_TCP:     return "TCP";
    case ESP_CONN_ONLINE:  return "ONLINE";
    case ESP_CONN_BACKOFF: return "BACKOFF";
    default:               return "UNKNOWN";
    }
}

/* ============================================================
 * 9. 中断回调（main.c ISR 调用）
 * ============================================================ */

void App_ESP01S_Tx_DMA_Done_IRQHandler(void)
{
    Bsp_UART_Tx_OnDMAComplete(&ESP_Uart_Dev);
}

void App_ESP01S_RX_IRQHandler(uint16_t rx_len)
{
    if (rx_len > ESP01S_DMA_RX_BUF_SIZE) {
        rx_len = ESP01S_DMA_RX_BUF_SIZE;
    }
    if (rx_len > s_esp_rx_dma_published) {
        Bsp_UART_Rx_PushStreamBytes(&ESP_Uart_Dev,
                                    &g_esp_rx_dma_buf[s_esp_rx_dma_published],
                                    (uint16_t)(rx_len - s_esp_rx_dma_published));
    }
    HW_ESP_UART_RestartRx();
}

void App_ESP01S_RX_DMA_Done_IRQHandler(void)
{
    App_ESP01S_RX_IRQHandler(ESP01S_DMA_RX_BUF_SIZE);
}
