/**
 * @file    bsp_uart.c
 * @brief   UART 核心逻辑实现（BSP — 纯逻辑，跨平台）
 * @note    FIFO 环形缓冲 + 帧长队列，硬件无关
 *
 * 数据流：
 *   RX: HW ISR → Bsp_UART_Rx_OnFrame() → 字节FIFO + 帧长队列 → Bsp_UART_Get_Frame()
 *   TX: Bsp_UART_Send() → 字节FIFO → Bsp_UART_Tx_Kick() → DMA → ISR → 链接下一块
 */
#include "bsp_uart.h"
#include <string.h>

/* ============================================================
 *  临界区保护（PRIMASK 保存/恢复，可重入安全）
 *  使用内联汇编，不依赖任何平台头文件，保持 BSP 跨平台性
 *  适用于 Cortex-M 全系列（M0+/M3/M4/M7/M33）
 * ============================================================ */

/* 保存并关闭全局中断，返回原 PRIMASK 值 */
static inline uint32_t bsp_irq_save(void)
{
    uint32_t primask;
#if defined(__GNUC__) || defined(__clang__) || defined(__TI_COMPILER_VERSION__) || defined(__ARMCC_VERSION)
    __asm volatile ("mrs %0, primask\n"
                    "cpsid i" : "=r"(primask) :: "memory");
#else
    primask = __get_PRIMASK();
    __disable_irq();
#endif
    return primask;
}

/* 恢复 PRIMASK（恢复中断状态） */
static inline void bsp_irq_restore(uint32_t primask)
{
#if defined(__GNUC__) || defined(__clang__) || defined(__TI_COMPILER_VERSION__) || defined(__ARMCC_VERSION)
    __asm volatile ("msr primask, %0" :: "r"(primask) : "memory");
#else
    __set_PRIMASK(primask);
#endif
}

#define BSP_UART_ENTER_CRITICAL()  uint32_t __pm = bsp_irq_save()
#define BSP_UART_EXIT_CRITICAL()   bsp_irq_restore(__pm)

/* ============================================================
 *  内部 FIFO 辅助（静态，仅本文件使用）
 * ============================================================ */

/* ---- TX FIFO ---- */
static inline uint16_t tx_fifo_used(const bsp_uart_t *u)
{
    uint16_t h = u->tx_fifo_head, t = u->tx_fifo_tail;
    return (h >= t) ? (uint16_t)(h - t) : (uint16_t)(u->tx_fifo_size - t + h);
}
static inline bool tx_fifo_empty(const bsp_uart_t *u)
{
    return u->tx_fifo_head == u->tx_fifo_tail;
}

/* ---- RX 字节 FIFO ---- */
static inline bool rx_fifo_empty(const bsp_uart_t *u)
{
    return u->rx_fifo_head == u->rx_fifo_tail;
}

/* ---- RX 帧长队列 ---- */
static inline bool rx_frame_empty(const bsp_uart_t *u)
{
    return u->rx_frame_head == u->rx_frame_tail;
}

/* ============================================================
 *  1. 初始化
 * ============================================================ */
void Bsp_UART_Init(bsp_uart_t *uart)
{
    if (uart == NULL) return;

    uart->rx_fifo_head  = 0;
    uart->rx_fifo_tail  = 0;
    uart->rx_frame_head = 0;
    uart->rx_frame_tail = 0;
    uart->tx_fifo_head  = 0;
    uart->tx_fifo_tail  = 0;
    uart->tx_chunk_len  = 0;
    uart->tx_busy       = false;
    uart->tx_overflow        = false;
    uart->rx_overflow        = false;
    uart->rx_frame_overflow  = false;

    if (uart->Init) uart->Init();
}

/* ============================================================
 *  2. TX —— 启动 DMA 块（假定已在临界区或 ISR 中）
 * ============================================================ */
static void tx_kick_locked(bsp_uart_t *u)
{
    if (u->tx_busy) return;
    if (tx_fifo_empty(u)) {
        if (u->StopTx) u->StopTx();
        return;
    }
    /* 计算从 tail 起的连续可读块（DMA 不支持环形，取连续段） */
    uint16_t tail  = u->tx_fifo_tail;
    uint16_t head  = u->tx_fifo_head;
    uint16_t chunk = (head > tail) ? (uint16_t)(head - tail)
                                   : (uint16_t)(u->tx_fifo_size - tail);
    u->tx_chunk_len = chunk;
    u->tx_busy      = true;
    if (u->StartTxChunk) u->StartTxChunk(&u->tx_fifo_buf[tail], chunk);
}

/* 主循环调用 —— 临界区保护 */
static void tx_kick(bsp_uart_t *u)
{
    BSP_UART_ENTER_CRITICAL();
    tx_kick_locked(u);
    BSP_UART_EXIT_CRITICAL();
}

/* ============================================================
 *  3. TX —— 发送接口
 * ============================================================ */
void Bsp_UART_Send(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    if (uart == NULL || data == NULL || len == 0) return;

    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((uart->tx_fifo_head + 1) % uart->tx_fifo_size);
        if (next == uart->tx_fifo_tail) {
            uart->tx_overflow = true;   /* FIFO 满，丢弃剩余字节 */
            break;
        }
        uart->tx_fifo_buf[uart->tx_fifo_head] = data[i];
        uart->tx_fifo_head = next;
    }
    tx_kick(uart);
}

bool Bsp_UART_TrySendFrame(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    if (uart == NULL || data == NULL || len == 0u ||
        len >= uart->tx_fifo_size) {
        return false;
    }

    BSP_UART_ENTER_CRITICAL();
    if (uart->tx_busy || !tx_fifo_empty(uart)) {
        BSP_UART_EXIT_CRITICAL();
        return false;
    }

    /* Rebase each standalone frame so DMA never has to continue the same
     * protocol frame across the ring-buffer wrap boundary. */
    uart->tx_fifo_head = 0u;
    uart->tx_fifo_tail = 0u;
    memcpy(uart->tx_fifo_buf, data, len);
    uart->tx_fifo_head = len;
    tx_kick_locked(uart);
    BSP_UART_EXIT_CRITICAL();
    return true;
}

void Bsp_UART_Send_Block(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    if (uart == NULL || data == NULL || len == 0) return;

    for (uint16_t i = 0; i < len; i++) {
        uint16_t next;
        do {
            next = (uint16_t)((uart->tx_fifo_head + 1) % uart->tx_fifo_size);
        } while (next == uart->tx_fifo_tail);   /* 自旋等待空闲 */
        uart->tx_fifo_buf[uart->tx_fifo_head] = data[i];
        uart->tx_fifo_head = next;
    }
    tx_kick(uart);
}

void Bsp_UART_SendString(bsp_uart_t *uart, const char *s)
{
    if (uart == NULL || s == NULL) return;
    Bsp_UART_Send_Block(uart, (const uint8_t *)s, (uint16_t)strlen(s));
}

void Bsp_UART_Flush(bsp_uart_t *uart)
{
    if (uart == NULL) return;
    /* 等待 FIFO 排空且 DMA 空闲 */
    while (!tx_fifo_empty(uart) || uart->tx_busy) {}
}

uint16_t Bsp_UART_TxPending(bsp_uart_t *uart)
{
    if (uart == NULL) return 0;
    return tx_fifo_used(uart);
}

/* ============================================================
 *  4. TX —— DMA 完成中断回调（在 DMA ISR 中调用）
 * ============================================================ */
void Bsp_UART_Tx_OnDMAComplete(bsp_uart_t *uart)
{
    if (uart == NULL) return;
    /* 推进 tail（DMA 已读走 chunk 字节） */
    uart->tx_fifo_tail =
        (uint16_t)((uart->tx_fifo_tail + uart->tx_chunk_len) % uart->tx_fifo_size);
    uart->tx_busy = false;
    /* 链接下一块（已在 ISR，无需再进临界区） */
    tx_kick_locked(uart);
}

/* ============================================================
 *  5. RX —— 帧到达回调（在 UART ISR 中调用）
 *      将 data[0..len) 推入字节 FIFO，并在帧长队列记录一帧
 * ============================================================ */
void Bsp_UART_Rx_OnFrame(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    Bsp_UART_Rx_PushBytes(uart, data, len);

    if (uart && uart->RestartRx) uart->RestartRx();
}

static void rx_push_bytes(bsp_uart_t *uart, const uint8_t *data, uint16_t len,
                          bool record_frame)
{
    if (uart == NULL) {
        return;
    }
    if (len == 0) {
        return;
    }
    if (data == NULL) {
        return;
    }

    /* PumpRx runs in the main loop while RX timeout can publish concurrently
     * from the UART ISR. Keep both producers from interleaving FIFO updates. */
    BSP_UART_ENTER_CRITICAL();

    /* ---- 推入字节 FIFO ---- */
    uint16_t pushed = 0;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((uart->rx_fifo_head + 1) % uart->rx_fifo_size);
        if (next == uart->rx_fifo_tail) {
            uart->rx_overflow = true;   /* FIFO 满，丢弃剩余字节 */
            break;
        }
        uart->rx_fifo_buf[uart->rx_fifo_head] = data[i];
        uart->rx_fifo_head = next;
        pushed++;
    }

    /* ---- 记录帧长（仅当有字节推入） ---- */
    if (record_frame && pushed > 0) {
        uint16_t fhead = uart->rx_frame_head;
        uint16_t fnext = (uint16_t)((fhead + 1) % BSP_UART_FRAME_FIFO_SIZE);
        if (fnext == uart->rx_frame_tail) {
            /* 帧队列满：合并到上一帧（不丢字节，仅丢帧边界） */
            uint16_t last = (uint16_t)((fhead + BSP_UART_FRAME_FIFO_SIZE - 1)
                                       % BSP_UART_FRAME_FIFO_SIZE);
            uart->rx_frame_lens[last] =
                (uint16_t)(uart->rx_frame_lens[last] + pushed);
            uart->rx_frame_overflow = true;
        } else {
            uart->rx_frame_lens[fhead] = pushed;
            uart->rx_frame_head = fnext;
        }
    }

    BSP_UART_EXIT_CRITICAL();
}

void Bsp_UART_Rx_PushBytes(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    rx_push_bytes(uart, data, len, true);
}

void Bsp_UART_Rx_PushStreamBytes(bsp_uart_t *uart, const uint8_t *data, uint16_t len)
{
    rx_push_bytes(uart, data, len, false);
}

void Bsp_UART_PumpRx(bsp_uart_t *uart)
{
    if (uart != NULL && uart->PumpRx != NULL) {
        uart->PumpRx();
    }
}

/* ============================================================
 *  6. RX —— 帧弹出 / 字节弹出 / 查询
 * ============================================================ */
bool Bsp_UART_Get_Frame(bsp_uart_t *uart, uint8_t *out_buf, uint16_t *out_len)
{
    if (uart == NULL) return false;
    if (rx_frame_empty(uart)) return false;

    uint16_t len = uart->rx_frame_lens[uart->rx_frame_tail];

    /* 安全限制：若帧长超出 FIFO 实际可用字节（溢出后可能不一致），截断 */
    uint16_t avail = Bsp_UART_Available(uart);
    if (len > avail) len = avail;

    if (out_buf != NULL && len > 0) {
        for (uint16_t i = 0; i < len; i++) {
            out_buf[i] = uart->rx_fifo_buf[uart->rx_fifo_tail];
            uart->rx_fifo_tail =
                (uint16_t)((uart->rx_fifo_tail + 1) % uart->rx_fifo_size);
        }
    } else if (len > 0) {
        /* 仅丢弃，不拷出 */
        uart->rx_fifo_tail =
            (uint16_t)((uart->rx_fifo_tail + len) % uart->rx_fifo_size);
    }

    if (out_len != NULL) *out_len = len;
    uart->rx_frame_tail =
        (uint16_t)((uart->rx_frame_tail + 1) % BSP_UART_FRAME_FIFO_SIZE);
    return true;
}

uint16_t Bsp_UART_Available(bsp_uart_t *uart)
{
    if (uart == NULL) return 0;
    uint16_t h = uart->rx_fifo_head, t = uart->rx_fifo_tail;
    return (h >= t) ? (uint16_t)(h - t) : (uint16_t)(uart->rx_fifo_size - t + h);
}

bool Bsp_UART_ReadByte(bsp_uart_t *uart, uint8_t *out_byte)
{
    if (uart == NULL || rx_fifo_empty(uart)) return false;
    if (out_byte) *out_byte = uart->rx_fifo_buf[uart->rx_fifo_tail];
    uart->rx_fifo_tail =
        (uint16_t)((uart->rx_fifo_tail + 1) % uart->rx_fifo_size);
    return true;
}

uint16_t Bsp_UART_Frame_Count(bsp_uart_t *uart)
{
    if (uart == NULL) return 0;
    uint16_t h = uart->rx_frame_head, t = uart->rx_frame_tail;
    return (h >= t) ? (uint16_t)(h - t)
                    : (uint16_t)(BSP_UART_FRAME_FIFO_SIZE - t + h);
}
