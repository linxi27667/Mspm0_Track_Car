/**
 * @file    bsp_uart.h
 * @brief   UART 抽象驱动层（BSP — 纯逻辑，硬件无关）
 * @note    架构：
 *          RX: DMA + IDLE/Timeout 帧检测 + 字节FIFO + 帧长环形队列（多帧缓冲，突发不丢帧）
 *          TX: 字节FIFO + DMA 零拷贝（DMA 直接从 FIFO 读取，完成中断自动链接下一块）
 */
#ifndef __Bsp_UART_H__
#define __Bsp_UART_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============ 缓冲区尺寸 ============ */
#define BSP_UART_RX_BUF_SIZE      256    /* DMA 单次接收暂存（HW 层使用） */
#define BSP_UART_TX_FIFO_SIZE     1024   /* TX 软件 FIFO（环形） */
#define BSP_UART_RX_FIFO_SIZE     1024   /* RX 软件 FIFO（环形） */
#define BSP_UART_FRAME_FIFO_SIZE  16     /* RX 帧长描述符环形队列深度 */

/* ============ 驱动对象 ============ */
typedef struct {
    /* ---- HW 钩子（由 APP/HW 层提供） ---- */
    void (*Init)(void);
    void (*StartTxChunk)(const uint8_t *src, uint16_t len);  /* 启动一段 DMA TX */
    void (*StopTx)(void);                                     /* 停止 UART TX DMA 触发 */
    void (*RestartRx)(void);                                  /* 帧捕获后重启 DMA RX */
    void (*PumpRx)(void);                                     /* 可选：轮询搬运 DMA 增量 RX */
    void (*SendCmdBlocking)(const uint8_t *data, uint16_t len); /* 可选：AT命令轮询发送 */

    /* ---- RX 字节 FIFO（ISR 写 / 主循环读） ---- */
    uint8_t           *rx_fifo_buf;
    uint16_t           rx_fifo_size;
    volatile uint16_t  rx_fifo_head;   /* ISR 推入位置 */
    volatile uint16_t  rx_fifo_tail;   /* 主循环弹出位置 */

    /* ---- RX 帧长环形队列（每帧到达时记录长度） ---- */
    volatile uint16_t  rx_frame_lens[BSP_UART_FRAME_FIFO_SIZE];
    volatile uint16_t  rx_frame_head;  /* ISR 写 */
    volatile uint16_t  rx_frame_tail;  /* 主循环读 */

    /* ---- TX 字节 FIFO（主循环写 / ISR 读） ---- */
    uint8_t           *tx_fifo_buf;
    uint16_t           tx_fifo_size;
    volatile uint16_t  tx_fifo_head;   /* 主循环推入位置 */
    volatile uint16_t  tx_fifo_tail;   /* ISR(DMA) 读取位置 */
    volatile uint16_t  tx_chunk_len;   /* 当前 DMA 传输块长度 */
    volatile bool      tx_busy;        /* DMA TX 进行中 */

    /* ---- 诊断标志 ---- */
    volatile bool      tx_overflow;    /* TX FIFO 溢出（非阻塞发送时丢字节） */
    volatile bool      rx_overflow;    /* RX 字节 FIFO 溢出（丢字节） */
    volatile bool      rx_frame_overflow; /* RX 帧队列溢出（帧合并） */
} bsp_uart_t;

/* ============ 对外 API ============ */
void     Bsp_UART_Init(bsp_uart_t *uart);

/* TX —— 非阻塞（FIFO 满则丢字节并置 tx_overflow） */
void     Bsp_UART_Send(bsp_uart_t *uart, const uint8_t *data, uint16_t len);
bool     Bsp_UART_TrySendFrame(bsp_uart_t *uart, const uint8_t *data,
                               uint16_t len);
/* TX —— 阻塞直到全部入队（不丢数据） */
void     Bsp_UART_Send_Block(bsp_uart_t *uart, const uint8_t *data, uint16_t len);
/* TX —— 发送字符串（阻塞） */
void     Bsp_UART_SendString(bsp_uart_t *uart, const char *s);
/* TX —— 阻塞等待 FIFO 排空 + DMA 完成 */
void     Bsp_UART_Flush(bsp_uart_t *uart);
/* TX —— 查询待发送字节数 */
uint16_t Bsp_UART_TxPending(bsp_uart_t *uart);

/* TX —— DMA 完成中断回调（在 DMA ISR 中调用） */
void     Bsp_UART_Tx_OnDMAComplete(bsp_uart_t *uart);

/* RX —— 帧到达回调（在 UART ISR 中调用，自动推入 FIFO + 重启 DMA） */
void     Bsp_UART_Rx_OnFrame(bsp_uart_t *uart, const uint8_t *data, uint16_t len);
/* RX —— 推入字节流，不重启硬件接收（供轮询 DMA 增量搬运） */
void     Bsp_UART_Rx_PushBytes(bsp_uart_t *uart, const uint8_t *data, uint16_t len);
/* RX byte stream without frame descriptors (AT parsers consume bytes only). */
void     Bsp_UART_Rx_PushStreamBytes(bsp_uart_t *uart, const uint8_t *data, uint16_t len);
/* RX —— 调用可选硬件轮询钩子 */
void     Bsp_UART_PumpRx(bsp_uart_t *uart);
/* RX —— 弹出一帧（返回 false 表示无完整帧） */
bool     Bsp_UART_Get_Frame(bsp_uart_t *uart, uint8_t *out_buf, uint16_t *out_len);
/* RX —— 查询可读字节数 */
uint16_t Bsp_UART_Available(bsp_uart_t *uart);
/* RX —— 弹出一个字节 */
bool     Bsp_UART_ReadByte(bsp_uart_t *uart, uint8_t *out_byte);
/* RX —— 查询待处理帧数 */
uint16_t Bsp_UART_Frame_Count(bsp_uart_t *uart);

#endif /* __Bsp_UART_H__ */
