/**
 * @file    app_esp01s_cfg.h
 * @brief   ESP-01S 应用层集中配置（WiFi / 服务器 / UART / DMA / 缓冲区 / 超时）
 * @note    所有可调参数集中于此，便于现场调试与多环境切换。
 *          使用前请：
 *            1. 修改 WiFi SSID/PASS 为现场热点
 *            2. 修改 SERVER_IP/PORT 为运行上位机的 PC（TCP 服务器）地址
 *            3. 在 SysConfig 中启用 UART1 + DMA CH2(RX)/CH3(TX)，
 *               并使下方宏与 SysConfig 生成定义保持一致
 *
 *          AI 闭环调参拓扑：
 *            [小车 MSPM0] ←UART1→ [ESP-01S] ←WiFi→ [PC 上位机 TCP server] ←HTTP→ [GLM AI]
 *            上行：小车遥测 JSON（ADC/陀螺仪/编码器/PID 状态）
 *            下行：AI 决策 JSON（新 PID 参数 / 指令）
 */
#ifndef APP_ESP01S_CFG_H
#define APP_ESP01S_CFG_H

#include "app_common.h"

/* ============================================================
 * 1. WiFi 热点（现场修改）
 * ============================================================ */

/* UART1 PA8/PA9 has two exclusive roles:
 *   APP_ESP01S_ENABLE=0: external USB-TTL diagnostic console at 115200 8N1.
 *   APP_ESP01S_ENABLE=1: ESP-01S AT/TCP remote tuning, using DMA RX/TX.
 *
 * Current default is diagnostic console because PA8/PA9 were just verified with
 * USB-TTL. Set APP_ESP01S_ENABLE to 1 when ESP-01S is connected. */
#ifndef APP_UART1_DIAG_CONSOLE_ENABLE
#define APP_UART1_DIAG_CONSOLE_ENABLE  (!APP_ESP01S_ENABLE)
#endif

/* ============================================================
 * 2. TCP 服务器（PC 上位机）
 *    上位机监听此端口，小车主动 TCP CONNECT 上去
 * ============================================================ */
/* ============================================================
 * 3. ESP-01S UART 硬件绑定（引用 SysConfig 生成的符号，避免双份定义）
 *    对应 SysConfig 实例 UART_ESP01S（UART1 外设，PA8 TX / PA9 RX）
 *    DMA CH2(RX) / CH3(TX)
 *    注意：UART0 已被主调试串口占用，ESP 必须用第二个 UART
 * ============================================================ */
#include "ti_msp_dl_config.h"

#define ESP01S_UART_INST            UART_ESP01S_INST
#define ESP01S_UART_IRQ_HANDLER     UART_ESP01S_INST_IRQHandler
#define ESP01S_UART_INT_IRQn        UART_ESP01S_INST_INT_IRQN

#define ESP01S_DMA_CH_RX            DMA_CH2_CHAN_ID   /* DMA 通道 2：UART1 RX */
#define ESP01S_DMA_CH_TX            DMA_CH3_CHAN_ID   /* DMA 通道 3：UART1 TX */
#define ESP01S_DMA_RX_TRIG          UART_ESP01S_INST_DMA_TRIGGER_0
#define ESP01S_DMA_TX_TRIG          UART_ESP01S_INST_DMA_TRIGGER_1
#define ESP01S_DMA_EVENT_RX         DL_DMA_EVENT_IIDX_DMACH2
#define ESP01S_DMA_EVENT_TX         DL_DMA_EVENT_IIDX_DMACH3

/* ============================================================
 * 4. 缓冲区尺寸
 * ============================================================ */
#define ESP01S_LINE_BUF_SIZE        256     /* AT 响应流缓冲（够装最长 AT 响应） */
#define ESP01S_IPD_BUF_SIZE         512     /* +IPD 数据缓冲（一帧 JSON 上限） */
#define ESP01S_TX_FIFO_SIZE         1024    /* ESP UART TX FIFO */
#define ESP01S_RX_FIFO_SIZE         1024    /* ESP UART RX FIFO */
#define ESP01S_DMA_RX_BUF_SIZE      256     /* ESP UART DMA 单次接收暂存 */

/* ============================================================
 * 5. AT 指令超时与重试
 * ============================================================ */
#define ESP01S_AT_TIMEOUT_SHORT     1000    /* AT / CWMODE 等快速指令 */
#define ESP01S_AT_TIMEOUT_LONG      8000    /* CWJAP 连 AP */
#define ESP01S_AT_TIMEOUT_CONNECT   5000    /* CIPSTART 连服务器 */
#define ESP01S_AT_TIMEOUT_SEND      10000   /* CIPSEND + 数据；ESP-01S WiFi 忙时 SEND OK 可能较慢 */
#define ESP01S_AT_RETRIES           5       /* 常规指令重试次数 */
#define ESP01S_AT_RETRIES_AP        20      /* 连 AP 重试次数（信号不稳） */
#define ESP01S_AT_RETRY_DELAY_MS    500

/* ============================================================
 * 6. ESP-01S 启动延时（上电稳定时间）
 *    ESP-01S 上电后需 ~500ms 跑完 bootloader 才能响应 AT
 * ============================================================ */
#define ESP01S_BOOT_DELAY_MS        800

/* ESP transparent-mode escape requires a silent guard interval before and
 * after the raw +++ sequence. This runs only while the MCU is starting. */
#define ESP01S_ESCAPE_GUARD_MS       1000

#endif /* APP_ESP01S_CFG_H */
