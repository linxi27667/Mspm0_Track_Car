/**
 * @file    app_esp01s.h
 * @brief   ESP-01S 应用层（非阻塞连接状态机 + 优先级 TX 队列 + JSON 收发）
 *
 *          连接状态机 (每轮 Poll 推进一步):
 *            BOOT → AT_INIT → WIFI → TCP → ONLINE
 *                                          ↓
 *                                       BACKOFF (1/2/4/8/30s) → AT_INIT
 *
 *          TX 队列优先级 (固定):
 *            HIGH (ack/nack/pong) > 最新 TELEMETRY > 可丢弃 LOG
 *
 *          用法:
 *            App_ESP01S_Init(progress);   // 非阻塞, 仅初始化 HW + 启动状态机
 *            App_ESP01S_RegisterRx(cb);
 *            while (1) App_ESP01S_Poll(); // 主循环驱动状态机 + TX
 */
#ifndef APP_ESP01S_H
#define APP_ESP01S_H

#include "bsp_esp_at.h"
#include "app_esp01s_cfg.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 回调类型
 * ============================================================ */

/* 建连进度回调 (step: 1=boot, 2=wifi, 3=tcp, 4=online, 0xFF=fail) */
typedef void (*App_ESP01S_ProgressCb)(uint8_t step, const char *msg);

/* 接收到一帧 JSON 的回调 (data 非 NUL 结尾, 长度由 len 给出) */
typedef void (*App_ESP01S_RxCallback)(const uint8_t *data, uint16_t len);

/* ============================================================
 * 生命周期
 * ============================================================ */

/* 非阻塞初始化: 初始化 UART/DMA/AT 引擎 + 启动 boot 延时状态
 * 返回 true = HW 初始化成功, 状态机已启动; false = HW 初始化失败
 * WiFi/TCP 建连由后续 Poll() 驱动, 不阻塞调用方 */
bool App_ESP01S_Init(App_ESP01S_ProgressCb progress);

/* 主循环轮询: 推进连接状态机 + TX 队列 + +IPD 重组
 * 非阻塞, 建议每个主循环迭代调用一次 */
void App_ESP01S_Poll(void);

/* 注册 JSON 接收回调 (仅允许注册一个) */
void App_ESP01S_RegisterRx(App_ESP01S_RxCallback cb);

/* ============================================================
 * 数据收发
 * ============================================================ */

/* 发送一帧数据到 PC (自动检测优先级: ack/nack/pong=HIGH, tele=TELEMETRY, 其他=LOG)
 * HIGH: 永不丢弃; TELEMETRY: 覆盖旧遥测; LOG: 队列满时丢弃
 * 返回 true = 已入队; false = 未连接或长度越界 */
bool App_ESP01S_SendJSON(const uint8_t *data, uint16_t len);

/* 发送 C 字符串 JSON */
bool App_ESP01S_SendJSONString(const char *s);

/* ============================================================
 * 状态查询
 * ============================================================ */
bool     App_ESP01S_IsWiFiConnected(void);
bool     App_ESP01S_IsTCPConnected(void);
uint32_t App_ESP01S_GetATFailCount(void);
uint32_t App_ESP01S_GetIPDOverflowCount(void);
uint32_t App_ESP01S_GetTxDropCount(void);
uint32_t App_ESP01S_GetTelemetryOverwriteCount(void);
uint32_t App_ESP01S_GetIPDDispatchCount(void);

/* 获取连接状态机当前状态名 (诊断用) */
const char *App_ESP01S_GetStateName(void);

/* ============================================================
 * 中断回调 (main.c ISR 中调用)
 * ============================================================ */
void App_ESP01S_Tx_DMA_Done_IRQHandler(void);   /* DMA CH3 完成 */
void App_ESP01S_RX_DMA_Done_IRQHandler(void);   /* DMA CH2 接收缓冲满 */
void App_ESP01S_RX_IRQHandler(uint16_t rx_len); /* UART1 IDLE/Timeout */

#endif /* APP_ESP01S_H */
