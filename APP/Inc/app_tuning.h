/**
 * @file    app_tuning.h
 * @brief   TC1 远程调参协议层（TC1 解帧 + 参数应用 + 遥测组包 + ACK）
 *
 *          架构定位：APP 层调参协议，对接 App_ESP01S 的 Rx 回调与 SendJSON API。
 *          使用 app_tc1 的零堆分配字节流解帧器处理入站 TC1 帧。
 *
 *          协议设计 (TC1 帧: "TC1 <len> <CRC32> <JSON>\n"):
 *          下行 (PC→小车):
 *            auth  — {"v":1,"t":"auth","token":"..."}
 *            set   — {"v":1,"t":"set","id":N,"set":{"KP":2.5,...}}
 *            cmd   — {"v":1,"t":"cmd","id":N,"cmd":"start"}
 *            ping  — {"v":1,"t":"ping","ms":T}
 *
 *          上行 (小车→PC):
 *            ack   — {"v":1,"t":"ack","id":N,"ok":true,"kind":"set","applied":{...}}
 *            pong  — {"v":1,"t":"pong","ms":T}
 *            tele  — {"v":1,"t":"tele","seq":N,"ms":T,...}  10Hz
 *            log   — {"v":1,"t":"log","msg":"..."}
 *
 *          安全:
 *            - TUNING 模式: 入站命令需 auth；心跳和主机连接只用于诊断，
 *              不参与车辆运动授权
 *            - RACE 模式: 编译禁用入站命令处理，参数使用固件固化值
 */
#ifndef APP_TUNING_H
#define APP_TUNING_H

#include "app_common.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 配置
 * ============================================================ */

/* TUNING 模式本地认证令牌 (RACE 模式不使用)
 * 现场可修改, 仅用于台架调试时防止误连接 */
/* Communication-only rotation command test. A valid rotate request is ACKed
 * without arming motors or entering the rotation state. Set to 0 before any
 * vehicle rotation test to restore the READY/IMU/Safety checks. */
/* ============================================================
 * 接口
 * ============================================================ */

/* 初始化 (注册 Rx 回调到 App_ESP01S, 初始化 TC1 解帧器) */
void App_Tuning_Init(void);

/* 主循环调用: 按 10Hz 节拍上报 TC1 遥测帧 */
void App_Tuning_Poll(void);

/* ESP01S Rx 回调 (由 App_Tuning_Init 注册)
 * 喂入字节流到 TC1 解帧器, 完整帧解码后派发 set/cmd/ping/auth */
void App_Tuning_OnRx(const uint8_t *data, uint16_t len);

/* 获取遥测序列号 (用于测试/诊断) */
uint32_t App_Tuning_GetTelemetrySeq(void);

/* 获取 TC1 解帧统计 (用于诊断) */
uint32_t App_Tuning_GetFramesOk(void);
uint32_t App_Tuning_GetFramesBadCrc(void);
uint32_t App_Tuning_GetFramesBadSchema(void);

#endif /* APP_TUNING_H */
