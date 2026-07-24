/**
 * @file    app_common.h
 * @brief   APP层公共基础设施 — 硬件抽象 + 电平约定
 * @note    APP层是唯一可包含硬件头文件的层
 */
#ifndef APP_COMMON_H
#define APP_COMMON_H

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "bsp_common.h"

/* ============================================================
 * 用户统一配置入口
 * 修改本区域后必须重新编译、烧录。SysConfig、芯片寄存器与第三方库宏
 * 不在此处定义，避免破坏生成链。
 * ============================================================ */

/* ESP-01S AT/TCP、网页调参和遥测总开关：0 时 UART1 进入本地诊断模式。 */
#define APP_ESP01S_ENABLE                 1

/* ESP-01S 现场网络与上位机 TCP 端点。 */
#define ESP01S_WIFI_SSID                  "Ll"
#define ESP01S_WIFI_PASS                  "ljc12345"
#define ESP01S_SERVER_IP                  "192.168.137.1"
#define ESP01S_SERVER_PORT                8888

/* 1=TUNING：允许网页认证、调参和命令；0=RACE：禁止入站远程命令。 */
#define APP_TUNING_MODE                   1
#define APP_TUNING_AUTH_TOKEN             "track_car_2025"
#define APP_TUNING_TELEM_INTERVAL_MS      100u
#define APP_PATH_FISH_FRAME_TIMEOUT_MS    50u

/* 0=真实旋转；1=仅回 ACK 的链路测试，绝不驱动电机。 */
#define APP_TUNING_ROTATE_LINK_TEST_MODE  0

/* 1=上电自动自检；自检含电机测试，日常运行必须保持 0。 */
#define SELFTEST_MODE                     0

/* 日志分类开关：0 时对应类别在编译期移除，不占 RTT 或 ESP 日志带宽。 */
/* 常用：按看到的 RTT 日志行选择开关。*/
#define APP_LOG_GRAY_ADC_ENABLE           1  /* 鱼眼12路位图、帧龄和通信错误 */
#define APP_LOG_GRAY_STATE_ENABLE         1  /* 鱼眼初始化、极性与线型诊断 */
#define APP_LOG_ROTATE_ENABLE             1  /* 原地旋转：拒绝原因、开始、结束结果和最终角度 */

/* 高级：定位通信、驱动或循迹状态机问题时再打开。*/
#define APP_LOG_ESP01S_ENABLE             0  /* ESP AT/WiFi/TCP 状态；关闭后 ESP 链路和网页命令仍工作 */
#define APP_LOG_VEHICLE_ENABLE            1  /* 仅运动时输出车辆/电机/编码器周期状态 */
#define APP_LOG_TRACK_ENABLE              1  /* 循迹状态、校准、丢线、直角和十字 */
#define APP_LOG_TUNING_ENABLE             1  /* 认证、命令、参数与协议诊断 */
#define APP_LOG_IMU_ENABLE                1  /* ICM42688 初始化与静止校准，不周期刷运行数据 */
#define APP_LOG_DIAGNOSTIC_ENABLE         1  /* 手动自检、比例标定、IOMUX、GPIO 等诊断 */

/* 循迹、PID、速度和远程参数仍集中于此文件，供上位机一致性测试读取。 */
#include "user_tuning.h"

/* ============================================================
 * GPIO 端口类型别名
 * ============================================================ */
typedef GPIO_Regs *gpio_port_t;
typedef uint32_t    gpio_pin_t;

/* ============================================================
 * GPIO 读写抽象（MSPM0 DriverLib 映射）
 * ============================================================ */
#define GPIO_SetBits(port, pins)       DL_GPIO_setPins((port), (pins))
#define GPIO_ClearBits(port, pins)     DL_GPIO_clearPins((port), (pins))
#define GPIO_ToggleBits(port, pins)    DL_GPIO_togglePins((port), (pins))
#define GPIO_ReadPin(port, pin)        ((DL_GPIO_readPins((port), (pin)) != 0) ? 1 : 0)
#define GPIO_WritePin(port, pin, lvl)  do { \
    if (lvl) DL_GPIO_setPins((port), (pin)); \
    else     DL_GPIO_clearPins((port), (pin)); \
} while (0)

/* ============================================================
 * 电平约定
 * ============================================================ */
// 板子1.0状态LED由PB4向LED3供电，高电平点亮
#define LED_ON      1
#define LED_OFF     0

#endif /* APP_COMMON_H */
