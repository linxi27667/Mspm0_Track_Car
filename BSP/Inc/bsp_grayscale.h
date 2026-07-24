/**
 * @file    bsp_grayscale.h
 * @brief   八路灰度传感器驱动核心框架（BSP — 纯逻辑，跨平台）
 * @note    适用于感为无MCU八路灰度传感器（74HC4051模拟输出型）
 *          原理：3根GPIO地址线(AD0/AD1/AD2)选通道，1根ADC读模拟量
 */
#ifndef __BSP_GRAYSCALE_H__
#define __BSP_GRAYSCALE_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define BSP_GRAY_CHANNEL_NUM    8       /* 通道数 */
#define BSP_GRAY_SETTLE_MS      1       /* 通道切换稳定时间(ms)，基础款1ms，青春款10ms */
#define BSP_GRAY_ADC_MAX        4095    /* ADC满量程（12位） */
#define BSP_GRAY_ADC_INVALID    UINT16_MAX /* ADC读取失败哨兵值 */

/* --- 抽象GPIO对象 --- */
typedef struct {
    void     *port;
    uint32_t  pin;
} bsp_gray_gpio_t;

/* --- 灰度传感器对象 --- */
typedef struct {
    /* --- 1. 物理配置 (Config) --- */
    bsp_gray_gpio_t ad0;                        /* 地址线0 */
    bsp_gray_gpio_t ad1;                        /* 地址线1 */
    bsp_gray_gpio_t ad2;                        /* 地址线2 */
    uint32_t        settle_ms;                  /* 通道切换稳定时间(ms) */

    /* --- 2. 运行状态 (Status) --- */
    uint8_t  current_ch;                        /* 当前通道 0~7 */
    uint32_t last_switch_tick;                  /* 上次切换通道的时间戳 */
    uint16_t analog[BSP_GRAY_CHANNEL_NUM];      /* 最近完整帧ADC快照 */
    float    norm[BSP_GRAY_CHANNEL_NUM];        /* 最近完整帧归一化快照 */
    uint16_t white[BSP_GRAY_CHANNEL_NUM];       /* 白纸校准值 */
    uint16_t black[BSP_GRAY_CHANNEL_NUM];       /* 黑纸校准值 */
    uint8_t  digital[BSP_GRAY_CHANNEL_NUM];     /* 最近完整帧: 1=黑, 0=白 */
    uint8_t  black_count;                       /* 最近完整帧黑色通道数 */
    uint8_t  data_ready;                        /* 一轮扫描完成标志 */
    uint8_t  frame_fault;                       /* 最近完整帧存在ADC读取失败 */
    uint8_t  calibrated;                        /* 白色校准完成标志 */
    uint8_t  black_calibrated;                  /* 黑色手动校准完成标志(K4) */

    /* 逐路扫描工作区。只有8路全部完成后才一次发布到上面的快照，
     * 防止控制器读到“上一帧后半段+下一帧前半段”的撕裂图案。 */
    uint16_t scan_analog[BSP_GRAY_CHANNEL_NUM];
    float    scan_norm[BSP_GRAY_CHANNEL_NUM];
    uint8_t  scan_digital[BSP_GRAY_CHANNEL_NUM];
    uint8_t  scan_fault;                        /* 当前扫描帧故障锁存 */

    /* --- 3. 抽象硬件操作方法 (Function Pointers) --- */
    void     (*Init)(void);
    void     (*Gpio_Write)(void *port, uint32_t pin, uint8_t level);
    uint16_t (*ADC_Read)(void);
    uint32_t (*GetTick)(void);
} bsp_grayscale_t;

static inline bool Bsp_Grayscale_ContrastValid8(
    const uint16_t white[BSP_GRAY_CHANNEL_NUM],
    const uint16_t black[BSP_GRAY_CHANNEL_NUM],
    uint16_t minimum_contrast)
{
    if (white == NULL || black == NULL) return false;

    for (uint8_t i = 0u; i < BSP_GRAY_CHANNEL_NUM; i++) {
        uint16_t contrast = (white[i] >= black[i])
                          ? (uint16_t)(white[i] - black[i])
                          : (uint16_t)(black[i] - white[i]);
        if (contrast < minimum_contrast) return false;
    }
    return true;
}

/* --- 4. 对外核心API (跨平台通用) --- */
void Bsp_Grayscale_Init_Device(bsp_grayscale_t *gs);
void Bsp_Grayscale_Update(bsp_grayscale_t *gs);
void Bsp_Grayscale_Calibrate_White(bsp_grayscale_t *gs);
void Bsp_Grayscale_Calibrate_Black(bsp_grayscale_t *gs);
bool Bsp_Grayscale_Calibration_Valid(const bsp_grayscale_t *gs,
                                     uint16_t minimum_contrast);

#endif /* __BSP_GRAYSCALE_H__ */
