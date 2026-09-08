/**
 * @file    app_grayscale.h
 * @brief   八路灰度传感器应用层声明（硬件绑定 + 业务逻辑入口）
 * @note    引脚配置需在SysConfig中完成：
 *          - AD0/AD1/AD2 配置为数字输出
 *          - OUT引脚配置为ADC模拟输入（添加ADC12模块）
 */
#ifndef APP_GRAYSCALE_H
#define APP_GRAYSCALE_H

#include "bsp_grayscale.h"
#include "ti_msp_dl_config.h"

/* ============ 灰度传感器硬件配置 ============ */
/* 地址线 AD0/AD1/AD2 — 由SysConfig GPIO4/GPIO5配置 */
#define GRAY_AD0_PORT           Grayscale_ADDR_A_PORT
#define GRAY_AD0_PIN            Grayscale_ADDR_A_AD0_PIN
#define GRAY_AD1_PORT           Grayscale_ADDR_A_PORT
#define GRAY_AD1_PIN            Grayscale_ADDR_A_AD1_PIN
#define GRAY_AD2_PORT           Grayscale_ADDR_B_PORT
#define GRAY_AD2_PIN            Grayscale_ADDR_B_AD2_PIN

/* ADC实例 — PA27 = ADC0通道0 (A0-0) */
#define GRAY_ADC_INST           GRAYSCALE_ADC_INST

/* ============ 业务接口声明 ============ */
extern bsp_grayscale_t Grayscale_Sensor;

void App_Grayscale_Init(void);
void Grayscale_Update(void);
void Grayscale_Calibrate_White(void);
void Grayscale_Calibrate_Black(void);
bool Grayscale_Calibration_Valid(uint16_t minimum_contrast);

#endif /* APP_GRAYSCALE_H */
