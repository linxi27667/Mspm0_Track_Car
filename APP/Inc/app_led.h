/**
 * @file    app_led.h
 * @brief   LED应用层声明（硬件绑定 + 业务逻辑入口）
 */
#ifndef APP_LED_H
#define APP_LED_H

#include "bsp_led.h"
#include "ti_msp_dl_config.h"

/* ============ 板载状态LED硬件配置（板子1.0网表：PB4，高有效） ============ */
#define LED1_PORT       Board_Output_PORT
#define LED1_PIN        Board_Output_STATUS_LED_PIN

/* ============ 业务接口声明 ============ */
extern led_t Led_Devices[BSP_LED_NUM];

void App_Led_Init(void);
void Led_On(uint8_t index);
void Led_Off(uint8_t index);
void Led_Toggle(uint8_t index);

#endif /* APP_LED_H */
