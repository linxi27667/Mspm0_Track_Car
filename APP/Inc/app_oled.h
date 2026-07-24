/**
 * @file    app_oled.h
 * @brief   OLED应用层接口（I2C1: PA18 SDA / PA17 SCL）
 */
#ifndef APP_OLED_H
#define APP_OLED_H

#include "app_common.h"
#include "bsp_oled.h"
#include "ti_msp_dl_config.h"

extern bsp_oled_t Oled_Display;

void App_Oled_AttachTransport(void *context, bsp_oled_write_fn write);
bsp_oled_status_t App_Oled_Init(void);
void App_Oled_Poll(uint32_t now);
void App_Oled_SetLine(uint8_t line, const char *text);
bool App_Oled_IsAvailable(void);
uint32_t App_Oled_ErrorCount(void);
bool Oled_IsBound(void);
bsp_oled_status_t Oled_Clear(void);
bsp_oled_status_t Oled_ShowChar(uint8_t line, uint8_t column, char character);
bsp_oled_status_t Oled_ShowString(uint8_t line, uint8_t column, const char *string);
bsp_oled_status_t Oled_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
bsp_oled_status_t Oled_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length);
bsp_oled_status_t Oled_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
bsp_oled_status_t Oled_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
bsp_oled_status_t Oled_ShowChinese(uint8_t line, uint8_t column, uint8_t glyph_index);

#endif /* APP_OLED_H */
