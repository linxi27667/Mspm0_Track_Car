/**
 * @file    bsp_oled_font.h
 * @brief   SSD1306 OLED 8x16 ASCII与16x16中文字模声明
 */
#ifndef BSP_OLED_FONT_H
#define BSP_OLED_FONT_H

#include <stdint.h>

#define BSP_OLED_ASCII_FIRST          0x20u
#define BSP_OLED_ASCII_LAST           0x7Eu
#define BSP_OLED_ASCII_GLYPH_COUNT    95u
#define BSP_OLED_CHINESE_GLYPH_COUNT  34u

extern const uint8_t Bsp_Oled_Font8x16[BSP_OLED_ASCII_GLYPH_COUNT * 16u];
extern const uint8_t Bsp_Oled_Font16x16[BSP_OLED_CHINESE_GLYPH_COUNT][32];

#endif /* BSP_OLED_FONT_H */
