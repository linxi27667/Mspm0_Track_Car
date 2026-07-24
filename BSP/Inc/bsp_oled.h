/**
 * @file    bsp_oled.h
 * @brief   SSD1306-compatible 128x64 OLED驱动（传输层未绑定）
 * @note    所有显示接口均同步发送；不得在ISR或5 ms循迹控制路径调用。
 */
#ifndef BSP_OLED_H
#define BSP_OLED_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_OLED_WIDTH                 128u
#define BSP_OLED_HEIGHT                64u
#define BSP_OLED_PAGE_COUNT            8u
#define BSP_OLED_TEXT_LINE_COUNT       4u
#define BSP_OLED_TEXT_COLUMN_COUNT     16u
#define BSP_OLED_CHINESE_COLUMN_COUNT  8u
#define BSP_OLED_DEFAULT_ADDRESS       0x3Cu
#define BSP_OLED_CONTROL_COMMAND       0x00u
#define BSP_OLED_CONTROL_DATA          0x40u

typedef enum {
    BSP_OLED_STATUS_OK = 0,
    BSP_OLED_STATUS_UNBOUND,
    BSP_OLED_STATUS_NOT_INITIALIZED,
    BSP_OLED_STATUS_INVALID_ARGUMENT,
    BSP_OLED_STATUS_IO_ERROR
} bsp_oled_status_t;

/*
 * 返回0表示成功，非0表示I/O失败。实现必须在同一I2C事务中先发送
 * control_byte再发送data，并至少支持一次发送128 B数据。
 */
typedef uint8_t (*bsp_oled_write_fn)(void *context,
                                     uint8_t address_7bit,
                                     uint8_t control_byte,
                                     const uint8_t *data,
                                     uint16_t length);

typedef struct {
    void *transport_context;
    bsp_oled_write_fn Write;
    uint8_t address_7bit;
    bool initialized;
} bsp_oled_t;

void Bsp_Oled_AttachTransport(bsp_oled_t *oled,
                              void *context,
                              bsp_oled_write_fn write);
bool Bsp_Oled_IsBound(const bsp_oled_t *oled);
/* 调用者必须先满足模组上电稳定时间（常见模组至少100 ms）并处理可选RESET脚。 */
bsp_oled_status_t Bsp_Oled_Init(bsp_oled_t *oled);
bsp_oled_status_t Bsp_Oled_Clear(bsp_oled_t *oled);
bsp_oled_status_t Bsp_Oled_SetCursor(bsp_oled_t *oled, uint8_t page, uint8_t column);
bsp_oled_status_t Bsp_Oled_ShowChar(bsp_oled_t *oled,
                                    uint8_t line,
                                    uint8_t column,
                                    char character);
bsp_oled_status_t Bsp_Oled_ShowString(bsp_oled_t *oled,
                                      uint8_t line,
                                      uint8_t column,
                                      const char *string);
bsp_oled_status_t Bsp_Oled_ShowNum(bsp_oled_t *oled,
                                   uint8_t line,
                                   uint8_t column,
                                   uint32_t number,
                                   uint8_t length);
bsp_oled_status_t Bsp_Oled_ShowSignedNum(bsp_oled_t *oled,
                                         uint8_t line,
                                         uint8_t column,
                                         int32_t number,
                                         uint8_t length);
bsp_oled_status_t Bsp_Oled_ShowHexNum(bsp_oled_t *oled,
                                      uint8_t line,
                                      uint8_t column,
                                      uint32_t number,
                                      uint8_t length);
bsp_oled_status_t Bsp_Oled_ShowBinNum(bsp_oled_t *oled,
                                      uint8_t line,
                                      uint8_t column,
                                      uint32_t number,
                                      uint8_t length);
bsp_oled_status_t Bsp_Oled_ShowChinese(bsp_oled_t *oled,
                                       uint8_t line,
                                       uint8_t column,
                                       uint8_t glyph_index);

#endif /* BSP_OLED_H */
