/**
 * @file    bsp_oled.c
 * @brief   SSD1306-compatible 128x64 OLED显示逻辑
 */
#include "bsp_oled.h"
#include "bsp_oled_font.h"

static bsp_oled_status_t Bsp_Oled_Write(bsp_oled_t *oled,
                                        uint8_t control_byte,
                                        const uint8_t *data,
                                        uint16_t length)
{
    if (!Bsp_Oled_IsBound(oled))
    {
        return BSP_OLED_STATUS_UNBOUND;
    }
    if (data == NULL || length == 0u)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }

    if (oled->Write(oled->transport_context,
                    oled->address_7bit,
                    control_byte,
                    data,
                    length) != 0u)
    {
        return BSP_OLED_STATUS_IO_ERROR;
    }

    return BSP_OLED_STATUS_OK;
}

static uint32_t Bsp_Oled_Power(uint32_t base, uint8_t exponent)
{
    uint32_t result = 1u;

    while (exponent > 0u)
    {
        result *= base;
        exponent--;
    }

    return result;
}

static bsp_oled_status_t Bsp_Oled_ValidateTextSpan(uint8_t line,
                                                   uint8_t column,
                                                   uint8_t length)
{
    if (line == 0u || line > BSP_OLED_TEXT_LINE_COUNT ||
        column == 0u || column > BSP_OLED_TEXT_COLUMN_COUNT ||
        length == 0u ||
        (uint16_t)column + (uint16_t)length - 1u > BSP_OLED_TEXT_COLUMN_COUNT)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }

    return BSP_OLED_STATUS_OK;
}

static bsp_oled_status_t Bsp_Oled_ShowBaseNum(bsp_oled_t *oled,
                                              uint8_t line,
                                              uint8_t column,
                                              uint32_t number,
                                              uint8_t length,
                                              uint8_t base)
{
    bsp_oled_status_t status;
    uint32_t divisor;
    uint8_t i;

    status = Bsp_Oled_ValidateTextSpan(line, column, length);
    if (status != BSP_OLED_STATUS_OK)
    {
        return status;
    }

    divisor = Bsp_Oled_Power(base, (uint8_t)(length - 1u));
    for (i = 0u; i < length; i++)
    {
        uint8_t digit = (uint8_t)((number / divisor) % base);
        char character = digit < 10u ? (char)('0' + digit) : (char)('A' + digit - 10u);

        status = Bsp_Oled_ShowChar(oled, line, (uint8_t)(column + i), character);
        if (status != BSP_OLED_STATUS_OK)
        {
            return status;
        }
        if (divisor > 1u)
        {
            divisor /= base;
        }
    }

    return BSP_OLED_STATUS_OK;
}

void Bsp_Oled_AttachTransport(bsp_oled_t *oled,
                              void *context,
                              bsp_oled_write_fn write)
{
    if (oled == NULL)
    {
        return;
    }

    oled->transport_context = context;
    oled->Write = write;
    oled->initialized = false;
    if (oled->address_7bit == 0u)
    {
        oled->address_7bit = BSP_OLED_DEFAULT_ADDRESS;
    }
}

bool Bsp_Oled_IsBound(const bsp_oled_t *oled)
{
    return oled != NULL && oled->Write != NULL;
}

bsp_oled_status_t Bsp_Oled_Init(bsp_oled_t *oled)
{
    static const uint8_t init_commands[] = {
        0xAEu,
        0xD5u, 0x80u,
        0xA8u, 0x3Fu,
        0xD3u, 0x00u,
        0x40u,
        0xA1u,
        0xC8u,
        0xDAu, 0x12u,
        0x81u, 0xCFu,
        0xD9u, 0xF1u,
        0xDBu, 0x30u,
        0xA4u,
        0xA6u,
        0x8Du, 0x14u,
        0x20u, 0x02u,
    };
    static const uint8_t display_on = 0xAFu;
    bsp_oled_status_t status;

    if (!Bsp_Oled_IsBound(oled))
    {
        return BSP_OLED_STATUS_UNBOUND;
    }

    oled->initialized = false;
    status = Bsp_Oled_Write(oled,
                            BSP_OLED_CONTROL_COMMAND,
                            init_commands,
                            (uint16_t)sizeof(init_commands));
    if (status != BSP_OLED_STATUS_OK)
    {
        return status;
    }

    oled->initialized = true;
    status = Bsp_Oled_Clear(oled);
    if (status == BSP_OLED_STATUS_OK)
    {
        status = Bsp_Oled_Write(oled, BSP_OLED_CONTROL_COMMAND, &display_on, 1u);
    }
    if (status != BSP_OLED_STATUS_OK)
    {
        oled->initialized = false;
    }

    return status;
}

bsp_oled_status_t Bsp_Oled_Clear(bsp_oled_t *oled)
{
    static const uint8_t clear_page[BSP_OLED_WIDTH] = {0u};
    bsp_oled_status_t status;
    uint8_t page;

    if (oled == NULL || !oled->initialized)
    {
        return BSP_OLED_STATUS_NOT_INITIALIZED;
    }

    for (page = 0u; page < BSP_OLED_PAGE_COUNT; page++)
    {
        status = Bsp_Oled_SetCursor(oled, page, 0u);
        if (status != BSP_OLED_STATUS_OK)
        {
            return status;
        }
        status = Bsp_Oled_Write(oled,
                                BSP_OLED_CONTROL_DATA,
                                clear_page,
                                (uint16_t)sizeof(clear_page));
        if (status != BSP_OLED_STATUS_OK)
        {
            return status;
        }
    }

    return BSP_OLED_STATUS_OK;
}

bsp_oled_status_t Bsp_Oled_SetCursor(bsp_oled_t *oled, uint8_t page, uint8_t column)
{
    uint8_t commands[3];

    if (oled == NULL || !oled->initialized)
    {
        return BSP_OLED_STATUS_NOT_INITIALIZED;
    }
    if (page >= BSP_OLED_PAGE_COUNT || column >= BSP_OLED_WIDTH)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }

    commands[0] = (uint8_t)(0xB0u | page);
    commands[1] = (uint8_t)(0x10u | ((column & 0xF0u) >> 4));
    commands[2] = (uint8_t)(column & 0x0Fu);
    return Bsp_Oled_Write(oled, BSP_OLED_CONTROL_COMMAND, commands, 3u);
}

bsp_oled_status_t Bsp_Oled_ShowChar(bsp_oled_t *oled,
                                    uint8_t line,
                                    uint8_t column,
                                    char character)
{
    bsp_oled_status_t status;
    uint8_t code = (uint8_t)character;
    uint8_t index;

    status = Bsp_Oled_ValidateTextSpan(line, column, 1u);
    if (status != BSP_OLED_STATUS_OK)
    {
        return status;
    }
    if (code < BSP_OLED_ASCII_FIRST || code > BSP_OLED_ASCII_LAST)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (oled == NULL || !oled->initialized)
    {
        return BSP_OLED_STATUS_NOT_INITIALIZED;
    }

    index = (uint8_t)(code - BSP_OLED_ASCII_FIRST);
    status = Bsp_Oled_SetCursor(oled,
                                (uint8_t)((line - 1u) * 2u),
                                (uint8_t)((column - 1u) * 8u));
    if (status == BSP_OLED_STATUS_OK)
    {
        status = Bsp_Oled_Write(oled,
                                BSP_OLED_CONTROL_DATA,
                                &Bsp_Oled_Font8x16[(uint16_t)index * 16u],
                                8u);
    }
    if (status == BSP_OLED_STATUS_OK)
    {
        status = Bsp_Oled_SetCursor(oled,
                                    (uint8_t)((line - 1u) * 2u + 1u),
                                    (uint8_t)((column - 1u) * 8u));
    }
    if (status == BSP_OLED_STATUS_OK)
    {
        status = Bsp_Oled_Write(oled,
                                BSP_OLED_CONTROL_DATA,
                                &Bsp_Oled_Font8x16[(uint16_t)index * 16u + 8u],
                                8u);
    }

    return status;
}

bsp_oled_status_t Bsp_Oled_ShowString(bsp_oled_t *oled,
                                      uint8_t line,
                                      uint8_t column,
                                      const char *string)
{
    bsp_oled_status_t status;
    uint8_t current_column = column;
    uint8_t string_length = 0u;
    const char *cursor = string;

    if (string == NULL)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (oled == NULL || !oled->initialized)
    {
        return BSP_OLED_STATUS_NOT_INITIALIZED;
    }

    status = Bsp_Oled_ValidateTextSpan(line, column, 1u);
    if (status != BSP_OLED_STATUS_OK)
    {
        return status;
    }

    while (*cursor != '\0')
    {
        string_length++;
        if ((uint16_t)column + string_length - 1u > BSP_OLED_TEXT_COLUMN_COUNT)
        {
            return BSP_OLED_STATUS_INVALID_ARGUMENT;
        }
        cursor++;
    }

    while (*string != '\0')
    {
        status = Bsp_Oled_ShowChar(oled, line, current_column, *string);
        if (status != BSP_OLED_STATUS_OK)
        {
            return status;
        }
        current_column++;
        string++;
    }

    return BSP_OLED_STATUS_OK;
}

bsp_oled_status_t Bsp_Oled_ShowNum(bsp_oled_t *oled,
                                   uint8_t line,
                                   uint8_t column,
                                   uint32_t number,
                                   uint8_t length)
{
    if (length == 0u || length > 10u)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }

    return Bsp_Oled_ShowBaseNum(oled, line, column, number, length, 10u);
}

bsp_oled_status_t Bsp_Oled_ShowSignedNum(bsp_oled_t *oled,
                                         uint8_t line,
                                         uint8_t column,
                                         int32_t number,
                                         uint8_t length)
{
    bsp_oled_status_t status;
    uint32_t magnitude;
    char sign;

    if (length == 0u || length > 10u ||
        Bsp_Oled_ValidateTextSpan(line, column, (uint8_t)(length + 1u)) != BSP_OLED_STATUS_OK)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }

    if (number < 0)
    {
        sign = '-';
        magnitude = (uint32_t)(-(number + 1)) + 1u;
    }
    else
    {
        sign = '+';
        magnitude = (uint32_t)number;
    }

    status = Bsp_Oled_ShowChar(oled, line, column, sign);
    if (status != BSP_OLED_STATUS_OK)
    {
        return status;
    }

    return Bsp_Oled_ShowBaseNum(oled,
                                line,
                                (uint8_t)(column + 1u),
                                magnitude,
                                length,
                                10u);
}

bsp_oled_status_t Bsp_Oled_ShowHexNum(bsp_oled_t *oled,
                                      uint8_t line,
                                      uint8_t column,
                                      uint32_t number,
                                      uint8_t length)
{
    if (length == 0u || length > 8u)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }

    return Bsp_Oled_ShowBaseNum(oled, line, column, number, length, 16u);
}

bsp_oled_status_t Bsp_Oled_ShowBinNum(bsp_oled_t *oled,
                                      uint8_t line,
                                      uint8_t column,
                                      uint32_t number,
                                      uint8_t length)
{
    if (length == 0u || length > BSP_OLED_TEXT_COLUMN_COUNT)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }

    return Bsp_Oled_ShowBaseNum(oled, line, column, number, length, 2u);
}

bsp_oled_status_t Bsp_Oled_ShowChinese(bsp_oled_t *oled,
                                       uint8_t line,
                                       uint8_t column,
                                       uint8_t glyph_index)
{
    bsp_oled_status_t status;

    if (line == 0u || line > BSP_OLED_TEXT_LINE_COUNT ||
        column == 0u || column > BSP_OLED_CHINESE_COLUMN_COUNT ||
        glyph_index >= BSP_OLED_CHINESE_GLYPH_COUNT)
    {
        return BSP_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (oled == NULL || !oled->initialized)
    {
        return BSP_OLED_STATUS_NOT_INITIALIZED;
    }

    status = Bsp_Oled_SetCursor(oled,
                                (uint8_t)((line - 1u) * 2u),
                                (uint8_t)((column - 1u) * 16u));
    if (status == BSP_OLED_STATUS_OK)
    {
        status = Bsp_Oled_Write(oled,
                                BSP_OLED_CONTROL_DATA,
                                &Bsp_Oled_Font16x16[glyph_index][0],
                                16u);
    }
    if (status == BSP_OLED_STATUS_OK)
    {
        status = Bsp_Oled_SetCursor(oled,
                                    (uint8_t)((line - 1u) * 2u + 1u),
                                    (uint8_t)((column - 1u) * 16u));
    }
    if (status == BSP_OLED_STATUS_OK)
    {
        status = Bsp_Oled_Write(oled,
                                BSP_OLED_CONTROL_DATA,
                                &Bsp_Oled_Font16x16[glyph_index][16],
                                16u);
    }

    return status;
}
