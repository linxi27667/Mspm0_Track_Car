/**
 * @file    app_oled.c
 * @brief   OLED应用层封装和I2C1低优先级脏字符刷新。
 */
#include "app_oled.h"

#include <string.h>

#define OLED_POWER_STABLE_MS       100u
#define OLED_IO_TIMEOUT_MS           2u
#define OLED_DIRTY_FLUSH_PERIOD_MS   5u
#define OLED_I2C_DATA_CHUNK           7u

static char s_desired[BSP_OLED_TEXT_LINE_COUNT][BSP_OLED_TEXT_COLUMN_COUNT];
static char s_displayed[BSP_OLED_TEXT_LINE_COUNT][BSP_OLED_TEXT_COLUMN_COUNT];
static uint8_t s_scan_index;
static uint32_t s_last_flush_at;
static uint32_t s_error_count;
static bool s_init_attempted;
static bool s_available;

bsp_oled_t Oled_Display = {
    .transport_context = NULL,
    .Write = NULL,
    .address_7bit = BSP_OLED_DEFAULT_ADDRESS,
    .initialized = false,
};

static void HW_Oled_ResetTransfer(void)
{
    DL_I2C_disableController(OLED_I2C_INST);
    DL_I2C_resetControllerTransfer(OLED_I2C_INST);
    DL_I2C_flushControllerTXFIFO(OLED_I2C_INST);
    DL_I2C_enableController(OLED_I2C_INST);
}

static uint8_t HW_Oled_WritePacket(uint8_t address,
                                   uint8_t *packet,
                                   uint8_t length)
{
    uint32_t started_at = Bsp_GetTick();
    uint32_t status;

    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0u) {
        if ((uint32_t)(Bsp_GetTick() - started_at) >= OLED_IO_TIMEOUT_MS) {
            HW_Oled_ResetTransfer();
            return 1u;
        }
    }

    DL_I2C_fillControllerTXFIFO(OLED_I2C_INST, packet, length);
    DL_I2C_startControllerTransfer(OLED_I2C_INST, address,
                                   DL_I2C_CONTROLLER_DIRECTION_TX, length);

    while ((DL_I2C_getControllerStatus(OLED_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0u) {
        if ((uint32_t)(Bsp_GetTick() - started_at) >= OLED_IO_TIMEOUT_MS) {
            HW_Oled_ResetTransfer();
            return 1u;
        }
    }

    status = DL_I2C_getControllerStatus(OLED_I2C_INST);
    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0u) {
        HW_Oled_ResetTransfer();
        return 1u;
    }
    return 0u;
}

static uint8_t HW_Oled_Write(void *context,
                             uint8_t address,
                             uint8_t control,
                             const uint8_t *data,
                             uint16_t length)
{
    uint16_t offset = 0u;
    uint8_t packet[OLED_I2C_DATA_CHUNK + 1u];

    (void)context;
    if (data == NULL || length == 0u) return 1u;

    while (offset < length) {
        uint8_t count = (uint8_t)(length - offset);
        if (count > OLED_I2C_DATA_CHUNK) count = OLED_I2C_DATA_CHUNK;

        packet[0] = control;
        for (uint8_t i = 0u; i < count; i++) {
            packet[i + 1u] = data[offset + i];
        }
        if (HW_Oled_WritePacket(address, packet, (uint8_t)(count + 1u)) != 0u) {
            return 1u;
        }
        offset = (uint16_t)(offset + count);
    }
    return 0u;
}

void App_Oled_AttachTransport(void *context, bsp_oled_write_fn write)
{
    Bsp_Oled_AttachTransport(&Oled_Display, context, write);
}

bsp_oled_status_t App_Oled_Init(void)
{
    return Bsp_Oled_Init(&Oled_Display);
}

void App_Oled_SetLine(uint8_t line, const char *text)
{
    uint8_t row;
    uint8_t column = 0u;

    if (line == 0u || line > BSP_OLED_TEXT_LINE_COUNT || text == NULL) return;
    row = (uint8_t)(line - 1u);
    while (column < BSP_OLED_TEXT_COLUMN_COUNT && text[column] != '\0') {
        char value = text[column];
        s_desired[row][column] =
            (value >= ' ' && value <= '~') ? value : '?';
        column++;
    }
    while (column < BSP_OLED_TEXT_COLUMN_COUNT) {
        s_desired[row][column++] = ' ';
    }
}

void App_Oled_Poll(uint32_t now)
{
    if (!s_init_attempted) {
        if (now < OLED_POWER_STABLE_MS) return;

        s_init_attempted = true;
        memset(s_desired, ' ', sizeof(s_desired));
        memset(s_displayed, 0, sizeof(s_displayed));
        App_Oled_AttachTransport(NULL, HW_Oled_Write);
        if (App_Oled_Init() != BSP_OLED_STATUS_OK) {
            s_error_count++;
            s_available = false;
            return;
        }
        s_available = true;
    }

    if (!s_available ||
        (uint32_t)(now - s_last_flush_at) < OLED_DIRTY_FLUSH_PERIOD_MS) {
        return;
    }
    s_last_flush_at = now;

    for (uint8_t checked = 0u;
         checked < BSP_OLED_TEXT_LINE_COUNT * BSP_OLED_TEXT_COLUMN_COUNT;
         checked++) {
        uint8_t index = s_scan_index;
        uint8_t row = (uint8_t)(index / BSP_OLED_TEXT_COLUMN_COUNT);
        uint8_t column = (uint8_t)(index % BSP_OLED_TEXT_COLUMN_COUNT);

        s_scan_index = (uint8_t)(
            (index + 1u) %
            (BSP_OLED_TEXT_LINE_COUNT * BSP_OLED_TEXT_COLUMN_COUNT));
        if (s_desired[row][column] == s_displayed[row][column]) continue;

        if (Oled_ShowChar((uint8_t)(row + 1u), (uint8_t)(column + 1u),
                          s_desired[row][column]) == BSP_OLED_STATUS_OK) {
            s_displayed[row][column] = s_desired[row][column];
        } else {
            s_error_count++;
            s_available = false;
        }
        return; /* one character is at most 28 I2C bytes */
    }
}

bool App_Oled_IsAvailable(void)
{
    return s_available;
}

uint32_t App_Oled_ErrorCount(void)
{
    return s_error_count;
}

bool Oled_IsBound(void)
{
    return Bsp_Oled_IsBound(&Oled_Display);
}

bsp_oled_status_t Oled_Clear(void)
{
    return Bsp_Oled_Clear(&Oled_Display);
}

bsp_oled_status_t Oled_ShowChar(uint8_t line, uint8_t column, char character)
{
    return Bsp_Oled_ShowChar(&Oled_Display, line, column, character);
}

bsp_oled_status_t Oled_ShowString(uint8_t line, uint8_t column, const char *string)
{
    return Bsp_Oled_ShowString(&Oled_Display, line, column, string);
}

bsp_oled_status_t Oled_ShowNum(uint8_t line,
                               uint8_t column,
                               uint32_t number,
                               uint8_t length)
{
    return Bsp_Oled_ShowNum(&Oled_Display, line, column, number, length);
}

bsp_oled_status_t Oled_ShowSignedNum(uint8_t line,
                                     uint8_t column,
                                     int32_t number,
                                     uint8_t length)
{
    return Bsp_Oled_ShowSignedNum(&Oled_Display, line, column, number, length);
}

bsp_oled_status_t Oled_ShowHexNum(uint8_t line,
                                  uint8_t column,
                                  uint32_t number,
                                  uint8_t length)
{
    return Bsp_Oled_ShowHexNum(&Oled_Display, line, column, number, length);
}

bsp_oled_status_t Oled_ShowBinNum(uint8_t line,
                                  uint8_t column,
                                  uint32_t number,
                                  uint8_t length)
{
    return Bsp_Oled_ShowBinNum(&Oled_Display, line, column, number, length);
}

bsp_oled_status_t Oled_ShowChinese(uint8_t line, uint8_t column, uint8_t glyph_index)
{
    return Bsp_Oled_ShowChinese(&Oled_Display, line, column, glyph_index);
}
