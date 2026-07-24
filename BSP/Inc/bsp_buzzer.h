/**
 * @file    bsp_buzzer.h
 * @brief   非阻塞蜂鸣器节奏驱动（BSP — 纯逻辑，硬件无关）
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    BSP_BUZZER_STATE_IDLE = 0,
    BSP_BUZZER_STATE_ON,
    BSP_BUZZER_STATE_GAP
} bsp_buzzer_state_t;

typedef struct {
    void *port;
    uint32_t pin;
    uint32_t deadline_ms;
    uint16_t on_time_ms;
    uint16_t gap_time_ms;
    uint8_t active_level;
    uint8_t remaining_beeps;
    bsp_buzzer_state_t state;
    void (*Gpio_Write)(void *port, uint32_t pin, uint8_t level);
} bsp_buzzer_t;

void Bsp_Buzzer_Init(bsp_buzzer_t *buzzer);
void Bsp_Buzzer_Request(bsp_buzzer_t *buzzer, uint8_t beep_count, uint32_t now_ms);
void Bsp_Buzzer_Update(bsp_buzzer_t *buzzer, uint32_t now_ms);
void Bsp_Buzzer_Stop(bsp_buzzer_t *buzzer);
bool Bsp_Buzzer_IsBusy(const bsp_buzzer_t *buzzer);

#endif /* BSP_BUZZER_H */
