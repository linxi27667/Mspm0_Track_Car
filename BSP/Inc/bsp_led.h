/**
 * @file    bsp_led.h
 * @brief   LED 抽象驱动层（BSP — 纯逻辑，硬件无关）
 */
#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include <stdint.h>
#include <stddef.h>

/* --- 抽象 GPIO 对象 --- */
typedef struct {
    void    *port;
    uint32_t pin;
    uint8_t  state;
    void (*Gpio_Write)(void *port, uint32_t pin, uint8_t level);
} led_t;

#define BSP_LED_NUM 1

void Bsp_Led_Init(led_t *led);
void Bsp_Led_On(led_t *led);
void Bsp_Led_Off(led_t *led);

#endif /* __BSP_LED_H__ */
