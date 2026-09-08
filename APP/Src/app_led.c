/**
 * @file    app_led.c
 * @brief   LED硬件绑定实现（MSPM0G3507）
 * @note    GPIO引脚和输出使能由SysConfig配置，无需手动初始化
 */
#include "app_led.h"
#include "app_common.h"

/* ================= 1. 硬件底层函数 (HW_ 前缀) ================= */

static void HW_Gpio_Write(void *port, uint32_t pin, uint8_t level)
{
    if (port == NULL) return;
    GPIO_WritePin((GPIO_Regs *)port, pin, level);
}

/* ================= 2. 对象实例化与引脚拼装 ================= */

led_t Led_Devices[BSP_LED_NUM] = {
    {LED1_PORT, LED1_PIN, 0, HW_Gpio_Write},
};

/* ================= 3. 对外业务切入点 ================= */

void App_Led_Init(void)
{
    Bsp_Led_Init(Led_Devices);
}

void Led_On(uint8_t index)
{
    if (index < BSP_LED_NUM) Bsp_Led_On(&Led_Devices[index]);
}

void Led_Off(uint8_t index)
{
    if (index < BSP_LED_NUM) Bsp_Led_Off(&Led_Devices[index]);
}

void Led_Toggle(uint8_t index)
{
    if (index >= BSP_LED_NUM) return;
    if (Led_Devices[index].state) {
        Bsp_Led_Off(&Led_Devices[index]);
    } else {
        Bsp_Led_On(&Led_Devices[index]);
    }
}
