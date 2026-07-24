/**
 * @file    app_buzzer.c
 * @brief   MSPM0G3507蜂鸣器硬件绑定实现
 */
#include "app_buzzer.h"
#include "app_common.h"

/* ================= 1. 硬件底层函数 (HW_ 前缀) ================= */

static void HW_Buzzer_GpioWrite(void *port, uint32_t pin, uint8_t level)
{
    if (port == NULL)
    {
        return;
    }

    GPIO_WritePin((GPIO_Regs *)port, pin, level);
}

/* ================= 2. 对象实例化与引脚拼装 ================= */

bsp_buzzer_t Buzzer_Device = {
    .port = BUZZER_PORT,
    .pin = BUZZER_PIN,
    .on_time_ms = BUZZER_ON_TIME_MS,
    .gap_time_ms = BUZZER_GAP_TIME_MS,
    .active_level = BUZZER_ACTIVE_LEVEL,
    .Gpio_Write = HW_Buzzer_GpioWrite,
};

/* ================= 3. 对外业务切入点 ================= */

void App_Buzzer_Init(void)
{
    Bsp_Buzzer_Init(&Buzzer_Device);
}

void Buzzer_Request(uint8_t beep_count, uint32_t now_ms)
{
    Bsp_Buzzer_Request(&Buzzer_Device, beep_count, now_ms);
}

void Buzzer_Update(uint32_t now_ms)
{
    Bsp_Buzzer_Update(&Buzzer_Device, now_ms);
}

void Buzzer_Stop(void)
{
    Bsp_Buzzer_Stop(&Buzzer_Device);
}

bool Buzzer_IsBusy(void)
{
    return Bsp_Buzzer_IsBusy(&Buzzer_Device);
}
