/**
 * @file    bsp_buzzer.c
 * @brief   非阻塞蜂鸣器节奏状态机
 */
#include "bsp_buzzer.h"

static void Bsp_Buzzer_WriteActive(bsp_buzzer_t *buzzer, bool active)
{
    uint8_t level;

    if (buzzer == NULL || buzzer->Gpio_Write == NULL)
    {
        return;
    }

    level = active ? buzzer->active_level : (uint8_t)!buzzer->active_level;
    buzzer->Gpio_Write(buzzer->port, buzzer->pin, level);
}

static bool Bsp_Buzzer_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

void Bsp_Buzzer_Init(bsp_buzzer_t *buzzer)
{
    if (buzzer == NULL)
    {
        return;
    }

    buzzer->deadline_ms = 0u;
    buzzer->remaining_beeps = 0u;
    buzzer->state = BSP_BUZZER_STATE_IDLE;
    Bsp_Buzzer_WriteActive(buzzer, false);
}

void Bsp_Buzzer_Request(bsp_buzzer_t *buzzer, uint8_t beep_count, uint32_t now_ms)
{
    if (buzzer == NULL || buzzer->Gpio_Write == NULL || beep_count == 0u)
    {
        return;
    }

    if (buzzer->state != BSP_BUZZER_STATE_IDLE)
    {
        if (beep_count > (uint8_t)(UINT8_MAX - buzzer->remaining_beeps))
        {
            buzzer->remaining_beeps = UINT8_MAX;
        }
        else
        {
            buzzer->remaining_beeps = (uint8_t)(buzzer->remaining_beeps + beep_count);
        }
        return;
    }

    buzzer->remaining_beeps = beep_count;
    buzzer->state = BSP_BUZZER_STATE_ON;
    buzzer->deadline_ms = now_ms + buzzer->on_time_ms;
    Bsp_Buzzer_WriteActive(buzzer, true);
}

void Bsp_Buzzer_Update(bsp_buzzer_t *buzzer, uint32_t now_ms)
{
    if (buzzer == NULL || buzzer->state == BSP_BUZZER_STATE_IDLE ||
        !Bsp_Buzzer_TimeReached(now_ms, buzzer->deadline_ms))
    {
        return;
    }

    if (buzzer->state == BSP_BUZZER_STATE_ON)
    {
        Bsp_Buzzer_WriteActive(buzzer, false);
        if (buzzer->remaining_beeps > 0u)
        {
            buzzer->remaining_beeps--;
        }

        if (buzzer->remaining_beeps == 0u)
        {
            buzzer->state = BSP_BUZZER_STATE_IDLE;
        }
        else
        {
            buzzer->state = BSP_BUZZER_STATE_GAP;
            buzzer->deadline_ms = now_ms + buzzer->gap_time_ms;
        }
    }
    else
    {
        Bsp_Buzzer_WriteActive(buzzer, true);
        buzzer->state = BSP_BUZZER_STATE_ON;
        buzzer->deadline_ms = now_ms + buzzer->on_time_ms;
    }
}

void Bsp_Buzzer_Stop(bsp_buzzer_t *buzzer)
{
    if (buzzer == NULL)
    {
        return;
    }

    Bsp_Buzzer_WriteActive(buzzer, false);
    buzzer->remaining_beeps = 0u;
    buzzer->state = BSP_BUZZER_STATE_IDLE;
}

bool Bsp_Buzzer_IsBusy(const bsp_buzzer_t *buzzer)
{
    return buzzer != NULL && buzzer->state != BSP_BUZZER_STATE_IDLE;
}
