/**
 * @file    app_buzzer.h
 * @brief   板子1.0蜂鸣器硬件绑定与一次业务接口
 */
#ifndef APP_BUZZER_H
#define APP_BUZZER_H

#include "bsp_buzzer.h"
#include "ti_msp_dl_config.h"

/* PB5蜂鸣器控制按实物接线配置为低有效。 */
#define BUZZER_PORT             Board_Output_PORT
#define BUZZER_PIN              Board_Output_BUZZER_PIN
#define BUZZER_ACTIVE_LEVEL     0u
#define BUZZER_ON_TIME_MS       60u
#define BUZZER_GAP_TIME_MS      60u

extern bsp_buzzer_t Buzzer_Device;

void App_Buzzer_Init(void);
void Buzzer_Request(uint8_t beep_count, uint32_t now_ms);
void Buzzer_Update(uint32_t now_ms);
void Buzzer_Stop(void);
bool Buzzer_IsBusy(void);

#endif /* APP_BUZZER_H */
