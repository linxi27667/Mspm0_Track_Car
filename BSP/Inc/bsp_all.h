/**
 * @file    bsp_all.h
 * @brief   BSP 统一包含头文件 — 业务层只需包含此文件
 */
#ifndef __BSP_ALL_H__
#define __BSP_ALL_H__

/* 公共基础设施 */
#include "bsp_common.h"

/* 硬件驱动层 (BSP) */
#include "bsp_led.h"
#include "bsp_buzzer.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "bsp_encoder_motor.h"
#include "bsp_sw_i2c.h"
#include "bsp_oled.h"
#include "bsp_imu660rb.h"
#include "bsp_path_fish.h"
#include "bsp_esp_at.h"

#endif /* __BSP_ALL_H__ */
