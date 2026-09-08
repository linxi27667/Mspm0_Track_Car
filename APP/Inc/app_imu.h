/**
 * @file    app_imu.h
 * @brief   IMU660RB hardware binding and non-blocking IMU services.
 */
#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_imu660rb.h"

extern imu_sensor_t Imu_Sensor;

bool App_Imu_Init(void);
void App_Imu_Request_Recovery(void);
void App_Imu_Recovery_Update(uint32_t now, bool motion_active);
bool Imu_Read_Data(void);
void Imu_Calibration_Start(void);
imu_cal_state_t Imu_Calibration_Update(void);
bool Imu_Update_Heading(float dt);
bool Imu_Update_Stationary(float dt);

#endif /* APP_IMU_H */
