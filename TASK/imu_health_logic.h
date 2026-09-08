#ifndef IMU_HEALTH_LOGIC_H
#define IMU_HEALTH_LOGIC_H

#include <stdbool.h>

#include "BSP/Inc/bsp_imu660rb.h"

static inline bool Imu_Health_AllowsMotion(bool initialized,
                                           bool data_valid,
                                           imu_cal_state_t calibration_state)
{
    return initialized && data_valid && calibration_state == IMU_CAL_DONE;
}

#endif /* IMU_HEALTH_LOGIC_H */
