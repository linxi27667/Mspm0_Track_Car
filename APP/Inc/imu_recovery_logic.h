#ifndef IMU_RECOVERY_LOGIC_H
#define IMU_RECOVERY_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_RECOVERY_BASE_DELAY_MS 500u
#define IMU_RECOVERY_MAX_DELAY_MS  5000u

static inline uint32_t Imu_Recovery_BackoffMs(uint8_t failures)
{
    uint32_t delay = IMU_RECOVERY_BASE_DELAY_MS;
    uint8_t shifts = failures > 4u ? 4u : failures;

    delay <<= shifts;
    return delay > IMU_RECOVERY_MAX_DELAY_MS
         ? IMU_RECOVERY_MAX_DELAY_MS : delay;
}

static inline bool Imu_Recovery_Due(uint32_t now, uint32_t scheduled_at,
                                    uint32_t delay_ms)
{
    return (uint32_t)(now - scheduled_at) >= delay_ms;
}

#endif /* IMU_RECOVERY_LOGIC_H */
