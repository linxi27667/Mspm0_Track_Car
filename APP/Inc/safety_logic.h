#ifndef SAFETY_LOGIC_H
#define SAFETY_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SAFETY_LOGIC_OK = 0,
    SAFETY_LOGIC_GRAYSCALE_EXPIRED,
    SAFETY_LOGIC_SENSOR_ABNORMAL,
} safety_logic_result_t;

typedef struct {
    bool grayscale_required;
    bool grayscale_seen;
    bool sensor_abnormal;
    uint32_t last_grayscale_tick;
} safety_health_t;

static inline bool Safety_Logic_CanEnterRotation(uint8_t state)
{
    /* Rotation is IMU/encoder based and does not consume grayscale data.
     * BOOT here means only that grayscale white+black calibration is pending;
     * the caller still validates IMU health. */
    return state == 0u || state == 2u; /* BOOT or READY */
}

static inline bool Safety_Logic_FinishedHoldElapsed(uint32_t now,
                                                     uint32_t finished_at,
                                                     uint32_t hold_ms)
{
    return (uint32_t)(now - finished_at) >= hold_ms;
}

static inline safety_logic_result_t Safety_Logic_Check(
    uint32_t now, const safety_health_t *health,
    uint32_t grayscale_timeout)
{
    if (health->sensor_abnormal) return SAFETY_LOGIC_SENSOR_ABNORMAL;
    if (health->grayscale_required &&
        (!health->grayscale_seen ||
         (uint32_t)(now - health->last_grayscale_tick) > grayscale_timeout)) {
        return SAFETY_LOGIC_GRAYSCALE_EXPIRED;
    }
    return SAFETY_LOGIC_OK;
}

#endif
