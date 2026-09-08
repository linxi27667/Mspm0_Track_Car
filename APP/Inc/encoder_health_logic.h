#ifndef ENCODER_HEALTH_LOGIC_H
#define ENCODER_HEALTH_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

static inline bool Encoder_Health_DeltaValid(int32_t delta,
                                             int32_t max_delta)
{
    return delta >= -max_delta && delta <= max_delta;
}

#endif
