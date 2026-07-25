#ifndef MOTOR_SPEED_LOGIC_H
#define MOTOR_SPEED_LOGIC_H

#include <stdint.h>

static inline uint32_t Motor_Pwm_DutyToCompare(uint32_t duty,
                                               uint32_t period)
{
    if (duty > period) duty = period;
    return duty;
}

static inline float Motor_Speed_LowPass(float previous, float sample, float alpha)
{
    return previous + alpha * (sample - previous);
}

/* The speed correction may coast, but it must not reverse the bridge against
 * the requested wheel direction. The caller only requests minimum drive while
 * the filtered wheel speed is near zero. */
static inline int32_t Motor_Speed_ConstrainPwm(int32_t target_speed,
                                               int32_t pwm,
                                               int32_t minimum_drive)
{
    if (target_speed == 0) return 0;

    if (target_speed > 0) {
        if (pwm <= 0) return 0;
        if (minimum_drive > 0 && pwm < minimum_drive) pwm = minimum_drive;
        return (pwm > 100) ? 100 : pwm;
    }

    if (pwm >= 0) return 0;
    if (minimum_drive > 0 && pwm > -minimum_drive) pwm = -minimum_drive;
    return (pwm < -100) ? -100 : pwm;
}

#endif /* MOTOR_SPEED_LOGIC_H */
