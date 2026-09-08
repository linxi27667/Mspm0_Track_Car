#ifndef ENCODER_QUADRATURE_LOGIC_H
#define ENCODER_QUADRATURE_LOGIC_H

#include <stdint.h>

/* Return one 4x quadrature count for a valid Gray-code transition.
 * Repeated states and impossible two-bit jumps are ignored. */
static inline int8_t Encoder_Quadrature_Delta(uint8_t previous,
                                               uint8_t current)
{
    static const int8_t transition[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    return transition[((previous & 0x03u) << 2u) | (current & 0x03u)];
}

#endif /* ENCODER_QUADRATURE_LOGIC_H */
