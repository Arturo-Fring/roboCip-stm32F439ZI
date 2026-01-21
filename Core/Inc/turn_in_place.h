#ifndef TURN_IN_PLACE_H
#define TURN_IN_PLACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Поворот на месте по гироскопу: angle_deg (знак важен)
    void TurnByAngleDeg(float angle_deg, int16_t pwm_max);

#ifdef __cplusplus
}
#endif

#endif // TURN_IN_PLACE.H
