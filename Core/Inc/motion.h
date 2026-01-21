#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "stm32f429xx.h"
#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MOTION_IDLE = 0,
        MOTION_RUNNING,
        MOTION_DONE
    } MotionState_t;

    void Motion_Init(void);
    void Motion_Stop(void);

    MotionState_t Motion_GetState(void);

    // Неблокирующее движение: запускаешь -> в main вызываешь Motion_Update()
    void Motion_MoveDistance_mm(int32_t distance_mm, int16_t base_pwm);
    void Motion_Update(void);

    // Блокирующие обёртки (по желанию)
    uint8_t Motion_RunDistance_Blocking(int32_t distance_mm, int16_t base_pwm, uint32_t timeout_ms);

    // Простой поворот "bang-bang" (если нужен), работает стабильно, но грубо
    uint8_t Motion_TurnSimple_Blocking(float angle_deg, int16_t pwm, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // MOTION_H
