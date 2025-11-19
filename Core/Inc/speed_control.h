// speed_control.h
#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdint.h>
#include "pid.h"
#include "motor.h"
#include "encoder.h"

void SpeedControl_Init(void);

// Задание целевой скорости в "оборотах в секунду" (или в тиках/сек — как удобнее)
void SpeedControl_SetTarget(float left_rps, float right_rps);

// Вызывать периодически раз в dt (например, каждые 10 мс)
// dt_sec — время в секундах (0.01f для 10 мс)
void SpeedControl_Update(float dt_sec);

#endif // SPEED_CONTROL_H
