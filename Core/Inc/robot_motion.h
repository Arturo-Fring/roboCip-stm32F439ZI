#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include "stm32f4xx.h"

/* Едем distance_mm мм с заданным PWM (по модулю),
 * знак направления будет задаваться через MoveForward/MoveBackward.
 */
void DriveDistanceMM(float distance_mm, int16_t speed);

/* Удобные обёртки */
void MoveForwardMM(float distance_mm, int16_t pwm);
void MoveBackwardMM(float distance_mm, int16_t pwm);

#endif // ROBOT_MOTION_H
