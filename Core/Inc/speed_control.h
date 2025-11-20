#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "stm32f4xx.h"

/* Инициализация ПИД-регуляторов скорости */
void SpeedControl_Init(void);

/*
 * Задание целевой скорости колёс:
 *  left_rps / right_rps — обороты в СЕКУНДУ (без знака, >= 0),
 *  dir_left / dir_right — направление (+1 вперёд, -1 назад).
 */
void SpeedControl_SetTarget(float left_rps, float right_rps,
                            int8_t dir_left, int8_t dir_right);

/* Остановка (обе цели = 0, моторы в ноль) */
void SpeedControl_Stop(void);

/* Обновление ПИД по скорости, вызывать с периодом dt_sec (например, каждые 10–20 мс) */
void SpeedControl_Update(float dt_sec);

#endif /* SPEED_CONTROL_H */
