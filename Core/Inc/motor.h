// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"
#include "stm32f4xx.h"

/*
 * Два мотора:
 *  - MOTOR_A: ENA = TIM1_CH1 (PE9), IN1/IN2 = PD11/PD12
 *  - MOTOR_B: ENB = TIM1_CH2 (PE11), IN3/IN4 = PD13/PD14
 */
#define MOTOR_TIM1_PSC 83U           /* Делитель: (PSC+1) = 84 */
#define MOTOR_TIM1_ARR 99U           /* Предел счётчика: (ARR+1) = 100 */
#define MOTOR_PWM_MAX MOTOR_TIM1_ARR /* Максимальное значение скважности 0..99 */

typedef enum
{
    MOTOR_A = 0, /* Левый / первый мотор, на CH1 (PE9) */
    MOTOR_B = 1  /* Правый / второй мотор, на CH2 (PE11) */
} MotorId;

/*
 * Максимальное значение "скорости" (по модулю), соответствующее 100% скважности.
 * Должно совпадать с ARR таймера TIM1 (мы выбрали ARR = 99).
 */
// #define MOTOR_PWM_MAX ((uint16_t)99U)

/* Инициализация:
 *  - включает тактирование TIM1, GPIOD, GPIOE
 *  - настраивает PD11..PD14 как выходы (IN1..IN4)
 *  - настраивает PE9/PE11 как PWM (TIM1_CH1/CH2, 20 кГц)
 *  - останавливает моторы
 */
void Motor_Init(void);

/* Установка скорости:
 *  - speed > 0: вперёд
 *  - speed < 0: назад
 *  - speed = 0: стоп (тормоз)
 *  - |speed| в пределах 0..MOTOR_PWM_MAX
 */


void Motor_SetSpeed(MotorId id, int16_t speed);

/* Явная остановка (эквивалент Motor_SetSpeed(id, 0)) */
void Motor_Stop(MotorId id);

#endif /* MOTOR_H */
