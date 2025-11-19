// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "stm32f4xx.h"

// Настроим пины (если нужно - поменяешь под свою плату)
#define ENC_L_GPIO GPIOA
#define ENC_L_PIN 0U
#define ENC_L_GPIO_CLK RCC_AHB1ENR_GPIOAEN
#define ENC_L_EXTI_LINE 0U
#define ENC_L_IRQN EXTI0_IRQn

#define ENC_R_GPIO GPIOA
#define ENC_R_PIN 1U
#define ENC_R_GPIO_CLK RCC_AHB1ENR_GPIOAEN
#define ENC_R_EXTI_LINE 1U
#define ENC_R_IRQN EXTI1_IRQn

// Эмпирически: сколько тиков на один оборот колеса.
// Gри настройке на Arduino получалось ≈40 импульсов/оборот.
// Возьми это значение как PULSES_PER_REV, можно будет уточнить.
#define ENC_PULSES_PER_REV 40U

// Минимальный интервал между "честными" тиками, мс (аналог MIN_TICK_INTERVAL)
#define ENC_MIN_TICK_INTERVAL_MS 10U

void Encoder_Init(void);

// Забрать тики за интервал и обнулить "секундные" счётчики
void Encoder_GetAndResetTicks(uint32_t *leftTicks, uint32_t *rightTicks);

// Получить общее количество тиков (не обнуляется)
uint32_t Encoder_GetTotalLeft(void);
uint32_t Encoder_GetTotalRight(void);

#endif // ENCODER_H
