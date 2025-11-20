// encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx.h"

/* ==== Настройки пинов энкодеров ==== */
/* Левый энкодер: PA5, EXTI5 */
#define ENC_L_GPIO GPIOA
#define ENC_L_GPIO_CLK RCC_AHB1ENR_GPIOAEN
#define ENC_L_PIN 5
#define ENC_L_EXTI_LINE 5

/* Правый энкодер: PA6, EXTI6 */
#define ENC_R_GPIO GPIOA
#define ENC_R_GPIO_CLK RCC_AHB1ENR_GPIOAEN
#define ENC_R_PIN 6
#define ENC_R_EXTI_LINE 6

/* Линии 5..9 сидят на одном IRQ: EXTI9_5_IRQn */
#define ENC_IRQN EXTI9_5_IRQn

/* ==== Параметры энкодера ==== */
/* Поставь реальное число импульсов на оборот (дырки * делитель и т.д.) */
#define ENC_PULSES_PER_REV 40U // твои реальные данные

#define WHEEL_DIAMETER_MM 65.0f
#define WHEEL_CIRCUMFERENCE_MM (3.1415926f * WHEEL_DIAMETER_MM)

#define ENC_MM_PER_TICK (WHEEL_CIRCUMFERENCE_MM / ENC_PULSES_PER_REV) * 0.95
#define ENC_M_PER_TICK (ENC_MM_PER_TICK / 1000.0f)

/* Минимальный интервал между тиками (мс) — защита от дребезга */
#define ENC_MIN_TICK_INTERVAL_MS 2U

#ifdef __cplusplus
extern "C"
{
#endif

    void Encoder_Init(void);

    /* За заданный интервал времени (dt) забрать тики и обнулить */
    void Encoder_GetAndResetTicks(uint32_t *leftTicks, uint32_t *rightTicks);

    /* Общие накопленные тики с момента включения */
    uint32_t Encoder_GetTotalLeft(void);
    uint32_t Encoder_GetTotalRight(void);

    // Перевод тиков в мм и в м
    float Encoder_TicksToMM(uint32_t ticks);
    float Encoder_TicksToMeters(uint32_t ticks);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
