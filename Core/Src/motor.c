// motor.c
// ============================================================
// Драйвер управления двумя DC-моторами через H-мост
// MCU: STM32F429ZI
//
// Используется:
//  - TIM1 для генерации PWM (20 кГц)
//  - GPIO для задания направления вращения
//
// Логика:
//  speed > 0  → вращение вперёд
//  speed < 0  → вращение назад
//  speed = 0  → электротормоз
// ============================================================

#include "motor.h"
#include "stm32f4xx.h" // <-- ОБЯЗАТЕЛЬНО
/* ============================================================
 *                АППАРАТНАЯ РАСКЛАДКА
 * ============================================================ *
 *
 * MOTOR_A (левый):
 *   PWM  → PE9   (TIM1_CH1)
 *   IN1  → PD4
 *   IN2  → PD5
 *
 * MOTOR_B (правый):
 *   PWM  → PE11  (TIM1_CH2)
 *   IN3  → PD6
 *   IN4  → PD7
 *
 * Направление задаётся парами INx:
 *
 *   Вперёд: INx1 = 0, INx2 = 1
 *   Назад:  INx1 = 1, INx2 = 0
 *   Стоп:   INx1 = 0, INx2 = 0   (электротормоз)
 *
 * PWM:
 *   - генерируется TIM1
 *   - частота 20 кГц (без слышимого писка)
 *   - CCRx ∈ [0 .. MOTOR_PWM_MAX]
 *
 * ============================================================ */

/* ============================================================
 *                 ЛОКАЛЬНЫЕ ПРОТОТИПЫ
 * ============================================================ */

static void Motor_ClockInit(void);
static void Motor_GPIO_DirPins_Init(void);
static void Motor_GPIO_PwmPins_Init(void);
static void Motor_TIM1_Init(void);

static void Motor_SetDir(MotorId id, int8_t dir);
static void Motor_SetPwm(MotorId id, uint16_t value);

/* ============================================================
 *                 ВКЛЮЧЕНИЕ ТАКТИРОВАНИЯ
 * ============================================================ */

static void Motor_ClockInit(void)
{
    // GPIO для направлений
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);

    // GPIO для PWM
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);

    // Таймер TIM1 (APB2)
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
}

/* ============================================================
 *             GPIO — ПИНЫ НАПРАВЛЕНИЯ
 * ============================================================ */

static void Motor_GPIO_DirPins_Init(void)
{
    /*
     * PD4 → IN1 (Motor A)
     * PD5 → IN2 (Motor A)
     * PD6 → IN3 (Motor B)
     * PD7 → IN4 (Motor B)
     */

    // MODE = output (01)
    MODIFY_REG(GPIOD->MODER,
               GPIO_MODER_MODER4_Msk |
                   GPIO_MODER_MODER5_Msk |
                   GPIO_MODER_MODER6_Msk |
                   GPIO_MODER_MODER7_Msk,
               (1U << GPIO_MODER_MODER4_Pos) |
                   (1U << GPIO_MODER_MODER5_Pos) |
                   (1U << GPIO_MODER_MODER6_Pos) |
                   (1U << GPIO_MODER_MODER7_Pos));

    // Push-pull
    CLEAR_BIT(GPIOD->OTYPER,
              GPIO_OTYPER_OT_4 |
                  GPIO_OTYPER_OT_5 |
                  GPIO_OTYPER_OT_6 |
                  GPIO_OTYPER_OT_7);

    // High speed (быстрый отклик)
    MODIFY_REG(GPIOD->OSPEEDR,
               GPIO_OSPEEDR_OSPEED4_Msk |
                   GPIO_OSPEEDR_OSPEED5_Msk |
                   GPIO_OSPEEDR_OSPEED6_Msk |
                   GPIO_OSPEEDR_OSPEED7_Msk,
               (3U << GPIO_OSPEEDR_OSPEED4_Pos) |
                   (3U << GPIO_OSPEEDR_OSPEED5_Pos) |
                   (3U << GPIO_OSPEEDR_OSPEED6_Pos) |
                   (3U << GPIO_OSPEEDR_OSPEED7_Pos));

    // Без подтяжек
    MODIFY_REG(GPIOD->PUPDR,
               GPIO_PUPDR_PUPD4_Msk |
                   GPIO_PUPDR_PUPD5_Msk |
                   GPIO_PUPDR_PUPD6_Msk |
                   GPIO_PUPDR_PUPD7_Msk,
               0U);

    // Начальное состояние: стоп (электротормоз)
    GPIOD->BSRR = GPIO_BSRR_BR_4 |
                  GPIO_BSRR_BR_5 |
                  GPIO_BSRR_BR_6 |
                  GPIO_BSRR_BR_7;
}

/* ============================================================
 *                GPIO — PWM ПИНЫ
 * ============================================================ */

static void Motor_GPIO_PwmPins_Init(void)
{
    /*
     * PE9  → TIM1_CH1 → Motor A
     * PE11 → TIM1_CH2 → Motor B
     */

    // Alternate Function
    MODIFY_REG(GPIOE->MODER,
               GPIO_MODER_MODER9_Msk |
                   GPIO_MODER_MODER11_Msk,
               (2U << GPIO_MODER_MODER9_Pos) |
                   (2U << GPIO_MODER_MODER11_Pos));

    // Push-pull
    CLEAR_BIT(GPIOE->OTYPER, GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_11);

    // High speed
    MODIFY_REG(GPIOE->OSPEEDR,
               GPIO_OSPEEDR_OSPEED9_Msk |
                   GPIO_OSPEEDR_OSPEED11_Msk,
               (3U << GPIO_OSPEEDR_OSPEED9_Pos) |
                   (3U << GPIO_OSPEEDR_OSPEED11_Pos));

    // Без подтяжек
    MODIFY_REG(GPIOE->PUPDR,
               GPIO_PUPDR_PUPD9_Msk |
                   GPIO_PUPDR_PUPD11_Msk,
               0U);

    // AF1 = TIM1
    MODIFY_REG(GPIOE->AFR[1],
               GPIO_AFRH_AFSEL9_Msk |
                   GPIO_AFRH_AFSEL11_Msk,
               (1U << GPIO_AFRH_AFSEL9_Pos) |
                   (1U << GPIO_AFRH_AFSEL11_Pos));
}

/* ============================================================
 *                   TIM1 — PWM
 * ============================================================ */
#define MOTOR_TIM1_PSC 7
#define MOTOR_TIM1_ARR 1000
static void Motor_TIM1_Init(void)
{
    // Останавливаем таймер на время настройки
    CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);

    // Частота PWM = 20 кГц
    TIM1->PSC = MOTOR_TIM1_PSC;
    TIM1->ARR = MOTOR_TIM1_ARR;

    // Channel 1 — PWM mode 1
    MODIFY_REG(TIM1->CCMR1,
               TIM_CCMR1_CC1S_Msk |
                   TIM_CCMR1_OC1M_Msk |
                   TIM_CCMR1_OC1PE_Msk,
               (0U << TIM_CCMR1_CC1S_Pos) |
                   (6U << TIM_CCMR1_OC1M_Pos) |
                   TIM_CCMR1_OC1PE);

    // Channel 2 — PWM mode 1
    MODIFY_REG(TIM1->CCMR1,
               TIM_CCMR1_CC2S_Msk |
                   TIM_CCMR1_OC2M_Msk |
                   TIM_CCMR1_OC2PE_Msk,
               (0U << TIM_CCMR1_CC2S_Pos) |
                   (6U << TIM_CCMR1_OC2M_Pos) |
                   TIM_CCMR1_OC2PE);

    // Включаем выходы
    SET_BIT(TIM1->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);

    // Начальный PWM = 0
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;

    // Advanced timer enable
    SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);

    // Обновление регистров
    SET_BIT(TIM1->EGR, TIM_EGR_UG);

    // Запуск таймера
    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
}

/* ============================================================
 *                   НИЗКИЙ УРОВЕНЬ
 * ============================================================ */

// Установка направления вращения
static void Motor_SetDir(MotorId id, int8_t dir)
{
    if (id == MOTOR_A)
    {
        if (dir > 0) // вперёд
            GPIOD->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BS_5;
        else if (dir < 0) // назад
            GPIOD->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BR_5;
        else // стоп
            GPIOD->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5;
    }
    else if (id == MOTOR_B)
    {
        if (dir > 0)
            GPIOD->BSRR = GPIO_BSRR_BR_6 | GPIO_BSRR_BS_7;
        else if (dir < 0)
            GPIOD->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BR_7;
        else
            GPIOD->BSRR = GPIO_BSRR_BR_6 | GPIO_BSRR_BR_7;
    }
}

// Установка PWM-скважности
static void Motor_SetPwm(MotorId id, uint16_t value)
{
    if (value > MOTOR_PWM_MAX)
        value = MOTOR_PWM_MAX;

    if (id == MOTOR_A)
        TIM1->CCR1 = value;
    else
        TIM1->CCR2 = value;
}

/* ============================================================
 *                   ПУБЛИЧНЫЙ API
 * ============================================================ */

void Motor_Init(void)
{
    Motor_ClockInit();
    Motor_GPIO_DirPins_Init();
    Motor_GPIO_PwmPins_Init();
    Motor_TIM1_Init();

    // безопасное состояние
    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
}

void Motor_SetSpeed(MotorId id, int16_t speed)
{
    if (speed == 0)
    {
        Motor_SetPwm(id, 0);
        Motor_SetDir(id, 0);
        return;
    }

    int8_t dir = (speed > 0) ? +1 : -1;
    uint16_t pwm = (speed > 0) ? speed : -speed;

    Motor_SetDir(id, dir);
    Motor_SetPwm(id, pwm);
}

void Motor_Stop(MotorId id)
{
    Motor_SetSpeed(id, 0);
}
