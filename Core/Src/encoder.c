// encoder.c
//
// Энкодеры через EXTI прерывания (ТОЛЬКО по FALLING edge).
// Идея: один спад (falling) = один тик.
// Это уменьшает шум/дребезг и убирает двойные срабатывания от "оба фронта".

#include "encoder.h"

extern volatile uint32_t g_msTicks; // SysTick ms

/* --------------------------------------------------------------------------
 * Переменные модуля
 * -------------------------------------------------------------------------- */

static volatile uint32_t s_leftTicks = 0;
static volatile uint32_t s_rightTicks = 0;

static volatile uint32_t s_leftTotal = 0;
static volatile uint32_t s_rightTotal = 0;

/* Антидребезг по времени (минимальный интервал между тиками) */
static volatile uint32_t s_leftLastMs = 0;
static volatile uint32_t s_rightLastMs = 0;

/* Локальные функции */
static void Encoder_GPIO_Init(void);
static void Encoder_EXTI_Init(void);

static inline void Encoder_CountLeftTick(void)
{
    uint32_t now = g_msTicks;

    // Антидребезг по времени (если ENC_MIN_TICK_INTERVAL_MS = 0 -> отключён)
    if (ENC_MIN_TICK_INTERVAL_MS == 0U || (now - s_leftLastMs) >= ENC_MIN_TICK_INTERVAL_MS)
    {
        s_leftTicks++;
        s_leftTotal++;
        s_leftLastMs = now;
    }
}

static inline void Encoder_CountRightTick(void)
{
    uint32_t now = g_msTicks;

    if (ENC_MIN_TICK_INTERVAL_MS == 0U || (now - s_rightLastMs) >= ENC_MIN_TICK_INTERVAL_MS)
    {
        s_rightTicks++;
        s_rightTotal++;
        s_rightLastMs = now;
    }
}

/* --------------------------------------------------------------------------
 * Инициализация
 * -------------------------------------------------------------------------- */
void Encoder_Init(void)
{
    Encoder_GPIO_Init();
    Encoder_EXTI_Init();

    // Стартовые метки времени
    s_leftLastMs = g_msTicks;
    s_rightLastMs = g_msTicks;

    // Обнуляем счётчики
    s_leftTicks = s_rightTicks = 0;
    s_leftTotal = s_rightTotal = 0;
}

/* --------------------------------------------------------------------------
 * GPIO: PA5/PA6 input + pull-up (как у тебя было)
 * -------------------------------------------------------------------------- */
static void Encoder_GPIO_Init(void)
{
    // Тактирование GPIOA
    SET_BIT(RCC->AHB1ENR, ENC_L_GPIO_CLK);

    // PA5, PA6 -> input (00)
    MODIFY_REG(ENC_L_GPIO->MODER,
               GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER6_Msk,
               0U);

    // pull-up (01)
    MODIFY_REG(ENC_L_GPIO->PUPDR,
               GPIO_PUPDR_PUPD5_Msk | GPIO_PUPDR_PUPD6_Msk,
               (0x1UL << GPIO_PUPDR_PUPD5_Pos) |
                   (0x1UL << GPIO_PUPDR_PUPD6_Pos));
}

/* --------------------------------------------------------------------------
 * EXTI: ТОЛЬКО FALLING edge на линиях 5 и 6
 * -------------------------------------------------------------------------- */
static void Encoder_EXTI_Init(void)
{
    // SYSCFG clock
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

    // EXTI5/6 -> Port A (0 = PA)
    MODIFY_REG(SYSCFG->EXTICR[1],
               SYSCFG_EXTICR2_EXTI5_Msk | SYSCFG_EXTICR2_EXTI6_Msk,
               0U);

    // На всякий случай: отключить обе линии, почистить триггеры, сбросить pending
    CLEAR_BIT(EXTI->IMR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));
    CLEAR_BIT(EXTI->RTSR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));
    CLEAR_BIT(EXTI->FTSR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));
    SET_BIT(EXTI->PR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE)); // clear pending

    // Включаем ТОЛЬКО FALLING
    SET_BIT(EXTI->FTSR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));

    // Разрешаем линии
    SET_BIT(EXTI->IMR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));

    // NVIC EXTI5..9
    NVIC_SetPriority(ENC_IRQN, 5);
    NVIC_EnableIRQ(ENC_IRQN);
}

/* --------------------------------------------------------------------------
 * IRQ EXTI5..9
 * -------------------------------------------------------------------------- */
void EXTI9_5_IRQHandler(void)
{
    // Линия 5 (левый)
    if (READ_BIT(EXTI->PR, (1U << ENC_L_EXTI_LINE)))
    {
        SET_BIT(EXTI->PR, (1U << ENC_L_EXTI_LINE)); // clear pending
        Encoder_CountLeftTick();
    }

    // Линия 6 (правый)
    if (READ_BIT(EXTI->PR, (1U << ENC_R_EXTI_LINE)))
    {
        SET_BIT(EXTI->PR, (1U << ENC_R_EXTI_LINE));
        Encoder_CountRightTick();
    }
}

/* --------------------------------------------------------------------------
 * Публичные функции (как было)
 * -------------------------------------------------------------------------- */

void Encoder_GetAndResetTicks(uint32_t *leftTicks, uint32_t *rightTicks)
{
    uint32_t l, r;

    __disable_irq();
    l = s_leftTicks;
    r = s_rightTicks;
    s_leftTicks = 0;
    s_rightTicks = 0;
    __enable_irq();

    if (leftTicks)
        *leftTicks = l;
    if (rightTicks)
        *rightTicks = r;
}

uint32_t Encoder_GetTotalLeft(void)
{
    uint32_t v;
    __disable_irq();
    v = s_leftTotal;
    __enable_irq();
    return v;
}

uint32_t Encoder_GetTotalRight(void)
{
    uint32_t v;
    __disable_irq();
    v = s_rightTotal;
    __enable_irq();
    return v;
}

float Encoder_TicksToMM(uint32_t ticks)
{
    return ticks * ENC_MM_PER_TICK;
}

float Encoder_TicksToMeters(uint32_t ticks)
{
    return ticks * ENC_M_PER_TICK;
}
