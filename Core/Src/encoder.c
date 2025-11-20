// encoder.c
#include "encoder.h"

extern volatile uint32_t g_msTicks; // SysTick миллисекунды

// Тики за последний "интервал" (между вызовами GetAndReset)
static volatile uint32_t s_leftTicks = 0;
static volatile uint32_t s_rightTicks = 0;

// Общие тики за всё время
static volatile uint32_t s_leftTotal = 0;
static volatile uint32_t s_rightTotal = 0;

// Последнее состояние пинов (для детекта фронта HIGH -> LOW)
static volatile uint8_t s_leftLastState = 1;
static volatile uint8_t s_rightLastState = 1;

// Время последнего принятого тика (мс) — для антидребезга
static volatile uint32_t s_leftLastMs = 0;
static volatile uint32_t s_rightLastMs = 0;

/* Внутренние функции */
static void Encoder_GPIO_Init(void);
static void Encoder_EXTI_Init(void);

/* ==== Инициализация всего модуля ==== */
void Encoder_Init(void)
{
    Encoder_GPIO_Init();
    Encoder_EXTI_Init();

    // Начальные состояния пинов
    s_leftLastState = (ENC_L_GPIO->IDR & (1U << ENC_L_PIN)) ? 1U : 0U;
    s_rightLastState = (ENC_R_GPIO->IDR & (1U << ENC_R_PIN)) ? 1U : 0U;

    s_leftLastMs = g_msTicks;
    s_rightLastMs = g_msTicks;

    s_leftTicks = 0;
    s_rightTicks = 0;
    s_leftTotal = 0;
    s_rightTotal = 0;
}

/* ==== Настройка GPIO: PA5, PA6 как входы с pull-up ==== */
static void Encoder_GPIO_Init(void)
{
    // Включаем тактирование порта A
    SET_BIT(RCC->AHB1ENR, ENC_L_GPIO_CLK); // для обоих энкодеров GPIOA

    // PA5, PA6: input (00), pull-up

    // MODER5/6 = 00 -> вход
    MODIFY_REG(ENC_L_GPIO->MODER,
               GPIO_MODER_MODER5_Msk |
                   GPIO_MODER_MODER6_Msk,
               0U);

    // PUPDR5/6 = 01 -> pull-up
    MODIFY_REG(ENC_L_GPIO->PUPDR,
               GPIO_PUPDR_PUPD5_Msk |
                   GPIO_PUPDR_PUPD6_Msk,
               (0x1UL << GPIO_PUPDR_PUPD5_Pos) |
                   (0x1UL << GPIO_PUPDR_PUPD6_Pos));
}

/* ==== Настройка EXTI на линии 5 и 6 (PA5, PA6) ==== */
static void Encoder_EXTI_Init(void)
{
    // Включаем тактирование SYSCFG
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

    // Привязываем EXTI5 и EXTI6 к порту A
    //
    // EXTI4..7 -> EXTICR[1]
    // EXTI5 -> EXTICR[1] bits 7:4
    // EXTI6 -> EXTICR[1] bits 11:8
    // 0000 = PAx
    MODIFY_REG(SYSCFG->EXTICR[1],
               SYSCFG_EXTICR2_EXTI5_Msk |
                   SYSCFG_EXTICR2_EXTI6_Msk,
               0U);

    // Разрешаем линии 5 и 6 в маске прерываний
    SET_BIT(EXTI->IMR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));

    // Включаем прерывания по обоим фронтам (потом внутри отфильтруем)
    SET_BIT(EXTI->RTSR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));
    SET_BIT(EXTI->FTSR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));

    // NVIC: включаем общий IRQ для EXTI5..9
    NVIC_SetPriority(ENC_IRQN, 5);
    NVIC_EnableIRQ(ENC_IRQN);
}

/* ==== Внутренняя обработка фронта левго энкодера ==== */
static inline void Encoder_HandleEdge_Left(void)
{
    uint8_t state = (ENC_L_GPIO->IDR & (1U << ENC_L_PIN)) ? 1U : 0U;
    uint32_t now = g_msTicks;

    // Ищем переход HIGH -> LOW
    if (s_leftLastState == 1U && state == 0U)
    {
        if ((now - s_leftLastMs) > ENC_MIN_TICK_INTERVAL_MS)
        {
            s_leftTicks++;
            s_leftTotal++;
            s_leftLastMs = now;
        }
    }

    s_leftLastState = state;
}

/* ==== Внутренняя обработка фронта правого энкодера ==== */
static inline void Encoder_HandleEdge_Right(void)
{
    uint8_t state = (ENC_R_GPIO->IDR & (1U << ENC_R_PIN)) ? 1U : 0U;
    uint32_t now = g_msTicks;

    // Ищем переход HIGH -> LOW
    if (s_rightLastState == 1U && state == 0U)
    {
        if ((now - s_rightLastMs) > ENC_MIN_TICK_INTERVAL_MS)
        {
            s_rightTicks++;
            s_rightTotal++;
            s_rightLastMs = now;
        }
    }

    s_rightLastState = state;
}

/* ==== Общий обработчик EXTI5..9 ==== */
void EXTI9_5_IRQHandler(void)
{
    // Линия 5 (левый энкодер)
    if (READ_BIT(EXTI->PR, (1U << ENC_L_EXTI_LINE)))
    {
        // Сбрасываем флаг
        SET_BIT(EXTI->PR, (1U << ENC_L_EXTI_LINE));
        // Обрабатываем фронт
        Encoder_HandleEdge_Left();
    }

    // Линия 6 (правый энкодер)
    if (READ_BIT(EXTI->PR, (1U << ENC_R_EXTI_LINE)))
    {
        SET_BIT(EXTI->PR, (1U << ENC_R_EXTI_LINE));
        Encoder_HandleEdge_Right();
    }
}

/* ==== Публичные функции ==== */

/* Забрать тики за интервал и обнулить счётчики "интервала" */
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

/* Общие накопленные тики (левый) */
uint32_t Encoder_GetTotalLeft(void)
{
    uint32_t v;
    __disable_irq();
    v = s_leftTotal;
    __enable_irq();
    return v;
}

/* Общие накопленные тики (правый) */
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
