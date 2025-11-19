// encoder.c
#include "encoder.h"

extern volatile uint32_t g_msTicks; // SysTick миллисекунды

// Секундные (или "интервальные") тики
static volatile uint32_t s_leftTicks = 0;
static volatile uint32_t s_rightTicks = 0;

// Общие тики
static volatile uint32_t s_leftTotal = 0;
static volatile uint32_t s_rightTotal = 0;

// Последнее состояние на пине (для фронта HIGH->LOW)
static volatile uint8_t s_leftLastState = 1;
static volatile uint8_t s_rightLastState = 1;

// Время последнего принятого тика (мс)
static volatile uint32_t s_leftLastMs = 0;
static volatile uint32_t s_rightLastMs = 0;

/* Внутренние функции */
static void Encoder_GPIO_Init(void);
static void Encoder_EXTI_Init(void);

void Encoder_Init(void)
{
    Encoder_GPIO_Init();
    Encoder_EXTI_Init();

    // Инициализируем начальное состояние
    s_leftLastState = (ENC_L_GPIO->IDR & (1U << ENC_L_PIN)) ? 1U : 0U;
    s_rightLastState = (ENC_R_GPIO->IDR & (1U << ENC_R_PIN)) ? 1U : 0U;

    s_leftLastMs = g_msTicks;
    s_rightLastMs = g_msTicks;

    // Обнулим счётчики
    s_leftTicks = s_rightTicks = 0;
    s_leftTotal = s_rightTotal = 0;
}

/* Настройка GPIO: вход с подтяжкой вверх */
static void Encoder_GPIO_Init(void)
{
    // Включаем тактирование порта A (если ещё не)
    SET_BIT(RCC->AHB1ENR, ENC_L_GPIO_CLK);

    // PA0, PA1: input (00), pull-up
    // MODER0/1 = 00 -> input
    MODIFY_REG(ENC_L_GPIO->MODER,
               GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk,
               0U);

    // PUPDR0/1 = 01 -> pull-up
    MODIFY_REG(ENC_L_GPIO->PUPDR,
               GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD1_Msk,
               (0x1UL << GPIO_PUPDR_PUPD0_Pos) |
                   (0x1UL << GPIO_PUPDR_PUPD1_Pos));
}

/* Настройка EXTI на линии 0 и 1 по CHANGE (потом внутри фильтруем до falling) */
static void Encoder_EXTI_Init(void)
{
    // Включаем тактирование SYSCFG
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

    // Привяжем EXTI0 и EXTI1 к порту A
    // EXTI0 -> EXTICR[0] bits 3:0, EXTI1 -> bits 7:4
    MODIFY_REG(SYSCFG->EXTICR[0],
               SYSCFG_EXTICR1_EXTI0_Msk | SYSCFG_EXTICR1_EXTI1_Msk,
               0U); // 0000: PAx

    // Разрешаем линию 0 и 1 в маске прерываний
    SET_BIT(EXTI->IMR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));

    // Включаем прерывание по обоим фронтам (RTSR и FTSR)
    SET_BIT(EXTI->RTSR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));
    SET_BIT(EXTI->FTSR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));

    // NVIC: включаем EXTI0 и EXTI1
    NVIC_SetPriority(ENC_L_IRQN, 5);
    NVIC_EnableIRQ(ENC_L_IRQN);

    NVIC_SetPriority(ENC_R_IRQN, 5);
    NVIC_EnableIRQ(ENC_R_IRQN);
}

/* Общая логика обработки фронта для одного энкодера */
static inline void Encoder_HandleEdge_Left(void)
{
    uint8_t state = (ENC_L_GPIO->IDR & (1U << ENC_L_PIN)) ? 1U : 0U;
    uint32_t now = g_msTicks;

    // Ищем переход HIGH -> LOW (как в Arduino-примере)
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

static inline void Encoder_HandleEdge_Right(void)
{
    uint8_t state = (ENC_R_GPIO->IDR & (1U << ENC_R_PIN)) ? 1U : 0U;
    uint32_t now = g_msTicks;

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

/* Обработчики прерываний EXTI0 и EXTI1 */
void EXTI0_IRQHandler(void)
{
    if (READ_BIT(EXTI->PR, (1U << ENC_L_EXTI_LINE)))
    {
        // Сбрасываем флаг
        SET_BIT(EXTI->PR, (1U << ENC_L_EXTI_LINE));
        // Обрабатываем фронт
        Encoder_HandleEdge_Left();
    }
}

void EXTI1_IRQHandler(void)
{
    if (READ_BIT(EXTI->PR, (1U << ENC_R_EXTI_LINE)))
    {
        SET_BIT(EXTI->PR, (1U << ENC_R_EXTI_LINE));
        Encoder_HandleEdge_Right();
    }
}

/* Публичные функции */

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
