// encoder.c
//
// Модуль подсчёта тиков энкодеров через EXTI прерывания.
//
// Реализует:
//  - Инициализацию GPIO (PA5, PA6) как входов с pull-up
//  - Настройку EXTI5 и EXTI6 на оба фронта
//  - Защиту от дребезга по времени
//  - Подсчёт:
//        * тиков за интервал (GetAndResetTicks)
//        * суммарных тиков с момента старта
//
// Логика детекции:
//   - EXTI вызывает IRQ на любом фронте.
//   - В обработчике читаем реальное состояние пина.
//   - Если обнаружен переход HIGH -> LOW, считаем как "тик".
//   - Интервал между тиками фильтруем по ENC_MIN_TICK_INTERVAL_MS.
//

#include "encoder.h"

extern volatile uint32_t g_msTicks; // Глобальная миллисекундная метка SysTick

/* --------------------------------------------------------------------------
 * Переменные модуля (статические)
 * -------------------------------------------------------------------------- */

// Счётчики тиков за последний "интервал" (между вызовами GetAndResetTicks)
static volatile uint32_t s_leftTicks = 0;
static volatile uint32_t s_rightTicks = 0;

// Общие суммарные тики с момента запуска системы
static volatile uint32_t s_leftTotal = 0;
static volatile uint32_t s_rightTotal = 0;

// Последнее состояние пинов (1/0) — используется для детекции перехода HIGH→LOW
static volatile uint8_t s_leftLastState = 1;
static volatile uint8_t s_rightLastState = 1;

// Метка времени последнего принятого тика — антидребезг
static volatile uint32_t s_leftLastMs = 0;
static volatile uint32_t s_rightLastMs = 0;

/* Локальные функции */
static void Encoder_GPIO_Init(void);
static void Encoder_EXTI_Init(void);

/* --------------------------------------------------------------------------
 * Инициализация энкодеров
 * -------------------------------------------------------------------------- */
void Encoder_Init(void)
{
    Encoder_GPIO_Init();
    Encoder_EXTI_Init();

    // Считываем реальные текущие уровни пинов
    s_leftLastState = (ENC_L_GPIO->IDR & (1U << ENC_L_PIN)) ? 1U : 0U;
    s_rightLastState = (ENC_R_GPIO->IDR & (1U << ENC_R_PIN)) ? 1U : 0U;

    // Начальные метки времени (для антидребезга)
    s_leftLastMs = g_msTicks;
    s_rightLastMs = g_msTicks;

    // Обнуляем счётчики
    s_leftTicks = s_rightTicks = 0;
    s_leftTotal = s_rightTotal = 0;
}

/* --------------------------------------------------------------------------
 * Настройка GPIO: PA5 и PA6 как входы с pull-up
 * -------------------------------------------------------------------------- */
static void Encoder_GPIO_Init(void)
{
    // Включаем тактирование GPIOA
    SET_BIT(RCC->AHB1ENR, ENC_L_GPIO_CLK);

    // PA5 и PA6 → режим входа (MODER = 00)
    MODIFY_REG(ENC_L_GPIO->MODER,
               GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER6_Msk,
               0U);

    // Устанавливаем подтяжку "pull-up" (PUPDR = 01)
    MODIFY_REG(ENC_L_GPIO->PUPDR,
               GPIO_PUPDR_PUPD5_Msk | GPIO_PUPDR_PUPD6_Msk,
               (0x1UL << GPIO_PUPDR_PUPD5_Pos) |
                   (0x1UL << GPIO_PUPDR_PUPD6_Pos));
}

/* --------------------------------------------------------------------------
 * Настройка EXTI для линий 5 и 6
 * -------------------------------------------------------------------------- */
static void Encoder_EXTI_Init(void)
{
    // Включаем модуль SYSCFG — нужен для привязки EXTI к GPIO
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);

    // Привязываем EXTI линий 5 и 6 к PA5 и PA6
    MODIFY_REG(SYSCFG->EXTICR[1],
               SYSCFG_EXTICR2_EXTI5_Msk | SYSCFG_EXTICR2_EXTI6_Msk,
               0U); // 0 = порт A

    // Разрешаем прерывания по линиям 5 и 6
    SET_BIT(EXTI->IMR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));

    // Включаем реакцию на оба фронта (Rising и Falling)
    SET_BIT(EXTI->RTSR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));
    SET_BIT(EXTI->FTSR, (1U << ENC_L_EXTI_LINE) | (1U << ENC_R_EXTI_LINE));

    // Разрешаем IRQ группы EXTI5–9
    NVIC_SetPriority(ENC_IRQN, 5);
    NVIC_EnableIRQ(ENC_IRQN);
}

/* --------------------------------------------------------------------------
 * Обработка фронта левого энкодера
 * -------------------------------------------------------------------------- */
static inline void Encoder_HandleEdge_Left(void)
{
    // Текущее логическое состояние входа
    uint8_t state = (ENC_L_GPIO->IDR & (1U << ENC_L_PIN)) ? 1U : 0U;
    uint32_t now = g_msTicks;

    // Детектор "тик" = переход HIGH → LOW
    if (s_leftLastState == 1U && state == 0U)
    {
        // Антидребезг по времени
        if ((now - s_leftLastMs) > ENC_MIN_TICK_INTERVAL_MS)
        {
            s_leftTicks++; // tики за интервал
            s_leftTotal++; // суммарные тики
            s_leftLastMs = now;
        }
    }

    s_leftLastState = state;
}

/* --------------------------------------------------------------------------
 * Обработка фронта правого энкодера
 * -------------------------------------------------------------------------- */
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

/* --------------------------------------------------------------------------
 * Основной обработчик EXTI5..9
 * -------------------------------------------------------------------------- */
void EXTI9_5_IRQHandler(void)
{
    // Проверяем: пришло ли прерывание с линии 5?
    if (READ_BIT(EXTI->PR, (1U << ENC_L_EXTI_LINE)))
    {
        SET_BIT(EXTI->PR, (1U << ENC_L_EXTI_LINE)); // сброс флага
        Encoder_HandleEdge_Left();
    }

    // Линия 6
    if (READ_BIT(EXTI->PR, (1U << ENC_R_EXTI_LINE)))
    {
        SET_BIT(EXTI->PR, (1U << ENC_R_EXTI_LINE));
        Encoder_HandleEdge_Right();
    }
}

/* --------------------------------------------------------------------------
 * Публичные функции
 * -------------------------------------------------------------------------- */

/* Возвращает тики за интервал dt и обнуляет счётчики */
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

/* Возвращает суммарные тики левого колеса */
uint32_t Encoder_GetTotalLeft(void)
{
    uint32_t v;
    __disable_irq();
    v = s_leftTotal;
    __enable_irq();
    return v;
}

/* Возвращает суммарные тики правого колеса */
uint32_t Encoder_GetTotalRight(void)
{
    uint32_t v;
    __disable_irq();
    v = s_rightTotal;
    __enable_irq();
    return v;
}

/* Перевод тиков в миллиметры */
float Encoder_TicksToMM(uint32_t ticks)
{
    return ticks * ENC_MM_PER_TICK;
}

/* Перевод тиков в метры */
float Encoder_TicksToMeters(uint32_t ticks)
{
    return ticks * ENC_M_PER_TICK;
}
