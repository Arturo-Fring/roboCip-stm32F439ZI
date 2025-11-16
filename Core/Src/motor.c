#include "motor.h"
// motor.c

/* --- ЛОКАЛЬНЫЕ ПРОТОТИПЫ --- */

static void Motor_ClockInit(void);
static void Motor_GPIO_DirPins_Init(void);
static void Motor_GPIO_PwmPins_Init(void);
static void Motor_TIM1_Init(void);

static void Motor_SetDir(MotorId id, int8_t dir);
static void Motor_SetPwm(MotorId id, uint16_t value);

// Включение тактирования для портов мотора и таймеров
static void Motor_ClockInit(void)
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);

    // Включаем тактирование TIM1 (шина APB2)
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
}

// IN1..IN4: PD11–PD14 как обычные выходы
static void Motor_GPIO_DirPins_Init(void)
{
    /* --- 0. Включаем тактирование порта D --- */
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);

    /* --- 1. MODER: PD11–PD14 = Output (01b) --- */
    MODIFY_REG(GPIOD->MODER,
               GPIO_MODER_MODER11_Msk |
                   GPIO_MODER_MODER12_Msk |
                   GPIO_MODER_MODER13_Msk |
                   GPIO_MODER_MODER14_Msk,
               (0x1UL << GPIO_MODER_MODER11_Pos) |
                   (0x1UL << GPIO_MODER_MODER12_Pos) |
                   (0x1UL << GPIO_MODER_MODER13_Pos) |
                   (0x1UL << GPIO_MODER_MODER14_Pos));
    /* 01b = General purpose output mode */

    /* --- 2. OTYPER: Push-pull (0) --- */
    CLEAR_BIT(GPIOD->OTYPER,
              GPIO_OTYPER_OT_11 |
                  GPIO_OTYPER_OT_12 |
                  GPIO_OTYPER_OT_13 |
                  GPIO_OTYPER_OT_14);

    /* --- 3. OSPEEDR: High speed (11b) --- */
    MODIFY_REG(GPIOD->OSPEEDR,
               GPIO_OSPEEDR_OSPEED11_Msk |
                   GPIO_OSPEEDR_OSPEED12_Msk |
                   GPIO_OSPEEDR_OSPEED13_Msk |
                   GPIO_OSPEEDR_OSPEED14_Msk,
               (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos) |
                   (0x3UL << GPIO_OSPEEDR_OSPEED12_Pos) |
                   (0x3UL << GPIO_OSPEEDR_OSPEED13_Pos) |
                   (0x3UL << GPIO_OSPEEDR_OSPEED14_Pos));

    /* --- 4. PUPDR: No pull-up / pull-down (00b) --- */
    MODIFY_REG(GPIOD->PUPDR,
               GPIO_PUPDR_PUPD11_Msk |
                   GPIO_PUPDR_PUPD12_Msk |
                   GPIO_PUPDR_PUPD13_Msk |
                   GPIO_PUPDR_PUPD14_Msk,
               0U);

    /* --- 5. На старте: все INx = 0 (тормоз) --- */
    /* BSRR: BRx (верхние 16 бит) — сброс пина */
    SET_BIT(GPIOD->BSRR,
            GPIO_BSRR_BR_11 |
                GPIO_BSRR_BR_12 |
                GPIO_BSRR_BR_13 |
                GPIO_BSRR_BR_14);
}

// Настройка PE9, PE11 на TIM1_CH1, TIM1_CH2 (режим MODER как 10, alt.func.)
static void Motor_GPIO_PwmPins_Init(void)
{
    /* --- 0. Включаем тактирование порта E --- */
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);

    /* --- 1. MODER: Alternate function (10b) --- */
    MODIFY_REG(GPIOE->MODER,
               GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER11_Msk,
               (0x2UL << GPIO_MODER_MODER9_Pos) |
                   (0x2UL << GPIO_MODER_MODER11_Pos));

    /* --- 2. OTYPER: Push-pull (0) --- */
    CLEAR_BIT(GPIOE->OTYPER, GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_11);

    /* --- 3. OSPEEDR: High speed (11b) --- */
    MODIFY_REG(GPIOE->OSPEEDR,
               GPIO_OSPEEDR_OSPEED9_Msk | GPIO_OSPEEDR_OSPEED11_Msk,
               (0x3UL << GPIO_OSPEEDR_OSPEED9_Pos) |
                   (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos));

    /* --- 4. PUPDR: No pull-up / pull-down (00b) --- */
    MODIFY_REG(GPIOE->PUPDR,
               GPIO_PUPDR_PUPD9_Msk | GPIO_PUPDR_PUPD11_Msk,
               0U);

    /* --- 5. AFR[1]: Alternate function AF1 (TIM1) --- */
    MODIFY_REG(GPIOE->AFR[1],
               GPIO_AFRH_AFSEL9_Msk | GPIO_AFRH_AFSEL11_Msk,
               (0x1UL << GPIO_AFRH_AFSEL9_Pos) |
                   (0x1UL << GPIO_AFRH_AFSEL11_Pos));
}

/*
 * ПАРАМЕТРЫ ШИМ:
 *
 * f_TIM1  — частота тактирования таймера TIM1 (после APB2 и умножителя).
 * В учебном примере предположим:
 *     f_TIM1 = 168 МГц  (это зависит от настройки RCC! см. SystemClock)
 *
 * Хотим получить частоту PWM:
 *     f_PWM = 20 кГц
 *
 * Формула:
 *   f_PWM = f_TIM1 / ((PSC + 1) * (ARR + 1))
 *
 * Отсюда:
 *   (PSC + 1) * (ARR + 1) = f_TIM1 / f_PWM
 *   (PSC + 1) * (ARR + 1) = 168 000 000 / 20 000 = 8400
 *
 * Выбираем удобную пару:
 *   PSC + 1 = 42  => PSC = 83
 *   ARR + 1 = 100 => ARR = 99
 *
 * Тогда:
 *   f_PWM = 168 000 000 / (84 * 100) = 20 000 Гц (20 кГц)
 */

/*
 * Настройка TIM1 в режим PWM:
 *
 * 1) Останавливаем таймер (CR1.CEN = 0).
 * 2) Задаём PSC и ARR под нужную частоту (20 кГц).
 * 3) Настраиваем канал 1 и 2:
 *      - CCxS = 00 (канал как выход)
 *      - OCxM = 110 (PWM mode 1)
 *      - OCxPE = 1 (предзагрузка CCRx включена)
 * 4) Разрешаем выходы каналов в CCER (CC1E, CC2E).
 * 5) Устанавливаем начальную скважность (CCR1/CCR2 = 0).
 * 6) Включаем ARPE (CR1.ARPE = 1), чтобы ARR тоже был буферизирован.
 * 7) Включаем BDTR.MOE — главный выход (обязательно для TIM1).
 * 8) Делаем EGR.UG, чтобы применить настройки.
 * 9) Включаем таймер (CR1.CEN = 1).
 */
static void Motor_TIM1_Init(void)
{
    /* --- 1. Остановить таймер перед настройкой --- */
    CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);

    /* --- 2. Задать делитель (PSC) и период (ARR) --- */
    WRITE_REG(TIM1->PSC, MOTOR_TIM1_PSC); /* делитель: f_cnt = f_tim / (PSC+1) */
    WRITE_REG(TIM1->ARR, MOTOR_TIM1_ARR); /* период: CNT считает от 0 до ARR */

    /* --- 3. Канал 1: PWM mode 1, preload ---
     *
     * CCMR1, поле для CH1:
     *   CC1S[1:0] = 00 -> канал работает как выход
     *   OC1M[2:0] = 110 -> PWM mode 1
     *   OC1PE = 1      -> предзагрузка CCR1 (CCR1 в "теневом" регистре)
     */
    MODIFY_REG(TIM1->CCMR1,
               TIM_CCMR1_CC1S_Msk |
                   TIM_CCMR1_OC1M_Msk |
                   TIM_CCMR1_OC1PE_Msk,
               (0x0UL << TIM_CCMR1_CC1S_Pos) |     /* CC1S = 00: output */
                   (0x6UL << TIM_CCMR1_OC1M_Pos) | /* OC1M = 110: PWM mode 1 */
                   TIM_CCMR1_OC1PE);               /* включаем preload CCR1 */

    /* --- 4. Канал 2: PWM mode 1, preload ---
     *
     * Для CH2 те же поля, но CC2S, OC2M, OC2PE.
     */
    MODIFY_REG(TIM1->CCMR1,
               TIM_CCMR1_CC2S_Msk |
                   TIM_CCMR1_OC2M_Msk |
                   TIM_CCMR1_OC2PE_Msk,
               (0x0UL << TIM_CCMR1_CC2S_Pos) |     /* CC2S = 00: output */
                   (0x6UL << TIM_CCMR1_OC2M_Pos) | /* OC2M = 110: PWM mode 1 */
                   TIM_CCMR1_OC2PE);               /* preload CCR2 */

    /* --- 5. CCER: включаем выходы CH1 и CH2, полярность "активный высокий" ---
     *
     * CC1E = 1 -> канал 1 включён, сигнал идёт на пин PE9.
     * CC1P = 0 -> активный высокий (логическая "1" = высокий уровень).
     * Аналогично CC2E/CC2P для канала 2 (PE11).
     */
    MODIFY_REG(TIM1->CCER,
               TIM_CCER_CC1E_Msk | TIM_CCER_CC1P_Msk |
                   TIM_CCER_CC2E_Msk | TIM_CCER_CC2P_Msk,
               TIM_CCER_CC1E |     /* CH1 enable, polarity high */
                   TIM_CCER_CC2E); /* CH2 enable, polarity high */

    /* --- 6. Начальная скважность: 0% ---
     * CNT в начале периода всегда < CCRx? Нет, мы просто делаем CCRx=0,
     * значит "высокий" участок будет нулевой длины -> моторы стоят.
     */
    WRITE_REG(TIM1->CCR1, 0U); /* PWM = 0% для мотора A */
    WRITE_REG(TIM1->CCR2, 0U); /* PWM = 0% для мотора B */

    /* --- 7. Разрешаем буферизацию ARR (ARPE) ---
     * Теперь ARR тоже буферизуется, и его изменение не рвёт период.
     */
    SET_BIT(TIM1->CR1, TIM_CR1_ARPE);

    /* --- 8. BDTR: включаем главный выход (MOE) ---
     *
     * Для advanced-timer (TIM1/TIM8) ВСЕ PWM-выходы физически блокируются,
     * пока не установлен бит MOE (Main Output Enable).
     * Поэтому без этого шага CH1/CH2 на ногах не появятся.
     */
    SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);

    /* --- 9. Генерируем событие обновления (EGR.UG) ---
     *
     * Это "подхватывает" значения PSC, ARR, CCRx из теневых регистров.
     * После этого таймер готов к работе с обновлёнными параметрами.
     */
    SET_BIT(TIM1->EGR, TIM_EGR_UG);

    /* --- 10. Запускаем таймер (CR1.CEN = 1) --- */
    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
}

/* --- 5. Внутренняя функция: направление мотора --- */
/* dir > 0: вперёд, dir < 0: назад, dir = 0: оба INx = 0 (тормоз) */
static void Motor_SetDir(MotorId id, int8_t dir)
{
    switch (id)
    {
    case MOTOR_A:
        if (dir > 0)
        {
            /* Вперёд: IN1=0 (PD11=0), IN2=1 (PD12=1) */
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR_11);
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BS_12);
        }
        else if (dir < 0)
        {
            /* Назад: IN1=1, IN2=0 */
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BS_11);
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR_12);
        }
        else
        {
            /* Стоп (тормоз): IN1=0, IN2=0 */
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR_11 | GPIO_BSRR_BR_12);
        }
        break;

    case MOTOR_B:
        if (dir > 0)
        {
            /* Вперёд: IN3=0 (PD13=0), IN4=1 (PD14=1) */
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR_13);
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BS_14);
        }
        else if (dir < 0)
        {
            /* Назад: IN3=1, IN4=0 */
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BS_13);
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR_14);
        }
        else
        {
            /* Стоп: IN3=0, IN4=0 */
            SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR_13 | GPIO_BSRR_BR_14);
        }
        break;

    default:
        /* Неверный id — ничего не делаем */
        break;
    }
}

/* --- 6. Внутренняя функция: установка скважности (CCR) --- */
/* value: 0..MOTOR_PWM_MAX */
static void Motor_SetPwm(MotorId id, uint16_t value)
{
    if (value > MOTOR_PWM_MAX)
    {
        value = MOTOR_PWM_MAX;
    }

    if (id == MOTOR_A)
    {
        WRITE_REG(TIM1->CCR1, value);
    }
    else if (id == MOTOR_B)
    {
        WRITE_REG(TIM1->CCR2, value);
    }
}

/* --- 7. Публичные функции --- */

void Motor_Init(void)
{
    Motor_ClockInit();
    Motor_GPIO_DirPins_Init();
    Motor_GPIO_PwmPins_Init();
    Motor_TIM1_Init();

    /* На всякий случай ещё раз убедимся, что моторы стоят */
    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
}

void Motor_SetSpeed(MotorId id, int16_t speed)
{
    if (id != MOTOR_A && id != MOTOR_B)
    {
        return;
    }

    if (speed == 0)
    {
        /* Полный стоп: PWM=0 и "тормоз" по INx */
        Motor_SetPwm(id, 0);
        Motor_SetDir(id, 0);
        return;
    }

    int8_t dir;
    uint16_t mag;

    if (speed > 0)
    {
        dir = +1;
        mag = (uint16_t)speed;
    }
    else
    {
        dir = -1;
        mag = (uint16_t)(-speed);
    }

    if (mag > MOTOR_PWM_MAX)
    {
        mag = MOTOR_PWM_MAX;
    }

    Motor_SetDir(id, dir);
    Motor_SetPwm(id, mag);
}

void Motor_Stop(MotorId id)
{
    Motor_SetSpeed(id, 0);
}