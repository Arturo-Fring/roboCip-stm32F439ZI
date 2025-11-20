#include "motor.h"

/******************************************************************************
 *                           MOTOR DRIVER — motor.c
 *
 * Управляет двумя моторами постоянного тока через драйвер H-моста.
 *
 * Каждый мотор имеет:
 *   - ДВЕ линии направления (INx)
 *   - Один PWM-сигнал от TIM1 (канал 1 или 2)
 *
 * Конструкция подключения (важно для понимания):
 *
 *   MOTOR_A (левый):
 *      IN1 = PE2
 *      IN2 = PD11
 *      PWM = TIM1_CH1 (PE9)
 *
 *   MOTOR_B (правый):
 *      IN3 = PD12
 *      IN4 = PD13
 *      PWM = TIM1_CH2 (PE11)
 *
 * PWM генерируется таймером TIM1 (частота 20 кГц).
 * Направление задаётся комбинациями INx:
 *
 *      Вперёд:   IN1=0, IN2=1
 *      Назад:    IN1=1, IN2=0
 *      Стоп:     IN1=0, IN2=0   (электротормоз)
 *
 *****************************************************************************/

/* --- ЛОКАЛЬНЫЕ ПРОТОТИПЫ --- */
static void Motor_ClockInit(void);
static void Motor_GPIO_DirPins_Init(void);
static void Motor_GPIO_PwmPins_Init(void);
static void Motor_TIM1_Init(void);

static void Motor_SetDir(MotorId id, int8_t dir);
static void Motor_SetPwm(MotorId id, uint16_t value);

/******************************************************************************
 *                           Motor_ClockInit()
 *
 * Включает тактирование:
 *   - GPIO портов: D и E
 *   - таймера TIM1 (через APB2)
 *
 * Без включения тактирования любые попытки записи в регистры GPIO/TIM1
 * будут либо проигнорированы, либо вызовут ошибку.
 *****************************************************************************/
static void Motor_ClockInit(void)
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);

    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
}

/******************************************************************************
 *                  Motor_GPIO_DirPins_Init()
 *
 * Настраивает линии направления IN1–IN4 как цифровые выходы.
 *
 * Пины:
 *     IN1 = PE2
 *     IN2 = PD11
 *     IN3 = PD12
 *     IN4 = PD13
 *
 * Все пины настраиваются как:
 *      MODE  = output (01)
 *      OTYPER = push-pull
 *      OSPEED = high speed (для лучшего отклика)
 *      PUPDR = no pull
 *
 * Затем выставляется начальное состояние всех INx = 0 (тормоз).
 *****************************************************************************/
static void Motor_GPIO_DirPins_Init(void)
{
    /* Тактирование портов */
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);

    /************** IN1 = PE2 **************/
    MODIFY_REG(GPIOE->MODER, GPIO_MODER_MODER2_Msk,
               (0x1UL << GPIO_MODER_MODER2_Pos)); // output mode

    CLEAR_BIT(GPIOE->OTYPER, GPIO_OTYPER_OT_2); // push-pull

    MODIFY_REG(GPIOE->OSPEEDR, GPIO_OSPEEDR_OSPEED2_Msk,
               (0x3UL << GPIO_OSPEEDR_OSPEED2_Pos)); // high speed

    MODIFY_REG(GPIOE->PUPDR, GPIO_PUPDR_PUPD2_Msk, 0U);

    /************** IN2=PD11, IN3=PD12, IN4=PD13 **************/

    MODIFY_REG(GPIOD->MODER,
               GPIO_MODER_MODER11_Msk |
                   GPIO_MODER_MODER12_Msk |
                   GPIO_MODER_MODER13_Msk,
               (0x1UL << GPIO_MODER_MODER11_Pos) |
                   (0x1UL << GPIO_MODER_MODER12_Pos) |
                   (0x1UL << GPIO_MODER_MODER13_Pos));

    CLEAR_BIT(GPIOD->OTYPER,
              GPIO_OTYPER_OT_11 |
                  GPIO_OTYPER_OT_12 |
                  GPIO_OTYPER_OT_13);

    MODIFY_REG(GPIOD->OSPEEDR,
               GPIO_OSPEEDR_OSPEED11_Msk |
                   GPIO_OSPEEDR_OSPEED12_Msk |
                   GPIO_OSPEEDR_OSPEED13_Msk,
               (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos) |
                   (0x3UL << GPIO_OSPEEDR_OSPEED12_Pos) |
                   (0x3UL << GPIO_OSPEEDR_OSPEED13_Pos));

    MODIFY_REG(GPIOD->PUPDR,
               GPIO_PUPDR_PUPD11_Msk |
                   GPIO_PUPDR_PUPD12_Msk |
                   GPIO_PUPDR_PUPD13_Msk,
               0U);

    /* Начальное состояние — стоп (все INx = 0) */
    SET_BIT(GPIOE->BSRR, GPIO_BSRR_BR_2);
    SET_BIT(GPIOD->BSRR,
            GPIO_BSRR_BR_11 |
                GPIO_BSRR_BR_12 |
                GPIO_BSRR_BR_13);
}

/******************************************************************************
 *                    Motor_GPIO_PwmPins_Init()
 *
 * Настройка PE9 (CH1) и PE11 (CH2) в режим PWM:
 *    - MODER = Alternate function (10b)
 *    - AF = AF1  (TIM1)
 *    - High speed
 *
 * TIM1 управляет этими двумя пинами в режиме PWM.
 *****************************************************************************/
static void Motor_GPIO_PwmPins_Init(void)
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);

    MODIFY_REG(GPIOE->MODER,
               GPIO_MODER_MODER9_Msk |
                   GPIO_MODER_MODER11_Msk,
               (0x2UL << GPIO_MODER_MODER9_Pos) |
                   (0x2UL << GPIO_MODER_MODER11_Pos)); // AF

    CLEAR_BIT(GPIOE->OTYPER, GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_11);

    MODIFY_REG(GPIOE->OSPEEDR,
               GPIO_OSPEEDR_OSPEED9_Msk |
                   GPIO_OSPEEDR_OSPEED11_Msk,
               (0x3UL << GPIO_OSPEEDR_OSPEED9_Pos) |
                   (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos)); // high

    MODIFY_REG(GPIOE->PUPDR,
               GPIO_PUPDR_PUPD9_Msk | GPIO_PUPDR_PUPD11_Msk,
               0U);

    MODIFY_REG(GPIOE->AFR[1],
               GPIO_AFRH_AFSEL9_Msk |
                   GPIO_AFRH_AFSEL11_Msk,
               (0x1UL << GPIO_AFRH_AFSEL9_Pos) |
                   (0x1UL << GPIO_AFRH_AFSEL11_Pos)); // AF1=TIM1
}

/******************************************************************************
 *                           Motor_TIM1_Init()
 *
 * Полная настройка TIM1 на генерацию PWM 20 кГц на CH1 и CH2.
 *
 * Параметры PWM:
 *     PSC = 83   (делит 168 МГц → 2 МГц)
 *     ARR = 99   (2 МГц / 100 → 20 кГц)
 *
 * Каналы настроены:
 *     - PWM mode 1 (OCxM = 110)
 *     - Preload CCR включён (OCxPE = 1)
 *     - Выходы CH1/CH2 разрешены в CCER
 *     - MOE = 1 (обязательно для TIM1)
 *****************************************************************************/
static void Motor_TIM1_Init(void)
{
    CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN); // стоп

    WRITE_REG(TIM1->PSC, MOTOR_TIM1_PSC);
    WRITE_REG(TIM1->ARR, MOTOR_TIM1_ARR);

    /*** Канал 1 (Motor A) ***/
    MODIFY_REG(TIM1->CCMR1,
               TIM_CCMR1_CC1S_Msk |
                   TIM_CCMR1_OC1M_Msk |
                   TIM_CCMR1_OC1PE_Msk,
               (0x0 << TIM_CCMR1_CC1S_Pos) |
                   (0x6 << TIM_CCMR1_OC1M_Pos) |
                   TIM_CCMR1_OC1PE);

    /*** Канал 2 (Motor B) ***/
    MODIFY_REG(TIM1->CCMR1,
               TIM_CCMR1_CC2S_Msk |
                   TIM_CCMR1_OC2M_Msk |
                   TIM_CCMR1_OC2PE_Msk,
               (0x0 << TIM_CCMR1_CC2S_Pos) |
                   (0x6 << TIM_CCMR1_OC2M_Pos) |
                   TIM_CCMR1_OC2PE);

    MODIFY_REG(TIM1->CCER,
               TIM_CCER_CC1E_Msk |
                   TIM_CCER_CC2E_Msk,
               TIM_CCER_CC1E | TIM_CCER_CC2E);

    WRITE_REG(TIM1->CCR1, 0);
    WRITE_REG(TIM1->CCR2, 0);

    SET_BIT(TIM1->CR1, TIM_CR1_ARPE);
    SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    SET_BIT(TIM1->EGR, TIM_EGR_UG);
    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
}

/******************************************************************************
 *                      Motor_SetDir() — направление
 *
 * Устанавливает логические уровни на INx в зависимости от dir:
 *
 *      dir > 0 → вперёд
 *      dir < 0 → назад
 *      dir = 0 → стоп (IN1=0, IN2=0)
 *
 * Реализация через GPIOx->BSRR:
 *   - BS_  → установить 1
 *   - BR_  → установить 0
 *
 *****************************************************************************/
static void Motor_SetDir(MotorId id, int8_t dir)
{
    switch (id)
    {
    case MOTOR_A:
        if (dir > 0) // вперёд
        {
            GPIOE->BSRR = GPIO_BSRR_BR_2;  // IN1 = 0
            GPIOD->BSRR = GPIO_BSRR_BS_11; // IN2 = 1
        }
        else if (dir < 0) // назад
        {
            GPIOE->BSRR = GPIO_BSRR_BS_2;  // IN1 = 1
            GPIOD->BSRR = GPIO_BSRR_BR_11; // IN2 = 0
        }
        else // стоп
        {
            GPIOE->BSRR = GPIO_BSRR_BR_2;
            GPIOD->BSRR = GPIO_BSRR_BR_11;
        }
        break;

    case MOTOR_B:
        if (dir > 0)
        {
            GPIOD->BSRR = GPIO_BSRR_BR_12;
            GPIOD->BSRR = GPIO_BSRR_BS_13;
        }
        else if (dir < 0)
        {
            GPIOD->BSRR = GPIO_BSRR_BS_12;
            GPIOD->BSRR = GPIO_BSRR_BR_13;
        }
        else
        {
            GPIOD->BSRR = GPIO_BSRR_BR_12 |
                          GPIO_BSRR_BR_13;
        }
        break;
    }
}

/******************************************************************************
 *                       Motor_SetPwm() — скважность PWM
 *
 * Записывает значение value в соответствующий регистр CCR:
 *
 *      MOTOR_A → CCR1
 *      MOTOR_B → CCR2
 *
 * value обязан быть в пределах 0..MOTOR_PWM_MAX.
 *****************************************************************************/
static void Motor_SetPwm(MotorId id, uint16_t value)
{
    if (value > MOTOR_PWM_MAX)
        value = MOTOR_PWM_MAX;

    if (id == MOTOR_A)
        WRITE_REG(TIM1->CCR1, value);
    else
        WRITE_REG(TIM1->CCR2, value);
}

/******************************************************************************
 *                                 Motor_Init()
 *
 * Полная инициализация драйвера:
 *   1. Включить тактирование TIM1, GPIO
 *   2. Настроить IN1-In4 как выходы
 *   3. Настроить PWM-пины (PE9/PE11) под TIM1 AF1
 *   4. Настроить TIM1 под PWM 20 кГц
 *   5. Остановить оба мотора
 *****************************************************************************/
void Motor_Init(void)
{
    Motor_ClockInit();
    Motor_GPIO_DirPins_Init();
    Motor_GPIO_PwmPins_Init();
    Motor_TIM1_Init();

    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
}

/******************************************************************************
 *                        Motor_SetSpeed(id, speed)
 *
 * Универсальное управление мотором:
 *
 *      speed > 0 → вперёд
 *      speed < 0 → назад
 *      speed = 0 → стоп
 *
 * По модулю |speed| берётся PWM-скважность.
 * НАПРАВЛЕНИЕ задаётся INx.
 *****************************************************************************/
void Motor_SetSpeed(MotorId id, int16_t speed)
{
    if (id != MOTOR_A && id != MOTOR_B)
        return;

    if (speed == 0)
    {
        Motor_SetPwm(id, 0);
        Motor_SetDir(id, 0);
        return;
    }

    int8_t dir = (speed > 0) ? +1 : -1;
    uint16_t mag = (speed > 0) ? speed : -speed;

    if (mag > MOTOR_PWM_MAX)
        mag = MOTOR_PWM_MAX;

    Motor_SetDir(id, dir);
    Motor_SetPwm(id, mag);
}

/******************************************************************************
 *                             Motor_Stop()
 *
 * Полный стоп → PWM=0 + INx=0 + электротормоз.
 *****************************************************************************/
void Motor_Stop(MotorId id)
{
    Motor_SetSpeed(id, 0);
}
