#include "../Inc/init.h"
#include "init.h"

void PB7_Init(void)
{
    /* 1. Включаем тактирование порта B */
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

    /* 2. Настраиваем PB7 как обычный выход (General Purpose Output) */

    // MODER7[1:0] = 01 (Output)
    MODIFY_REG(GPIOB->MODER,
               GPIO_MODER_MODER7,
               GPIO_MODER_MODER7_0); // 01b

    // Тип выхода: Push-pull (OT7 = 0)
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT7);

    // Скорость: medium (10) или high — не критично для светодиода
    MODIFY_REG(GPIOB->OSPEEDR,
               GPIO_OSPEEDR_OSPEED7,
               GPIO_OSPEEDR_OSPEED7_0); // 01b - medium

    // Подтяжки: нет (PUPDR7 = 00)
    MODIFY_REG(GPIOB->PUPDR,
               GPIO_PUPDR_PUPD7,
               0U << GPIO_PUPDR_PUPD7_Pos);

    /* 3. Погасим светодиод по умолчанию (предположим, что "1" = включен, "0" = выключен) */
    // Сброс бита через BSRR (нижние 16 бит - reset)
    GPIOB->BSRR = (uint32_t)(1U << (7 + 16)); // сбросить PB7
}

void SysTick_Init_1ms(void)
{
    /* Останавливаем SysTick на время настройки */
    CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);
    /* Счётчик перезагрузки:
       частота / 1000 - 1  → период 1 мс */
    uint32_t reload = 168000000U / 1000U - 1U;
    // CLEAR_REG(SysTick->)
    WRITE_REG(SysTick->LOAD, reload); // значение перезагрузки
    WRITE_REG(SysTick->VAL, 0U);      // сбрасываем текущий счётчик

    /* Источник — системный такт (SYSCLK), включаем прерывания и сам таймер */
    CLEAR_BIT(SysTick->CTRL,
              SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk |
                  SysTick_CTRL_ENABLE_Msk);

    SET_BIT(SysTick->CTRL,
            SysTick_CTRL_CLKSOURCE_Msk |   // тактировать от ядра (SYSCLK)
                SysTick_CTRL_TICKINT_Msk | // разрешить прерывания
                SysTick_CTRL_ENABLE_Msk);  // включить счётчик
}
// #### //
volatile uint32_t g_msTicks = 0U;
