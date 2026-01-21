#include "../Inc/init.h"
#include "init.h"

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
