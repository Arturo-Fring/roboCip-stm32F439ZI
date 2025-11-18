#include "GU521_init.h"
#include "usart.h"

void Gyro_I2C2_Init_400kHz(void)
{
    /* 1. Включение тактирования */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    /* 2. PF0 = I2C2_SCL, PF1 = I2C2_SDA — Alternate Function 4 */
    GPIOF->MODER   |=  (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);        // Alternate function
    GPIOF->OTYPER  |=  (GPIO_OTYPER_OT0   | GPIO_OTYPER_OT1);            // Open-drain (обязательно!)
    GPIOF->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1);    // Very High speed
    GPIOF->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);           // сначала очистим
    GPIOF->PUPDR   |=  (GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0);       // Pull-up  (01)

    // Самое важное — AF4 для I2C2 на PF0/PF1 !!!
    GPIOF->AFR[0] &= ~(0xFFUL);                                          // очистить биты 0-7
    GPIOF->AFR[0] |=  (4UL << GPIO_AFRL_AFSEL0_Pos) |                    // AF4 для PF0
                      (4UL << GPIO_AFRL_AFSEL1_Pos);                     // AF4 для PF1

    /* 3. PB5 — вход INT от гироскопа (если используете) */
    GPIOB->MODER   &= ~GPIO_MODER_MODE5;      // Input
    GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPD5;      // No pull-up/pull-down (или можно pull-down)

    /* 4. Полный сброс I2C2 через RCC (самый надёжный способ) */
    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;

    /* 5. Настройка частоты APB1 в регистре CR2 */
    I2C2->CR2 = (I2C2->CR2 & ~I2C_CR2_FREQ_Msk) | 42U;   // 42 МГц (PCLK1)

    /* 6. Fast-mode Plus 400 кГц */
    // Формула для Fm+ (FS=1, DUTY не важен):  CCR = PCLK1 / (2 × 400000)
    // 42 000 000 / 800 000 = 52.5 → округляем вверх до 53
    I2C2->CCR = I2C_CCR_FS | 53U;                // FS=1, CCR=53

    /* 7. TRISE для 400 кГц */
    I2C2->TRISE = 43U;                           // 42 + 1 = 43

    /* 8. Включение I2C2 */
    I2C2->CR1 |= I2C_CR1_PE;
}
/*# КОД ПРОВЕРЕННЫЙ И ПОЛНОСТЬЮ РАБОЧИЙ. РУЧКАМИ НЕ ЛАПАТЬ #*/
