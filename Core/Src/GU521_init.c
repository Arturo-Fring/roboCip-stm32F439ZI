#include "GU521_init.h"
#include "usart.h"

void GY521_I2C1_Init(void)
{
    /* 1. Тактирование GPIOB и I2C1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    
    /* 2. PB8 (SCL), PB9 (SDA) → AF4, open-drain, pull-up */

    // MODER: AF
    GPIOB->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk);
    GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1); // 10: AF

    // OTYPER: open-drain
    GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9;

    // OSPEEDR: high
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_Msk | GPIO_OSPEEDR_OSPEED9_Msk;

    // PUPDR: pull-up
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk | GPIO_PUPDR_PUPD9_Msk);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0);

    // AF4
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8_Msk | GPIO_AFRH_AFSEL9_Msk);
    GPIOB->AFR[1] |= (4U << GPIO_AFRH_AFSEL8_Pos) |
                     (4U << GPIO_AFRH_AFSEL9_Pos);

    /* 3. Полный сброс и конфиг I2C1 */

    // Сначала полностью выключим
    I2C1->CR1 = 0;
    I2C1->CR2 = 0;
    I2C1->OAR1 = 0;
    I2C1->OAR2 = 0;
    I2C1->CCR = 0;
    I2C1->TRISE = 0;

    // Частота APB1 = 42 МГц
    I2C1->CR2 = 42U;

    // Standard Mode 100 кГц
    uint32_t i2c_freq = 42000000UL;
    uint32_t i2c_speed = 100000UL;
    uint32_t ccr_value = i2c_freq / (2U * i2c_speed);
    if (ccr_value < 4U)
        ccr_value = 4U;
    if (ccr_value > 0xFFFU)
        ccr_value = 0xFFFU;

    I2C1->CCR = ccr_value;  // FS=0 (standard)
    I2C1->TRISE = 42U + 1U; // стандартный режим

    // Включаем ACK заранее
    I2C1->CR1 |= I2C_CR1_ACK;

    // И наконец включаем I2C
    I2C1->CR1 |= I2C_CR1_PE;

    // Для контроля выведем CR1/CR2
    USART_Print("I2C1 CR1 = 0x");
    USART_PrintHex(I2C1->CR1);
    USART_Print(" CR2 = 0x");
    USART_PrintlnHex(I2C1->CR2);
}
