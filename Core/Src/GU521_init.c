#include "GU521_init.h"

void Gyro_init(void)
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN_Msk | RCC_AHB1ENR_GPIOBEN_Msk); // Включили тактирования порта F, шины APB1 для I2C2, и порта B
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN_Msk);

    CLEAR_REG(GPIOF->AFR[0]);
    SET_BIT(GPIOF->AFR[0], GPIO_AFRL_AFRL0_2 | GPIO_AFRL_AFRL1_2); // 

    SET_BIT(GPIOF->MODER, GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1); // Настроили пины PF0 (SDA) и PF1 (SCL) на альтернативный режим
    SET_BIT(GPIOF->OTYPER, GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1); // Режим open-drain для PF0 и PF1
    SET_BIT(GPIOF->OSPEEDR, GPIO_OSPEEDR_OSPEED0_Msk | GPIO_OSPEEDR_OSPEED1_Msk); // Настроили работу выводов PF0 и PF1 на быстрый
    CLEAR_REG(GPIOF->PUPDR); // Все без пулап или пулдаун

    CLEAR_BIT(GPIOB->MODER, GPIO_MODER_MODE5_Msk); // Пин PB5 на вход. Отвечает в гироскопе за INT
    CLEAR_BIT(GPIOB->PUPDR, GPIO_PUPDR_PUPD5_Msk); // Пин PB5 без пулап и пулдаун
    CLEAR_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED5_Msk); // Входу не нужна высокая скорость
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT5_Msk); // Пуш-пул 

    // Настройка USART2

    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Msk | RCC_AHB1ENR_GPIODEN_Msk); // тактирование портов A и D

    SET_BIT(GPIOD->MODER, GPIO_MODER_MODE5_1);
    SET_BIT(GPIOA->MODER, GPIO_MODER_MODE3_1);

    CLEAR_BIT(GPIOD->AFR[0], GPIO_AFRL_AFRL5_0 | GPIO_AFRL_AFRL5_1 | GPIO_AFRL_AFRL5_2 | GPIO_AFRL_AFRL5_3 );
    CLEAR_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL3_0 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_2 | GPIO_AFRL_AFRL3_3 );

    SET_BIT(GPIOA->AFR[0], GPIO_AFRL_AFRL3_2 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_0); // Настроили на RX
    SET_BIT(GPIOD->AFR[0], GPIO_AFRL_AFRL5_2 | GPIO_AFRL_AFRL5_1 | GPIO_AFRL_AFRL5_0); // Настроили на TX

    MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED3_Msk, 0x1);
    MODIFY_REG(GPIOD->OSPEEDR, GPIO_OSPEEDR_OSPEED5_Msk, 0x1);

    CLEAR_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT3_Msk);
    CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT5_Msk);

    CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD3_Msk);
    CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPD5_Msk);

    /**/

    //BRR = 42000000/115200; = 0x16C // Скорость передачи сигнала МГц/бод

    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN_Msk); // Подали тактирование на шину APB1
    CLEAR_BIT(USART2->CR1, USART_CR1_UE_Msk);
    WRITE_REG(USART2->BRR, 0x16C);
    USART2->CR1 = USART_CR1_TE_Msk | USART_CR1_RE_Msk | 0; // Запустили RX/TX. Остальные биты без изменений

    CLEAR_REG(USART2->CR2);
    CLEAR_REG(USART2->CR3);

    SET_BIT(USART2->CR1, USART_CR1_UE);

    // Настройка I2C2

    SET_BIT(I2C2->CR1, I2C_CR1_SWRST);  // SWRST = 1

    CLEAR_BIT(I2C2->CR1, I2C_CR1_SWRST); // SWRST = 0

    MODIFY_REG(I2C2->CR2, I2C_CR2_FREQ_Msk, 42 << I2C_CR2_FREQ_Pos); // Настроили частоту на 42 МГц

    uint32_t i2c_freq = 42000000UL;
    uint32_t i2c_speed = 400000UL;
    uint32_t ccr_value = (i2c_freq / (3 * i2c_speed));

    MODIFY_REG(I2C2->CCR, I2C_CCR_CCR_Msk | I2C_CCR_DUTY | I2C_CCR_FS, 
           ccr_value << I2C_CCR_CCR_Pos | I2C_CCR_FS | I2C_CCR_DUTY);

    MODIFY_REG(I2C2->TRISE, I2C_TRISE_TRISE_Msk, ((i2c_freq/1000000) + 1)); // Ограничивает максимальную длительность цикла ожидания. Короче чтобы I2C работал норм
    SET_BIT(I2C2->CR1, I2C_CR1_PE_Msk); // Включили шину I2C2 на частоте 42МГц

    //
}