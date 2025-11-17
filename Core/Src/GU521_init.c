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
    // USART2 настройка
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Msk | RCC_AHB1ENR_GPIODEN_Msk);

    // MODER - альтернативная функция
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER3_Msk, 2 << GPIO_MODER_MODER3_Pos);
    MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODER5_Msk, 2 << GPIO_MODER_MODER5_Pos);

    // AFR - AF7 
    MODIFY_REG(GPIOA->AFR[0], 0xF << 12, 7 << 12);  // PA3: биты 12-15
    MODIFY_REG(GPIOD->AFR[0], 0xF << 20, 7 << 20);  // PD5: биты 20-

    // USART
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN_Msk);
    CLEAR_BIT(USART2->CR1, USART_CR1_UE_Msk);
    WRITE_REG(USART2->BRR, 0x16D);
    WRITE_REG(USART2->CR2, 0);
    WRITE_REG(USART2->CR3, 0);
    SET_BIT(USART2->CR1, USART_CR1_TE_Msk | USART_CR1_RE_Msk | USART_CR1_UE_Msk);

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

void Test_USART_Menu(void)
{
    USART2_SendString("\r\n=== USART Test Menu ===\r\n");
    USART2_SendString("1. Send test string\r\n");
    USART2_SendString("2. Echo test\r\n");
    USART2_SendString("3. Check communication\r\n");
    USART2_SendString("Select option (1-3): ");
}

void USART2_Init(void)
{
    // Отключение USART перед настройкой
    USART2->CR1 &= ~USART_CR1_UE;
    
    // Настройка скорости (Baud Rate)
    // Для F_APB1 = 36MHz, Baud = 115200
    USART2->BRR = (39 << 4) | (1 << 0); // 39.0625 = 36000000/115200
    
    // Настройка контроля:
    // 8 бит данных, 1 стоп-бит, без контроля четности
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE; // Включение TX и RX
    USART2->CR2 = 0;                           // 1 стоп-бит
    USART2->CR3 = 0;                           // Без аппаратного контроля
    
    // Включение USART
    USART2->CR1 |= USART_CR1_UE;
}