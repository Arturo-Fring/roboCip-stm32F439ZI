#include "../Inc/init.h"
#include "init.h"

void ITR_Init(void)
{
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    /*//Это шутка, которая позволяет настроить мультиплексоры. Разного рода
    регистры: SYSCFG. Объединение нулевой линии (нулевые порты контроллера)
    EXTI1 - первая линия и т.д.
    Мы подключаем кнопку на PC13, на уроке PC12. Настраиваем для 12 линии
    SYSCFG разделили на 2 области, 16 битов старш другому, 16 битов младшему именно EXTI
    Там разделение на 4 линии. Во второй от 4 до 7, в третьем от 8 до 11, в четвёртом от 12 до ...
    Мы будем брать PC12, следовательно значение для PC12.
    APB2ENR (записали 1 для тактирвоания)
    */
    SET_BIT(SYSCFG->EXTICR[3], SYSCFG_EXTICR4_EXTI13_PC);
    /*то что на 13pc, будет выходить на контроллер прерываний*/

    /*Пропишем сами регистры EXTI, 12.3*/
    // Не хотим маскировать
    SET_BIT(EXTI->IMR, EXTI_IMR_MR13);
    // EMR пропускаем

    // Rising cl. rising trigger selection reg. по фронту, з. на RT 1
    SET_BIT(EXTI->RTSR, EXTI_RTSR_TR13);

    // Теперь спад.
    CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_TR13);

    // Нужно настроить NVIC
    // см. programming manual 4.3,
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(EXTI15_10_IRQn); // вкючаем по вектору. Все вектора в ассемблерном файле  (ext interrupts)
}

void UART3_GPIO_Init(void)
{
    /* 1. Включаем тактирование порта D */
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
    /* 2. Настраиваем PD8 и PD9 в режим альтернативной функции (AF) */

    // MODER8[1:0] = 10 (AF)
    // MODER9[1:0] = 10 (AF)
    MODIFY_REG(GPIOD->MODER,
               GPIO_MODER_MODER8 | GPIO_MODER_MODER9,
               (GPIO_MODER_MODER8_1) | (GPIO_MODER_MODER9_1));

    /* 3. Скорость портов — высокая, чтобы фронты были покруче */
    // OSPEED8[1:0] = 11 (Very High speed)
    // OSPEED9[1:0] = 11
    MODIFY_REG(GPIOD->OSPEEDR,
               GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9,
               (GPIO_OSPEEDR_OSPEED8) | (GPIO_OSPEEDR_OSPEED9));

    /* 4. Подтяжки:
     *    - TX (PD8) можно оставить без подтяжки (floating).
     *    - RX (PD9) часто делают pull-up.
     */
    MODIFY_REG(GPIOD->PUPDR,
               GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9,
               (0U << GPIO_PUPDR_PUPD8_Pos) | // PD8: no pull
                   (GPIO_PUPDR_PUPD9_0));     // PD9: pull-up (01) pull-up, чтобы вход не «висел в воздухе».

    /* 5. Альтернативная функция AF7 для USART3 на PD8/PD9
     *    PD8 -> AFRH[3:0]
     *    PD9 -> AFRH[7:4]
     *    AF7 = 0b0111
     *
     * AFR[0] — пины 0..7
       AFR[1] — пины 8..15 //На каждый пин — 4 бита (AF0..AF15)
     */
    MODIFY_REG(GPIOD->AFR[1],
               GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9,
               (7U << GPIO_AFRH_AFSEL8_Pos) |
                   (7U << GPIO_AFRH_AFSEL9_Pos));
}

void UART3_Init(void)
{
    /* 1. Включаем тактирование USART3 на шине APB1 */
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN); // 1 — USART3 получает тактирование от PCLK1 (42 МГц в нашей схеме).

    /* 2. Настраиваем пины PD8/PD9 под AF7 USART3 */
    UART3_GPIO_Init();
    
    /* 3. Перед конфигурированием — выключаем USART3 (UE = 0) */
    CLEAR_BIT(USART3->CR1, USART_CR1_UE); // USART vkl/vikl

    /* 4. Настройка формата кадра:
     *    - 8 бит данных: M = 0
     *    - без чётности: PCE = 0
     *    - 1 стоп-бит: STOP[1:0] = 00
     */
    CLEAR_BIT(USART3->CR1, USART_CR1_M);         // 8 data bits
    CLEAR_BIT(USART3->CR1, USART_CR1_PCE);       // no parity //бит-чётности не используется
    MODIFY_REG(USART3->CR2, USART_CR2_STOP, 0U); // 1 stop bit итого схема 8N1 (8 data, no parity, 1 stop)

    /* 5. Oversampling = 16 (OVER8 = 0, по умолчанию и так 0) */
    CLEAR_BIT(USART3->CR1, USART_CR1_OVER8);

    /* 6. Скорость: 115200 при PCLK1 = 42 МГц -> BRR = 0x016D */
    WRITE_REG(USART3->BRR, 0x016D);

    /* 7. Включаем передатчик и приёмник */
    SET_BIT(USART3->CR1, USART_CR1_TE); // Transmitter enable
    SET_BIT(USART3->CR1, USART_CR1_RE); // Receiver enable

    /* 8. Включаем USART3 (UE = 1) */
    SET_BIT(USART3->CR1, USART_CR1_UE);
}

void UART3_SendChar(char c)
{
    // Ждём, пока буфер передачи освободится (TXE = 1)
    while ((READ_BIT(USART3->SR, USART_SR_TXE) == 0U))
    {
        // wait
    }

    // Пишем байт в регистр данных
    WRITE_REG(USART3->DR, (uint16_t)c);
}

void UART3_SendString(const char *s)
{
    while (*s)
    {
        UART3_SendChar(*s++);
    }

    // Ждём завершения последней передачи (TC = 1)
    while ((READ_BIT(USART3->SR, USART_SR_TC) == 0U))
    {
        // wait
    }
}

char UART3_ReceiveChar(void)
{
    // Ждём, пока придет байт (RXNE = 1)
    while ((READ_BIT(USART3->SR, USART_SR_RXNE) == 0U))
    {
        // wait
    }

    // Читаем DR — важно маскировать до 8 бит
    return (char)(READ_REG(USART3->DR) & 0xFF);
}

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

// #### //
 