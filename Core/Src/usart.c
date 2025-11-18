#include "usart.h"
#include <stdio.h> // для sprintf (если не нужен float/инты через sprintf — можно убрать)

/* ===== Локальная функция: инициализация GPIO под USART2 =====
 * TX = PD5 (AF7)
 * RX = PA3 (AF7)
 */
// usart.c — версия под USART3 PD8/PD9

static void USART3_GPIO_Init(void)
{
    // Включаем порт D
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN_Msk;

    // PD8, PD9 -> AF7 (USART3)
    GPIOD->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
    GPIOD->MODER |= (2U << GPIO_MODER_MODER8_Pos) | (2U << GPIO_MODER_MODER9_Pos);

    // AFRH: PD8 = AF7, PD9 = AF7
    GPIOD->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOD->AFR[1] |= (7U << ((8 - 8) * 4)) | (7U << ((9 - 8) * 4));

    // TX: push-pull, fast; RX: с pull-up
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT8_Msk | GPIO_OTYPER_OT9_Msk);
    GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_Msk | GPIO_OSPEEDR_OSPEED9_Msk;

    GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD8_Msk | GPIO_PUPDR_PUPD9_Msk);
    GPIOD->PUPDR |= (1U << GPIO_PUPDR_PUPD9_Pos); // pull-up на RX (PD9), TX можно без
}

void USART3_Init(uint32_t baudrate)
{
    // Тактирование USART3
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN_Msk;

    USART3_GPIO_Init();

    USART3->CR1 &= ~USART_CR1_UE;

    uint32_t brr = (USART_PCLK1_HZ + baudrate / 2U) / baudrate;
    USART3->BRR = brr;

    USART3->CR1 = USART_CR1_TE | USART_CR1_RE; // 8N1
    USART3->CR2 = 0;
    USART3->CR3 = 0;

    USART3->CR1 |= USART_CR1_UE;
}

void USART_WriteChar(char c)
{
    while ((USART3->SR & USART_SR_TXE) == 0)
    {
    }
    USART3->DR = (uint16_t)c;
}

void USART_WriteString(const char *s)
{
    while (*s)
        USART_WriteChar(*s++);
}

/* ===== Arduino-style Print / Println ===== */

void USART_Print(const char *s)
{
    USART_WriteString(s);
}

void USART_Println(const char *s)
{
    USART_WriteString(s);
    USART_WriteString("\r\n");
}

/* --- Печать целых чисел --- */

void USART_PrintInt(int32_t value)
{
    char buf[32];
    sprintf(buf, "%ld", (long)value);
    USART_WriteString(buf);
}

void USART_PrintlnInt(int32_t value)
{
    USART_PrintInt(value);
    USART_WriteString("\r\n");
}

/* --- Печать hex (удобно для регистра/байта датчика) --- */

void USART_PrintHex(uint32_t value)
{
    char buf[16];
    sprintf(buf, "0x%08lX", (unsigned long)value);
    USART_WriteString(buf);
}

void USART_PrintlnHex(uint32_t value)
{
    USART_PrintHex(value);
    USART_WriteString("\r\n");
}

// Печать float (если нужно)

void USART_PrintFloat(float value, uint8_t digits)
{
    char fmt[8];
    char buf[64];

    // Формат, например "%.2f"
    sprintf(fmt, "%%.%df", digits);
    sprintf(buf, fmt, value);

    USART_WriteString(buf);
}

void USART_PrintlnFloat(float value, uint8_t digits)
{
    USART_PrintFloat(value, digits);
    USART_WriteString("\r\n");
}

/* ===== RX (если захочешь читать из UART) ===== */

uint8_t USART_IsDataReceived(void)
{
    return (USART2->SR & USART_SR_RXNE) ? 1U : 0U;
}

char USART_ReadChar(void)

{
    while ((USART2->SR & USART_SR_RXNE) == 0)
    {
    }
    return (char)(USART2->DR & 0xFFU);
}

///////  ДЛЯ FLOAT ЧИСЕЛ
void USART_PrintFloatSimple(float value, uint8_t digits)
{
    if (digits > 6)
        digits = 6; // ограничим, чтобы не расходиться

    // Знак
    if (value < 0.0f)
    {
        USART_WriteChar('-');
        value = -value;
    }

    // Целая часть
    int32_t intPart = (int32_t)value;
    USART_PrintInt(intPart);

    USART_WriteChar('.');

    // Дробная часть
    float frac = value - (float)intPart;

    for (uint8_t i = 0; i < digits; i++)
    {
        frac *= 10.0f;
        int32_t digit = (int32_t)frac;
        if (digit > 9)
            digit = 9; // на всякий случай
        USART_WriteChar('0' + digit);
        frac -= (float)digit;
    }
}