#include "GU521_init.h"
#include "clock.h"
#include <string.h>

void USART2_SendChar(uint8_t ch)
{
    while (!(USART2->SR & USART_SR_TXE)); // Ждем готовности передатчика
    USART2->DR = ch;
}

void USART2_SendString(const char *str)
{
    while (*str) {
        USART2_SendChar(*str++);
    }
}

// Функция для проверки приема
uint8_t USART2_ReceiveChar(void)
{
    while (!(USART2->SR & USART_SR_RXNE)); // Ждем приема данных
    return USART2->DR;
}

int main(void)
{
    Clock_Init();
    Gyro_init();
    
    // Даем время на инициализацию
    for(volatile int i = 0; i < 1000000; i++);
    
    // Отправляем тестовое сообщение
    USART2_SendString("\r\n=== USART2 Test Started ===\r\n");
    USART2_SendString("STM32 is ready!\r\n");
    USART2_SendString("Type something...\r\n");
    
    char counter = 0;
    
    while(1) {
        // Простой эхо-тест
        if (USART2->SR & USART_SR_RXNE) {
            uint8_t received = USART2_ReceiveChar();
            USART2_SendString("Echo: ");
            USART2_SendChar(received);
            USART2_SendString("\r\n");
        }
        
        // Периодическая отправка сообщения
        for(volatile int i = 0; i < 1000000; i++);
        counter++;
        USART2_SendString("Counter: ");
        USART2_SendChar('0' + (counter % 10));
        USART2_SendString("\r\n");
    }
}