#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"
#include <stdint.h>

/*
 * Здесь мы считаем, что:
 *  - частота шины APB1 = 42 МГц (классическая схема при SysClk = 168 МГц)
 * Если у тебя другая — поменяй значение ниже.
 */
#define USART_PCLK1_HZ 42000000UL // если APB1 = 42 МГц

void USART3_Init(uint32_t baudrate);


/* Базовые функции */
void USART_WriteChar(char c);
void USART_WriteString(const char *s);

/* Arduino-style */
void USART_Print(const char *s);
void USART_Println(const char *s);

void USART_PrintInt(int32_t value);
void USART_PrintlnInt(int32_t value);

void USART_PrintHex(uint32_t value);
void USART_PrintlnHex(uint32_t value);

/* По желанию: печать float (через sprintf) */
void USART_PrintFloat(float value, uint8_t digits);
void USART_PrintlnFloat(float value, uint8_t digits);

/* Если захочешь читать из порта */
uint8_t USART_IsDataReceived(void);
char USART_ReadChar(void);

#endif // USART_H
