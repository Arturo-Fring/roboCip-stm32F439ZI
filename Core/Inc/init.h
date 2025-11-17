#include <stdint.h>
#include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"
#include "stm32f4xx.h"

void GPIO_INIT(void);

// LB 2

void ITR_Init(void);

// UART
void UART3_Init(void);
void UART3_SendString(const char *s);
char UART3_ReceiveChar(void);

// pb7
void PB7_Init(void); // PB7 как вывод
