#ifndef INIT_H
#define INIT_H

#include <stdint.h>
#include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"
#include "stm32f4xx.h"

// pb7
void PB7_Init(void); // PB7 как вывод
void SysTick_Init_1ms(void);
extern volatile uint32_t g_msTicks; // только extern, БЕЗ = 0!

#endif