#include <stdint.h>
#include "init.h"
#include "Interrupt.h"
#include "clock.h"
#include "motor.h"

// Могу ошибаться, но это курсовая
//comment
static void delay_ms(volatile uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 10000U; i++)
    {
        __NOP();
    }
}


static inline void PB7_On(void)
{
    // Установить PB7 в 1: пишем в нижние 16 бит BSRR
    GPIOB->BSRR = (1U << 7);
}

static inline void PB7_Off(void)
{
    // Сброс PB7: пишем в старшие 16 бит (7 + 16)
    GPIOB->BSRR = (1U << (7 + 16));
}

static inline void PB7_Toggle(void)
{
    // Самый простой вариант: XOR по ODR
    GPIOB->ODR ^= (1U << 7);
}

// Привет всем из первой лабораторной работы!
int main(void)
{

    PB7_Init();   // PB7 как вывод
    Clock_Init(); // 168 МГц, PCLK1=42М, PCLK2=84М, TIM1=168МГц
    PB7_Init();   // PB7 как вывод
    Motor_Init(); // TIM1 + GPIO для моторов
    // GPIO_INIT();
    //  UART3_Init();

    // UART3_SendString("Hello from NUCLEO-F429ZI @ 115200 8N1!\r\n");
    // UART3_SendString("Type something, I will echo it.\r\n");

    // Жёстко задаём направление вперёд
    Motor_SetSpeed(MOTOR_A, -MOTOR_PWM_MAX); // полный вперёд
    Motor_SetSpeed(MOTOR_B, -MOTOR_PWM_MAX);

    while (1)
    {
        __NOP();
    }
}