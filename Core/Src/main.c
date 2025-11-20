#include "clock.h"
#include "init.h"
#include "usart.h"
#include "motor.h"
#include "encoder.h"
#include "robot_motion.h"
#include "stm32f4xx.h"

extern volatile uint32_t g_msTicks;

static void Delay_ms(uint32_t ms)
{
    uint32_t start = g_msTicks;
    while ((g_msTicks - start) < ms)
    {
    }
}

int main(void)
{
    Clock_Init();
    SysTick_Init_1ms();
    USART3_Init(115200);

    USART_Println("=== SIMPLE DIST TEST (SIGN CALIB) ===");

    Motor_Init();
    Encoder_Init();

    USART_Println("Init OK");
    Delay_ms(1000);

    // ПРОБА 1: Вперёд 30 см, PWM 50
    MoveForwardMM(300.0f, 65);
    Delay_ms(1000);

    // ПРОБА 2: Назад 30 см, PWM 50
    MoveBackwardMM(300.0f, 65);

    USART_Println("=== SCRIPT DONE ===");

    while (1)
    {
        Delay_ms(1000);
    }
}
