
// test. Вращения
#include "clock.h"
#include "init.h" // тут объявлена SysTick_Init_1ms()
#include "usart.h"
#include "motor.h"
#include "stm32f4xx.h"

extern volatile uint32_t g_msTicks; // счётчик миллисекунд из SysTick

static void Delay_ms(uint32_t ms)
{
    uint32_t start = g_msTicks;
    while ((g_msTicks - start) < ms)
    {
        // пустой цикл
    }
}

int mainj(void)
{
    /* 1. Системное тактирование 168 МГц */
    Clock_Init();

    /* 2. SysTick с периодом 1 мс (g_msTicks) */
    SysTick_Init_1ms();

    /* 3. UART для логов (USART3 PD8/PD9) */
    USART3_Init(115200);
    USART_Println("=== MOTOR TEST START ===");

    /* 4. Инициализация моторов (TIM1 + GPIO) */
    Motor_Init();
    USART_Println("Motor_Init done");

    /* На всякий случай остановим оба мотора */
    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);

    USART_Println("Test sequence begins...");

    while (1)
    {
        // --- Шаг 1: Левый вперёд, правый стоп ---
        USART_Println("Step 1: A forward, B stop");
        Motor_SetSpeed(MOTOR_A, 70); // подбери по факту: 30..80
        Motor_SetSpeed(MOTOR_B, 0);
        Delay_ms(2000);

        // --- Шаг 2: Левый стоп, правый вперёд ---
        USART_Println("Step 2: A stop, B forward");
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 70);
        Delay_ms(2000);

        // --- Шаг 3: оба вперёд ---
        USART_Println("Step 3: A forward, B forward");
        Motor_SetSpeed(MOTOR_A, 70);
        Motor_SetSpeed(MOTOR_B, 70);
        Delay_ms(2000);

        // --- Шаг 4: оба стоп ---
        USART_Println("Step 4: A stop, B stop");
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 0);
        Delay_ms(2000);

        // --- Шаг 5: оба назад ---
        USART_Println("Step 5: A backward, B backward");
        Motor_SetSpeed(MOTOR_A, -70);
        Motor_SetSpeed(MOTOR_B, -70);
        Delay_ms(2000);

        // --- Шаг 6: разворот: A вперёд, B назад ---
        USART_Println("Step 6: A forward, B backward (turn in place)");
        Motor_SetSpeed(MOTOR_A, 70);
        Motor_SetSpeed(MOTOR_B, -70);
        Delay_ms(2000);

        // --- Шаг 7: полный стоп ---
        USART_Println("Step 7: FULL STOP");
        Motor_SetSpeed(MOTOR_A, 0);
        Motor_SetSpeed(MOTOR_B, 0);
        Delay_ms(2000);
    }
}
