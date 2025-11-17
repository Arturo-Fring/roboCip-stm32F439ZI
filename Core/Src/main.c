#include "GU521_init.h"
#include "clock.h"
#include <string.h>
#include "motor.h"
#include "usart.h"
#include "init.h"

int main(void)
{
    /*  =============== ИНИЦИАЛИЗАЦИЯ =================== */
    Clock_Init();
    SysTick_Init_1ms();
    Gyro_init();
    Motor_Init(); // настраивает TIM1, GPIO и т.д.
    USART3_Init(115200);
    USART_Println("=== Robot debug UART ready ===");
    /*  ============================================ */

    /*  ===============  ПЕРЕМЕННЫЕ =================== */
    int32_t speedL = 0;
    int32_t speedR = 0;
    float yaw = 0.0f;
    char counter = 0;
    /*  ============================================ */

    while (1)
    {
        static uint32_t lastPrint = 0;
        uint32_t now = g_msTicks;

        if ((now - lastPrint) >= 1000)
        { // раз в 100 мс
            lastPrint = now;

            USART_Print("SpeedL=");
            USART_PrintInt(speedL);
            USART_Print(" SpeedR=");
            USART_PrintInt(speedR);
            USART_Print(" Yaw=");
            USART_PrintlnFloat(yaw, 2);
        }
    }
}