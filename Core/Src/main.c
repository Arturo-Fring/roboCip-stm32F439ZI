// main.c
// ============================================================
// Основной файл программы
// - инициализация системы
// - выполнение заданной траектории
// - остановка и ожидание
// ============================================================
#include "../../CMSIS/Devices/STM32F4xx/Inc/STM32F429ZI/stm32f429xx.h"

#include "clock.h"
#include "init.h"
#include "usart.h"

#include "motor.h"
#include "encoder.h"
#include "gyro_yaw.h"
#include "motion.h"
#include "turn_in_place.h"
#include "debug_log.h"

#include <stdint.h>

/* ============================================================
 *  extern (используются в main)
 * ============================================================ */

// системное время (SysTick, 1 мс)
extern volatile uint32_t g_msTicks;

// текущий угол yaw (гироскоп)
extern volatile float g_yaw_deg;

/* ============================================================
 *  local helpers
 * ============================================================ */

// простая задержка в миллисекундах (busy wait)
static void Delay_ms(uint32_t ms)
{
    uint32_t t0 = g_msTicks;
    while ((uint32_t)(g_msTicks - t0) < ms)
    {
        // busy wait
    }
}

/**
 * @brief Подготовка робота к повороту
 *        Обязательно вызывать между Motion и Turn
 */
static void PrepareForTurn(void)
{
    // гарантируем, что линейное движение остановлено
    Motion_Stop();

    // даём механике и гироскопу полностью успокоиться
    Delay_ms(200);

    // сбрасываем yaw ТОЛЬКО в покое
    Gyro_ResetYaw(0.0f);

    // даём Gyro_Update пройти хотя бы раз
    Delay_ms(30);
}

/* ============================================================
 *  main
 * ============================================================ */

int main(void)
{
    /* ---------- базовая инициализация ---------- */

    Clock_Init();       // SYSCLK = 168 МГц
    SysTick_Init_1ms(); // системный таймер 1 мс

    USART3_Init(115200); // отладочный UART
    USART_Println("==== ROBOT START ====");

    /* ---------- драйверы ---------- */

    Motor_Init();     // моторы + PWM
    Encoder_Init();   // энкодеры
    Gyro_Init();      // MPU6050 + калибровка
    Motion_Init();    // движение по расстоянию
    Debug_Log_Init(); // логгер (MCUViewer / UART)

    /* ========================================================
     *  ТРАЕКТОРИЯ
     * ======================================================== */

    /* ===== сторона 1 ===== */
    Motion_RunDistance_Blocking(600, 160, 7000);
    Delay_ms(500);
    PrepareForTurn();
    TurnByAngleDeg(-120.0f, 660);
    Delay_ms(500);

    /* ===== сторона 2 ===== */
    Motion_RunDistance_Blocking(1200, 160, 7000);
    Delay_ms(500);
    PrepareForTurn();
    TurnByAngleDeg(120.0f, 660);
    Delay_ms(500);

    /* ===== сторона 3 ===== */
    Motion_RunDistance_Blocking(600, 160, 7000);
    Delay_ms(500);
    PrepareForTurn();
    TurnByAngleDeg(120.0f, 660);
    Delay_ms(500);

    /* ===== сторона 4 ===== */
    Motion_RunDistance_Blocking(1200, 160, 7000);
    Delay_ms(500);
    PrepareForTurn();
    TurnByAngleDeg(-120.0f, 660);
    Delay_ms(500);

    /* ========================================================
     *  стоп и ожидание
     * ======================================================== */

    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);

    USART_Println("==== END OF PROGRAM ====");

    while (1)
    {
        // idle
    }
}
