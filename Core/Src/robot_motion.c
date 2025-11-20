#include "robot_motion.h"
#include "motor.h"
#include "encoder.h"
#include "usart.h"

extern volatile uint32_t g_msTicks;

/* === КОНФИГ === */

/* Какой знак ШИМа означает "ФИЗИЧЕСКИ вперёд".
 * Если сейчас MoveForwardMM едет назад — просто поменяй +1 на -1.
 */
#define MOTOR_FORWARD_SIGN (+1) // при необходимости поменяешь на (-1)

/* минимальный рабочий PWM, чтобы колёса точно крутились */
#define PWM_MIN_START 60
#define PWM_MAX MOTOR_PWM_MAX

/* При желании можно чуть "поддушить" одно колесо, если будет вести.
 * Пока ставим одинаково.
 */
#define WHEEL_BALANCE_A 1.00f // множитель для MOTOR_A
#define WHEEL_BALANCE_B 1.00f // множитель для MOTOR_B

/* === ВСПОМОГАТЕЛЬНОЕ === */

static void Delay_ms(uint32_t ms)
{
    uint32_t start = g_msTicks;
    while ((g_msTicks - start) < ms)
    {
    }
}

static int16_t clamp_pwm_mag(int16_t v)
{
    if (v < 0)
        v = -v;
    if (v < PWM_MIN_START)
        v = PWM_MIN_START;
    if (v > PWM_MAX)
        v = PWM_MAX;
    return v;
}

/* === ОСНОВНАЯ ФУНКЦИЯ === */

void DriveDistanceMM(float distance_mm, int16_t speed)
{
    if (distance_mm <= 0.0f || speed == 0)
        return;

    /* модуль PWM */
    int16_t pwm_mag = clamp_pwm_mag(speed);

    /* знак направления:
     * >0  → "вперёд" с точки зрения функции,
     * <0  → "назад".
     */
    int8_t dir_sign = (speed > 0) ? +1 : -1;

    /* итоговый знак для моторов с учётом конфигурации "что такое вперёд" */
    int8_t motor_dir = (dir_sign > 0) ? MOTOR_FORWARD_SIGN : -MOTOR_FORWARD_SIGN;

    USART_Print("Drive ");
    USART_PrintFloat(distance_mm, 1);
    USART_Print(" mm, pwm_mag=");
    USART_PrintInt(pwm_mag);
    USART_Print(" dir_sign=");
    USART_PrintlnInt(dir_sign);

    uint32_t startL = Encoder_GetTotalLeft();
    uint32_t startR = Encoder_GetTotalRight();

    /* базовые PWM для обоих моторов (можно будет подкорректировать балансом) */
    int16_t pwmA = (int16_t)(pwm_mag * WHEEL_BALANCE_A);
    int16_t pwmB = (int16_t)(pwm_mag * WHEEL_BALANCE_B);

    /* применяем знак направления */
    pwmA *= motor_dir;
    pwmB *= motor_dir;

    Motor_SetSpeed(MOTOR_A, pwmA);
    Motor_SetSpeed(MOTOR_B, pwmB);

    uint32_t lastPrint = g_msTicks;

    while (1)
    {
        uint32_t curL = Encoder_GetTotalLeft();
        uint32_t curR = Encoder_GetTotalRight();

        float distL = Encoder_TicksToMM(curL - startL);
        float distR = Encoder_TicksToMM(curR - startR);
        float dist = 0.5f * (distL + distR);

        if (g_msTicks - lastPrint > 100)
        {
            lastPrint = g_msTicks;
            USART_Print("Distance: ");
            USART_PrintFloat(dist, 1);
            USART_Print(" mm   L=");
            USART_PrintFloat(distL, 1);
            USART_Print(" R=");
            USART_PrintFloat(distR, 1);
            USART_Println(" mm");
        }

        if (dist >= distance_mm)
            break;
    }

    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);

    USART_Println("STOP!");
}

/* === ОБЁРТКИ === */

void MoveForwardMM(float distance_mm, int16_t pwm)
{
    if (pwm < 0)
        pwm = -pwm;
    /* Положительный speed → "вперёд" в логике функции,
       а реальное направление задаётся MOTOR_FORWARD_SIGN. */
    DriveDistanceMM(distance_mm, +pwm);
}

void MoveBackwardMM(float distance_mm, int16_t pwm)
{
    if (pwm < 0)
        pwm = -pwm;
    /* Отрицательный speed → "назад" в логике функции. */
    DriveDistanceMM(distance_mm, -pwm);
}
