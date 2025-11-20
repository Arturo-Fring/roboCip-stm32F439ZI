// speed_control.c
#include "speed_control.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h" // твой модуль PID

extern volatile uint32_t g_msTicks;

// PID для левого и правого мотора
static PID_t pid_left;
static PID_t pid_right;

// целевые скорости (об/сек) и направления
static float target_left_rps = 0.0f;
static float target_right_rps = 0.0f;
static int8_t dir_left = +1;
static int8_t dir_right = +1;

// перевод тиков за dt в обороты/сек (всегда >= 0)
static float ticks_to_rps(uint32_t ticks, float dt_sec)
{
    float tps = (float)ticks / dt_sec;           // ticks per second
    float rps = tps / (float)ENC_PULSES_PER_REV; // revolutions per second
    return rps;
}

void SpeedControl_Init(void)
{
    /* Более мягкие коэффициенты ПИД:
     * начинаем с небольшого Kp, Ki, без D.
     * Диапазон выхода [0 .. MOTOR_PWM_MAX].
     */
    PID_Init(&pid_left, 8.0f, 1.0f, 0.0f, 0.0f, (float)MOTOR_PWM_MAX);
    PID_Init(&pid_right, 8.0f, 1.0f, 0.0f, 0.0f, (float)MOTOR_PWM_MAX);

    target_left_rps = 0.0f;
    target_right_rps = 0.0f;
    dir_left = +1;
    dir_right = +1;

    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
}

void SpeedControl_SetTarget(float left_rps, float right_rps,
                            int8_t dirL, int8_t dirR)
{
    if (left_rps < 0.0f)
        left_rps = 0.0f;
    if (right_rps < 0.0f)
        right_rps = 0.0f;

    target_left_rps = left_rps;
    target_right_rps = right_rps;

    dir_left = (dirL >= 0) ? +1 : -1;
    dir_right = (dirR >= 0) ? +1 : -1;
}

void SpeedControl_Stop(void)
{
    target_left_rps = 0.0f;
    target_right_rps = 0.0f;

    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
}

void SpeedControl_Update(float dt_sec)
{
    uint32_t ticksL, ticksR;
    Encoder_GetAndResetTicks(&ticksL, &ticksR);

    // измеренная скорость
    float measL = ticks_to_rps(ticksL, dt_sec);
    float measR = ticks_to_rps(ticksR, dt_sec);

    // ПИД-выход — желаемый |pwm| в [0 .. MOTOR_PWM_MAX]
    float outL = PID_Update(&pid_left, target_left_rps, measL, dt_sec);
    float outR = PID_Update(&pid_right, target_right_rps, measR, dt_sec);

    if (outL < 0.0f)
        outL = 0.0f;
    if (outR < 0.0f)
        outR = 0.0f;

    int16_t pwmL = (int16_t)outL;
    int16_t pwmR = (int16_t)outR;

    /* --- Минимальный PWM для сдвига мотора --- */
    const int16_t PWM_MIN_START = 60; // подбирал бы в районе 35–50

    if (target_left_rps > 0.0f && pwmL > 0 && pwmL < PWM_MIN_START)
        pwmL = PWM_MIN_START;
    if (target_right_rps > 0.0f && pwmR > 0 && pwmR < PWM_MIN_START)
        pwmR = PWM_MIN_START;

    // учитываем направление
    pwmL *= dir_left;
    pwmR *= dir_right;

    Motor_SetSpeed(MOTOR_A, pwmL);
    Motor_SetSpeed(MOTOR_B, pwmR);
}
