// speed_control.c
#include "speed_control.h"

extern volatile uint32_t g_msTicks;

// PID для левого и правого мотора
static PID_t pid_left;
static PID_t pid_right;

// целевые скорости (об/сек) для колёс
static float target_left_rps = 0.0f;
static float target_right_rps = 0.0f;

// Небольшой хелпер для перевода тиков за dt в об/сек
static float ticks_to_rps(uint32_t ticks, float dt_sec)
{
    // ticks / dt = тиков/сек
    float tps = (float)ticks / dt_sec;
    // делим на количество тиков за оборот
    float rps = tps / (float)ENC_PULSES_PER_REV;
    return rps;
}

void SpeedControl_Init(void)
{
    // Настраиваем PID так, чтобы его выход был в диапазоне PWM [-MOTOR_PWM_MAX .. +MOTOR_PWM_MAX]
    // Коэффициенты Kp, Ki, Kd — подбирать опытно
    PID_Init(&pid_left, 50.0f, 5.0f, 0.0f, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    PID_Init(&pid_right, 50.0f, 5.0f, 0.0f, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);

    target_left_rps = 0.0f;
    target_right_rps = 0.0f;
}

void SpeedControl_SetTarget(float left_rps, float right_rps)
{
    target_left_rps = left_rps;
    target_right_rps = right_rps;
}

void SpeedControl_Update(float dt_sec)
{
    uint32_t ticksL, ticksR;
    Encoder_GetAndResetTicks(&ticksL, &ticksR);

    // измеренная скорость
    float measL = ticks_to_rps(ticksL, dt_sec);
    float measR = ticks_to_rps(ticksR, dt_sec);

    // ПИД-выход — это "желательный pwm" в диапазоне [-MOTOR_PWM_MAX .. +MOTOR_PWM_MAX]
    float outL = PID_Update(&pid_left, target_left_rps, measL, dt_sec);
    float outR = PID_Update(&pid_right, target_right_rps, measR, dt_sec);

    // Преобразуем float → int16_t и даём в Motor_SetSpeed
    Motor_SetSpeed(MOTOR_A, (int16_t)outL);
    Motor_SetSpeed(MOTOR_B, (int16_t)outR);
}
