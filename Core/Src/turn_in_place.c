// robot_motion.c
// УЛУЧШЕННЫЙ ПИД ПОВОРОТ
// ====================

#include "turn_in_place.h"
#include "motor.h"
#include "gyro_yaw.h"
#include <math.h>
#include <stdint.h>

extern volatile uint32_t g_msTicks;
extern volatile float g_yaw_deg;
extern volatile float g_gz_fast_dps;

extern volatile float ang_err;

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_msTicks;
    while ((g_msTicks - start) < ms)
    {
    }
}

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

void TurnByAngleDeg(float angle_deg, int16_t pwm_max)
{
    if (pwm_max < 600)
        pwm_max = 600;
    if (pwm_max > 730)
        pwm_max = 730;

    Gyro_ResetYaw(0.0f);
    uint8_t dir = (angle_deg >= 0);
    float target = angle_deg + ang_err;

    // Более агрессивные коэффициенты
    float Kp = 8.0f;  // Увеличил для более быстрого реагирования
    float Kd = 2.5f;  // Увеличил для лучшего торможения
    float Ki = 0.02f; // Маленький интеграл для устранения остаточной ошибки

    float error_prev = 0, integral = 0;
    uint32_t start = g_msTicks, last = start;

    // Стартовый импульс
    Motor_SetSpeed(MOTOR_A, dir ? -700 : 700); // Увеличил
    Motor_SetSpeed(MOTOR_B, dir ? 700 : -700); // Увеличил
    delay_ms(60);                              // Увеличил

    while (1)
    {

        Gyro_Update();
        float yaw = g_yaw_deg;
        float error = dir ? (target - yaw) : (yaw - target);

        // Основное условие завершения
        if (fabsf(error) < 0.8f)
        { // Уменьшил порог
            break;
        }

        uint32_t now = g_msTicks;
        float dt = (now - last) * 0.001f;
        if (dt < 0.001f)
            dt = 0.001f;

        // Интеграл только для малых ошибок (чтобы не накапливался)
        if (fabsf(error) < 10.0f)
        {
            integral += error * dt;
        }
        integral = clampf(integral, -50.0f, 50.0f); // Уменьшил пределы

        float derivative = (error - error_prev) / dt;
        float pid = Kp * error + Ki * integral + Kd * derivative;

        // PWM с АГРЕССИВНЫМ управлением
        float pwm = pid;
        float abs_error = fabsf(error);

        // Более плавное уменьшение скорости
        if (abs_error < 5.0f)
        {
            // В зоне 5 градусов - очень медленно
            float min_pwm = 150.0f; // Минимальная скорость для доводки
            if (fabsf(pwm) > min_pwm)
            {
                pwm = copysignf(min_pwm, pid);
            }
        }
        else if (abs_error < 15.0f)
        {
            // В зоне 15 градусов - средняя скорость
            float scale = 0.3f + 0.7f * (abs_error / 15.0f); // 30-100%
            pwm = copysignf(450.0f * scale, pid);            // 135-450
        }
        else if (abs_error > 30.0f)
        {
            pwm = copysignf(650.0f, pid); // Максимальная
        }
        else
        {
            pwm = copysignf(550.0f, pid); // Средняя
        }

        // Ограничение
        pwm = clampf(pwm, -pwm_max, pwm_max);

        // Применяем с гарантией минимального усилия
        int16_t pwm_int = (int16_t)pwm;
        if (abs_error > 2.0f)
        { // Если еще далеко, применяем полную мощность
            Motor_SetSpeed(MOTOR_A, dir ? -pwm_int : pwm_int);
            Motor_SetSpeed(MOTOR_B, dir ? pwm_int : -pwm_int);
        }

        error_prev = error;
        last = now;

        // Увеличил таймаут
        if ((g_msTicks - start) > 3000)
        {
            break;
        }

        delay_ms(10);
    }

    // СНАЧАЛА останавливаем моторы
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);

    // Ждем стабилизации
    delay_ms(150); // Увеличил

    // Проверяем точность
    Gyro_Update();
    float final_yaw = g_yaw_deg;
    float final_err = dir ? (target - final_yaw) : (final_yaw - target);

    // АГРЕССИВНАЯ коррекция микроошибок
    if (fabsf(final_err) > 0.5f)
    { // Очень строгий порог!
        int16_t fix_pwm;

        // Выбираем мощность коррекции в зависимости от ошибки
        if (fabsf(final_err) > 4.0f)
        {
            fix_pwm = 250; // Большая ошибка - сильная коррекция
        }
        else if (fabsf(final_err) > 1.5f)
        {
            fix_pwm = 180; // Средняя ошибка
        }
        else
        {
            fix_pwm = 100; // Маленькая ошибка - слабая коррекция
        }

        // Время коррекции пропорционально ошибке
        uint16_t fix_time = (uint16_t)(fabsf(final_err) * 15.0f); // 7.5мс на градус
        if (fix_time < 10)
            fix_time = 10;
        if (fix_time > 100)
            fix_time = 100;

        if (final_err > 0)
        {
            // НЕДОВОРОТ - доворачиваем
            Motor_SetSpeed(MOTOR_A, dir ? -fix_pwm : fix_pwm);
            Motor_SetSpeed(MOTOR_B, dir ? fix_pwm : -fix_pwm);
        }
        else
        {
            // ПЕРЕВОРОТ - возвращаем
            Motor_SetSpeed(MOTOR_A, dir ? fix_pwm : -fix_pwm);
            Motor_SetSpeed(MOTOR_B, dir ? -fix_pwm : fix_pwm);
        }

        delay_ms(fix_time);
        Motor_Stop(MOTOR_A);
        Motor_Stop(MOTOR_B);

        // Проверяем еще раз после коррекции
        delay_ms(100);
        Gyro_Update();
        final_yaw = g_yaw_deg;
        final_err = dir ? (target - final_yaw) : (final_yaw - target);

        // Если все еще есть ошибка, делаем супер-медленную доводку
        if (fabsf(final_err) > 0.3f)
        {
            int16_t super_slow = 60;
            if (final_err > 0)
            {
                Motor_SetSpeed(MOTOR_A, dir ? -super_slow : super_slow);
                Motor_SetSpeed(MOTOR_B, dir ? super_slow : -super_slow);
            }
            else
            {
                Motor_SetSpeed(MOTOR_A, dir ? super_slow : -super_slow);
                Motor_SetSpeed(MOTOR_B, dir ? -super_slow : super_slow);
            }
            delay_ms(30);
            Motor_Stop(MOTOR_A);
            Motor_Stop(MOTOR_B);
        }
    }

    delay_ms(50);
}