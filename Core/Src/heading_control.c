// heading_control.c
#include "heading_control.h"

// Простой P-регулятор по углу
static float target_yaw = 0.0f;
static const float Kp_yaw = 0.05f; // подбирать!

// Удобная функция ошибки угла с учётом -180..+180
static float angle_error(float target, float current)
{
    float e = target - current;
    // Нормализуем в диапазон -180..+180
    while (e > 180.0f)
        e -= 360.0f;
    while (e < -180.0f)
        e += 360.0f;
    return e;
}

void Heading_SetTarget(float yaw_deg)
{
    // Нормализуем
    while (yaw_deg > 180.0f)
        yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f)
        yaw_deg += 360.0f;
    target_yaw = yaw_deg;
}

void Heading_Compute(float yaw_deg, float base_rps, float *outL, float *outR)
{
    float e = angle_error(target_yaw, yaw_deg);
    float w = Kp_yaw * e; // "угловая скорость" в условных единицах

    // Левое/правое колесо — базовая скорость плюс-минус коррекция
    float vL = base_rps - w;
    float vR = base_rps + w;

    if (outL)
        *outL = vL;
    if (outR)
        *outR = vR;
}
