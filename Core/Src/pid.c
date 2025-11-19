// pid.c
#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float outMin, float outMax)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->outMin = outMin;
    pid->outMax = outMax;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // P
    float P = pid->Kp * error;

    // I
    pid->integrator += error * dt;
    float I = pid->Ki * pid->integrator;

    // D
    float derivative = (error - pid->prevError) / dt;
    float D = pid->Kd * derivative;

    float out = P + I + D;

    // Сатурация выхода
    if (out > pid->outMax)
        out = pid->outMax;
    if (out < pid->outMin)
        out = pid->outMin;

    // Антивиндап: если выходим за пределы, можно чуть обрезать интегратор
    // (по-простому: не даём интегратору раздуваться)
    if (out == pid->outMax && error > 0.0f)
        pid->integrator -= error * dt;
    else if (out == pid->outMin && error < 0.0f)
        pid->integrator -= error * dt;

    pid->prevError = error;
    return out;
}
