// pid.h
#ifndef PID_H
#define PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float integrator;
    float prevError;

    float outMin;
    float outMax;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float outMin, float outMax);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

#endif // PID_H
