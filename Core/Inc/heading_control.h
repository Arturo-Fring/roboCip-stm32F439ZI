// heading_control.h
#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdint.h>

// Задать целевой угол курса (в градусах, -180..+180)
void Heading_SetTarget(float yaw_deg);

// Обновить управляющие воздействия: на выходе получаем желаемые rps для левого/правого колеса
// yaw_deg    — текущий угол (из гироскопа)
// base_rps   — "средняя" линейная скорость (об/сек)
// *outL / R  — результат (об/сек) для левого/правого
void Heading_Compute(float yaw_deg, float base_rps, float *outL, float *outR);

#endif // HEADING_CONTROL_H
