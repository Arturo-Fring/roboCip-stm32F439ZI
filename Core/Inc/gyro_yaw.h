#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>
#include "stm32f429xx.h"
#ifdef __cplusplus
extern "C"
{
#endif

    // Глобальные значения (обновляются в Gyro_Update)
    extern volatile float g_yaw_deg; // [-180..180]
    extern volatile float g_gz_dps;  // dps (bias removed), signed
    // extern volatile float g_gzFilt_dps;  // dps low-pass, signed
    extern volatile float g_gz_fast_dps; // dps low-pass, signed
    extern volatile uint8_t g_turn_active;

    void Gyro_Init(void);
    void Gyro_Update(void);
    void Gyro_ResetYaw(float yaw_deg);
    float Gyro_GetBiasZ(void);

#ifdef __cplusplus
}
#endif

#endif // GYRO_H
