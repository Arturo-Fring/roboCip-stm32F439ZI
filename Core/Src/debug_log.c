#include "debug_log.h"
#include "usart.h"
#include "gyro_yaw.h"
#include "encoder.h"

extern volatile uint32_t g_msTicks;
extern volatile uint8_t g_mpu_last_ok;

static uint32_t s_lastLog = 0;
static uint32_t s_t0 = 0;
static float s_yaw0 = 0.0f;

void Debug_Log_Init(void)
{
    s_lastLog = g_msTicks;
    s_t0 = g_msTicks;
    s_yaw0 = g_yaw_deg;

    USART_Println("DBG: t_ms yaw gz filt bias ok encL encR");
}

void Debug_Log_Update(void)
{
    if ((uint32_t)(g_msTicks - s_lastLog) < 100U)
        return;
    s_lastLog += 100U;

    // ВАЖНО: Gyro_Update() тут НЕ вызываем. Логгер только печатает.
    // (иначе начнётся двойной Update из разных мест)

    int32_t eL = (int32_t)Encoder_GetTotalLeft();
    int32_t eR = (int32_t)Encoder_GetTotalRight();

    USART_Print("t=");
    USART_PrintInt((int32_t)(g_msTicks - s_t0));

    USART_Print(" yaw=");
    USART_PrintFixed1(g_yaw_deg);

    USART_Print(" gz=");
    USART_PrintFixed1(g_gz_dps);

    USART_Print(" f=");
    USART_PrintFixed1(g_gz_fast_dps);

    USART_Print(" b=");
    USART_PrintFixed1(Gyro_GetBiasZ());

    USART_Print(" ok=");
    USART_PrintInt(g_mpu_last_ok ? 1 : 0);

    USART_Print(" eL=");
    USART_PrintInt(eL);

    USART_Print(" eR=");
    USART_PrintInt(eR);

    USART_Println("");
}
