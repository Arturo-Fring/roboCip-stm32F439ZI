#include "clock.h"
#include "init.h"
#include "usart.h"
#include "GU521_init.h"
#include "MPU6050.h"
#include "stm32f4xx.h"

extern volatile uint32_t g_msTicks;

static void Delay_ms(uint32_t ms)
{
    uint32_t start = g_msTicks;
    while ((g_msTicks - start) < ms)
    {
    }
}

int main(void)
{
    /* 1. Тактирование ядра и шин */
    Clock_Init();

    /* 2. SysTick на 1 мс (g_msTicks) */
    SysTick_Init_1ms();

    /* 3. UART для отладочного вывода (USART3 на PD8/PD9) */
    USART3_Init(115200);
    USART_Println("=== Simple MPU6050 test (I2C1 PB8/PB9) ===");
    USART_Println("=== Robot gyro test (yaw) ===");

    /* 4. I2C1 на PB8/PB9 (GY-521) */
    GY521_I2C1_Init();
    USART_Println("I2C1 init done");

    /* Небольшая пауза, чтобы модуль проснулся */
    Delay_ms(100);

    /* 5. Инициализация MPU6050 */
    MPU6050_Init();
    USART_Println("MPU6050 Init DONE");

    /* 6. Проверка WHO_AM_I */
    uint8_t id = MPU6050_ReadWhoAmI();
    USART_Print("WHO_AM_I = 0x");
    USART_PrintlnHex(id);

    if (id != 0x68)
    {
        USART_Println("ERROR: MPU6050 not responding!");
        while (1)
        {
            // висим, если датчик не отвечает
        }
    }
    USART_Println("MPU6050 OK!");

    // --- Калибровка гироскопа ---
    USART_Println("Calibrating gyro, do NOT move the robot...");
    float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
    MPU6050_CalibrateGyro(&gyro_bias_x, &gyro_bias_y, &gyro_bias_z);

    USART_Print("Gyro bias [dps]: ");
    USART_PrintFloat(gyro_bias_x, 3);
    USART_Print(" ");
    USART_PrintFloat(gyro_bias_y, 3);
    USART_Print(" ");
    USART_PrintFloat(gyro_bias_z, 3);
    USART_Println("");

    USART_Println("Calibration done.");

    /* 7. Основной цикл: читаем аксель/гиро/температуру */

    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp_raw;

    float yaw_deg = 0.0f;
    uint32_t lastTicks = g_msTicks;

    while (1)
    {

        uint32_t now = g_msTicks;
        float dt = (now - lastTicks) / 1000.0f; // мс → сек
        lastTicks = now;

        MPU6050_ReadRaw(accel, gyro, &temp_raw);

        // переводим сырые значения в dps
        float gx = MPU6050_GyroLSB_to_dps(gyro[0]) - gyro_bias_x;
        float gy = MPU6050_GyroLSB_to_dps(gyro[1]) - gyro_bias_y;
        float gz = MPU6050_GyroLSB_to_dps(gyro[2]) - gyro_bias_z;

        // интеграция yaw по оси Z
        yaw_deg += gz * dt; // gz в °/с, dt в сек → градусы

        // нормализация угла (чтобы не разрастался до бесконечности)
        if (yaw_deg > 180.0f)
            yaw_deg -= 360.0f;
        if (yaw_deg < -180.0f)
            yaw_deg += 360.0f;

        // Можешь печатать вместе с A, G и T, но главное — yaw_deg:
        USART_Print("Yaw[deg]: ");
        USART_PrintFloat(yaw_deg, 2);
        USART_Print("\r\n");

        Delay_ms(20); // ~50 Гц

        // /* Читаем сырые данные */
        // MPU6050_ReadRaw(accel, gyro, &temp_raw);

        // /* Переводим в физические единицы */

        // float ax_g = MPU6050_AccelLSB_to_g(accel[0]);
        // float ay_g = MPU6050_AccelLSB_to_g(accel[1]);
        // float az_g = MPU6050_AccelLSB_to_g(accel[2]);

        // float gx_dps = MPU6050_GyroLSB_to_dps(gyro[0]);
        // float gy_dps = MPU6050_GyroLSB_to_dps(gyro[1]);
        // float gz_dps = MPU6050_GyroLSB_to_dps(gyro[2]);

        // float temp_C = MPU6050_TempLSB_to_C(temp_raw);

        // /* Выводим */

        // USART_Print("A[g]: ");
        // USART_PrintFloat(ax_g, 3);
        // USART_Print(" ");
        // USART_PrintFloat(ay_g, 3);
        // USART_Print(" ");
        // USART_PrintlnFloat(az_g, 3);

        // USART_Print("G[dps]: ");
        // USART_PrintFloat(gx_dps, 2);
        // USART_Print(" ");
        // USART_PrintFloat(gy_dps, 2);
        // USART_Print(" ");
        // USART_PrintlnFloat(gz_dps, 2);

        // USART_Print("T[degC]: ");
        // USART_PrintlnFloat(temp_C, 2);

        // USART_Println("----------------");

        // /* Частота обновления ~10 Гц */
        // Delay_ms(100);
    }
}
