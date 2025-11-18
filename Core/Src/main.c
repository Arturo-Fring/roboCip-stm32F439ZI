#include "GU521_init.h"
#include "clock.h"
#include <string.h>
#include "motor.h"
#include "usart.h"
#include "init.h"
#include "MPU6050.h"

int main(void)
{
    /*  =============== ИНИЦИАЛИЗАЦИЯ =================== */
    Clock_Init();               // твоя настройка тактирования (42/84/168 МГц и т.д.)
    SysTick_Init_1ms();         // g_msTicks уже тикает
    Gyro_init();                // I2C2 + пины PF0/PF1
    Motor_Init();
    USART3_Init(115200);

    USART_Println("=== Robot debug UART ready ===");
    MPU6050_Init();             // сброс + настройка диапазонов

    /* Ждём стабилизации гироскопа после включения */
    for(uint8_t i = 0; i < 100000; i++);

    /* Проверка WHO_AM_I */
    uint8_t id = 0;
    I2C_WriteReg(MPU6050_ADDR, MPU6050_REG_WHO_AM_I, 0x00); // dummy write
    // простое чтение одного байта
    while (I2C2->SR2 & I2C_SR2_BUSY);
    SET_BIT(I2C2->CR1, I2C_CR1_START);
    while (!READ_BIT(I2C2->SR1, I2C_SR1_SB));
    I2C2->DR = (MPU6050_ADDR << 1) | 1;
    while (!READ_BIT(I2C2->SR1, I2C_SR1_ADDR));
    (void)I2C2->SR1; (void)I2C2->SR2;
    CLEAR_BIT(I2C2->CR1, I2C_CR1_ACK);
    SET_BIT(I2C2->CR1, I2C_CR1_STOP);
    while (!READ_BIT(I2C2->SR1, I2C_SR1_RXNE));
    id = I2C2->DR;

    USART_Print("MPU6050 WHO_AM_I = 0x");
    USART_PrintlnHex(id);
    if (id != 0x68) {
        USART_Println("ERROR: MPU6050 not responding!");
        while(1);
    }
    USART_Println("MPU6050 OK!");

    /*  ===============  ПЕРЕМЕННЫЕ =================== */
    int16_t accel[3] = {0};
    int16_t gyro[3]  = {0};

    int32_t speedL = 0;
    int32_t speedR = 0;

    float yaw = 0.0f;                   // текущий угол поворота (градусы)
    float gyroZ_offset = 0.0f;          // смещение нуля (калибруется при старте)

    uint32_t lastTime = g_msTicks;
    uint32_t lastPrint = g_msTicks;

    char counter = 0;

    /* ========== Калибровка смещения гироскопа (10 секунд покоя) ========== */
    USART_Println("Calibrating gyro... Keep robot STILL for 10 sec!");
    int32_t sum_gyroZ = 0;
    uint16_t samples = 0;

    for (int i = 0; i < 10000; i++) // ~10 секунд при 1 кГц выборки
    {
        MPU6050_ReadData(accel, gyro);
        sum_gyroZ += gyro[2];
        samples++;
        for(uint8_t i = 0; i < 1000; i++);
    }

    gyroZ_offset = (float)sum_gyroZ / samples / 131.0f;  // LSB/°/s при ±2000 dps
    USART_Print("Gyro Z offset = ");
    USART_PrintlnFloat(gyroZ_offset, 4);

    USART_Println("Calibration done! Starting main loop...");

    /* ====================== ОСНОВНОЙ ЦИКЛ ====================== */
    while (1)
    {
        uint32_t now = g_msTicks;

        /* Читаем данные с MPU6050 (как можно чаще) */
        MPU6050_ReadData(accel, gyro);

        /* Интеграция угла Yaw по оси Z */
        uint32_t dt_ms = now - lastTime;
        if (dt_ms > 0)
        {
            float dt_sec = dt_ms / 1000.0f;
            float gyroZ_dps = (gyro[2] / 131.0f) - gyroZ_offset;  // ±2000 dps → 131 LSB/°/s
            yaw += gyroZ_dps * dt_sec;
            lastTime = now;
        }

        /* Выводим данные раз в секунду (или чаще/реже — как хочешь) */
        if ((now - lastPrint) >= 1000)  // каждую секунду
        {
            lastPrint = now;

            USART_Print("A: ");
            USART_PrintInt(accel[0]); USART_Print(" ");
            USART_PrintInt(accel[1]); USART_Print(" ");
            USART_PrintlnInt(accel[2]);

            USART_Print("G: ");
            USART_PrintInt(gyro[0]); USART_Print(" ");
            USART_PrintInt(gyro[1]); USART_Print(" ");
            USART_PrintlnInt(gyro[2]);

            USART_Print("Yaw = ");
            USART_PrintlnFloat(yaw, 2);
            USART_Println("------------------------");
        }

        /* Здесь будет управление моторами по yaw, PID и т.д. */
        // speedL = ...
        // speedR = ...

        // Пока просто моргаем счётчиком
        counter++;
        if (counter > 50) counter = 0;
    }
}