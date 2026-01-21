// MPU6050.c
// ============================================================
// Низкоуровневый драйвер MPU6050 (GY-521)
//
// Отвечает ТОЛЬКО за:
//  - запись / чтение регистров по I2C
//  - инициализацию датчика
//  - чтение сырых данных гироскопа / акселя
//
// НЕ занимается:
//  - фильтрацией
//  - интеграцией угла
//  - логикой движения
// ============================================================

#include "MPU6050.h"
#include "usart.h"

/* ===================== НАСТРОЙКИ I2C ===================== */

#define I2C_DEV I2C1         // используемый I2C модуль
#define I2C_TIMEOUT 100000UL // таймаут ожидания флагов

volatile uint8_t g_mpu_last_ok = 1; // флаг успешности последнего чтения

/* ==========================================================
 *                     LOW-LEVEL I2C
 * ========================================================== */

// Ожидание установки флага в SR1 (SB, ADDR, TXE, RXNE, BTF)
static uint8_t I2C_WaitSR1(uint32_t flag)
{
    uint32_t t = 0;
    while (!(I2C_DEV->SR1 & flag))
    {
        if (t++ > I2C_TIMEOUT)
            return 0; // таймаут → ошибка
    }
    return 1;
}

// Запись одного байта в регистр MPU6050
static uint8_t I2C_WriteReg(uint8_t reg, uint8_t data)
{
    /* START */
    I2C_DEV->CR1 |= I2C_CR1_START;
    if (!I2C_WaitSR1(I2C_SR1_SB))
        goto error;

    /* Адрес + WRITE */
    I2C_DEV->DR = (MPU6050_ADDR << 1);
    if (!I2C_WaitSR1(I2C_SR1_ADDR))
        goto error;

    // Сброс ADDR (обязательно)
    (void)I2C_DEV->SR1;
    (void)I2C_DEV->SR2;

    /* Адрес регистра */
    if (!I2C_WaitSR1(I2C_SR1_TXE))
        goto error;
    I2C_DEV->DR = reg;

    /* Данные */
    if (!I2C_WaitSR1(I2C_SR1_TXE))
        goto error;
    I2C_DEV->DR = data;

    /* Завершение передачи */
    if (!I2C_WaitSR1(I2C_SR1_BTF))
        goto error;
    I2C_DEV->CR1 |= I2C_CR1_STOP;

    return 1;

error:
    I2C_DEV->CR1 |= I2C_CR1_STOP; // обязательно отпускаем шину
    return 0;
}

// Чтение одного байта из регистра MPU6050
static uint8_t I2C_Read1(uint8_t reg, uint8_t *val)
{
    if (!val)
        return 0;

    /* --- WRITE: номер регистра --- */
    I2C_DEV->CR1 |= I2C_CR1_START;
    if (!I2C_WaitSR1(I2C_SR1_SB))
        goto error;

    I2C_DEV->DR = (MPU6050_ADDR << 1);
    if (!I2C_WaitSR1(I2C_SR1_ADDR))
        goto error;

    (void)I2C_DEV->SR1;
    (void)I2C_DEV->SR2;

    if (!I2C_WaitSR1(I2C_SR1_TXE))
        goto error;
    I2C_DEV->DR = reg;

    if (!I2C_WaitSR1(I2C_SR1_BTF))
        goto error;

    /* --- READ: один байт --- */
    I2C_DEV->CR1 |= I2C_CR1_START;
    if (!I2C_WaitSR1(I2C_SR1_SB))
        goto error;

    I2C_DEV->DR = (MPU6050_ADDR << 1) | 1;
    if (!I2C_WaitSR1(I2C_SR1_ADDR))
        goto error;

    (void)I2C_DEV->SR1;
    (void)I2C_DEV->SR2;

    // читаем 1 байт → ACK=0 + STOP
    I2C_DEV->CR1 &= ~I2C_CR1_ACK;
    I2C_DEV->CR1 |= I2C_CR1_STOP;

    if (!I2C_WaitSR1(I2C_SR1_RXNE))
        goto error;

    *val = I2C_DEV->DR;
    I2C_DEV->CR1 |= I2C_CR1_ACK;

    return 1;

error:
    I2C_DEV->CR1 |= I2C_CR1_STOP;
    return 0;
}

/* ==========================================================
 *                   MPU6050 API
 * ========================================================== */

// Инициализация MPU6050
void MPU6050_Init(void)
{
    USART_Println("MPU6050 init");

    // 1) Reset
    if (!I2C_WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_DEVICE_RESET))
        return;

    // небольшая задержка после reset
    for (volatile uint32_t i = 0; i < 500000; i++)
        __NOP();

    // 2) Тактирование от PLL (по X-gyro)
    I2C_WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);

    // 3) DLPF (умеренный фильтр)
    I2C_WriteReg(MPU6050_REG_CONFIG, 0x03);

    // 4) Частота выборки: 1kHz / (7+1) = 125 Hz
    I2C_WriteReg(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_7DIV);

    // 5) Диапазон гироскопа ±2000 dps
    I2C_WriteReg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG_2000);

    // 6) Диапазон акселерометра ±8g
    I2C_WriteReg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_8G);

    // 7) Прерывание Data Ready (не используется, но безопасно)
    I2C_WriteReg(MPU6050_REG_INT_ENABLE, MPU6050_INT_DATA_RDY);
}

// Проверка связи с датчиком
uint8_t MPU6050_ReadWhoAmI(void)
{
    uint8_t id = 0;

    if (!I2C_Read1(MPU6050_REG_WHO_AM_I, &id))
        return 0;

    return id; // ожидаем 0x68
}

// Чтение всех сырых данных (аксель + гироскоп + температура)
void MPU6050_ReadRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buf[14];
    const uint8_t base = MPU6050_REG_ACCEL_XOUT_H;

    for (uint8_t i = 0; i < 14; i++)
    {
        if (!I2C_Read1(base + i, &buf[i]))
        {
            g_mpu_last_ok = 0;
            return;
        }
    }

    g_mpu_last_ok = 1;

    // акселерометр
    accel[0] = (buf[0] << 8) | buf[1];
    accel[1] = (buf[2] << 8) | buf[3];
    accel[2] = (buf[4] << 8) | buf[5];

    // температура
    if (temp)
        *temp = (buf[6] << 8) | buf[7];

    // гироскоп
    gyro[0] = (buf[8] << 8) | buf[9];
    gyro[1] = (buf[10] << 8) | buf[11];
    gyro[2] = (buf[12] << 8) | buf[13];
}

// Быстрое чтение ТОЛЬКО Z-оси гироскопа (используется в gyro.c)
uint8_t MPU6050_ReadGyroZRaw(int16_t *gz_raw)
{
    uint8_t hi, lo;

    if (!gz_raw)
        return 0;

    if (!I2C_Read1(MPU6050_REG_GYRO_ZOUT_H, &hi))
        goto error;
    if (!I2C_Read1(MPU6050_REG_GYRO_ZOUT_L, &lo))
        goto error;

    *gz_raw = (int16_t)((hi << 8) | lo);
    g_mpu_last_ok = 1;
    return 1;

error:
    g_mpu_last_ok = 0;
    return 0;
}

/* ==========================================================
 *               ПРЕОБРАЗОВАНИЕ ЕДИНИЦ
 * ========================================================== */

float MPU6050_AccelLSB_to_g(int16_t raw)
{
    return (float)raw / 4096.0f; // ±8g
}

float MPU6050_GyroLSB_to_dps(int16_t raw)
{
    return (float)raw / 16.4f; // ±2000 dps
}

float MPU6050_TempLSB_to_C(int16_t raw)
{
    return 36.53f + raw / 340.0f;
}
