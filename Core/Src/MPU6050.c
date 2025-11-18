#include "MPU6050.h"
#include "usart.h"

#define I2C_DEV I2C1
#define I2C_TIMEOUT 100000UL

/* ===== Вспомогательная функция ожидания флага SR1 с таймаутом ===== */
static uint8_t I2C_WaitSR1(uint32_t flag)
{
    uint32_t t = 0;
    while (!(I2C_DEV->SR1 & flag))
    {
        if (t++ > I2C_TIMEOUT)
            return 0;
    }
    return 1;
}

/* ===== Запись одного байта в регистр MPU6050 ===== */
static uint8_t I2C_WriteReg(uint8_t reg, uint8_t data)
{
    uint32_t t;

    // START
    I2C_DEV->CR1 |= I2C_CR1_START;
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_SB))
    {
        if (t++ > I2C_TIMEOUT)
        {
            USART_Println("I2C_WriteReg: timeout SB");
            goto error;
        }
    }

    // адрес + write
    I2C_DEV->DR = (MPU6050_ADDR << 1) | 0;
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_ADDR))
    {
        if (t++ > I2C_TIMEOUT)
        {
            USART_Println("I2C_WriteReg: timeout ADDR");
            goto error;
        }
    }
    (void)I2C_DEV->SR1;
    (void)I2C_DEV->SR2;

    // байт регистра
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_TXE))
    {
        if (t++ > I2C_TIMEOUT)
        {
            USART_Println("I2C_WriteReg: timeout TXE (reg)");
            goto error;
        }
    }
    I2C_DEV->DR = reg;

    // байт данных
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_TXE))
    {
        if (t++ > I2C_TIMEOUT)
        {
            USART_Println("I2C_WriteReg: timeout TXE (data)");
            goto error;
        }
    }
    I2C_DEV->DR = data;

    // ждём BTF
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_BTF))
    {
        if (t++ > I2C_TIMEOUT)
        {
            USART_Println("I2C_WriteReg: timeout BTF");
            goto error;
        }
    }

    // STOP
    I2C_DEV->CR1 |= I2C_CR1_STOP;
    return 1;

error:
{
    uint32_t sr1 = I2C_DEV->SR1;
    uint32_t sr2 = I2C_DEV->SR2;
    I2C_DEV->CR1 |= I2C_CR1_STOP;

    USART_Print("I2C_WriteReg ERROR, SR1 = 0x");
    USART_PrintHex(sr1);
    USART_Print(" SR2 = 0x");
    USART_PrintlnHex(sr2);
}
    return 0;
}

/* ===== Уже проверенная функция I2C_Read (1 байт) ===== */
/* Используем только для len=1, чтобы не мучиться с мультибайтом */

static uint8_t I2C_Read1(uint8_t reg, uint8_t *val)
{
    uint32_t t;

    if (!val)
        return 0;

    // USART_Println("I2C_Read: start");

    /* --- ЭТАП 1: пишем номер регистра (WRITE) --- */

    // START
    I2C_DEV->CR1 |= I2C_CR1_START;
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_SB))
    {
        if (t++ > I2C_TIMEOUT)
        {
            //  USART_Println("I2C_Read: timeout SB (stage1)");
            goto error;
        }
    }

    // адрес + write
    I2C_DEV->DR = (MPU6050_ADDR << 1) | 0;
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_ADDR))
    {
        if (t++ > I2C_TIMEOUT)
        {
            // USART_Println("I2C_Read: timeout ADDR (stage1)");
            goto error;
        }
    }
    (void)I2C_DEV->SR1;
    (void)I2C_DEV->SR2;

    // байт регистра
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_TXE))
    {
        if (t++ > I2C_TIMEOUT)
        {
            // USART_Println("I2C_Read: timeout TXE (reg)");
            goto error;
        }
    }
    I2C_DEV->DR = reg;

    // ждём BTF
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_BTF))
    {
        if (t++ > I2C_TIMEOUT)
        {
            // USART_Println("I2C_Read: timeout BTF (before restart)");
            goto error;
        }
    }

    /* --- ЭТАП 2: Re-START и чтение 1 байта --- */

    // Re-START
    I2C_DEV->CR1 |= I2C_CR1_START;
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_SB))
    {
        if (t++ > I2C_TIMEOUT)
        {
            // USART_Println("I2C_Read: timeout SB (stage2)");
            goto error;
        }
    }

    // адрес + read
    I2C_DEV->DR = (MPU6050_ADDR << 1) | 1;
    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_ADDR))
    {
        if (t++ > I2C_TIMEOUT)
        {
            // USART_Println("I2C_Read: timeout ADDR (stage2)");
            goto error;
        }
    }
    (void)I2C_DEV->SR1;
    (void)I2C_DEV->SR2;

    // один байт: ACK=0, STOP, читаем
    I2C_DEV->CR1 &= ~I2C_CR1_ACK;
    I2C_DEV->CR1 |= I2C_CR1_STOP;

    t = 0;
    while (!(I2C_DEV->SR1 & I2C_SR1_RXNE))
    {
        if (t++ > I2C_TIMEOUT)
        {
            // USART_Println("I2C_Read: timeout RXNE (1 byte)");
            goto error_after_stop;
        }
    }
    *val = I2C_DEV->DR;

    I2C_DEV->CR1 |= I2C_CR1_ACK;
    // USART_Println("I2C_Read: done (1 byte)");
    return 1;

error_after_stop:
    I2C_DEV->CR1 |= I2C_CR1_STOP;

error:
{
    uint32_t sr1 = I2C_DEV->SR1;
    uint32_t sr2 = I2C_DEV->SR2;
    I2C_DEV->CR1 |= I2C_CR1_STOP;

    // USART_Print("I2C_Read ERROR, SR1 = 0x");
    // USART_PrintHex(sr1);
    // USART_Print(" SR2 = 0x");
    // USART_PrintlnHex(sr2);
}
    return 0;
}

/* ======== ВЫСОКИЙ УРОВЕНЬ MPU6050 ======== */

void MPU6050_Init(void)
{
    USART_Println("MPU6050_Init: start");

    // Сброс
    USART_Println("MPU6050_Init: reset device");
    if (!I2C_WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_DEVICE_RESET))
    {
        USART_Println("MPU6050_Init: failed to write reset");
        return;
    }

    for (volatile uint32_t i = 0; i < 500000; i++)
        __NOP();

    // Тактирование от PLL по X-gyro
    if (!I2C_WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO))
        USART_Println("MPU6050_Init: failed to set clock source");

    // DLPF
    if (!I2C_WriteReg(MPU6050_REG_CONFIG, 0x03))
        USART_Println("MPU6050_Init: failed to set DLPF");

    // Частота выборки
    if (!I2C_WriteReg(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_7DIV))
        USART_Println("MPU6050_Init: failed to set SMPLRT_DIV");

    // Диапазон гироскопа
    if (!I2C_WriteReg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG_2000))
        USART_Println("MPU6050_Init: failed to set GYRO_CONFIG");

    // Диапазон акселерометра
    if (!I2C_WriteReg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_8G))
        USART_Println("MPU6050_Init: failed to set ACCEL_CONFIG");

    // Прерывания
    if (!I2C_WriteReg(MPU6050_REG_INT_PIN_CFG, 0x00))
        USART_Println("MPU6050_Init: failed to set INT_PIN_CFG");

    if (!I2C_WriteReg(MPU6050_REG_INT_ENABLE, MPU6050_INT_DATA_RDY))
        USART_Println("MPU6050_Init: failed to set INT_ENABLE");

    USART_Println("MPU6050_Init: done");
}

uint8_t MPU6050_ReadWhoAmI(void)
{
    uint8_t id = 0;

    if (!I2C_Read1(MPU6050_REG_WHO_AM_I, &id))
    {
        USART_Println("MPU6050_ReadWhoAmI: I2C_Read failed");
        return 0;
    }

    USART_Print("MPU6050_ReadWhoAmI: 0x");
    USART_PrintlnHex(id);

    return id;
}

/* ===== Чтение сырых данных с акселя/гиро ===== */

void MPU6050_ReadRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t b;

    uint8_t ok = 1;

    uint8_t buf[14];

    // читаем 14 регистров по одному байту:
    uint8_t regs[14] = {
        0x3B, // ACCEL_XOUT_H
        0x3C,
        0x3D,
        0x3E,
        0x3F,
        0x40,
        0x41, // TEMP_OUT_H
        0x42,
        0x43, // GYRO_XOUT_H
        0x44,
        0x45,
        0x46,
        0x47,
        0x48 // GYRO_ZOUT_L
    };

    for (int i = 0; i < 14; i++)
    {
        if (!I2C_Read1(regs[i], &b))
        {
            ok = 0;
            break;
        }
        buf[i] = b;
    }

    if (!ok)
    {
        USART_Println("MPU6050_ReadRaw: I2C_Read failed");
        return;
    }

    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);

    int16_t t = (int16_t)((buf[6] << 8) | buf[7]);
    if (temp)
        *temp = t;

    gyro[0] = (int16_t)((buf[8] << 8) | buf[9]);
    gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);
}

float MPU6050_AccelLSB_to_g(int16_t raw)
{
    // ±8g → 4096 LSB/g
    return (float)raw / 4096.0f;
}

float MPU6050_GyroLSB_to_dps(int16_t raw)
{
    // ±2000 dps → 16.4 LSB/°/s
    return (float)raw / 16.4f;
}

float MPU6050_TempLSB_to_C(int16_t raw)
{
    return 36.53f + (float)raw / 340.0f;
}

void MPU6050_CalibrateGyro(float *bias_x, float *bias_y, float *bias_z)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t accel[3], gyro[3], temp;
    const int N = 5000;

    for (int i = 0; i < N; i++)
    {
        MPU6050_ReadRaw(accel, gyro, &temp);
        sum_x += gyro[0];
        sum_y += gyro[1];
        sum_z += gyro[2];

        // маленькая пауза, чтобы не грузить шину
        for (volatile uint32_t d = 0; d < 2000; d++)
            __NOP();
    }

    if (bias_x)
        *bias_x = MPU6050_GyroLSB_to_dps(sum_x / (float)N);
    if (bias_y)
        *bias_y = MPU6050_GyroLSB_to_dps(sum_y / (float)N);
    if (bias_z)
        *bias_z = MPU6050_GyroLSB_to_dps(sum_z / (float)N);
}