#include "MPU6050.h"   

void I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t data)
{
    while(I2C2->SR2 & I2C_SR2_BUSY) __NOP();// Ждем освобождения шины (после предыдущей передачи или чужого мастера)

    SET_BIT(I2C2->CR1, I2C_CR1_START); // Включили передачу данных по шине I2C (Генерируем Старт)
    while(!READ_BIT(I2C2->SR1, I2C_SR1_SB)) __NOP();// Пока SB не равен 1 ждем правильного старта I2C. То есть ждем аппаратно формирования условия Старт на линии SB

    I2C2->DR = addr; // Вписали какой-то адрес (устройства, например) -> Передали бит записи

    while(!READ_BIT(I2C2->SR1, I2C_SR1_ADDR)) __NOP(); // Короче говоря, мы вписали на предыдущем шаге адрес устройства. I2C самовольно выставляет на шину 7битный адрес и бит R/W. Ждет 9 такт (так называемый ACK).
    // Дальше есть два варианта ответ ACK - я тут, готов слушать. NACK - никого нет, я занят.
    // ADDR = 1 значит, что STM32 отправил адрес на шину и ведомое устройство ответило и подтвердило свое существование и готовность к обмену данными
    // ждем подтверждения успешной отправки адреса

    (void)I2C2->SR1;
    (void)I2C2->SR2; // Сбросили флаг ADDR. Операция выглядит странно, однако, это единственный верный метод сброса флага ADDR. Если не сбросить, то передачи будут вечно блокироваться.

    while(!READ_BIT(I2C2->SR1, I2C_SR1_TXE)) __NOP(); // Ждем пока передатчик не отчистится
    I2C2->DR = reg; // Отправляем адрес регистра

    while(!READ_BIT(I2C2->SR1, I2C_SR1_TXE)) __NOP(); // Ждем пока передатчик не отчистится
    I2C2->DR = data; // Отправляем данные

    while(!READ_BIT(I2C2->SR1, I2C_SR1_BTF)) __NOP(); // Ждем успешной передачи данных
    // BTF = 1 значит, что DR у нас пуст, сдвиговый регистр пуст -> Последний байт ливнул

    SET_BIT(I2C2->CR1, I2C_CR1_STOP); // Выключили передачу
}


void MPU6050_Init(void)
{
    // Так как существует вероятность сохранения старых настроек, очистим. Также пробудим гироскоп
    I2C_WriteReg(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, MPU6050_DEVICE_RESET);
    for(uint32_t i = 0; i < 100000; i++); // Небольшая задержечка
    I2C_WriteReg(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_SOURSE_PLL_Xref); // PLL с Ox (вроде как точнее)

    I2C_WriteReg(MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_7DIV); // Предделитель 7 - частота выборки 1 кГц

    I2C_WriteReg(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG_2000); // Диапазон гироскопа +- 2000 градусов

    I2C_WriteReg(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_8g); // Диапазон акселерометра +- 8g

    I2C_WriteReg(MPU6050_ADDR, MPU6050_REG_INT_ENABLE, MPU6050_FSYNC_INT_EN); // Включили прерывания по INT
}

void MPU6050_ReadData(int16_t* accel, int16_t* gyro)
{
    uint8_t buf[14];

    while (I2C2->SR2 & I2C_SR2_BUSY) __NOP();

    /* ---------- START + адрес на запись ---------- */
    SET_BIT(I2C2->CR1, I2C_CR1_START);
    while (!READ_BIT(I2C2->SR1, I2C_SR1_SB)) __NOP();

    /* Правильный адрес: 7 бит << 1 + 0 для записи */
    I2C2->DR = (MPU6050_ADDR << 1);                     // 0xD0 или 0xD2
    while (!READ_BIT(I2C2->SR1, I2C_SR1_ADDR)) __NOP();
    (void)I2C2->SR1;
    (void)I2C2->SR2;

    /* Указываем старший регистр */
    while (!READ_BIT(I2C2->SR1, I2C_SR1_TXE)) __NOP();
    I2C2->DR = MPU6050_REG_ACCEL_XOUT_H;

    /* Ждём, пока байт реально уйдёт (очень важно перед Repeated START!) */
    while (!READ_BIT(I2C2->SR1, I2C_SR1_BTF)) __NOP();

    /* ---------- REPEATED START + адрес на чтение ---------- */
    SET_BIT(I2C2->CR1, I2C_CR1_START);
    while (!READ_BIT(I2C2->SR1, I2C_SR1_SB)) __NOP();

    I2C2->DR = (MPU6050_ADDR << 1) | 0x01;              // 0xD1 или 0xD3
    while (!READ_BIT(I2C2->SR1, I2C_SR1_ADDR)) __NOP();
    (void)I2C2->SR1;
    (void)I2C2->SR2;

    /* Читаем первые 13 байт с ACK */
    for (uint16_t i = 0; i < 13; i++)
    {
        SET_BIT(I2C2->CR1, I2C_CR1_ACK);                // ACK включаем
        while (!READ_BIT(I2C2->SR1, I2C_SR1_RXNE)) __NOP();
        buf[i] = I2C2->DR;
    }

    /* Последний (14-й) байт — без ACK */
    CLEAR_BIT(I2C2->CR1, I2C_CR1_ACK);
    while (!READ_BIT(I2C2->SR1, I2C_SR1_RXNE)) __NOP();
    buf[13] = I2C2->DR;

    SET_BIT(I2C2->CR1, I2C_CR1_STOP);

    /* Сборка 16-битных значений */
    accel[0] = (int16_t)(buf[0]  << 8 | buf[1]);
    accel[1] = (int16_t)(buf[2]  << 8 | buf[3]);
    accel[2] = (int16_t)(buf[4]  << 8 | buf[5]);

    gyro[0]  = (int16_t)(buf[8]  << 8 | buf[9]);
    gyro[1]  = (int16_t)(buf[10] << 8 | buf[11]);
    gyro[2]  = (int16_t)(buf[12] << 8 | buf[13]);
}