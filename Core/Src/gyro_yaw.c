// gyro.c
// ============================================================
// MPU6050 (GY-521) — обработка гироскопа по оси Z и угла yaw
//
// Что даёт модуль:
//  - g_gz_dps        : текущая угловая скорость Z (deg/s), bias вычтен
//  - g_gz_fast_dps   : быстрый LPF канал (для D-регулятора / демпфирования)
//  - g_gz_slow_dps   : медленный LPF канал (для интеграции yaw)
//  - g_yaw_deg       : угол yaw в диапазоне [-180..180]
//
// Внутри:
//  - s_bias_z_dps    : оценка смещения гироскопа (дрейф нуля)
//  - s_yaw_unwrap_deg: "развёрнутый" угол (без wrap), чтобы интеграция была непрерывной
// ============================================================

#include "gyro_yaw.h"
#include "gyro_i2c.h"
#include "MPU6050.h"
#include "usart.h"

#include <math.h>
#include <stdint.h>

volatile uint8_t g_turn_active = 0; // 1 = сейчас идёт поворот (важно для bias-адаптации)

/* ===================== НАСТРОЙКИ (ТЮНИНГ) ===================== */

// Если при TurnByAngleDeg(+90) yaw уменьшается (идёт в минус) — ставь -1.0f
#define GYRO_Z_SIGN (+1.0f) // знак оси Z (под конкретный монтаж датчика)

// Быстрый фильтр скорости (меньше alpha => быстрее реагирует, больше шума)
#define GZ_FAST_ALPHA 0.70f // для D-части регулятора (нужна реактивность)

// Медленный фильтр скорости (больше alpha => сильнее сглаживание, меньше шума, больше лаг)
#define GZ_SLOW_ALPHA 0.96f // для интеграции yaw (нужна стабильность)

// Подстройка bias (смещения нуля) во время простоя
#define BIAS_ADAPT_THR_DPS 1.5f // подстраиваем bias только если |gz| меньше этого порога
#define BIAS_ADAPT_GAIN 0.002f  // скорость подстройки bias (слишком большая => "съест" реальное вращение)

// Защита: если dt внезапно большой (подвисание цикла), ограничим влияние на интеграцию
#define MAX_DT_SEC 0.2f // 200 мс максимум на шаг

/* ===================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (видны в других модулях) ===================== */

extern volatile uint32_t g_msTicks;    // системное время в миллисекундах (SysTick)
extern volatile uint8_t g_mpu_last_ok; // 1 = последняя операция чтения по I2C успешна

volatile float g_yaw_deg = 0.0f;     // итоговый yaw [-180..180]
volatile float g_gz_dps = 0.0f;      // скорость Z (deg/s) после вычитания bias (почти "сырая")
volatile float g_gz_fast_dps = 0.0f; // быстрый LPF
volatile float g_gz_slow_dps = 0.0f; // медленный LPF

/* ===================== ВНУТРЕННЕЕ СОСТОЯНИЕ (только внутри gyro.c) ===================== */

static float s_bias_z_dps = 0.0f;     // текущая оценка дрейфа нуля гиры (deg/s)
static uint32_t s_last_ms = 0;        // метка времени последнего Update (ms)
static float s_yaw_unwrap_deg = 0.0f; // "развёрнутый" yaw, без wrap (накапливается бесконечно)

/* ===================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ===================== */

// Нормализуем угол в [-180..180], чтобы удобно сравнивать ошибки по углу
static float wrap180(float a)
{
    while (a > 180.0f)
        a -= 360.0f; // если >180, уводим назад на оборот
    while (a < -180.0f)
        a += 360.0f; // если <-180, уводим вперёд на оборот
    return a;        // возвращаем нормализованное значение
}

// Выдать текущий bias (для логов / отладки)
float Gyro_GetBiasZ(void)
{
    return s_bias_z_dps; // просто отдаём внутреннее значение
}

/* ===================== RESET YAW ===================== */

// Сброс yaw в заданное значение (обычно 0), плюс сброс фильтров
void Gyro_ResetYaw(float yaw_deg)
{
    g_yaw_deg = wrap180(yaw_deg); // публичный yaw всегда держим в [-180..180]
    s_yaw_unwrap_deg = yaw_deg;   // развёрнутый угол ставим точно yaw_deg (без wrap)

    g_gz_fast_dps = 0.0f; // сброс быстрого фильтра
    g_gz_slow_dps = 0.0f; // сброс медленного фильтра

    s_last_ms = g_msTicks; // чтобы следующий Update правильно посчитал dt
}

/* ===================== INIT + КАЛИБРОВКА ===================== */

void Gyro_Init(void)
{
    USART_Println("Gyro init..."); // лог старта

    // 1) Инициализация I2C и самого MPU6050
    GY521_I2C1_Init(); // настройка I2C1 (PB8/PB9)
    MPU6050_Init();    // конфиг MPU: DLPF, диапазоны, sample rate

    // 2) Проверка WHO_AM_I, чтобы понять что датчик реально отвечает
    uint8_t id = MPU6050_ReadWhoAmI();
    if (id != 0x68) // ожидаем 0x68 при AD0=GND
    {
        USART_Print("WHO_AM_I error: 0x");
        USART_PrintlnHex(id);
        while (1)
        {
            ;
        } // фатальная ошибка: без IMU проект бессмысленен
    }

    USART_Println("Gyro calibration (keep still)...");

    // 3) Калибровка bias: усредняем gz, пока робот неподвижен
    float sum = 0.0f; // сумма измерений (deg/s)
    int count = 0;    // количество успешных замеров

    uint32_t t0 = g_msTicks; // старт калибровки
    uint32_t last = t0;      // тайминг 5мс

    // около 2.5 сек, с шагом ~5мс => ~500 измерений (если I2C успевает)
    while ((uint32_t)(g_msTicks - t0) < 2500U)
    {
        if ((uint32_t)(g_msTicks - last) < 5U)
            continue; // ждём следующий тик 5мс
        last += 5U;

        int16_t gz_raw = 0; // сырое значение Z из регистра MPU
        if (MPU6050_ReadGyroZRaw(&gz_raw))
        {
            float dps = MPU6050_GyroLSB_to_dps(gz_raw) * GYRO_Z_SIGN; // перевод в deg/s + знак оси
            sum += dps;                                               // накапливаем
            count++;                                                  // увеличиваем число выборок
        }
    }

    // Защита: даже если чтения были плохие — не делим на слишком маленькое число
    if (count < 100)
        count = 100;

    s_bias_z_dps = sum / (float)count; // оценка смещения нуля (bias)

    USART_Print("Gyro Z bias = ");
    USART_PrintFloat(s_bias_z_dps, 4);
    USART_Println(" dps");

    // Сброс выходных значений, чтобы старт был "чистый"
    g_gz_dps = 0.0f;
    g_gz_fast_dps = 0.0f;
    g_gz_slow_dps = 0.0f;

    Gyro_ResetYaw(0.0f); // стартовый yaw = 0
}

/* ===================== UPDATE (вызывается из Motion_Update / Turn) ===================== */

void Gyro_Update(void)
{
    // 1) dt (секунды)
    uint32_t now = g_msTicks;         // текущее время (ms)
    uint32_t dt_ms = now - s_last_ms; // шаг по времени с прошлого вызова
    if (dt_ms == 0U)
        return;      // если вызвали дважды в один ms — нечего делать
    s_last_ms = now; // запоминаем время для следующего шага

    float dt = (float)dt_ms * 0.001f; // ms -> sec
    if (dt > MAX_DT_SEC)
        dt = MAX_DT_SEC; // защита от "скачка" dt

    // 2) чтение гироскопа Z
    int16_t gz_raw = 0;                 // сырое значение из MPU
    if (!MPU6050_ReadGyroZRaw(&gz_raw)) // читаем 2 байта (H/L)
    {
        g_mpu_last_ok = 0; // флаг ошибки (для логов)
        // НЕ интегрируем yaw на плохом чтении: лучше "заморозить" состояние
        return;
    }
    g_mpu_last_ok = 1; // чтение ок

    // 3) перевод в deg/s и вычитание bias
    float raw_dps = MPU6050_GyroLSB_to_dps(gz_raw) * GYRO_Z_SIGN; // deg/s с нужным знаком
    float gz = raw_dps - s_bias_z_dps;                            // компенсируем дрейф нуля
    g_gz_dps = gz;                                                // публикуем "почти сырой" gz (bias removed)

    // 4) авто-адаптация bias
    // Идея: когда НЕ поворачиваем и скорость близка к нулю — bias слегка подстраиваем,
    // чтобы yaw не уплывал со временем.
    if (!g_turn_active && fabsf(gz) < BIAS_ADAPT_THR_DPS)
    {
        // простая адаптация: bias += k * gz
        // если gz > 0 (мы якобы "крутимся"), значит bias недооценён -> увеличим его
        s_bias_z_dps += BIAS_ADAPT_GAIN * gz;
    }

    // 5) фильтры скорости
    // Быстрый (для D): быстрее реагирует, но шумнее
    g_gz_fast_dps =
        GZ_FAST_ALPHA * g_gz_fast_dps +
        (1.0f - GZ_FAST_ALPHA) * gz;

    // Медленный (для yaw): сильно гасит шум, меньше дрейфа в интеграции
    g_gz_slow_dps =
        GZ_SLOW_ALPHA * g_gz_slow_dps +
        (1.0f - GZ_SLOW_ALPHA) * gz;

    // 6) интеграция yaw
    // Интегрируем ТОЛЬКО медленный канал (так yaw получается стабильнее)
    s_yaw_unwrap_deg += g_gz_slow_dps * dt; // deg/s * s = deg
    g_yaw_deg = wrap180(s_yaw_unwrap_deg);  // публичный yaw держим в [-180..180]
}
