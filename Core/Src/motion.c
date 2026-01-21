// motion.c
// ============================================================
// Движение по прямой линии с удержанием курса (yaw hold)
//
// Управление:
//  - PI по расстоянию (энкодеры)
//  - PID по курсу (гироскоп)
//
// Модуль НЕ занимается поворотами.
// ============================================================

#include "motion.h"
#include "motor.h"
#include "encoder.h"
#include "gyro_yaw.h"

#include <stdint.h>
#include <math.h>

/* ============================================================
 * DEBUG VARIABLES (MCUViewer)
 * ============================================================ */

// геометрия
volatile float dbg_yaw_ref = 0.0f;
volatile float dbg_yaw = 0.0f;
volatile float dbg_yaw_err = 0.0f;

// динамика
volatile float dbg_gz = 0.0f;

// PID yaw
volatile float dbg_yaw_P = 0.0f;
volatile float dbg_yaw_I = 0.0f;
volatile float dbg_yaw_D = 0.0f;
volatile float dbg_yaw_u = 0.0f;

// базовая скорость
volatile int16_t dbg_base_pwm = 0;

/* ============================================================
 * НАСТРОЙКИ
 * ============================================================ */

// кинематика
#define TICKS_PER_M 175.7f

// период управления
#define LOOP_MS 20U

// PWM
#define PWM_MAX 1000
#define PWM_MIN 700

// базовая скорость
#define BASE_MIN 700
#define BASE_MAX 750

// --- расстояние (PI) ---
#define K_DIST_P 0.35f
#define K_DIST_I 0.05f
#define DIST_I_LIM 30.0f

// --- удержание курса (PID) ---
#define K_YAW_P 6.5f
#define K_YAW_I 0.20f
#define K_YAW_D 1.05f

#define YAW_I_LIM 20.0f
#define YAW_D_DEADBAND 2.0f // deg/s
#define GYRO_CORR_LIM 80.0f

// мягкая остановка
#define FINISH_ZONE_TICKS 5
#define BASE_FINISH 630

/* ============================================================
 * extern
 * ============================================================ */

extern volatile uint32_t g_msTicks;
extern volatile float g_yaw_deg;
extern volatile float g_gz_fast_dps;

/* ============================================================
 * утилиты
 * ============================================================ */

static inline int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static inline int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static float wrap180(float a)
{
    while (a > 180.0f)
        a -= 360.0f;
    while (a < -180.0f)
        a += 360.0f;
    return a;
}

/* ============================================================
 * состояние движения
 * ============================================================ */

static MotionState_t s_state = MOTION_IDLE;

// цель
static int32_t s_goal_ticks = 0;
static int32_t s_dir = +1;

// стартовые энкодеры
static int32_t s_startL = 0;
static int32_t s_startR = 0;

// тайминг
static uint32_t s_lastLoop = 0;

// интеграторы
static float s_dist_i = 0.0f;
static float s_yaw_i = 0.0f;

// опорный курс
static float s_yaw_ref = 0.0f;

// фактический курс после завершения движения
volatile float ang_err = 0.0f;

/* ============================================================
 * API
 * ============================================================ */

MotionState_t Motion_GetState(void)
{
    return s_state;
}

void Motion_Stop(void)
{
    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);
    s_state = MOTION_IDLE;
}

void Motion_Init(void)
{
    Motion_Stop();
}

/* ============================================================
 * helpers
 * ============================================================ */

static int32_t mm_to_ticks(int32_t mm)
{
    return (int32_t)lroundf(fabsf((float)mm * TICKS_PER_M / 1000.0f));
}

/* ============================================================
 * start
 * ============================================================ */

void Motion_MoveDistance_mm(int32_t distance_mm, int16_t base_pwm)
{
    (void)base_pwm; // параметр сохранён для совместимости

    s_dir = (distance_mm >= 0) ? +1 : -1;
    s_goal_ticks = mm_to_ticks(distance_mm);

    s_startL = Encoder_GetTotalLeft();
    s_startR = Encoder_GetTotalRight();

    s_dist_i = 0.0f;
    s_yaw_i = 0.0f;

    Gyro_ResetYaw(0.0f);
    Gyro_Update();
    s_yaw_ref = g_yaw_deg;

    s_lastLoop = g_msTicks;
    s_state = MOTION_RUNNING;
}

/* ============================================================
 * update
 * ============================================================ */

void Motion_Update(void)
{
    if (s_state != MOTION_RUNNING)
        return;

    if ((uint32_t)(g_msTicks - s_lastLoop) < LOOP_MS)
        return;
    s_lastLoop += LOOP_MS;

    Gyro_Update();
    const float dt = LOOP_MS * 0.001f;

    /* ---------- энкодеры ---------- */

    int32_t dL = iabs32(Encoder_GetTotalLeft() - s_startL);
    int32_t dR = iabs32(Encoder_GetTotalRight() - s_startR);

    int32_t done = (dL + dR) / 2;
    int32_t rem = s_goal_ticks - done;

    if (rem <= 0)
    {
        // угловая ошибка, накопленная за прямолинейное движение
        ang_err = wrap180(g_yaw_deg - s_yaw_ref);
        Motion_Stop();
        s_state = MOTION_DONE;
        return;
    }

    /* ---------- PI по расстоянию ---------- */

    s_dist_i += rem * dt;
    if (s_dist_i > DIST_I_LIM)
        s_dist_i = DIST_I_LIM;

    float base_f = K_DIST_P * rem + K_DIST_I * s_dist_i;

    // мягкое торможение
    if (rem < FINISH_ZONE_TICKS)
    {
        float k = (float)rem / FINISH_ZONE_TICKS;
        base_f = BASE_FINISH + k * (base_f - BASE_FINISH);
    }

    int16_t base = clamp_i16((int16_t)lroundf(base_f),
                             BASE_MIN, BASE_MAX);
    base *= s_dir;

    /* ---------- удержание курса (PID) ---------- */

    float yaw_err = wrap180(s_yaw_ref - g_yaw_deg);
    float rate = g_gz_fast_dps;

    // интегратор только при движении
    if (abs(base) > BASE_MIN + 10)
        s_yaw_i += yaw_err * dt;

    s_yaw_i = clamp_i16(s_yaw_i, -YAW_I_LIM, YAW_I_LIM);

    if (fabsf(rate) < YAW_D_DEADBAND)
        rate = 0.0f;

    float u_yaw =
        K_YAW_P * yaw_err +
        K_YAW_I * s_yaw_i -
        K_YAW_D * rate;

    if (u_yaw > GYRO_CORR_LIM)
        u_yaw = GYRO_CORR_LIM;
    if (u_yaw < -GYRO_CORR_LIM)
        u_yaw = -GYRO_CORR_LIM;

    int16_t corr = (int16_t)lroundf(u_yaw);

    /* ---------- DEBUG ---------- */

    dbg_yaw_ref = s_yaw_ref;
    dbg_yaw = g_yaw_deg;
    dbg_yaw_err = yaw_err;

    dbg_gz = rate;
    dbg_yaw_P = K_YAW_P * yaw_err;
    dbg_yaw_I = K_YAW_I * s_yaw_i;
    dbg_yaw_D = -K_YAW_D * rate;
    dbg_yaw_u = u_yaw;

    dbg_base_pwm = base;

    /* ---------- моторы ---------- */

    int16_t pwmL = base - corr;
    int16_t pwmR = base + corr;

    pwmL = clamp_i16(pwmL, -PWM_MAX, PWM_MAX);
    pwmR = clamp_i16(pwmR, -PWM_MAX, PWM_MAX);

    if (pwmL > 0 && pwmL < PWM_MIN)
        pwmL = PWM_MIN;
    if (pwmR > 0 && pwmR < PWM_MIN)
        pwmR = PWM_MIN;

    Motor_SetSpeed(MOTOR_A, pwmL);
    Motor_SetSpeed(MOTOR_B, pwmR);
}

/* ============================================================
 * blocking API
 * ============================================================ */

uint8_t Motion_RunDistance_Blocking(int32_t distance_mm,
                                    int16_t base_pwm,
                                    uint32_t timeout_ms)
{
    Motion_MoveDistance_mm(distance_mm, base_pwm);

    uint32_t t0 = g_msTicks;

    while (Motion_GetState() == MOTION_RUNNING)
    {
        Motion_Update();

        if (timeout_ms &&
            (uint32_t)(g_msTicks - t0) > timeout_ms)
        {
            Motion_Stop();
            return 0;
        }
    }
    return 1;
}
