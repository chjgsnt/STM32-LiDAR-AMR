#include "app_odometry.h"

#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <stdint.h>

/*
 * Positive chassis forward command uses CHASSIS_LEFT_SIGN=-1 and
 * CHASSIS_RIGHT_SIGN=1 on the verified hardware. These macros convert raw
 * timer deltas into positive-forward wheel travel. If forward logs show one
 * side negative, flip that side here.
 */
#ifndef ODOM_LEFT_TICK_SIGN
#define ODOM_LEFT_TICK_SIGN CHASSIS_LEFT_SIGN
#endif

#ifndef ODOM_RIGHT_TICK_SIGN
#define ODOM_RIGHT_TICK_SIGN CHASSIS_RIGHT_SIGN
#endif

#define ODO_LOG_INTERVAL_MS 1000U
#define ODOM_PI 3.14159265358979323846f

static OdomPose_t odom_pose = {0.0f, 0.0f, 0.0f};
static OdomSample_t odom_last_sample = {0};
static int32_t odom_prev_left = 0;
static int32_t odom_prev_right = 0;
static float odom_linear_velocity_mps = 0.0f;
static float odom_angular_velocity_radps = 0.0f;
static uint32_t odom_last_update_ms = 0U;
static uint32_t odom_last_log_ms = 0U;
static uint8_t odom_initialized = 0U;

static float Odom_TicksToMeters(int32_t ticks);
static float Odom_NormalizeAngle(float angle_rad);
static int32_t Odom_ScaleFloatRounded(float value, float multiplier);
static const char *Odom_FixedSign(int32_t value);
static uint32_t Odom_FixedWhole(int32_t value, int32_t decimal_scale);
static uint32_t Odom_FixedFraction(int32_t value, int32_t decimal_scale);
static void Odom_Log(uint32_t now_ms);

void Odom_Init(void)
{
    odom_prev_left = MotorDriver_GetEncoderA();
    odom_prev_right = MotorDriver_GetEncoderB();
    odom_pose.x_m = 0.0f;
    odom_pose.y_m = 0.0f;
    odom_pose.theta_rad = 0.0f;
    odom_last_sample.raw_left_delta = 0;
    odom_last_sample.raw_right_delta = 0;
    odom_last_sample.signed_left_delta = 0;
    odom_last_sample.signed_right_delta = 0;
    odom_last_sample.delta_left_m = 0.0f;
    odom_last_sample.delta_right_m = 0.0f;
    odom_last_sample.last_update_ms = HAL_GetTick();
    odom_linear_velocity_mps = 0.0f;
    odom_angular_velocity_radps = 0.0f;
    odom_last_update_ms = odom_last_sample.last_update_ms;
    odom_last_log_ms = odom_last_sample.last_update_ms;
    odom_initialized = 1U;

    APP_LOG("ODO: init radius_mm=%lu wheel_base_mm=%lu ticks_per_rev=%lu gear_x100=%lu left_sign=%d right_sign=%d",
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_WHEEL_RADIUS_M, 1000.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_WHEEL_BASE_M, 1000.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_ENCODER_TICKS_PER_REV, 1.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_GEAR_RATIO, 100.0f), 1),
            (int)ODOM_LEFT_TICK_SIGN,
            (int)ODOM_RIGHT_TICK_SIGN);
}

void Odom_Update(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;
    float dt_s;

    if (odom_initialized == 0U)
    {
        Odom_Init();
    }

    now_ms = HAL_GetTick();
    elapsed_ms = (now_ms >= odom_last_update_ms) ? (now_ms - odom_last_update_ms) : 0U;
    dt_s = (elapsed_ms > 0U) ? ((float)elapsed_ms / 1000.0f) : 0.0f;

    AppOdo_Update(dt_s);
}

void AppOdo_Update(float dt_s)
{
    int32_t current_left;
    int32_t current_right;
    int32_t raw_left_delta;
    int32_t raw_right_delta;
    int32_t signed_left_delta;
    int32_t signed_right_delta;
    float dl;
    float dr;
    float dc;
    float dtheta;
    float theta_mid;
    uint32_t now_ms = HAL_GetTick();

    if (odom_initialized == 0U)
    {
        Odom_Init();
    }

    current_left = MotorDriver_GetEncoderA();
    current_right = MotorDriver_GetEncoderB();
    raw_left_delta = current_left - odom_prev_left;
    raw_right_delta = current_right - odom_prev_right;
    odom_prev_left = current_left;
    odom_prev_right = current_right;

    signed_left_delta = raw_left_delta * ODOM_LEFT_TICK_SIGN;
    signed_right_delta = raw_right_delta * ODOM_RIGHT_TICK_SIGN;
    dl = Odom_TicksToMeters(signed_left_delta);
    dr = Odom_TicksToMeters(signed_right_delta);
    dc = (dl + dr) * 0.5f;
    dtheta = (dr - dl) / ODO_WHEEL_BASE_M;
    theta_mid = odom_pose.theta_rad + (dtheta * 0.5f);

    odom_pose.x_m += dc * cosf(theta_mid);
    odom_pose.y_m += dc * sinf(theta_mid);
    odom_pose.theta_rad = Odom_NormalizeAngle(odom_pose.theta_rad + dtheta);
    if (dt_s > 0.0001f)
    {
        odom_linear_velocity_mps = dc / dt_s;
        odom_angular_velocity_radps = dtheta / dt_s;
    }
    else
    {
        odom_linear_velocity_mps = 0.0f;
        odom_angular_velocity_radps = 0.0f;
    }

    odom_last_sample.raw_left_delta = raw_left_delta;
    odom_last_sample.raw_right_delta = raw_right_delta;
    odom_last_sample.signed_left_delta = signed_left_delta;
    odom_last_sample.signed_right_delta = signed_right_delta;
    odom_last_sample.delta_left_m = dl;
    odom_last_sample.delta_right_m = dr;
    odom_last_sample.last_update_ms = now_ms;
    odom_last_update_ms = now_ms;

    if ((now_ms - odom_last_log_ms) >= ODO_LOG_INTERVAL_MS)
    {
        Odom_Log(now_ms);
        odom_last_log_ms = now_ms;
    }
}

void Odom_Reset(void)
{
    odom_prev_left = MotorDriver_GetEncoderA();
    odom_prev_right = MotorDriver_GetEncoderB();
    odom_pose.x_m = 0.0f;
    odom_pose.y_m = 0.0f;
    odom_pose.theta_rad = 0.0f;
    odom_linear_velocity_mps = 0.0f;
    odom_angular_velocity_radps = 0.0f;
    odom_last_sample.raw_left_delta = 0;
    odom_last_sample.raw_right_delta = 0;
    odom_last_sample.signed_left_delta = 0;
    odom_last_sample.signed_right_delta = 0;
    odom_last_sample.delta_left_m = 0.0f;
    odom_last_sample.delta_right_m = 0.0f;
    odom_last_sample.last_update_ms = HAL_GetTick();
    odom_last_update_ms = odom_last_sample.last_update_ms;
    odom_last_log_ms = odom_last_sample.last_update_ms;
    odom_initialized = 1U;

    APP_LOG("[ODOM] reset");
}

void AppOdo_Init(void)
{
    Odom_Init();
}

void AppOdo_Reset(void)
{
    Odom_Reset();
}

bool Odom_GetPose(OdomPose_t *pose)
{
    if ((pose == NULL) || (odom_initialized == 0U))
    {
        return false;
    }

    *pose = odom_pose;

    return true;
}

bool Odom_GetLastSample(OdomSample_t *sample)
{
    if ((sample == NULL) || (odom_initialized == 0U))
    {
        return false;
    }

    *sample = odom_last_sample;

    return true;
}

void AppOdo_GetPose(float *x, float *y, float *theta)
{
    if (odom_initialized == 0U)
    {
        Odom_Init();
    }

    if (x != NULL)
    {
        *x = odom_pose.x_m;
    }

    if (y != NULL)
    {
        *y = odom_pose.y_m;
    }

    if (theta != NULL)
    {
        *theta = odom_pose.theta_rad;
    }
}

void AppOdo_GetVelocity(float *v, float *w)
{
    if (v != NULL)
    {
        *v = odom_linear_velocity_mps;
    }

    if (w != NULL)
    {
        *w = odom_angular_velocity_radps;
    }
}

static float Odom_TicksToMeters(int32_t ticks)
{
    return ((float)ticks * (2.0f * ODOM_PI * ODO_WHEEL_RADIUS_M)) /
           (ODO_ENCODER_TICKS_PER_REV * ODO_GEAR_RATIO);
}

static float Odom_NormalizeAngle(float angle_rad)
{
    while (angle_rad > ODOM_PI)
    {
        angle_rad -= (2.0f * ODOM_PI);
    }

    while (angle_rad < -ODOM_PI)
    {
        angle_rad += (2.0f * ODOM_PI);
    }

    return angle_rad;
}

static int32_t Odom_ScaleFloatRounded(float value, float multiplier)
{
    float scaled = value * multiplier;

    if (scaled < 0.0f)
    {
        return (int32_t)(scaled - 0.5f);
    }

    return (int32_t)(scaled + 0.5f);
}

static uint32_t Odom_FixedWhole(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;

    return (uint32_t)(abs_value / decimal_scale);
}

static const char *Odom_FixedSign(int32_t value)
{
    return (value < 0) ? "-" : "";
}

static uint32_t Odom_FixedFraction(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;

    return (uint32_t)(abs_value % decimal_scale);
}

static void Odom_Log(uint32_t now_ms)
{
    int32_t x_mm = Odom_ScaleFloatRounded(odom_pose.x_m, 1000.0f);
    int32_t y_mm = Odom_ScaleFloatRounded(odom_pose.y_m, 1000.0f);
    int32_t theta_mrad = Odom_ScaleFloatRounded(odom_pose.theta_rad, 1000.0f);
    int32_t v_mmps = Odom_ScaleFloatRounded(odom_linear_velocity_mps, 1000.0f);
    int32_t w_mradps = Odom_ScaleFloatRounded(odom_angular_velocity_radps, 1000.0f);

    (void)now_ms;

    APP_LOG("ODO: x=%s%lu.%03lu y=%s%lu.%03lu th=%s%lu.%03lu v=%s%lu.%03lu w=%s%lu.%03lu",
            Odom_FixedSign(x_mm),
            (unsigned long)Odom_FixedWhole(x_mm, 1000),
            (unsigned long)Odom_FixedFraction(x_mm, 1000),
            Odom_FixedSign(y_mm),
            (unsigned long)Odom_FixedWhole(y_mm, 1000),
            (unsigned long)Odom_FixedFraction(y_mm, 1000),
            Odom_FixedSign(theta_mrad),
            (unsigned long)Odom_FixedWhole(theta_mrad, 1000),
            (unsigned long)Odom_FixedFraction(theta_mrad, 1000),
            Odom_FixedSign(v_mmps),
            (unsigned long)Odom_FixedWhole(v_mmps, 1000),
            (unsigned long)Odom_FixedFraction(v_mmps, 1000),
            Odom_FixedSign(w_mradps),
            (unsigned long)Odom_FixedWhole(w_mradps, 1000),
            (unsigned long)Odom_FixedFraction(w_mradps, 1000));
}
