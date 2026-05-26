#include "app_odometry.h"

#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <stdint.h>

/*
 * Encoder diagnostic showed left-wheel forward ticks are positive in odometry
 * coordinates. Keep odometry signs independent from chassis motor polarity.
 */
#ifndef ODOM_LEFT_TICK_SIGN
#define ODOM_LEFT_TICK_SIGN (1)
#endif

#ifndef ODOM_RIGHT_TICK_SIGN
#define ODOM_RIGHT_TICK_SIGN (1)
#endif

#if APP_ODO_AUTO_PRINT_ENABLE
#define ODO_LOG_INTERVAL_MS 1000U
#endif
#define ODO_WARN_INTERVAL_MS 1000U
#define ODOM_PI 3.14159265358979323846f

static OdomPose_t odom_pose = {0.0f, 0.0f, 0.0f};
static OdomSample_t odom_last_sample = {0};
static int32_t odom_prev_left = 0;
static int32_t odom_prev_right = 0;
static int32_t odom_debug_total_raw_left = 0;
static int32_t odom_debug_total_raw_right = 0;
static int32_t odom_debug_total_signed_left = 0;
static int32_t odom_debug_total_signed_right = 0;
static float odom_linear_velocity_mps = 0.0f;
static float odom_angular_velocity_radps = 0.0f;
static uint32_t odom_last_update_ms = 0U;
#if APP_ODO_AUTO_PRINT_ENABLE
static uint32_t odom_last_log_ms = 0U;
#endif
static uint32_t odom_last_warn_ms = 0U;
static uint8_t odom_initialized = 0U;
static uint8_t odom_freeze = 0U;

static void Odom_ClearDebugTotals(void);
static int32_t Odom_LeftDelta(int32_t current, int32_t previous);
static int32_t Odom_RightDelta(int32_t current, int32_t previous);
static float Odom_TicksToMeters(int32_t ticks);
static float Odom_NormalizeAngle(float angle_rad);
static float Odom_AbsFloat(float value);
static int32_t Odom_ScaleFloatRounded(float value, float multiplier);
static const char *Odom_FixedSign(int32_t value);
static uint32_t Odom_FixedWhole(int32_t value, int32_t decimal_scale);
static uint32_t Odom_FixedFraction(int32_t value, int32_t decimal_scale);
#if APP_ODO_AUTO_PRINT_ENABLE
static void Odom_Log(uint32_t now_ms);
#endif
static void Odom_LogLargeStep(uint32_t now_ms, float ds_m, float dtheta_rad, float dt_s);
static void Odom_LogDtSkip(uint32_t now_ms, uint32_t dt_ms, uint8_t reason);

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
    odom_last_sample.total_raw_left_delta = 0;
    odom_last_sample.total_raw_right_delta = 0;
    odom_last_sample.total_signed_left_delta = 0;
    odom_last_sample.total_signed_right_delta = 0;
    odom_last_sample.delta_left_m = 0.0f;
    odom_last_sample.delta_right_m = 0.0f;
    odom_last_sample.delta_center_m = 0.0f;
    odom_last_sample.delta_theta_rad = 0.0f;
    odom_last_sample.total_delta_left_m = 0.0f;
    odom_last_sample.total_delta_right_m = 0.0f;
    odom_last_sample.total_delta_center_m = 0.0f;
    odom_last_sample.total_delta_theta_rad = 0.0f;
    odom_last_sample.dt_s = 0.0f;
    odom_last_sample.last_update_ms = HAL_GetTick();
    odom_last_sample.step_skipped = 0U;
    odom_last_sample.skip_reason = APP_ODO_SKIP_NONE;
    odom_last_sample.frozen = odom_freeze;
    odom_linear_velocity_mps = 0.0f;
    odom_angular_velocity_radps = 0.0f;
    odom_last_update_ms = odom_last_sample.last_update_ms;
#if APP_ODO_AUTO_PRINT_ENABLE
    odom_last_log_ms = odom_last_sample.last_update_ms;
#endif
    odom_last_warn_ms = 0U;
    odom_initialized = 1U;
    Odom_ClearDebugTotals();

    APP_LOG("ODO: init radius_mm=%lu wheel_base_mm=%lu ticks_per_rev=%lu gear_x100=%lu left_sign=%d right_sign=%d max_step_mm=%lu max_dth_mrad=%lu dt_ms=%u..%u",
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_WHEEL_RADIUS_M, 1000.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_WHEEL_BASE_M, 1000.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_ENCODER_TICKS_PER_REV, 1.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_GEAR_RATIO, 100.0f), 1),
            (int)ODOM_LEFT_TICK_SIGN,
            (int)ODOM_RIGHT_TICK_SIGN,
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(APP_ODO_MAX_LINEAR_STEP_M, 1000.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(APP_ODO_MAX_ANGULAR_STEP_RAD, 1000.0f), 1),
            (unsigned int)APP_ODO_MIN_DT_MS,
            (unsigned int)APP_ODO_MAX_DT_MS);
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
    uint8_t step_skipped = 0U;
    uint8_t skip_reason = APP_ODO_SKIP_NONE;
    uint32_t now_ms = HAL_GetTick();
    int32_t dt_ms_signed;
    uint32_t dt_ms;

    if (odom_initialized == 0U)
    {
        Odom_Init();
    }

    dt_ms_signed = Odom_ScaleFloatRounded(dt_s, 1000.0f);
    dt_ms = (dt_ms_signed > 0) ? (uint32_t)dt_ms_signed : 0U;

    current_left = MotorDriver_GetEncoderA();
    current_right = MotorDriver_GetEncoderB();
    raw_left_delta = Odom_LeftDelta(current_left, odom_prev_left);
    raw_right_delta = Odom_RightDelta(current_right, odom_prev_right);
    odom_prev_left = current_left;
    odom_prev_right = current_right;

    signed_left_delta = raw_left_delta * ODOM_LEFT_TICK_SIGN;
    signed_right_delta = raw_right_delta * ODOM_RIGHT_TICK_SIGN;
    dl = Odom_TicksToMeters(signed_left_delta);
    dr = Odom_TicksToMeters(signed_right_delta);
    dc = (dl + dr) * 0.5f;
    dtheta = (dr - dl) / ODO_WHEEL_BASE_M;
    odom_debug_total_raw_left += raw_left_delta;
    odom_debug_total_raw_right += raw_right_delta;
    odom_debug_total_signed_left += signed_left_delta;
    odom_debug_total_signed_right += signed_right_delta;

    if (dt_ms > APP_ODO_MAX_DT_MS)
    {
        skip_reason = APP_ODO_SKIP_DT_TOO_LARGE;
        Odom_LogDtSkip(now_ms, dt_ms, skip_reason);
    }
    else if (dt_ms < APP_ODO_MIN_DT_MS)
    {
        skip_reason = APP_ODO_SKIP_DT_TOO_SMALL;
        Odom_LogDtSkip(now_ms, dt_ms, skip_reason);
    }
    else if ((Odom_AbsFloat(dc) > APP_ODO_MAX_LINEAR_STEP_M) ||
             (Odom_AbsFloat(dtheta) > APP_ODO_MAX_ANGULAR_STEP_RAD))
    {
        skip_reason = APP_ODO_SKIP_LARGE_STEP;
        Odom_LogLargeStep(now_ms, dc, dtheta, dt_s);
    }
    else if (odom_freeze != 0U)
    {
        skip_reason = APP_ODO_SKIP_FROZEN;
    }

    step_skipped = (skip_reason == APP_ODO_SKIP_NONE) ? 0U : 1U;

    if (skip_reason == APP_ODO_SKIP_NONE)
    {
        theta_mid = odom_pose.theta_rad + (dtheta * 0.5f);
        odom_pose.x_m += dc * cosf(theta_mid);
        odom_pose.y_m += dc * sinf(theta_mid);
        odom_pose.theta_rad = Odom_NormalizeAngle(odom_pose.theta_rad + dtheta);
    }
    else
    {
        dc = 0.0f;
        dtheta = 0.0f;
    }

    if ((skip_reason == APP_ODO_SKIP_NONE) && (dt_s > 0.0001f))
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
    odom_last_sample.total_raw_left_delta = odom_debug_total_raw_left;
    odom_last_sample.total_raw_right_delta = odom_debug_total_raw_right;
    odom_last_sample.total_signed_left_delta = odom_debug_total_signed_left;
    odom_last_sample.total_signed_right_delta = odom_debug_total_signed_right;
    odom_last_sample.delta_left_m = dl;
    odom_last_sample.delta_right_m = dr;
    odom_last_sample.delta_center_m = (dl + dr) * 0.5f;
    odom_last_sample.delta_theta_rad = (dr - dl) / ODO_WHEEL_BASE_M;
    odom_last_sample.total_delta_left_m = Odom_TicksToMeters(odom_debug_total_signed_left);
    odom_last_sample.total_delta_right_m = Odom_TicksToMeters(odom_debug_total_signed_right);
    odom_last_sample.total_delta_center_m = (odom_last_sample.total_delta_left_m + odom_last_sample.total_delta_right_m) * 0.5f;
    odom_last_sample.total_delta_theta_rad = (odom_last_sample.total_delta_right_m - odom_last_sample.total_delta_left_m) / ODO_WHEEL_BASE_M;
    odom_last_sample.dt_s = dt_s;
    odom_last_sample.last_update_ms = now_ms;
    odom_last_sample.step_skipped = step_skipped;
    odom_last_sample.skip_reason = skip_reason;
    odom_last_sample.frozen = odom_freeze;
    odom_last_update_ms = now_ms;

#if APP_ODO_AUTO_PRINT_ENABLE
    if ((now_ms - odom_last_log_ms) >= ODO_LOG_INTERVAL_MS)
    {
        Odom_Log(now_ms);
        odom_last_log_ms = now_ms;
    }
#else
    (void)now_ms;
#endif
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
    odom_last_sample.total_raw_left_delta = 0;
    odom_last_sample.total_raw_right_delta = 0;
    odom_last_sample.total_signed_left_delta = 0;
    odom_last_sample.total_signed_right_delta = 0;
    odom_last_sample.delta_left_m = 0.0f;
    odom_last_sample.delta_right_m = 0.0f;
    odom_last_sample.delta_center_m = 0.0f;
    odom_last_sample.delta_theta_rad = 0.0f;
    odom_last_sample.total_delta_left_m = 0.0f;
    odom_last_sample.total_delta_right_m = 0.0f;
    odom_last_sample.total_delta_center_m = 0.0f;
    odom_last_sample.total_delta_theta_rad = 0.0f;
    odom_last_sample.dt_s = 0.0f;
    odom_last_sample.last_update_ms = HAL_GetTick();
    odom_last_sample.step_skipped = 0U;
    odom_last_sample.skip_reason = APP_ODO_SKIP_NONE;
    odom_last_sample.frozen = odom_freeze;
    odom_last_update_ms = odom_last_sample.last_update_ms;
#if APP_ODO_AUTO_PRINT_ENABLE
    odom_last_log_ms = odom_last_sample.last_update_ms;
#endif
    odom_last_warn_ms = 0U;
    odom_initialized = 1U;
    Odom_ClearDebugTotals();

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

void AppOdo_SyncBaseline(void)
{
    uint32_t now_ms;

    if (odom_initialized == 0U)
    {
        Odom_Init();
        return;
    }

    now_ms = HAL_GetTick();
    odom_prev_left = MotorDriver_GetEncoderA();
    odom_prev_right = MotorDriver_GetEncoderB();
    odom_last_update_ms = now_ms;
    odom_linear_velocity_mps = 0.0f;
    odom_angular_velocity_radps = 0.0f;

    odom_last_sample.raw_left_delta = 0;
    odom_last_sample.raw_right_delta = 0;
    odom_last_sample.signed_left_delta = 0;
    odom_last_sample.signed_right_delta = 0;
    odom_last_sample.delta_left_m = 0.0f;
    odom_last_sample.delta_right_m = 0.0f;
    odom_last_sample.delta_center_m = 0.0f;
    odom_last_sample.delta_theta_rad = 0.0f;
    odom_last_sample.dt_s = 0.0f;
    odom_last_sample.last_update_ms = now_ms;
    odom_last_sample.step_skipped = 0U;
    odom_last_sample.skip_reason = APP_ODO_SKIP_NONE;
    odom_last_sample.frozen = odom_freeze;
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

void AppOdo_PrintDebug(void)
{
    OdomSample_t sample;
    int32_t current_left;
    int32_t current_right;
    int32_t dl_mm;
    int32_t dr_mm;
    int32_t ds_mm;
    int32_t dtheta_mrad;
    int32_t total_dl_mm;
    int32_t total_dr_mm;
    int32_t total_ds_mm;
    int32_t total_dtheta_mrad;
    int32_t dt_ms;
    int32_t v_mmps = 0;
    int32_t w_mradps = 0;

    if (Odom_GetLastSample(&sample) == false)
    {
        APP_LOG("ODO_DBG: unavailable");
        return;
    }

    dl_mm = Odom_ScaleFloatRounded(sample.delta_left_m, 1000.0f);
    dr_mm = Odom_ScaleFloatRounded(sample.delta_right_m, 1000.0f);
    ds_mm = Odom_ScaleFloatRounded(sample.delta_center_m, 1000.0f);
    dtheta_mrad = Odom_ScaleFloatRounded(sample.delta_theta_rad, 1000.0f);
    total_dl_mm = Odom_ScaleFloatRounded(sample.total_delta_left_m, 1000.0f);
    total_dr_mm = Odom_ScaleFloatRounded(sample.total_delta_right_m, 1000.0f);
    total_ds_mm = Odom_ScaleFloatRounded(sample.total_delta_center_m, 1000.0f);
    total_dtheta_mrad = Odom_ScaleFloatRounded(sample.total_delta_theta_rad, 1000.0f);
    dt_ms = Odom_ScaleFloatRounded(sample.dt_s, 1000.0f);
    current_left = MotorDriver_GetEncoderA();
    current_right = MotorDriver_GetEncoderB();

    if ((sample.skip_reason == APP_ODO_SKIP_NONE) && (sample.dt_s > 0.0001f))
    {
        v_mmps = Odom_ScaleFloatRounded(sample.delta_center_m / sample.dt_s, 1000.0f);
        w_mradps = Odom_ScaleFloatRounded(sample.delta_theta_rad / sample.dt_s, 1000.0f);
    }

    APP_LOG("ODO_DBG: frozen=%u cntL=%ld cntR=%ld",
            (unsigned int)sample.frozen,
            (long)current_left,
            (long)current_right);
    APP_LOG("ODO_DBG: last rawL=%ld rawR=%ld signedL=%ld signedR=%ld dl_mm=%s%lu dr_mm=%s%lu ds_mm=%s%lu dth_mrad=%s%lu dt_ms=%lu skipped=%u reason=%s",
            (long)sample.raw_left_delta,
            (long)sample.raw_right_delta,
            (long)sample.signed_left_delta,
            (long)sample.signed_right_delta,
            Odom_FixedSign(dl_mm),
            (unsigned long)Odom_FixedWhole(dl_mm, 1),
            Odom_FixedSign(dr_mm),
            (unsigned long)Odom_FixedWhole(dr_mm, 1),
            Odom_FixedSign(ds_mm),
            (unsigned long)Odom_FixedWhole(ds_mm, 1),
            Odom_FixedSign(dtheta_mrad),
            (unsigned long)Odom_FixedWhole(dtheta_mrad, 1),
            (unsigned long)Odom_FixedWhole(dt_ms, 1),
            (unsigned int)sample.step_skipped,
            AppOdo_SkipReasonName(sample.skip_reason));
    APP_LOG("ODO_DBG: total rawL=%ld rawR=%ld signedL=%ld signedR=%ld",
            (long)sample.total_raw_left_delta,
            (long)sample.total_raw_right_delta,
            (long)sample.total_signed_left_delta,
            (long)sample.total_signed_right_delta);
    APP_LOG("ODO_DBG: total dl_mm=%s%lu dr_mm=%s%lu ds_mm=%s%lu dth_mrad=%s%lu v_mmps=%s%lu w_mradps=%s%lu",
            Odom_FixedSign(total_dl_mm),
            (unsigned long)Odom_FixedWhole(total_dl_mm, 1),
            Odom_FixedSign(total_dr_mm),
            (unsigned long)Odom_FixedWhole(total_dr_mm, 1),
            Odom_FixedSign(total_ds_mm),
            (unsigned long)Odom_FixedWhole(total_ds_mm, 1),
            Odom_FixedSign(total_dtheta_mrad),
            (unsigned long)Odom_FixedWhole(total_dtheta_mrad, 1),
            Odom_FixedSign(v_mmps),
            (unsigned long)Odom_FixedWhole(v_mmps, 1),
            Odom_FixedSign(w_mradps),
            (unsigned long)Odom_FixedWhole(w_mradps, 1));
    APP_LOG("ODO_DBG: signL=%d signR=%d radius_mm=%lu base_mm=%lu ticks_rev=%lu gear_x100=%lu max_step_mm=%lu max_dth_mrad=%lu dt_ms=%u..%u",
            (int)ODOM_LEFT_TICK_SIGN,
            (int)ODOM_RIGHT_TICK_SIGN,
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_WHEEL_RADIUS_M, 1000.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_WHEEL_BASE_M, 1000.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_ENCODER_TICKS_PER_REV, 1.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(ODO_GEAR_RATIO, 100.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(APP_ODO_MAX_LINEAR_STEP_M, 1000.0f), 1),
            (unsigned long)Odom_FixedWhole(Odom_ScaleFloatRounded(APP_ODO_MAX_ANGULAR_STEP_RAD, 1000.0f), 1),
            (unsigned int)APP_ODO_MIN_DT_MS,
            (unsigned int)APP_ODO_MAX_DT_MS);
}

void AppOdo_SetFreeze(uint8_t freeze)
{
    uint8_t next = (freeze != 0U) ? 1U : 0U;

    if (odom_freeze == next)
    {
        if (next != 0U)
        {
            AppOdo_SyncBaseline();
            Odom_ClearDebugTotals();
        }
        APP_LOG("ODO: freeze=%u", (unsigned int)odom_freeze);
        return;
    }

    odom_freeze = next;
    if (odom_freeze != 0U)
    {
        AppOdo_SyncBaseline();
        Odom_ClearDebugTotals();
    }
    else
    {
        AppOdo_SyncBaseline();
    }
    odom_linear_velocity_mps = 0.0f;
    odom_angular_velocity_radps = 0.0f;
    odom_last_sample.frozen = odom_freeze;
    APP_LOG("ODO: freeze=%u", (unsigned int)odom_freeze);
}

uint8_t AppOdo_IsFrozen(void)
{
    return odom_freeze;
}

const char *AppOdo_SkipReasonName(uint8_t reason)
{
    switch (reason)
    {
        case APP_ODO_SKIP_NONE:
            return "NONE";

        case APP_ODO_SKIP_LARGE_STEP:
            return "LARGE_STEP";

        case APP_ODO_SKIP_DT_TOO_LARGE:
            return "DT_TOO_LARGE";

        case APP_ODO_SKIP_DT_TOO_SMALL:
            return "DT_TOO_SMALL";

        case APP_ODO_SKIP_FROZEN:
            return "FROZEN";

        default:
            return "UNKNOWN";
    }
}

static void Odom_ClearDebugTotals(void)
{
    odom_debug_total_raw_left = 0;
    odom_debug_total_raw_right = 0;
    odom_debug_total_signed_left = 0;
    odom_debug_total_signed_right = 0;
    odom_last_sample.total_raw_left_delta = 0;
    odom_last_sample.total_raw_right_delta = 0;
    odom_last_sample.total_signed_left_delta = 0;
    odom_last_sample.total_signed_right_delta = 0;
    odom_last_sample.total_delta_left_m = 0.0f;
    odom_last_sample.total_delta_right_m = 0.0f;
    odom_last_sample.total_delta_center_m = 0.0f;
    odom_last_sample.total_delta_theta_rad = 0.0f;
}

static int32_t Odom_LeftDelta(int32_t current, int32_t previous)
{
    /*
     * Encoder A is TIM2 on NUCLEO-F446RE, a 32-bit timer in this project.
     * During calibration moves it behaves as a software-visible cumulative count.
     */
    return current - previous;
}

static int32_t Odom_RightDelta(int32_t current, int32_t previous)
{
    /*
     * Encoder B is read through a signed 16-bit TIM4 counter. Cast the
     * difference back to int16_t so wrap across +/-32768 produces a small delta.
     */
    return (int32_t)((int16_t)((int16_t)current - (int16_t)previous));
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

static float Odom_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
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

#if APP_ODO_AUTO_PRINT_ENABLE
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
#endif

static void Odom_LogLargeStep(uint32_t now_ms, float ds_m, float dtheta_rad, float dt_s)
{
    int32_t ds_mm;
    int32_t dtheta_mrad;
    int32_t dt_ms;

    if ((odom_last_warn_ms != 0U) && ((now_ms - odom_last_warn_ms) < ODO_WARN_INTERVAL_MS))
    {
        return;
    }

    odom_last_warn_ms = now_ms;
    ds_mm = Odom_ScaleFloatRounded(ds_m, 1000.0f);
    dtheta_mrad = Odom_ScaleFloatRounded(dtheta_rad, 1000.0f);
    dt_ms = Odom_ScaleFloatRounded(dt_s, 1000.0f);

    APP_LOG("ODO: warn large step ds=%s%lu.%03lu dth=%s%lu.%03lu dt=%lums skipped",
            Odom_FixedSign(ds_mm),
            (unsigned long)Odom_FixedWhole(ds_mm, 1000),
            (unsigned long)Odom_FixedFraction(ds_mm, 1000),
            Odom_FixedSign(dtheta_mrad),
            (unsigned long)Odom_FixedWhole(dtheta_mrad, 1000),
            (unsigned long)Odom_FixedFraction(dtheta_mrad, 1000),
            (unsigned long)Odom_FixedWhole(dt_ms, 1));
}

static void Odom_LogDtSkip(uint32_t now_ms, uint32_t dt_ms, uint8_t reason)
{
    if ((odom_last_warn_ms != 0U) && ((now_ms - odom_last_warn_ms) < ODO_WARN_INTERVAL_MS))
    {
        return;
    }

    odom_last_warn_ms = now_ms;

    if (reason == APP_ODO_SKIP_DT_TOO_LARGE)
    {
        APP_LOG("ODO: warn dt too large dt=%lums skipped integration",
                (unsigned long)dt_ms);
    }
    else if (reason == APP_ODO_SKIP_DT_TOO_SMALL)
    {
        APP_LOG("ODO: warn dt too small dt=%lums skipped integration",
                (unsigned long)dt_ms);
    }
}
