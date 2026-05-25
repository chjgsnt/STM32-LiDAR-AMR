#include "app_safety.h"

#include "amr_system.h"
#include "app_lidar.h"
#include "app_odometry.h"
#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"
#include "stm32f4xx_hal.h"

#define APP_SAFETY_LIDAR_TIMEOUT_MS 500U
#define APP_SAFETY_LIDAR_START_GRACE_MS 1000U
#define APP_SAFETY_STALL_PWM_THRESHOLD 250
#define APP_SAFETY_STALL_DELTA_THRESHOLD 2
#define APP_SAFETY_STALL_TIME_MS 1000U
#define APP_SAFETY_ODOM_SAMPLE_MAX_AGE_MS 250U

static AppSafetyStatus_t app_safety_status = {APP_FAULT_NONE, 0U, 0U, 0, 0, 0, 0};
static uint32_t app_safety_stall_start_ms = 0U;

static void App_Safety_CheckLidarTimeout(uint32_t now_ms, AMR_State_t state);
static void App_Safety_CheckEncoderStall(uint32_t now_ms, AMR_State_t state);
static uint8_t App_Safety_StateNeedsMotionSafety(AMR_State_t state);
static int32_t App_Safety_AbsI32(int32_t value);
static int16_t App_Safety_AbsI16(int16_t value);
static void App_Safety_EnterFault(AppFaultCode_t code, const char *reason);
static void App_Safety_StopOutputs(void);

void App_Safety_Init(void)
{
    app_safety_status.fault_code = APP_FAULT_NONE;
    app_safety_status.fault_time_ms = 0U;
    app_safety_status.last_lidar_update_ms = 0U;
    app_safety_status.pwm_left = 0;
    app_safety_status.pwm_right = 0;
    app_safety_status.raw_left_delta = 0;
    app_safety_status.raw_right_delta = 0;
    app_safety_stall_start_ms = 0U;

    APP_LOG("[SAFETY] init lidar_timeout_ms=%u stall_pwm=%d stall_delta=%d stall_ms=%u",
            (unsigned int)APP_SAFETY_LIDAR_TIMEOUT_MS,
            APP_SAFETY_STALL_PWM_THRESHOLD,
            APP_SAFETY_STALL_DELTA_THRESHOLD,
            (unsigned int)APP_SAFETY_STALL_TIME_MS);
}

void App_Safety_Update(void)
{
    uint32_t now_ms = HAL_GetTick();
    AMR_State_t state = AMR_GetState();

    if ((state == AMR_STATE_FAULT) || (state == AMR_STATE_ESTOP))
    {
        return;
    }

    App_Safety_CheckLidarTimeout(now_ms, state);

    if (AMR_GetState() == AMR_STATE_FAULT)
    {
        return;
    }

    App_Safety_CheckEncoderStall(now_ms, state);
}

void App_Safety_ClearFault(void)
{
    app_safety_status.fault_code = APP_FAULT_NONE;
    app_safety_status.fault_time_ms = 0U;
    app_safety_stall_start_ms = 0U;

    APP_LOG("[SAFETY] fault cleared");
}

bool App_Safety_GetStatus(AppSafetyStatus_t *status)
{
    if (status == NULL)
    {
        return false;
    }

    *status = app_safety_status;

    return true;
}

const char *App_Safety_FaultName(AppFaultCode_t code)
{
    switch (code)
    {
        case APP_FAULT_NONE:
            return "NONE";

        case APP_FAULT_LIDAR_TIMEOUT:
            return "LIDAR_TIMEOUT";

        case APP_FAULT_ENCODER_STALL:
            return "ENCODER_STALL";

        default:
            return "UNKNOWN";
    }
}

static void App_Safety_CheckLidarTimeout(uint32_t now_ms, AMR_State_t state)
{
    const AppLidarStatus *lidar;
    uint32_t last_update_ms;
    uint32_t lidar_age_ms;
    uint32_t state_age_ms;

    if (App_Safety_StateNeedsMotionSafety(state) == 0U)
    {
        return;
    }

    lidar = App_Lidar_GetStatus();
    if (lidar == NULL)
    {
        return;
    }

    last_update_ms = lidar->last_update_ms;
    lidar_age_ms = now_ms - last_update_ms;
    app_safety_status.last_lidar_update_ms = last_update_ms;
    state_age_ms = now_ms - AMR_GetStateEnterMs();

    if (state_age_ms < APP_SAFETY_LIDAR_START_GRACE_MS)
    {
        return;
    }

    if ((lidar->ready != 0U) &&
        (last_update_ms != 0U) &&
        (lidar_age_ms > APP_SAFETY_LIDAR_TIMEOUT_MS))
    {
        APP_LOG("[FAULT] LIDAR_TIMEOUT age=%lu ms last_update=%lu now=%lu",
                (unsigned long)lidar_age_ms,
                (unsigned long)last_update_ms,
                (unsigned long)now_ms);
        App_Safety_EnterFault(APP_FAULT_LIDAR_TIMEOUT, "lidar_timeout");
    }
    else if ((lidar->ready == 0U) || (last_update_ms == 0U))
    {
        APP_LOG("[FAULT] LIDAR_TIMEOUT age=%lu ms last_update=%lu now=%lu",
                (unsigned long)lidar_age_ms,
                (unsigned long)last_update_ms,
                (unsigned long)now_ms);
        App_Safety_EnterFault(APP_FAULT_LIDAR_TIMEOUT, "lidar_timeout");
    }
}

static void App_Safety_CheckEncoderStall(uint32_t now_ms, AMR_State_t state)
{
    ChassisCommandStatus_t command;
    OdomSample_t sample;
    uint8_t command_active;
    uint8_t encoder_still;

    if (App_Safety_StateNeedsMotionSafety(state) == 0U)
    {
        app_safety_stall_start_ms = 0U;
        return;
    }

    Chassis_GetLastCommand(&command);
    if (Odom_GetLastSample(&sample) == false)
    {
        app_safety_stall_start_ms = 0U;
        return;
    }

    app_safety_status.pwm_left = command.left_duty;
    app_safety_status.pwm_right = command.right_duty;
    app_safety_status.raw_left_delta = sample.raw_left_delta;
    app_safety_status.raw_right_delta = sample.raw_right_delta;

    if ((now_ms - sample.last_update_ms) > APP_SAFETY_ODOM_SAMPLE_MAX_AGE_MS)
    {
        app_safety_stall_start_ms = 0U;
        return;
    }

    command_active =
        ((App_Safety_AbsI16(command.left_duty) >= APP_SAFETY_STALL_PWM_THRESHOLD) ||
         (App_Safety_AbsI16(command.right_duty) >= APP_SAFETY_STALL_PWM_THRESHOLD)) ? 1U : 0U;
    encoder_still =
        ((App_Safety_AbsI32(sample.raw_left_delta) <= APP_SAFETY_STALL_DELTA_THRESHOLD) &&
         (App_Safety_AbsI32(sample.raw_right_delta) <= APP_SAFETY_STALL_DELTA_THRESHOLD)) ? 1U : 0U;

    if ((command_active == 0U) || (encoder_still == 0U))
    {
        app_safety_stall_start_ms = 0U;
        return;
    }

    if (app_safety_stall_start_ms == 0U)
    {
        app_safety_stall_start_ms = now_ms;
        return;
    }

    if ((now_ms - app_safety_stall_start_ms) >= APP_SAFETY_STALL_TIME_MS)
    {
        APP_LOG("[FAULT] ENCODER_STALL pwmL=%d pwmR=%d rawL=%ld rawR=%ld",
                (int)command.left_duty,
                (int)command.right_duty,
                (long)sample.raw_left_delta,
                (long)sample.raw_right_delta);
        App_Safety_EnterFault(APP_FAULT_ENCODER_STALL, "encoder_stall");
    }
}

static uint8_t App_Safety_StateNeedsMotionSafety(AMR_State_t state)
{
    return ((state == AMR_STATE_EXPLORE) ||
            (state == AMR_STATE_AVOID) ||
            (state == AMR_STATE_RETURN)) ? 1U : 0U;
}

static int32_t App_Safety_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int16_t App_Safety_AbsI16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static void App_Safety_EnterFault(AppFaultCode_t code, const char *reason)
{
    app_safety_status.fault_code = code;
    app_safety_status.fault_time_ms = HAL_GetTick();
    App_Safety_StopOutputs();
    (void)AMR_SetState(AMR_STATE_FAULT, reason);
}

static void App_Safety_StopOutputs(void)
{
    Chassis_Stop();
    MotorDriver_StopAll();
}
