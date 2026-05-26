#include "app_safety.h"

#include "amr_system.h"
#include "app_lidar.h"
#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"
#include "stm32f4xx_hal.h"

#define APP_SAFETY_LIDAR_RX_TIMEOUT_MS 1500U
#define APP_SAFETY_LIDAR_VALID_WARN_MS 2000U
#define APP_SAFETY_LIDAR_WARN_INTERVAL_MS 1000U
#if APP_DEBUG_VERBOSE
#define APP_SAFETY_HEARTBEAT_MS 1000U
#else
#define APP_SAFETY_HEARTBEAT_MS 5000U
#endif
#define APP_SAFETY_STALL_PWM_THRESHOLD 250
#define APP_SAFETY_STALL_DELTA_THRESHOLD 2
#define APP_SAFETY_STALL_TIME_MS 1000U

static AppSafetyStatus_t app_safety_status = {APP_FAULT_NONE, 0U, 0U, 0, 0, 0, 0};
static uint32_t app_safety_stall_start_ms = 0U;
static uint32_t app_safety_lidar_valid_warn_last_ms = 0U;
static uint32_t app_safety_last_heartbeat_ms = 0U;
static int32_t app_safety_prev_encoder_left = 0;
static int32_t app_safety_prev_encoder_right = 0;
static uint8_t app_safety_encoder_sample_valid = 0U;
static uint8_t app_safety_time_skew_warned = 0U;

static void App_Safety_SampleMotion(ChassisCommandStatus_t *command,
                                    int32_t *raw_left_delta,
                                    int32_t *raw_right_delta);
static void App_Safety_LogHeartbeat(uint32_t now_ms,
                                    AMR_State_t state,
                                    const ChassisCommandStatus_t *command,
                                    int32_t raw_left_delta,
                                    int32_t raw_right_delta);
static void App_Safety_CheckLidarTimeout(uint32_t now_ms, AMR_State_t state);
static void App_Safety_CheckEncoderStall(uint32_t now_ms,
                                         AMR_State_t state,
                                         const ChassisCommandStatus_t *command,
                                         int32_t raw_left_delta,
                                         int32_t raw_right_delta);
static uint32_t App_Safety_ElapsedMs(uint32_t now_ms, uint32_t then_ms);
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
    app_safety_lidar_valid_warn_last_ms = 0U;
    app_safety_last_heartbeat_ms = 0U;
    app_safety_prev_encoder_left = MotorDriver_GetEncoderA();
    app_safety_prev_encoder_right = MotorDriver_GetEncoderB();
    app_safety_encoder_sample_valid = 1U;
    app_safety_time_skew_warned = 0U;

    APP_LOG("[SAFETY] init lidar_rx_timeout_ms=%u lidar_valid_warn_ms=%u stall_pwm=%d stall_delta=%d stall_ms=%u",
            (unsigned int)APP_SAFETY_LIDAR_RX_TIMEOUT_MS,
            (unsigned int)APP_SAFETY_LIDAR_VALID_WARN_MS,
            APP_SAFETY_STALL_PWM_THRESHOLD,
            APP_SAFETY_STALL_DELTA_THRESHOLD,
            (unsigned int)APP_SAFETY_STALL_TIME_MS);
}

void App_Safety_Update(void)
{
    uint32_t now_ms = HAL_GetTick();
    AMR_State_t state = AMR_GetState();
    ChassisCommandStatus_t command = {0, 0, 0U};
    int32_t raw_left_delta = 0;
    int32_t raw_right_delta = 0;

    App_Safety_SampleMotion(&command, &raw_left_delta, &raw_right_delta);
    App_Safety_LogHeartbeat(now_ms, state, &command, raw_left_delta, raw_right_delta);

    if ((state == AMR_STATE_FAULT) || (state == AMR_STATE_ESTOP))
    {
        return;
    }

    App_Safety_CheckLidarTimeout(now_ms, state);

    if (AMR_GetState() == AMR_STATE_FAULT)
    {
        return;
    }

    App_Safety_CheckEncoderStall(now_ms, state, &command, raw_left_delta, raw_right_delta);
}

void App_Safety_ClearFault(void)
{
    app_safety_status.fault_code = APP_FAULT_NONE;
    app_safety_status.fault_time_ms = 0U;
    app_safety_stall_start_ms = 0U;
    app_safety_lidar_valid_warn_last_ms = 0U;
    app_safety_last_heartbeat_ms = 0U;
    app_safety_prev_encoder_left = MotorDriver_GetEncoderA();
    app_safety_prev_encoder_right = MotorDriver_GetEncoderB();
    app_safety_encoder_sample_valid = 1U;
    app_safety_time_skew_warned = 0U;

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

static void App_Safety_SampleMotion(ChassisCommandStatus_t *command,
                                    int32_t *raw_left_delta,
                                    int32_t *raw_right_delta)
{
    int32_t current_left = MotorDriver_GetEncoderA();
    int32_t current_right = MotorDriver_GetEncoderB();

    if (command != NULL)
    {
        Chassis_GetLastCommand(command);
        app_safety_status.pwm_left = command->left_duty;
        app_safety_status.pwm_right = command->right_duty;
    }

    if (app_safety_encoder_sample_valid == 0U)
    {
        app_safety_prev_encoder_left = current_left;
        app_safety_prev_encoder_right = current_right;
        app_safety_encoder_sample_valid = 1U;
    }

    if (raw_left_delta != NULL)
    {
        *raw_left_delta = current_left - app_safety_prev_encoder_left;
        app_safety_status.raw_left_delta = *raw_left_delta;
    }

    if (raw_right_delta != NULL)
    {
        *raw_right_delta = current_right - app_safety_prev_encoder_right;
        app_safety_status.raw_right_delta = *raw_right_delta;
    }

    app_safety_prev_encoder_left = current_left;
    app_safety_prev_encoder_right = current_right;
}

static void App_Safety_LogHeartbeat(uint32_t now_ms,
                                    AMR_State_t state,
                                    const ChassisCommandStatus_t *command,
                                    int32_t raw_left_delta,
                                    int32_t raw_right_delta)
{
    const AppLidarStatus *lidar;
    uint32_t last_rx_tick_ms;
    uint32_t last_valid_update_ms;
    uint32_t rx_age_ms;
    uint32_t valid_age_ms;

    if (App_Safety_StateNeedsMotionSafety(state) == 0U)
    {
        app_safety_last_heartbeat_ms = 0U;
        return;
    }

    if ((app_safety_last_heartbeat_ms != 0U) &&
        (App_Safety_ElapsedMs(now_ms, app_safety_last_heartbeat_ms) < APP_SAFETY_HEARTBEAT_MS))
    {
        return;
    }

    lidar = App_Lidar_GetStatus();
    last_rx_tick_ms = (lidar != NULL) ? lidar->last_rx_tick_ms : 0U;
    last_valid_update_ms = (lidar != NULL) ? lidar->last_valid_update_ms : 0U;
    rx_age_ms = App_Safety_ElapsedMs(now_ms, last_rx_tick_ms);
    valid_age_ms = App_Safety_ElapsedMs(now_ms, last_valid_update_ms);
    app_safety_last_heartbeat_ms = now_ms;

    APP_LOG("[SAFETY] state=%s rx_age=%lu valid_age=%lu pwmL=%d pwmR=%d rawL=%ld rawR=%ld",
            AMR_StateName(state),
            (unsigned long)rx_age_ms,
            (unsigned long)valid_age_ms,
            (int)((command != NULL) ? command->left_duty : 0),
            (int)((command != NULL) ? command->right_duty : 0),
            (long)raw_left_delta,
            (long)raw_right_delta);
}

static void App_Safety_CheckLidarTimeout(uint32_t now_ms, AMR_State_t state)
{
    const AppLidarStatus *lidar;
    uint32_t last_rx_tick_ms;
    uint32_t last_valid_update_ms;
    uint32_t rx_age_ms;
    uint32_t valid_age_ms;

    if (App_Safety_StateNeedsMotionSafety(state) == 0U)
    {
        return;
    }

    lidar = App_Lidar_GetStatus();
    if (lidar == NULL)
    {
        return;
    }

    last_rx_tick_ms = lidar->last_rx_tick_ms;
    last_valid_update_ms = lidar->last_valid_update_ms;
    rx_age_ms = App_Safety_ElapsedMs(now_ms, last_rx_tick_ms);
    valid_age_ms = App_Safety_ElapsedMs(now_ms, last_valid_update_ms);
    app_safety_status.last_lidar_update_ms = last_valid_update_ms;

    if ((last_rx_tick_ms != 0U) && (now_ms < last_rx_tick_ms))
    {
        if (app_safety_time_skew_warned == 0U)
        {
            app_safety_time_skew_warned = 1U;
            APP_LOG("[WARN] time_skew now=%lu last_rx=%lu",
                    (unsigned long)now_ms,
                    (unsigned long)last_rx_tick_ms);
        }

        return;
    }

    if (rx_age_ms > APP_SAFETY_LIDAR_RX_TIMEOUT_MS)
    {
        APP_LOG("[FAULT] LIDAR_TIMEOUT_RX rx_age=%lu ms valid_age=%lu ms last_rx=%lu last_valid=%lu now=%lu",
                (unsigned long)rx_age_ms,
                (unsigned long)valid_age_ms,
                (unsigned long)last_rx_tick_ms,
                (unsigned long)last_valid_update_ms,
                (unsigned long)now_ms);
        App_Safety_EnterFault(APP_FAULT_LIDAR_TIMEOUT, "lidar_timeout_rx");
        return;
    }

    if (valid_age_ms > APP_SAFETY_LIDAR_VALID_WARN_MS)
    {
        if ((app_safety_lidar_valid_warn_last_ms == 0U) ||
            (App_Safety_ElapsedMs(now_ms, app_safety_lidar_valid_warn_last_ms) >= APP_SAFETY_LIDAR_WARN_INTERVAL_MS))
        {
            app_safety_lidar_valid_warn_last_ms = now_ms;
            APP_LOG("[WARN] LIDAR_VALID_STALE rx_age=%lu ms valid_age=%lu ms last_rx=%lu last_valid=%lu now=%lu ready=%u",
                    (unsigned long)rx_age_ms,
                    (unsigned long)valid_age_ms,
                    (unsigned long)last_rx_tick_ms,
                    (unsigned long)last_valid_update_ms,
                    (unsigned long)now_ms,
                    (unsigned int)lidar->ready);
        }
    }
    else
    {
        app_safety_lidar_valid_warn_last_ms = 0U;
    }
}

static void App_Safety_CheckEncoderStall(uint32_t now_ms,
                                         AMR_State_t state,
                                         const ChassisCommandStatus_t *command,
                                         int32_t raw_left_delta,
                                         int32_t raw_right_delta)
{
    uint8_t command_active;
    uint8_t encoder_still;

    if (App_Safety_StateNeedsMotionSafety(state) == 0U)
    {
        app_safety_stall_start_ms = 0U;
        return;
    }

    if (command == NULL)
    {
        app_safety_stall_start_ms = 0U;
        return;
    }

    command_active =
        ((App_Safety_AbsI16(command->left_duty) >= APP_SAFETY_STALL_PWM_THRESHOLD) ||
         (App_Safety_AbsI16(command->right_duty) >= APP_SAFETY_STALL_PWM_THRESHOLD)) ? 1U : 0U;
    encoder_still =
        ((App_Safety_AbsI32(raw_left_delta) <= APP_SAFETY_STALL_DELTA_THRESHOLD) &&
         (App_Safety_AbsI32(raw_right_delta) <= APP_SAFETY_STALL_DELTA_THRESHOLD)) ? 1U : 0U;

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

    if (App_Safety_ElapsedMs(now_ms, app_safety_stall_start_ms) >= APP_SAFETY_STALL_TIME_MS)
    {
        APP_LOG("[FAULT] ENCODER_STALL pwmL=%d pwmR=%d rawL=%ld rawR=%ld",
                (int)command->left_duty,
                (int)command->right_duty,
                (long)raw_left_delta,
                (long)raw_right_delta);
        App_Safety_EnterFault(APP_FAULT_ENCODER_STALL, "encoder_stall");
    }
}

static uint8_t App_Safety_StateNeedsMotionSafety(AMR_State_t state)
{
    return ((state == AMR_STATE_EXPLORE) ||
            (state == AMR_STATE_AVOID) ||
            (state == AMR_STATE_RETURN)) ? 1U : 0U;
}

static uint32_t App_Safety_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    if (then_ms == 0U)
    {
        return UINT32_MAX;
    }

    if (now_ms >= then_ms)
    {
        return now_ms - then_ms;
    }

    return 0U;
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
