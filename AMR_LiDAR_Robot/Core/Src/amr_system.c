#include "amr_system.h"

#include "app_lidar.h"
#include "app_lidar_obstacle_avoidance.h"
#include "app_odometry.h"
#include "app_return_path.h"
#include "bringup_log.h"
#include "chassis.h"
#include "main.h"
#include "motor_driver.h"

#define AMR_ENABLE_LEGACY_START_BUTTON 0
#if AMR_ENABLE_LEGACY_START_BUTTON
#define AMR_START_BUTTON_DEBOUNCE_MS 250U
#endif
#define AMR_AVOID_ENTER_FRONT_MM 400U
#define AMR_AVOID_EXIT_FRONT_MM 520U

static volatile AMR_State_t amr_state = AMR_STATE_IDLE;
static uint32_t amr_state_enter_ms = 0U;
static uint8_t amr_initialized = 0U;
#if AMR_ENABLE_LEGACY_START_BUTTON
static uint8_t amr_last_start_button_active = 0U;
static uint32_t amr_last_start_button_ms = 0U;
#endif

static void AMR_EnsureInitialized(void);
static void AMR_ApplySafeStop(void);
static uint8_t AMR_IsMotionState(AMR_State_t state);
#if AMR_ENABLE_LEGACY_START_BUTTON
static void AMR_UpdateStartButton(uint32_t now_ms);
#endif
static void AMR_UpdateObstacleState(void);

void AMR_Init(void)
{
    amr_state = AMR_STATE_IDLE;
    amr_state_enter_ms = HAL_GetTick();
    amr_initialized = 1U;
#if AMR_ENABLE_LEGACY_START_BUTTON
    amr_last_start_button_active = 0U;
    amr_last_start_button_ms = 0U;
#endif

    /*
     * app_button_control.c owns the NUCLEO B1 user button for short-press
     * start/return/stop/reset and long-press ESTOP demo control.
     */
    APP_LOG("[AMR] state IDLE reason=boot");
}

void AMR_StateMachine_Update(void)
{
#if AMR_ENABLE_LEGACY_START_BUTTON
    uint32_t now_ms = HAL_GetTick();
#endif

    AMR_EnsureInitialized();
#if AMR_ENABLE_LEGACY_START_BUTTON
    AMR_UpdateStartButton(now_ms);
#endif
    AMR_UpdateObstacleState();
}

AMR_State_t AMR_GetState(void)
{
    AMR_EnsureInitialized();

    return amr_state;
}

uint32_t AMR_GetStateEnterMs(void)
{
    AMR_EnsureInitialized();

    return amr_state_enter_ms;
}

const char *AMR_StateName(AMR_State_t state)
{
    switch (state)
    {
        case AMR_STATE_IDLE:
            return "IDLE";

        case AMR_STATE_EXPLORE:
            return "EXPLORE";

        case AMR_STATE_AVOID:
            return "AVOID";

        case AMR_STATE_RETURN:
            return "RETURN";

        case AMR_STATE_FAULT:
            return "FAULT";

        case AMR_STATE_ESTOP:
            return "ESTOP";

        default:
            return "UNKNOWN";
    }
}

const char *AMR_StateShortName(AMR_State_t state)
{
    switch (state)
    {
        case AMR_STATE_IDLE:
            return "IDLE";

        case AMR_STATE_EXPLORE:
            return "EXP";

        case AMR_STATE_AVOID:
            return "AVD";

        case AMR_STATE_RETURN:
            return "RET";

        case AMR_STATE_FAULT:
            return "FLT";

        case AMR_STATE_ESTOP:
            return "EST";

        default:
            return "UNK";
    }
}

uint8_t AMR_SetState(AMR_State_t next_state, const char *reason)
{
    AMR_State_t previous_state;

    AMR_EnsureInitialized();
    previous_state = amr_state;

    if (reason == NULL)
    {
        reason = "unspecified";
    }

    if (previous_state == next_state)
    {
        if ((next_state == AMR_STATE_FAULT) || (next_state == AMR_STATE_ESTOP))
        {
            ReturnExecutor_Stop(reason);
            AMR_ApplySafeStop();
        }

        return 0U;
    }

    amr_state = next_state;
    amr_state_enter_ms = HAL_GetTick();

    if ((next_state == AMR_STATE_IDLE) ||
        (next_state == AMR_STATE_FAULT) ||
        (next_state == AMR_STATE_ESTOP))
    {
        ReturnExecutor_Stop(reason);
        AMR_ApplySafeStop();
    }
    else if (next_state == AMR_STATE_RETURN)
    {
        AMR_ApplySafeStop();
    }

    APP_LOG("[AMR] state %s -> %s reason=%s",
            AMR_StateName(previous_state),
            AMR_StateName(next_state),
            reason);

    if (next_state == AMR_STATE_ESTOP)
    {
        APP_LOG("[ESTOP] motor outputs disabled");
    }

#if APP_ENABLE_LIDAR_OBSTACLE_AVOIDANCE_TEST
    if ((AMR_IsMotionState(previous_state) == 0U) &&
        (AMR_IsMotionState(next_state) != 0U))
    {
        App_LidarObstacleAvoidance_Start();
    }
#endif

    return 1U;
}

void AMR_RequestStart(const char *reason)
{
    AMR_State_t state = AMR_GetState();

    if (state == AMR_STATE_IDLE)
    {
        (void)AMR_SetState(AMR_STATE_EXPLORE, reason);
    }
}

void AMR_RequestStop(const char *reason)
{
    AMR_State_t state = AMR_GetState();

    if ((state == AMR_STATE_EXPLORE) ||
        (state == AMR_STATE_AVOID) ||
        (state == AMR_STATE_RETURN))
    {
        (void)AMR_SetState(AMR_STATE_IDLE, reason);
    }
}

void AMR_RequestReturn(const char *reason)
{
    AMR_State_t state = AMR_GetState();

    if ((state != AMR_STATE_FAULT) && (state != AMR_STATE_ESTOP))
    {
        (void)AMR_SetState(AMR_STATE_RETURN, reason);
    }
}

void AMR_RequestEStop(const char *reason)
{
    (void)AMR_SetState(AMR_STATE_ESTOP, reason);
}

void AMR_RequestResetFault(const char *reason)
{
    AMR_State_t state = AMR_GetState();

    if ((state == AMR_STATE_FAULT) || (state == AMR_STATE_ESTOP))
    {
        (void)AMR_SetState(AMR_STATE_IDLE, reason);
    }
}

static void AMR_EnsureInitialized(void)
{
    if (amr_initialized == 0U)
    {
        AMR_Init();
    }
}

static void AMR_ApplySafeStop(void)
{
    Chassis_Stop();
    MotorDriver_StopAll();
    AppOdo_SyncBaseline();
}

static uint8_t AMR_IsMotionState(AMR_State_t state)
{
    return ((state == AMR_STATE_EXPLORE) || (state == AMR_STATE_AVOID)) ? 1U : 0U;
}

#if AMR_ENABLE_LEGACY_START_BUTTON
static void AMR_UpdateStartButton(uint32_t now_ms)
{
    uint8_t active = (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if ((active != 0U) &&
        (amr_last_start_button_active == 0U) &&
        ((now_ms - amr_last_start_button_ms) >= AMR_START_BUTTON_DEBOUNCE_MS))
    {
        amr_last_start_button_ms = now_ms;
        AMR_RequestStart("start_button");
    }

    amr_last_start_button_active = active;
}
#endif

static void AMR_UpdateObstacleState(void)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    AMR_State_t state = amr_state;

    if ((lidar == NULL) || (lidar->front_valid == 0U))
    {
        return;
    }

    if ((state == AMR_STATE_EXPLORE) &&
        (lidar->front_min_mm <= AMR_AVOID_ENTER_FRONT_MM))
    {
        (void)AMR_SetState(AMR_STATE_AVOID, "front_obstacle");
    }
    else if ((state == AMR_STATE_AVOID) &&
             (lidar->front_min_mm >= AMR_AVOID_EXIT_FRONT_MM))
    {
        (void)AMR_SetState(AMR_STATE_EXPLORE, "front_clear");
    }
}
