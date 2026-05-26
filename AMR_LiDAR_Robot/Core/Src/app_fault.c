#include "app_fault.h"

#include "amr_system.h"
#include "app_odometry.h"
#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"
#include "stm32f4xx_hal.h"

static AppFaultCode app_fault_code = FAULT_NONE;
static uint32_t app_fault_time_ms = 0U;
static uint8_t app_fault_initialized = 0U;

static void AppFault_EnsureInitialized(void);
static void AppFault_SafeStop(void);

void AppFault_Init(void)
{
    app_fault_code = FAULT_NONE;
    app_fault_time_ms = 0U;
    app_fault_initialized = 1U;
}

void AppFault_Set(AppFaultCode code)
{
    AppFault_EnsureInitialized();

    if (code == FAULT_NONE)
    {
        AppFault_Clear();
        return;
    }

    if (app_fault_code != FAULT_NONE)
    {
        AppFault_SafeStop();
        return;
    }

    app_fault_code = code;
    app_fault_time_ms = HAL_GetTick();

    APP_LOG("FAULT: code=%s", AppFault_Name(code));
    AppFault_SafeStop();
    APP_LOG("FAULT: safe stop");

    if (code == FAULT_USER_ESTOP)
    {
        (void)AMR_SetState(AMR_STATE_ESTOP, "fault_user_estop");
    }
    else
    {
        (void)AMR_SetState(AMR_STATE_FAULT, "fault_manager");
    }
}

void AppFault_Clear(void)
{
    AppFault_EnsureInitialized();

    if (app_fault_code != FAULT_NONE)
    {
        app_fault_code = FAULT_NONE;
        app_fault_time_ms = 0U;
        APP_LOG("FAULT: cleared");
    }
}

AppFaultCode AppFault_Get(void)
{
    AppFault_EnsureInitialized();

    return app_fault_code;
}

bool AppFault_IsActive(void)
{
    AppFault_EnsureInitialized();

    return (app_fault_code != FAULT_NONE);
}

const char *AppFault_Name(AppFaultCode code)
{
    switch (code)
    {
        case FAULT_NONE:
            return "NONE";

        case FAULT_LIDAR_TIMEOUT:
            return "LIDAR_TIMEOUT";

        case FAULT_ENCODER_STALL:
            return "ENCODER_STALL";

        case FAULT_PLANNER_STUCK:
            return "PLANNER_STUCK";

        case FAULT_USER_ESTOP:
            return "USER_ESTOP";

        default:
            return "UNKNOWN";
    }
}

static void AppFault_EnsureInitialized(void)
{
    if (app_fault_initialized == 0U)
    {
        AppFault_Init();
    }
}

static void AppFault_SafeStop(void)
{
    Chassis_Stop();
    MotorDriver_StopAll();
    AppOdo_SyncBaseline();
    (void)app_fault_time_ms;
}
