#include "app_benchmark_script.h"

#include "amr_system.h"
#include "app_fault.h"
#include "app_lidar.h"
#include "bringup_log.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define SCRIPT_OBSTACLE_STOP_MM 250U
#define SCRIPT_OBSTACLE_CLEAR_MM 450U
#define SCRIPT_LIDAR_STALE_MS 800U
#define SCRIPT_LOG_INTERVAL_MS 1000U
#define AUTO_FORWARD_DUTY 520
#define AUTO_TURN_DUTY 420
#define AUTO_TURN_RIGHT_MS 550U
#define AUTO_OBSTACLE_STOP_MM 250U
#define AUTO_OBSTACLE_CLEAR_MM 450U

typedef struct
{
    ScriptAction action;
    int16_t duty;
    uint32_t duration_ms;
    const char *note;
} BenchmarkScriptStep_t;

typedef struct
{
    const char *name;
    const BenchmarkScriptStep_t *steps;
    uint8_t count;
} BenchmarkScriptDef_t;

static const BenchmarkScriptStep_t exit_script_steps[] = {
    {SCRIPT_ACT_FORWARD, 520, 900U, "exit forward 1"},
    {SCRIPT_ACT_TURN_RIGHT, 420, 550U, "exit right turn"},
    {SCRIPT_ACT_FORWARD, 520, 900U, "exit forward 2"},
    {SCRIPT_ACT_STOP, 0, 200U, "exit stop"},
};

static const BenchmarkScriptStep_t return_script_steps[] = {
    {SCRIPT_ACT_TURN_RIGHT, 420, 1750U, "return turn around"},
    {SCRIPT_ACT_FORWARD, 520, 900U, "return forward 1"},
    {SCRIPT_ACT_TURN_LEFT, 420, 550U, "return left turn"},
    {SCRIPT_ACT_FORWARD, 520, 900U, "return forward 2"},
    {SCRIPT_ACT_STOP, 0, 200U, "return stop"},
};

static const BenchmarkScriptDef_t exit_script = {
    "exit_script",
    exit_script_steps,
    (uint8_t)(sizeof(exit_script_steps) / sizeof(exit_script_steps[0])),
};

static const BenchmarkScriptDef_t return_script = {
    "return_script",
    return_script_steps,
    (uint8_t)(sizeof(return_script_steps) / sizeof(return_script_steps[0])),
};

static BenchmarkScriptState_t script_state = SCRIPT_IDLE;
static const BenchmarkScriptDef_t *script_current = NULL;
static uint8_t script_step_index = 0U;
static uint32_t script_step_start_ms = 0U;
static uint32_t script_wait_start_ms = 0U;
static uint32_t script_last_log_ms = 0U;
static uint8_t script_initialized = 0U;
static uint8_t script_step_active = 0U;
static uint8_t script_step_just_started = 0U;
static ScriptAction script_current_action = SCRIPT_ACT_STOP;
static uint8_t script_current_action_valid = 0U;

static void AppBenchmarkScript_Start(const BenchmarkScriptDef_t *script);
static void AppBenchmarkScript_ClearRuntime(void);
static void AppBenchmarkScript_StartStep(uint32_t now_ms);
static void AppBenchmarkScript_ApplyAction(const BenchmarkScriptStep_t *step);
static void AppBenchmarkScript_FinishStep(uint32_t now_ms);
static void AppBenchmarkScript_ResumeCurrentStep(uint32_t now_ms);
static void AppBenchmarkScript_Abort(BenchmarkScriptState_t state, const char *reason);
static void AppBenchmarkScript_CheckForwardSafety(uint32_t now_ms);
static void AppBenchmarkScript_UpdateWaitObstacleClear(uint32_t now_ms);
static void AppBenchmarkScript_UpdateAuto(uint32_t now_ms);
static void AppBenchmarkScript_UpdateAutoForward(uint32_t now_ms);
static void AppBenchmarkScript_UpdateAutoTurnRight(uint32_t now_ms);
static void AppBenchmarkScript_UpdateAutoWaitClear(uint32_t now_ms);
static void AppBenchmarkScript_EnterAutoForward(uint32_t now_ms, const char *reason);
static void AppBenchmarkScript_EnterAutoTurnRight(uint32_t now_ms, uint32_t front_mm);
static void AppBenchmarkScript_EnterAutoWaitClear(uint32_t now_ms, const char *reason);
static const BenchmarkScriptStep_t *AppBenchmarkScript_CurrentStep(void);
static uint32_t AppBenchmarkScript_ElapsedMs(uint32_t now_ms, uint32_t then_ms);
static bool AppBenchmarkScript_LidarFrontClear(uint32_t now_ms);
static bool AppBenchmarkScript_LidarFrontBlocked(uint32_t now_ms);
static bool AppBenchmarkScript_LidarFrontStale(uint32_t now_ms);
static bool AppBenchmarkScript_AutoFrontClear(uint32_t now_ms);
static bool AppBenchmarkScript_AutoFrontBlocked(uint32_t now_ms);
static bool AppBenchmarkScript_IsAutoState(BenchmarkScriptState_t state);
static bool AppBenchmarkScript_IsActiveState(BenchmarkScriptState_t state);
static const char *AppBenchmarkScript_CurrentName(void);
static void AppBenchmarkScript_LogLowRate(uint32_t now_ms, const char *message);

void AppBenchmarkScript_Init(void)
{
    script_state = SCRIPT_IDLE;
    script_current = NULL;
    script_step_index = 0U;
    script_step_start_ms = 0U;
    script_wait_start_ms = 0U;
    script_last_log_ms = 0U;
    script_step_active = 0U;
    script_step_just_started = 0U;
    script_current_action = SCRIPT_ACT_STOP;
    script_current_action_valid = 0U;
    script_initialized = 1U;
    APP_LOG("SCRIPT: init obstacle_stop=%umm obstacle_clear=%umm lidar_stale=%ums",
            (unsigned int)SCRIPT_OBSTACLE_STOP_MM,
            (unsigned int)SCRIPT_OBSTACLE_CLEAR_MM,
            (unsigned int)SCRIPT_LIDAR_STALE_MS);
}

void AppBenchmarkScript_StartExit(void)
{
    AppBenchmarkScript_Start(&exit_script);
}

void AppBenchmarkScript_StartReturn(void)
{
    AppBenchmarkScript_Start(&return_script);
}

void AppBenchmarkScript_StartAuto(void)
{
    uint32_t now_ms;

    if (script_initialized == 0U)
    {
        AppBenchmarkScript_Init();
    }

    if (AppBenchmarkScript_IsActiveState(script_state))
    {
        APP_LOG("SCRIPT: rejected reason=busy state=%s", AppBenchmarkScript_StateName(script_state));
        return;
    }

    AppBenchmarkScript_ClearRuntime();

    if (AMR_GetState() != AMR_STATE_IDLE)
    {
        APP_LOG("SCRIPT: auto rejected reason=amr_state state=%s", AMR_StateName(AMR_GetState()));
        return;
    }

    if (AppFault_IsActive())
    {
        APP_LOG("SCRIPT: auto rejected reason=fault fault=%s", AppFault_Name(AppFault_Get()));
        return;
    }

    script_current = NULL;
    script_step_index = 0U;
    now_ms = HAL_GetTick();
    APP_LOG("SCRIPT: auto start forward_duty=%d turn_duty=%d turn_ms=%u stop=%umm clear=%umm",
            (int)AUTO_FORWARD_DUTY,
            (int)AUTO_TURN_DUTY,
            (unsigned int)AUTO_TURN_RIGHT_MS,
            (unsigned int)AUTO_OBSTACLE_STOP_MM,
            (unsigned int)AUTO_OBSTACLE_CLEAR_MM);
    AppBenchmarkScript_EnterAutoForward(now_ms, "start");
}

void AppBenchmarkScript_Stop(const char *reason)
{
    if (script_initialized == 0U)
    {
        AppBenchmarkScript_Init();
    }

    if (AppBenchmarkScript_IsActiveState(script_state))
    {
        Chassis_Stop();
        script_state = SCRIPT_ABORTED;
        AppBenchmarkScript_ClearRuntime();
        APP_LOG("SCRIPT: stopped reason=%s", (reason != NULL) ? reason : "unspecified");
    }
}

void AppBenchmarkScript_Reset(void)
{
    Chassis_Stop();
    script_state = SCRIPT_IDLE;
    script_current = NULL;
    script_step_index = 0U;
    AppBenchmarkScript_ClearRuntime();
    script_initialized = 1U;
    APP_LOG("SCRIPT: reset");
}

void AppBenchmarkScript_Update(void)
{
    uint32_t now_ms;
    const BenchmarkScriptStep_t *step;

    if (script_initialized == 0U)
    {
        AppBenchmarkScript_Init();
    }

    if (!AppBenchmarkScript_IsActiveState(script_state))
    {
        return;
    }

    now_ms = HAL_GetTick();

    if (AppFault_IsActive() || (AMR_GetState() == AMR_STATE_ESTOP) || (AMR_GetState() == AMR_STATE_FAULT))
    {
        AppBenchmarkScript_Abort(SCRIPT_FAULT, "fault_or_estop");
        return;
    }

    if (AppBenchmarkScript_IsAutoState(script_state))
    {
        AppBenchmarkScript_UpdateAuto(now_ms);
        return;
    }

    if (script_state == SCRIPT_WAIT_OBSTACLE_CLEAR)
    {
        AppBenchmarkScript_UpdateWaitObstacleClear(now_ms);
        return;
    }

    step = AppBenchmarkScript_CurrentStep();
    if (step == NULL)
    {
        script_state = SCRIPT_DONE;
        Chassis_Stop();
        AppBenchmarkScript_ClearRuntime();
        APP_LOG("SCRIPT: done %s", (script_current != NULL) ? script_current->name : "none");
        return;
    }

    if (script_step_active == 0U)
    {
        return;
    }

    if (step->action == SCRIPT_ACT_FORWARD)
    {
        AppBenchmarkScript_CheckForwardSafety(now_ms);
        if (script_state != SCRIPT_RUNNING)
        {
            return;
        }
    }

    if (script_step_just_started != 0U)
    {
        script_step_just_started = 0U;
        return;
    }

    if (AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms) >= step->duration_ms)
    {
        AppBenchmarkScript_FinishStep(now_ms);
    }
}

void AppBenchmarkScript_PrintStatus(void)
{
    const BenchmarkScriptStep_t *step = AppBenchmarkScript_CurrentStep();
    uint8_t show_step = 0U;
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = 0U;
    uint32_t remaining_ms = 0U;
    uint8_t display_step_index = 0U;
    const char *action_name = "NONE";
    const char *script_name = AppBenchmarkScript_CurrentName();

    if (((script_state == SCRIPT_RUNNING) || (script_state == SCRIPT_WAIT_OBSTACLE_CLEAR)) &&
        (step != NULL) &&
        (script_current_action_valid != 0U))
    {
        show_step = 1U;
        action_name = AppBenchmarkScript_ActionName(script_current_action);
        display_step_index = (uint8_t)(script_step_index + 1U);
    }
    else if (AppBenchmarkScript_IsAutoState(script_state) && (script_current_action_valid != 0U))
    {
        show_step = 1U;
        action_name = AppBenchmarkScript_ActionName(script_current_action);
    }
    else if ((script_state == SCRIPT_DONE) && (script_current != NULL))
    {
        display_step_index = script_current->count;
    }

    if ((show_step != 0U) && (script_step_active != 0U))
    {
        elapsed_ms = AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms);
    }

    if ((show_step != 0U) && (step != NULL))
    {
        remaining_ms = (elapsed_ms < step->duration_ms) ? (step->duration_ms - elapsed_ms) : 0U;
    }
    else if (script_state == SCRIPT_AUTO_TURN_RIGHT)
    {
        remaining_ms = (elapsed_ms < AUTO_TURN_RIGHT_MS) ? (AUTO_TURN_RIGHT_MS - elapsed_ms) : 0U;
    }

    APP_LOG("SCRIPT: status state=%s name=%s step=%u/%u action=%s elapsed=%lums remaining=%lums",
            AppBenchmarkScript_StateName(script_state),
            script_name,
            (unsigned int)display_step_index,
            (unsigned int)((script_current != NULL) ? script_current->count : 0U),
            action_name,
            (unsigned long)elapsed_ms,
            (unsigned long)remaining_ms);
}

uint8_t AppBenchmarkScript_IsActive(void)
{
    return AppBenchmarkScript_IsActiveState(script_state) ? 1U : 0U;
}

uint8_t AppBenchmarkScript_IsAutoActive(void)
{
    return AppBenchmarkScript_IsAutoState(script_state) ? 1U : 0U;
}

uint8_t AppBenchmarkScript_IsReturnActive(void)
{
    return (AppBenchmarkScript_IsActiveState(script_state) &&
            (script_current == &return_script)) ? 1U : 0U;
}

BenchmarkScriptState_t AppBenchmarkScript_GetState(void)
{
    return script_state;
}

const char *AppBenchmarkScript_StateName(BenchmarkScriptState_t state)
{
    switch (state)
    {
        case SCRIPT_IDLE:
            return "IDLE";
        case SCRIPT_RUNNING:
            return "RUNNING";
        case SCRIPT_WAIT_OBSTACLE_CLEAR:
            return "WAIT_OBSTACLE_CLEAR";
        case SCRIPT_AUTO_FORWARD:
            return "SCRIPT_AUTO_FORWARD";
        case SCRIPT_AUTO_TURN_RIGHT:
            return "SCRIPT_AUTO_TURN_RIGHT";
        case SCRIPT_AUTO_WAIT_CLEAR:
            return "SCRIPT_AUTO_WAIT_CLEAR";
        case SCRIPT_DONE:
            return "DONE";
        case SCRIPT_ABORTED:
            return "ABORTED";
        case SCRIPT_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

const char *AppBenchmarkScript_ActionName(ScriptAction action)
{
    switch (action)
    {
        case SCRIPT_ACT_STOP:
            return "STOP";
        case SCRIPT_ACT_WAIT:
            return "WAIT";
        case SCRIPT_ACT_FORWARD:
            return "FORWARD";
        case SCRIPT_ACT_BACKUP:
            return "BACKUP";
        case SCRIPT_ACT_TURN_LEFT:
            return "TURN_LEFT";
        case SCRIPT_ACT_TURN_RIGHT:
            return "TURN_RIGHT";
        default:
            return "UNKNOWN";
    }
}

static void AppBenchmarkScript_Start(const BenchmarkScriptDef_t *script)
{
    if (script_initialized == 0U)
    {
        AppBenchmarkScript_Init();
    }

    if (script == NULL)
    {
        APP_LOG("SCRIPT: rejected reason=no_script");
        return;
    }

    if (AppBenchmarkScript_IsActiveState(script_state))
    {
        APP_LOG("SCRIPT: rejected reason=busy state=%s", AppBenchmarkScript_StateName(script_state));
        return;
    }

    AppBenchmarkScript_ClearRuntime();

    if (AMR_GetState() != AMR_STATE_IDLE)
    {
        APP_LOG("SCRIPT: rejected reason=amr_state state=%s", AMR_StateName(AMR_GetState()));
        return;
    }

    if (AppFault_IsActive())
    {
        APP_LOG("SCRIPT: rejected reason=fault fault=%s", AppFault_Name(AppFault_Get()));
        return;
    }

    script_current = script;
    script_step_index = 0U;
    script_state = SCRIPT_RUNNING;
    APP_LOG("SCRIPT: start %s count=%u", script->name, (unsigned int)script->count);
    AppBenchmarkScript_StartStep(0U);
}

static void AppBenchmarkScript_ClearRuntime(void)
{
    script_step_active = 0U;
    script_step_just_started = 0U;
    script_step_start_ms = 0U;
    script_wait_start_ms = 0U;
    script_last_log_ms = 0U;
    script_current_action = SCRIPT_ACT_STOP;
    script_current_action_valid = 0U;
}

static void AppBenchmarkScript_StartStep(uint32_t now_ms)
{
    const BenchmarkScriptStep_t *step = AppBenchmarkScript_CurrentStep();
    uint32_t start_ms = (now_ms != 0U) ? now_ms : HAL_GetTick();

    if (step == NULL)
    {
        script_state = SCRIPT_DONE;
        Chassis_Stop();
        AppBenchmarkScript_ClearRuntime();
        APP_LOG("SCRIPT: done %s", (script_current != NULL) ? script_current->name : "none");
        return;
    }

    script_step_start_ms = start_ms;
    script_step_active = 1U;
    script_step_just_started = 1U;
    script_current_action = step->action;
    script_current_action_valid = 1U;
    APP_LOG("SCRIPT: step=%u/%u action=%s duty=%d dur=%lums note=%s",
            (unsigned int)(script_step_index + 1U),
            (unsigned int)((script_current != NULL) ? script_current->count : 0U),
            AppBenchmarkScript_ActionName(step->action),
            (int)step->duty,
            (unsigned long)step->duration_ms,
            (step->note != NULL) ? step->note : "");
    if (step->action == SCRIPT_ACT_FORWARD)
    {
        AppBenchmarkScript_CheckForwardSafety(start_ms);
        if (script_state != SCRIPT_RUNNING)
        {
            script_current_action = step->action;
            script_current_action_valid = 1U;
            return;
        }
    }

    AppBenchmarkScript_ApplyAction(step);
}

static void AppBenchmarkScript_ApplyAction(const BenchmarkScriptStep_t *step)
{
    if (step == NULL)
    {
        Chassis_Stop();
        return;
    }

    switch (step->action)
    {
        case SCRIPT_ACT_STOP:
        case SCRIPT_ACT_WAIT:
            Chassis_Stop();
            break;

        case SCRIPT_ACT_FORWARD:
            Chassis_Forward(step->duty);
            break;

        case SCRIPT_ACT_BACKUP:
            Chassis_Backward(step->duty);
            break;

        case SCRIPT_ACT_TURN_LEFT:
            Chassis_TurnLeft(step->duty);
            break;

        case SCRIPT_ACT_TURN_RIGHT:
            Chassis_TurnRight(step->duty);
            break;

        default:
            Chassis_Stop();
            break;
    }
}

static void AppBenchmarkScript_FinishStep(uint32_t now_ms)
{
    uint32_t elapsed_ms = AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms);

    if ((script_step_active == 0U) || (script_current == NULL) || (script_step_index >= script_current->count))
    {
        return;
    }

    APP_LOG("SCRIPT: step=%u/%u done elapsed=%lums",
            (unsigned int)(script_step_index + 1U),
            (unsigned int)((script_current != NULL) ? script_current->count : 0U),
            (unsigned long)elapsed_ms);

    script_step_index++;
    if ((script_current == NULL) || (script_step_index >= script_current->count))
    {
        script_state = SCRIPT_DONE;
        Chassis_Stop();
        AppBenchmarkScript_ClearRuntime();
        APP_LOG("SCRIPT: done %s", (script_current != NULL) ? script_current->name : "none");
        return;
    }

    AppBenchmarkScript_ClearRuntime();
    AppBenchmarkScript_StartStep(now_ms);
}

static void AppBenchmarkScript_ResumeCurrentStep(uint32_t now_ms)
{
    const BenchmarkScriptStep_t *step = AppBenchmarkScript_CurrentStep();

    if (step == NULL)
    {
        script_state = SCRIPT_DONE;
        Chassis_Stop();
        AppBenchmarkScript_ClearRuntime();
        APP_LOG("SCRIPT: done %s", (script_current != NULL) ? script_current->name : "none");
        return;
    }

    script_state = SCRIPT_RUNNING;
    script_wait_start_ms = 0U;
    APP_LOG("SCRIPT: obstacle_clear resume step=%u/%u action=%s",
            (unsigned int)(script_step_index + 1U),
            (unsigned int)((script_current != NULL) ? script_current->count : 0U),
            AppBenchmarkScript_ActionName(step->action));
    AppBenchmarkScript_StartStep(now_ms);
}

static void AppBenchmarkScript_Abort(BenchmarkScriptState_t state, const char *reason)
{
    Chassis_Stop();
    script_state = state;
    AppBenchmarkScript_ClearRuntime();
    APP_LOG("SCRIPT: abort state=%s reason=%s",
            AppBenchmarkScript_StateName(state),
            (reason != NULL) ? reason : "unspecified");
}

static void AppBenchmarkScript_CheckForwardSafety(uint32_t now_ms)
{
    if (AppBenchmarkScript_LidarFrontStale(now_ms))
    {
        Chassis_Stop();
        script_state = SCRIPT_WAIT_OBSTACLE_CLEAR;
        script_wait_start_ms = now_ms;
        script_step_active = 0U;
        script_step_just_started = 0U;
        APP_LOG("SCRIPT: wait reason=lidar_invalid_or_stale");
        return;
    }

    if (AppBenchmarkScript_LidarFrontBlocked(now_ms))
    {
        const AppLidarStatus *lidar = App_Lidar_GetStatus();
        Chassis_Stop();
        script_state = SCRIPT_WAIT_OBSTACLE_CLEAR;
        script_wait_start_ms = now_ms;
        script_step_active = 0U;
        script_step_just_started = 0U;
        APP_LOG("SCRIPT: wait reason=front_blocked front=%u stop=%u",
                (unsigned int)((lidar != NULL) ? lidar->front_min_mm : 0U),
                (unsigned int)SCRIPT_OBSTACLE_STOP_MM);
    }
}

static void AppBenchmarkScript_UpdateWaitObstacleClear(uint32_t now_ms)
{
    if (AppBenchmarkScript_LidarFrontClear(now_ms))
    {
        AppBenchmarkScript_ResumeCurrentStep(now_ms);
        return;
    }

    AppBenchmarkScript_LogLowRate(now_ms, "SCRIPT: waiting obstacle clear");
}

static void AppBenchmarkScript_UpdateAuto(uint32_t now_ms)
{
    switch (script_state)
    {
        case SCRIPT_AUTO_FORWARD:
            AppBenchmarkScript_UpdateAutoForward(now_ms);
            break;

        case SCRIPT_AUTO_TURN_RIGHT:
            AppBenchmarkScript_UpdateAutoTurnRight(now_ms);
            break;

        case SCRIPT_AUTO_WAIT_CLEAR:
            AppBenchmarkScript_UpdateAutoWaitClear(now_ms);
            break;

        default:
            break;
    }
}

static void AppBenchmarkScript_UpdateAutoForward(uint32_t now_ms)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();

    if (script_state != SCRIPT_AUTO_FORWARD)
    {
        return;
    }

    if (AppBenchmarkScript_LidarFrontStale(now_ms))
    {
        AppBenchmarkScript_EnterAutoWaitClear(now_ms, "lidar_invalid_or_stale");
        return;
    }

    if (AppBenchmarkScript_AutoFrontBlocked(now_ms))
    {
        uint32_t front_mm = (lidar != NULL) ? lidar->front_min_mm : 0U;
        Chassis_Stop();
        AppBenchmarkScript_EnterAutoTurnRight(now_ms, front_mm);
        return;
    }

    Chassis_Forward(AUTO_FORWARD_DUTY);
    AppBenchmarkScript_LogLowRate(now_ms, "SCRIPT: auto forward");
}

static void AppBenchmarkScript_UpdateAutoTurnRight(uint32_t now_ms)
{
    if (script_state != SCRIPT_AUTO_TURN_RIGHT)
    {
        return;
    }

    if (AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms) >= AUTO_TURN_RIGHT_MS)
    {
        APP_LOG("SCRIPT: auto turn_right done elapsed=%lums",
                (unsigned long)AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms));
        AppBenchmarkScript_EnterAutoForward(now_ms, "turn_done");
        return;
    }

    Chassis_TurnRight(AUTO_TURN_DUTY);
}

static void AppBenchmarkScript_UpdateAutoWaitClear(uint32_t now_ms)
{
    if (script_state != SCRIPT_AUTO_WAIT_CLEAR)
    {
        return;
    }

    Chassis_Stop();
    if (AppBenchmarkScript_AutoFrontClear(now_ms))
    {
        AppBenchmarkScript_EnterAutoForward(now_ms, "front_clear");
        return;
    }

    AppBenchmarkScript_LogLowRate(now_ms, "SCRIPT: auto waiting clear");
}

static void AppBenchmarkScript_EnterAutoForward(uint32_t now_ms, const char *reason)
{
    script_current = NULL;
    script_step_index = 0U;
    script_state = SCRIPT_AUTO_FORWARD;
    script_step_start_ms = now_ms;
    script_step_active = 1U;
    script_step_just_started = 0U;
    script_wait_start_ms = 0U;
    script_current_action = SCRIPT_ACT_FORWARD;
    script_current_action_valid = 1U;
    Chassis_Forward(AUTO_FORWARD_DUTY);
    APP_LOG("SCRIPT: auto state=FORWARD duty=%d reason=%s",
            (int)AUTO_FORWARD_DUTY,
            (reason != NULL) ? reason : "none");
    AppBenchmarkScript_UpdateAutoForward(now_ms);
}

static void AppBenchmarkScript_EnterAutoTurnRight(uint32_t now_ms, uint32_t front_mm)
{
    script_current = NULL;
    script_step_index = 0U;
    script_state = SCRIPT_AUTO_TURN_RIGHT;
    script_step_start_ms = now_ms;
    script_step_active = 1U;
    script_step_just_started = 0U;
    script_wait_start_ms = 0U;
    script_current_action = SCRIPT_ACT_TURN_RIGHT;
    script_current_action_valid = 1U;
    Chassis_TurnRight(AUTO_TURN_DUTY);
    APP_LOG("SCRIPT: auto state=TURN_RIGHT duty=%d dur=%ums front=%u stop=%u",
            (int)AUTO_TURN_DUTY,
            (unsigned int)AUTO_TURN_RIGHT_MS,
            (unsigned int)front_mm,
            (unsigned int)AUTO_OBSTACLE_STOP_MM);
}

static void AppBenchmarkScript_EnterAutoWaitClear(uint32_t now_ms, const char *reason)
{
    script_current = NULL;
    script_step_index = 0U;
    script_state = SCRIPT_AUTO_WAIT_CLEAR;
    script_step_start_ms = now_ms;
    script_wait_start_ms = now_ms;
    script_step_active = 1U;
    script_step_just_started = 0U;
    script_current_action = SCRIPT_ACT_WAIT;
    script_current_action_valid = 1U;
    Chassis_Stop();
    APP_LOG("SCRIPT: auto state=WAIT_CLEAR reason=%s clear=%u",
            (reason != NULL) ? reason : "none",
            (unsigned int)AUTO_OBSTACLE_CLEAR_MM);
}

static const BenchmarkScriptStep_t *AppBenchmarkScript_CurrentStep(void)
{
    if ((script_current == NULL) || (script_step_index >= script_current->count))
    {
        return NULL;
    }

    return &script_current->steps[script_step_index];
}

static uint32_t AppBenchmarkScript_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    if (now_ms >= then_ms)
    {
        return now_ms - then_ms;
    }

    return 0U;
}

static bool AppBenchmarkScript_LidarFrontClear(uint32_t now_ms)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();

    if ((lidar == NULL) || (lidar->front_valid == 0U))
    {
        return false;
    }

    if (AppBenchmarkScript_ElapsedMs(now_ms, lidar->last_valid_update_ms) > SCRIPT_LIDAR_STALE_MS)
    {
        return false;
    }

    return (lidar->front_min_mm > SCRIPT_OBSTACLE_CLEAR_MM);
}

static bool AppBenchmarkScript_LidarFrontBlocked(uint32_t now_ms)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();

    if ((lidar == NULL) || (lidar->front_valid == 0U))
    {
        return false;
    }

    if (AppBenchmarkScript_ElapsedMs(now_ms, lidar->last_valid_update_ms) > SCRIPT_LIDAR_STALE_MS)
    {
        return false;
    }

    return (lidar->front_min_mm < SCRIPT_OBSTACLE_STOP_MM);
}

static bool AppBenchmarkScript_LidarFrontStale(uint32_t now_ms)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();

    if ((lidar == NULL) || (lidar->front_valid == 0U))
    {
        return true;
    }

    return (AppBenchmarkScript_ElapsedMs(now_ms, lidar->last_valid_update_ms) > SCRIPT_LIDAR_STALE_MS);
}

static bool AppBenchmarkScript_AutoFrontClear(uint32_t now_ms)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();

    if ((lidar == NULL) || (lidar->front_valid == 0U))
    {
        return false;
    }

    if (AppBenchmarkScript_ElapsedMs(now_ms, lidar->last_valid_update_ms) > SCRIPT_LIDAR_STALE_MS)
    {
        return false;
    }

    return (lidar->front_min_mm > AUTO_OBSTACLE_CLEAR_MM);
}

static bool AppBenchmarkScript_AutoFrontBlocked(uint32_t now_ms)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();

    if ((lidar == NULL) || (lidar->front_valid == 0U))
    {
        return false;
    }

    if (AppBenchmarkScript_ElapsedMs(now_ms, lidar->last_valid_update_ms) > SCRIPT_LIDAR_STALE_MS)
    {
        return false;
    }

    return (lidar->front_min_mm < AUTO_OBSTACLE_STOP_MM);
}

static bool AppBenchmarkScript_IsAutoState(BenchmarkScriptState_t state)
{
    return ((state == SCRIPT_AUTO_FORWARD) ||
            (state == SCRIPT_AUTO_TURN_RIGHT) ||
            (state == SCRIPT_AUTO_WAIT_CLEAR));
}

static bool AppBenchmarkScript_IsActiveState(BenchmarkScriptState_t state)
{
    return ((state == SCRIPT_RUNNING) ||
            (state == SCRIPT_WAIT_OBSTACLE_CLEAR) ||
            AppBenchmarkScript_IsAutoState(state));
}

static const char *AppBenchmarkScript_CurrentName(void)
{
    if (script_current != NULL)
    {
        return script_current->name;
    }

    if (AppBenchmarkScript_IsAutoState(script_state))
    {
        return "script_auto";
    }

    return "none";
}

static void AppBenchmarkScript_LogLowRate(uint32_t now_ms, const char *message)
{
    if ((script_last_log_ms == 0U) ||
        (AppBenchmarkScript_ElapsedMs(now_ms, script_last_log_ms) >= SCRIPT_LOG_INTERVAL_MS))
    {
        script_last_log_ms = now_ms;
        APP_LOG("%s", (message != NULL) ? message : "SCRIPT: wait");
    }
}
