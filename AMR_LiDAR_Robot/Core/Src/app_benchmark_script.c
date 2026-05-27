#include "app_benchmark_script.h"

#include "amr_system.h"
#include "app_fault.h"
#include "app_lidar.h"
#include "bringup_log.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SCRIPT_OBSTACLE_STOP_MM 250U
#define SCRIPT_OBSTACLE_CLEAR_MM 450U
#define SCRIPT_LIDAR_STALE_MS 800U
#define SCRIPT_LOG_INTERVAL_MS 1000U
#define BENCH_FORWARD_LEFT_DUTY 498
#define BENCH_FORWARD_RIGHT_DUTY 500
#define AUTO_TURN_DUTY 420
#define AUTO_TURN_RIGHT_MS 550U
#define RETURN_AUTO_TURN_AROUND_MS 1900U
#define RETURN_AUTO_TURN_LEFT_MS 550U
#define AUTO_OBSTACLE_STOP_MM 360U
#define AUTO_OBSTACLE_CLEAR_MM 520U
#define ROUTE_MAX_TOKENS 64U
#define ROUTE_MAX_STEPS (ROUTE_MAX_TOKENS * 3U)
#define ROUTE_FORWARD_MS 2000U
#define ROUTE_HALF_FORWARD_MS 1000U
#define ROUTE_TURN_LEFT_MS 550U
#define ROUTE_TURN_RIGHT_MS 550U
#define ROUTE_WAIT_MS 200U
#define ROUTE_STOP_MS 200U
#define ROUTE_TEXT_MAX (ROUTE_MAX_TOKENS * 2U)
#define BENCH_FWD_MIN_MS 100U
#define BENCH_FWD_MAX_MS 3000U
#define BENCH_FWD_REJECT_FRONT_MM 250U

typedef enum
{
    ROUTE_SLOT_EXIT = 0,
    ROUTE_SLOT_RETURN
} ManualRouteSlot_t;

typedef enum
{
    ROUTE_PHASE_SINGLE = 0,
    ROUTE_PHASE_PRE_HALF,
    ROUTE_PHASE_TURN,
    ROUTE_PHASE_POST_HALF
} ManualRoutePhase_t;

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

typedef struct
{
    char token;
    ManualRoutePhase_t phase;
    uint8_t raw_index;
} ManualRouteStepMeta_t;

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

static BenchmarkScriptStep_t route_exit_steps[ROUTE_MAX_STEPS];
static BenchmarkScriptStep_t route_return_steps[ROUTE_MAX_STEPS];
static BenchmarkScriptStep_t route_parse_steps[ROUTE_MAX_STEPS];
static ManualRouteStepMeta_t route_exit_meta[ROUTE_MAX_STEPS];
static ManualRouteStepMeta_t route_return_meta[ROUTE_MAX_STEPS];
static ManualRouteStepMeta_t route_parse_meta[ROUTE_MAX_STEPS];
static char route_exit_tokens[ROUTE_MAX_TOKENS];
static char route_return_tokens[ROUTE_MAX_TOKENS];
static char route_parse_tokens[ROUTE_MAX_TOKENS];
static uint8_t route_exit_token_count = 0U;
static uint8_t route_return_token_count = 0U;
static BenchmarkScriptDef_t route_exit_script = {
    "route_exit",
    route_exit_steps,
    0U,
};
static BenchmarkScriptDef_t route_return_script = {
    "route_return",
    route_return_steps,
    0U,
};
static BenchmarkScriptStep_t bench_forward_steps[] = {
    {SCRIPT_ACT_FORWARD, BENCH_FORWARD_RIGHT_DUTY, ROUTE_FORWARD_MS, "bench forward"},
};
static BenchmarkScriptDef_t bench_forward_script = {
    "bench_fwd",
    bench_forward_steps,
    (uint8_t)(sizeof(bench_forward_steps) / sizeof(bench_forward_steps[0])),
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
static uint32_t bench_forward_duration_ms = ROUTE_FORWARD_MS;
static uint32_t bench_forward_last_elapsed_ms = 0U;

static void AppBenchmarkScript_Start(const BenchmarkScriptDef_t *script);
static void AppBenchmarkScript_ClearRuntime(void);
static void AppBenchmarkScript_StartStep(uint32_t now_ms);
static void AppBenchmarkScript_ApplyAction(const BenchmarkScriptStep_t *step);
static void AppBenchmarkScript_FinishStep(uint32_t now_ms);
static void AppBenchmarkScript_ResumeCurrentStep(uint32_t now_ms);
static void AppBenchmarkScript_Abort(BenchmarkScriptState_t state, const char *reason);
static void AppBenchmarkScript_ApplyForwardTrim(void);
static void AppBenchmarkScript_CheckForwardSafety(uint32_t now_ms);
static void AppBenchmarkScript_UpdateWaitObstacleClear(uint32_t now_ms);
static void AppBenchmarkScript_UpdateAuto(uint32_t now_ms);
static void AppBenchmarkScript_UpdateAutoForward(uint32_t now_ms);
static void AppBenchmarkScript_UpdateAutoTurnRight(uint32_t now_ms);
static void AppBenchmarkScript_UpdateAutoWaitClear(uint32_t now_ms);
static void AppBenchmarkScript_UpdateReturnAuto(uint32_t now_ms);
static void AppBenchmarkScript_UpdateReturnAutoTurnAround(uint32_t now_ms);
static void AppBenchmarkScript_UpdateReturnAutoForward(uint32_t now_ms);
static void AppBenchmarkScript_UpdateReturnAutoTurnLeft(uint32_t now_ms);
static void AppBenchmarkScript_UpdateReturnAutoWaitClear(uint32_t now_ms);
static void AppBenchmarkScript_EnterAutoForward(uint32_t now_ms, const char *reason);
static void AppBenchmarkScript_EnterAutoTurnRight(uint32_t now_ms, uint32_t front_mm);
static void AppBenchmarkScript_EnterAutoWaitClear(uint32_t now_ms, const char *reason);
static void AppBenchmarkScript_EnterReturnAutoTurnAround(uint32_t now_ms, const char *reason);
static void AppBenchmarkScript_EnterReturnAutoForward(uint32_t now_ms, const char *reason);
static void AppBenchmarkScript_EnterReturnAutoTurnLeft(uint32_t now_ms, uint32_t front_mm);
static void AppBenchmarkScript_EnterReturnAutoWaitClear(uint32_t now_ms, const char *reason);
static uint8_t AppBenchmarkScript_SetRoute(ManualRouteSlot_t slot, const char *tokens);
static uint8_t AppBenchmarkScript_AddRouteExpandedToken(char token,
                                                        uint8_t raw_index,
                                                        BenchmarkScriptStep_t *steps,
                                                        ManualRouteStepMeta_t *meta,
                                                        uint8_t *expanded_count);
static uint8_t AppBenchmarkScript_AddRoutePhase(char token,
                                                ManualRoutePhase_t phase,
                                                uint8_t raw_index,
                                                BenchmarkScriptStep_t *steps,
                                                ManualRouteStepMeta_t *meta,
                                                uint8_t *expanded_count);
static uint8_t AppBenchmarkScript_RouteTokenToStep(char token,
                                                   ManualRoutePhase_t phase,
                                                   BenchmarkScriptStep_t *step);
static uint8_t AppBenchmarkScript_IsRouteScript(const BenchmarkScriptDef_t *script);
static uint8_t AppBenchmarkScript_IsBenchScript(const BenchmarkScriptDef_t *script);
static char AppBenchmarkScript_CurrentRouteToken(void);
static uint8_t AppBenchmarkScript_CurrentRouteRawIndex(void);
static uint8_t AppBenchmarkScript_CurrentRouteRawCount(void);
static ManualRoutePhase_t AppBenchmarkScript_CurrentRoutePhase(void);
static const ManualRouteStepMeta_t *AppBenchmarkScript_CurrentRouteMeta(void);
static const char *AppBenchmarkScript_RoutePhaseName(ManualRoutePhase_t phase);
static const char *AppBenchmarkScript_RouteExpandedName(char token);
static void AppBenchmarkScript_LogRouteStepStart(const BenchmarkScriptStep_t *step);
static void AppBenchmarkScript_FormatRouteTokens(const char *tokens,
                                                 uint8_t count,
                                                 char *buffer,
                                                 uint32_t buffer_len);
static uint8_t AppBenchmarkScript_BenchFrontSafeToStart(uint32_t now_ms);
static const BenchmarkScriptStep_t *AppBenchmarkScript_CurrentStep(void);
static uint32_t AppBenchmarkScript_ElapsedMs(uint32_t now_ms, uint32_t then_ms);
static bool AppBenchmarkScript_LidarFrontClear(uint32_t now_ms);
static bool AppBenchmarkScript_LidarFrontBlocked(uint32_t now_ms);
static bool AppBenchmarkScript_LidarFrontStale(uint32_t now_ms);
static bool AppBenchmarkScript_AutoFrontClear(uint32_t now_ms);
static bool AppBenchmarkScript_AutoFrontBlocked(uint32_t now_ms);
static bool AppBenchmarkScript_IsAutoState(BenchmarkScriptState_t state);
static bool AppBenchmarkScript_IsReturnAutoState(BenchmarkScriptState_t state);
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
    APP_LOG("SCRIPT: auto start dutyL=%d dutyR=%d turn_duty=%d turn_ms=%u stop=%umm clear=%umm",
            (int)BENCH_FORWARD_LEFT_DUTY,
            (int)BENCH_FORWARD_RIGHT_DUTY,
            (int)AUTO_TURN_DUTY,
            (unsigned int)AUTO_TURN_RIGHT_MS,
            (unsigned int)AUTO_OBSTACLE_STOP_MM,
            (unsigned int)AUTO_OBSTACLE_CLEAR_MM);
    AppBenchmarkScript_EnterAutoForward(now_ms, "start");
}

void AppBenchmarkScript_StartReturnAuto(void)
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
        APP_LOG("SCRIPT: return_auto rejected reason=amr_state state=%s", AMR_StateName(AMR_GetState()));
        return;
    }

    if (AppFault_IsActive())
    {
        APP_LOG("SCRIPT: return_auto rejected reason=fault fault=%s", AppFault_Name(AppFault_Get()));
        return;
    }

    script_current = NULL;
    script_step_index = 0U;
    now_ms = HAL_GetTick();
    APP_LOG("SCRIPT: return_auto start dutyL=%d dutyR=%d turn_duty=%d turnaround_ms=%u turn_left_ms=%u stop=%umm clear=%umm",
            (int)BENCH_FORWARD_LEFT_DUTY,
            (int)BENCH_FORWARD_RIGHT_DUTY,
            (int)AUTO_TURN_DUTY,
            (unsigned int)RETURN_AUTO_TURN_AROUND_MS,
            (unsigned int)RETURN_AUTO_TURN_LEFT_MS,
            (unsigned int)AUTO_OBSTACLE_STOP_MM,
            (unsigned int)AUTO_OBSTACLE_CLEAR_MM);
    AppBenchmarkScript_EnterReturnAutoTurnAround(now_ms, "start");
}

void AppBenchmarkScript_SetRouteExit(const char *tokens)
{
    (void)AppBenchmarkScript_SetRoute(ROUTE_SLOT_EXIT, tokens);
}

void AppBenchmarkScript_SetRouteReturn(const char *tokens)
{
    (void)AppBenchmarkScript_SetRoute(ROUTE_SLOT_RETURN, tokens);
}

void AppBenchmarkScript_StartRouteExit(void)
{
    if (route_exit_script.count == 0U)
    {
        APP_LOG("ROUTE: run_exit rejected reason=empty_route");
        return;
    }

    APP_LOG("ROUTE: run_exit raw_count=%u expanded_count=%u centered_turn=enabled",
            (unsigned int)route_exit_token_count,
            (unsigned int)route_exit_script.count);
    AppBenchmarkScript_Start(&route_exit_script);
}

void AppBenchmarkScript_StartRouteReturn(void)
{
    if (route_return_script.count == 0U)
    {
        APP_LOG("ROUTE: run_return rejected reason=empty_route");
        return;
    }

    APP_LOG("ROUTE: run_return raw_count=%u expanded_count=%u centered_turn=enabled",
            (unsigned int)route_return_token_count,
            (unsigned int)route_return_script.count);
    AppBenchmarkScript_Start(&route_return_script);
}

uint8_t AppBenchmarkRoute_HasExit(void)
{
    return (route_exit_token_count != 0U) ? 1U : 0U;
}

uint8_t AppBenchmarkRoute_HasReturn(void)
{
    return (route_return_token_count != 0U) ? 1U : 0U;
}

void AppBenchmarkRoute_RunExit(void)
{
    AppBenchmarkScript_StartRouteExit();
}

void AppBenchmarkRoute_RunReturn(void)
{
    AppBenchmarkScript_StartRouteReturn();
}

uint8_t AppBenchmarkRoute_IsActive(void)
{
    return ((AppBenchmarkScript_IsRouteScript(script_current) != 0U) &&
            AppBenchmarkScript_IsActiveState(script_state)) ? 1U : 0U;
}

uint8_t AppBenchmarkRoute_IsReturnActive(void)
{
    return ((script_current == &route_return_script) &&
            AppBenchmarkScript_IsActiveState(script_state)) ? 1U : 0U;
}

void AppBenchmarkRoute_Stop(const char *reason)
{
    if (AppBenchmarkRoute_IsActive() != 0U)
    {
        AppBenchmarkScript_Stop((reason != NULL) ? reason : "route_stop");
    }

    Chassis_Stop();
}

void AppBenchmarkScript_PrintRouteStatus(void)
{
    char exit_text[ROUTE_TEXT_MAX];
    char return_text[ROUTE_TEXT_MAX];
    const BenchmarkScriptStep_t *step = AppBenchmarkScript_CurrentStep();
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = 0U;
    uint32_t remaining_ms = 0U;
    uint8_t display_index = 0U;
    uint8_t total_count = 0U;
    uint8_t raw_index = 0U;
    uint8_t raw_count = 0U;
    char token = '-';
    const char *route_name = "none";
    const char *action_name = "NONE";
    const char *phase_name = "none";

    AppBenchmarkScript_FormatRouteTokens(route_exit_tokens,
                                         route_exit_token_count,
                                         exit_text,
                                         (uint32_t)sizeof(exit_text));
    AppBenchmarkScript_FormatRouteTokens(route_return_tokens,
                                         route_return_token_count,
                                         return_text,
                                         (uint32_t)sizeof(return_text));

    if (AppBenchmarkScript_IsRouteScript(script_current) != 0U)
    {
        route_name = script_current->name;
        total_count = script_current->count;
        raw_count = AppBenchmarkScript_CurrentRouteRawCount();
        if (step != NULL)
        {
            display_index = (uint8_t)(script_step_index + 1U);
            token = AppBenchmarkScript_CurrentRouteToken();
            raw_index = AppBenchmarkScript_CurrentRouteRawIndex();
            phase_name = AppBenchmarkScript_RoutePhaseName(AppBenchmarkScript_CurrentRoutePhase());
            if (script_current_action_valid != 0U)
            {
                action_name = AppBenchmarkScript_ActionName(script_current_action);
            }

            if (script_step_active != 0U)
            {
                elapsed_ms = AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms);
            }

            remaining_ms = (elapsed_ms < step->duration_ms) ? (step->duration_ms - elapsed_ms) : 0U;
        }
        else
        {
            display_index = total_count;
        }
    }

    APP_LOG("ROUTE: exit raw_count=%u expanded_count=%u tokens=%s centered_turn=enabled",
            (unsigned int)route_exit_token_count,
            (unsigned int)route_exit_script.count,
            exit_text);
    APP_LOG("ROUTE: return raw_count=%u expanded_count=%u tokens=%s centered_turn=enabled",
            (unsigned int)route_return_token_count,
            (unsigned int)route_return_script.count,
            return_text);
    APP_LOG("ROUTE: status state=%s route=%s index=%u/%u raw_index=%u/%u token=%c action=%s phase=%s elapsed=%lums remaining=%lums centered_turn=enabled",
            AppBenchmarkScript_StateName(script_state),
            route_name,
            (unsigned int)display_index,
            (unsigned int)total_count,
            (unsigned int)raw_index,
            (unsigned int)raw_count,
            token,
            action_name,
            phase_name,
            (unsigned long)elapsed_ms,
            (unsigned long)remaining_ms);
}

void AppBenchmarkScript_ClearRoutes(void)
{
    if (AppBenchmarkScript_IsActiveState(script_state))
    {
        APP_LOG("ROUTE: clear rejected reason=busy state=%s", AppBenchmarkScript_StateName(script_state));
        return;
    }

    route_exit_script.count = 0U;
    route_return_script.count = 0U;
    route_exit_token_count = 0U;
    route_return_token_count = 0U;
    memset(route_exit_tokens, 0, sizeof(route_exit_tokens));
    memset(route_return_tokens, 0, sizeof(route_return_tokens));
    if (AppBenchmarkScript_IsRouteScript(script_current) != 0U)
    {
        script_current = NULL;
        script_step_index = 0U;
        script_state = SCRIPT_IDLE;
        AppBenchmarkScript_ClearRuntime();
    }

    APP_LOG("ROUTE: cleared max_tokens=%u", (unsigned int)ROUTE_MAX_TOKENS);
}

void AppBenchmarkScript_StartBenchForward(uint32_t duration_ms)
{
    uint32_t now_ms;

    if (script_initialized == 0U)
    {
        AppBenchmarkScript_Init();
    }

    if ((duration_ms < BENCH_FWD_MIN_MS) || (duration_ms > BENCH_FWD_MAX_MS))
    {
        APP_LOG("BENCH: fwd rejected reason=duration_range duration=%lums min=%u max=%u",
                (unsigned long)duration_ms,
                (unsigned int)BENCH_FWD_MIN_MS,
                (unsigned int)BENCH_FWD_MAX_MS);
        return;
    }

    if (AppBenchmarkScript_IsActiveState(script_state))
    {
        APP_LOG("BENCH: fwd rejected reason=busy state=%s", AppBenchmarkScript_StateName(script_state));
        return;
    }

    if (AMR_GetState() != AMR_STATE_IDLE)
    {
        APP_LOG("BENCH: fwd rejected reason=amr_state state=%s", AMR_StateName(AMR_GetState()));
        return;
    }

    if (AppFault_IsActive())
    {
        APP_LOG("BENCH: fwd rejected reason=fault fault=%s", AppFault_Name(AppFault_Get()));
        return;
    }

    now_ms = HAL_GetTick();
    if (AppBenchmarkScript_BenchFrontSafeToStart(now_ms) == 0U)
    {
        return;
    }

    bench_forward_duration_ms = duration_ms;
    bench_forward_last_elapsed_ms = 0U;
    bench_forward_steps[0].duration_ms = duration_ms;
    APP_LOG("BENCH: fwd start duration=%lums dutyL=%d dutyR=%d front_reject=%umm",
            (unsigned long)duration_ms,
            (int)BENCH_FORWARD_LEFT_DUTY,
            (int)BENCH_FORWARD_RIGHT_DUTY,
            (unsigned int)BENCH_FWD_REJECT_FRONT_MM);
    AppBenchmarkScript_Start(&bench_forward_script);
}

void AppBenchmarkScript_PrintBenchStatus(void)
{
    const BenchmarkScriptStep_t *step = AppBenchmarkScript_CurrentStep();
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = 0U;
    uint32_t remaining_ms = 0U;

    if (AppBenchmarkScript_IsBenchScript(script_current) != 0U)
    {
        if (step != NULL)
        {
            if (script_step_active != 0U)
            {
                elapsed_ms = AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms);
            }

            remaining_ms = (elapsed_ms < bench_forward_duration_ms) ? (bench_forward_duration_ms - elapsed_ms) : 0U;
        }
        else if (script_state == SCRIPT_DONE)
        {
            elapsed_ms = bench_forward_last_elapsed_ms;
        }
    }

    APP_LOG("BENCH: status state=%s duration=%lums elapsed=%lums remaining=%lums",
            AppBenchmarkScript_StateName(script_state),
            (unsigned long)bench_forward_duration_ms,
            (unsigned long)elapsed_ms,
            (unsigned long)remaining_ms);
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

    if (AppBenchmarkScript_IsReturnAutoState(script_state))
    {
        AppBenchmarkScript_UpdateReturnAuto(now_ms);
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
    else if ((AppBenchmarkScript_IsAutoState(script_state) ||
              AppBenchmarkScript_IsReturnAutoState(script_state)) &&
             (script_current_action_valid != 0U))
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
    else if (script_state == SCRIPT_RETURN_AUTO_TURN_AROUND)
    {
        remaining_ms = (elapsed_ms < RETURN_AUTO_TURN_AROUND_MS) ? (RETURN_AUTO_TURN_AROUND_MS - elapsed_ms) : 0U;
    }
    else if (script_state == SCRIPT_RETURN_AUTO_TURN_LEFT)
    {
        remaining_ms = (elapsed_ms < RETURN_AUTO_TURN_LEFT_MS) ? (RETURN_AUTO_TURN_LEFT_MS - elapsed_ms) : 0U;
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
            ((script_current == &return_script) || (script_current == &route_return_script))) ? 1U : 0U;
}

uint8_t AppBenchmarkScript_IsReturnAutoActive(void)
{
    return AppBenchmarkScript_IsReturnAutoState(script_state) ? 1U : 0U;
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
        case SCRIPT_RETURN_AUTO_TURN_AROUND:
            return "SCRIPT_RETURN_AUTO_TURN_AROUND";
        case SCRIPT_RETURN_AUTO_FORWARD:
            return "SCRIPT_RETURN_AUTO_FORWARD";
        case SCRIPT_RETURN_AUTO_TURN_LEFT:
            return "SCRIPT_RETURN_AUTO_TURN_LEFT";
        case SCRIPT_RETURN_AUTO_WAIT_CLEAR:
            return "SCRIPT_RETURN_AUTO_WAIT_CLEAR";
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
        case SCRIPT_ACT_TURN_AROUND:
            return "TURN_AROUND";
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
    AppBenchmarkScript_LogRouteStepStart(step);
    if (step->action == SCRIPT_ACT_FORWARD)
    {
        APP_LOG("SCRIPT: step=%u/%u action=%s duty=%d dutyL=%d dutyR=%d dur=%lums note=%s",
                (unsigned int)(script_step_index + 1U),
                (unsigned int)((script_current != NULL) ? script_current->count : 0U),
                AppBenchmarkScript_ActionName(step->action),
                (int)step->duty,
                (int)BENCH_FORWARD_LEFT_DUTY,
                (int)BENCH_FORWARD_RIGHT_DUTY,
                (unsigned long)step->duration_ms,
                (step->note != NULL) ? step->note : "");
    }
    else
    {
        APP_LOG("SCRIPT: step=%u/%u action=%s duty=%d dur=%lums note=%s",
                (unsigned int)(script_step_index + 1U),
                (unsigned int)((script_current != NULL) ? script_current->count : 0U),
                AppBenchmarkScript_ActionName(step->action),
                (int)step->duty,
                (unsigned long)step->duration_ms,
                (step->note != NULL) ? step->note : "");
    }
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
            AppBenchmarkScript_ApplyForwardTrim();
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

        case SCRIPT_ACT_TURN_AROUND:
            Chassis_TurnRight(step->duty);
            break;

        default:
            Chassis_Stop();
            break;
    }
}

static void AppBenchmarkScript_ApplyForwardTrim(void)
{
    Chassis_SetRaw(BENCH_FORWARD_LEFT_DUTY, BENCH_FORWARD_RIGHT_DUTY);
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

    if (AppBenchmarkScript_IsBenchScript(script_current) != 0U)
    {
        bench_forward_last_elapsed_ms = elapsed_ms;
    }

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

    AppBenchmarkScript_ApplyForwardTrim();
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

static void AppBenchmarkScript_UpdateReturnAuto(uint32_t now_ms)
{
    switch (script_state)
    {
        case SCRIPT_RETURN_AUTO_TURN_AROUND:
            AppBenchmarkScript_UpdateReturnAutoTurnAround(now_ms);
            break;

        case SCRIPT_RETURN_AUTO_FORWARD:
            AppBenchmarkScript_UpdateReturnAutoForward(now_ms);
            break;

        case SCRIPT_RETURN_AUTO_TURN_LEFT:
            AppBenchmarkScript_UpdateReturnAutoTurnLeft(now_ms);
            break;

        case SCRIPT_RETURN_AUTO_WAIT_CLEAR:
            AppBenchmarkScript_UpdateReturnAutoWaitClear(now_ms);
            break;

        default:
            break;
    }
}

static void AppBenchmarkScript_UpdateReturnAutoTurnAround(uint32_t now_ms)
{
    if (script_state != SCRIPT_RETURN_AUTO_TURN_AROUND)
    {
        return;
    }

    if (AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms) >= RETURN_AUTO_TURN_AROUND_MS)
    {
        AppBenchmarkScript_EnterReturnAutoForward(now_ms, "turnaround_done");
        return;
    }

    Chassis_TurnRight(AUTO_TURN_DUTY);
}

static void AppBenchmarkScript_UpdateReturnAutoForward(uint32_t now_ms)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();

    if (script_state != SCRIPT_RETURN_AUTO_FORWARD)
    {
        return;
    }

    if (AppBenchmarkScript_LidarFrontStale(now_ms))
    {
        AppBenchmarkScript_EnterReturnAutoWaitClear(now_ms, "lidar_invalid_or_stale");
        return;
    }

    if (AppBenchmarkScript_AutoFrontBlocked(now_ms))
    {
        uint32_t front_mm = (lidar != NULL) ? lidar->front_min_mm : 0U;
        Chassis_Stop();
        AppBenchmarkScript_EnterReturnAutoTurnLeft(now_ms, front_mm);
        return;
    }

    AppBenchmarkScript_ApplyForwardTrim();
    AppBenchmarkScript_LogLowRate(now_ms, "SCRIPT: return_auto forward");
}

static void AppBenchmarkScript_UpdateReturnAutoTurnLeft(uint32_t now_ms)
{
    if (script_state != SCRIPT_RETURN_AUTO_TURN_LEFT)
    {
        return;
    }

    if (AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms) >= RETURN_AUTO_TURN_LEFT_MS)
    {
        APP_LOG("SCRIPT: return_auto turn_left done elapsed=%lums",
                (unsigned long)AppBenchmarkScript_ElapsedMs(now_ms, script_step_start_ms));
        AppBenchmarkScript_EnterReturnAutoForward(now_ms, "turn_done");
        return;
    }

    Chassis_TurnLeft(AUTO_TURN_DUTY);
}

static void AppBenchmarkScript_UpdateReturnAutoWaitClear(uint32_t now_ms)
{
    if (script_state != SCRIPT_RETURN_AUTO_WAIT_CLEAR)
    {
        return;
    }

    Chassis_Stop();
    if (AppBenchmarkScript_AutoFrontClear(now_ms))
    {
        AppBenchmarkScript_EnterReturnAutoForward(now_ms, "front_clear");
        return;
    }

    AppBenchmarkScript_LogLowRate(now_ms, "SCRIPT: return_auto waiting clear");
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
    AppBenchmarkScript_ApplyForwardTrim();
    APP_LOG("SCRIPT: auto state=FORWARD dutyL=%d dutyR=%d reason=%s",
            (int)BENCH_FORWARD_LEFT_DUTY,
            (int)BENCH_FORWARD_RIGHT_DUTY,
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

static void AppBenchmarkScript_EnterReturnAutoTurnAround(uint32_t now_ms, const char *reason)
{
    script_current = NULL;
    script_step_index = 0U;
    script_state = SCRIPT_RETURN_AUTO_TURN_AROUND;
    script_step_start_ms = now_ms;
    script_step_active = 1U;
    script_step_just_started = 0U;
    script_wait_start_ms = 0U;
    script_current_action = SCRIPT_ACT_TURN_RIGHT;
    script_current_action_valid = 1U;
    Chassis_TurnRight(AUTO_TURN_DUTY);
    APP_LOG("SCRIPT: return_auto state=TURN_AROUND duty=%d dur=%ums reason=%s",
            (int)AUTO_TURN_DUTY,
            (unsigned int)RETURN_AUTO_TURN_AROUND_MS,
            (reason != NULL) ? reason : "none");
}

static void AppBenchmarkScript_EnterReturnAutoForward(uint32_t now_ms, const char *reason)
{
    script_current = NULL;
    script_step_index = 0U;
    script_state = SCRIPT_RETURN_AUTO_FORWARD;
    script_step_start_ms = now_ms;
    script_step_active = 1U;
    script_step_just_started = 0U;
    script_wait_start_ms = 0U;
    script_current_action = SCRIPT_ACT_FORWARD;
    script_current_action_valid = 1U;
    AppBenchmarkScript_ApplyForwardTrim();
    APP_LOG("SCRIPT: return_auto state=FORWARD dutyL=%d dutyR=%d reason=%s",
            (int)BENCH_FORWARD_LEFT_DUTY,
            (int)BENCH_FORWARD_RIGHT_DUTY,
            (reason != NULL) ? reason : "none");
    AppBenchmarkScript_UpdateReturnAutoForward(now_ms);
}

static void AppBenchmarkScript_EnterReturnAutoTurnLeft(uint32_t now_ms, uint32_t front_mm)
{
    script_current = NULL;
    script_step_index = 0U;
    script_state = SCRIPT_RETURN_AUTO_TURN_LEFT;
    script_step_start_ms = now_ms;
    script_step_active = 1U;
    script_step_just_started = 0U;
    script_wait_start_ms = 0U;
    script_current_action = SCRIPT_ACT_TURN_LEFT;
    script_current_action_valid = 1U;
    Chassis_TurnLeft(AUTO_TURN_DUTY);
    APP_LOG("SCRIPT: return_auto state=TURN_LEFT duty=%d dur=%ums front=%u stop=%u",
            (int)AUTO_TURN_DUTY,
            (unsigned int)RETURN_AUTO_TURN_LEFT_MS,
            (unsigned int)front_mm,
            (unsigned int)AUTO_OBSTACLE_STOP_MM);
}

static void AppBenchmarkScript_EnterReturnAutoWaitClear(uint32_t now_ms, const char *reason)
{
    script_current = NULL;
    script_step_index = 0U;
    script_state = SCRIPT_RETURN_AUTO_WAIT_CLEAR;
    script_step_start_ms = now_ms;
    script_wait_start_ms = now_ms;
    script_step_active = 1U;
    script_step_just_started = 0U;
    script_current_action = SCRIPT_ACT_WAIT;
    script_current_action_valid = 1U;
    Chassis_Stop();
    APP_LOG("SCRIPT: return_auto state=WAIT_CLEAR reason=%s clear=%u",
            (reason != NULL) ? reason : "none",
            (unsigned int)AUTO_OBSTACLE_CLEAR_MM);
}

static uint8_t AppBenchmarkScript_SetRoute(ManualRouteSlot_t slot, const char *tokens)
{
    uint8_t parsed_count = 0U;
    uint8_t expanded_count = 0U;
    const char *cursor = tokens;
    BenchmarkScriptStep_t *target_steps;
    ManualRouteStepMeta_t *target_meta;
    char *target_tokens;
    uint8_t *target_token_count;
    BenchmarkScriptDef_t *target_script;
    const char *target_name;

    if (script_initialized == 0U)
    {
        AppBenchmarkScript_Init();
    }

    if (AppBenchmarkScript_IsActiveState(script_state))
    {
        APP_LOG("ROUTE: set rejected reason=busy state=%s", AppBenchmarkScript_StateName(script_state));
        return 0U;
    }

    if ((tokens == NULL) || (tokens[0] == '\0'))
    {
        APP_LOG("ROUTE: set rejected reason=empty_tokens");
        return 0U;
    }

    while ((cursor != NULL) && (*cursor != '\0'))
    {
        char ch = *cursor;

        if ((ch == ' ') || (ch == ','))
        {
            cursor++;
            continue;
        }

        if ((ch >= 'A') && (ch <= 'Z'))
        {
            ch = (char)(ch + ('a' - 'A'));
        }

        if (parsed_count >= ROUTE_MAX_TOKENS)
        {
            APP_LOG("ROUTE: set rejected reason=too_many_tokens max=%u",
                    (unsigned int)ROUTE_MAX_TOKENS);
            return 0U;
        }

        if (AppBenchmarkScript_AddRouteExpandedToken(ch,
                                                     parsed_count,
                                                     route_parse_steps,
                                                     route_parse_meta,
                                                     &expanded_count) == 0U)
        {
            APP_LOG("ROUTE: set rejected reason=bad_token token=%c valid=F,H,L,R,U,W,S", ch);
            return 0U;
        }

        route_parse_tokens[parsed_count] = ch;
        parsed_count++;
        cursor++;
    }

    if (parsed_count == 0U)
    {
        APP_LOG("ROUTE: set rejected reason=empty_tokens");
        return 0U;
    }

    if (slot == ROUTE_SLOT_RETURN)
    {
        target_steps = route_return_steps;
        target_meta = route_return_meta;
        target_tokens = route_return_tokens;
        target_token_count = &route_return_token_count;
        target_script = &route_return_script;
        target_name = "return";
    }
    else
    {
        target_steps = route_exit_steps;
        target_meta = route_exit_meta;
        target_tokens = route_exit_tokens;
        target_token_count = &route_exit_token_count;
        target_script = &route_exit_script;
        target_name = "exit";
    }

    memcpy(target_steps, route_parse_steps, (size_t)expanded_count * sizeof(route_parse_steps[0]));
    memcpy(target_meta, route_parse_meta, (size_t)expanded_count * sizeof(route_parse_meta[0]));
    memcpy(target_tokens, route_parse_tokens, (size_t)parsed_count * sizeof(route_parse_tokens[0]));
    if (parsed_count < ROUTE_MAX_TOKENS)
    {
        memset(&target_tokens[parsed_count], 0, (size_t)(ROUTE_MAX_TOKENS - parsed_count));
    }

    target_script->count = expanded_count;
    *target_token_count = parsed_count;

    APP_LOG("ROUTE: set_%s raw_count=%u expanded_count=%u max_tokens=%u max_steps=%u centered_turn=enabled F=%ums H=%ums L=%ums R=%ums U=%ums turn_duty=%d stop=%ums",
            target_name,
            (unsigned int)parsed_count,
            (unsigned int)expanded_count,
            (unsigned int)ROUTE_MAX_TOKENS,
            (unsigned int)ROUTE_MAX_STEPS,
            (unsigned int)ROUTE_FORWARD_MS,
            (unsigned int)ROUTE_HALF_FORWARD_MS,
            (unsigned int)ROUTE_TURN_LEFT_MS,
            (unsigned int)ROUTE_TURN_RIGHT_MS,
            (unsigned int)RETURN_AUTO_TURN_AROUND_MS,
            (int)AUTO_TURN_DUTY,
            (unsigned int)ROUTE_STOP_MS);
    return 1U;
}

static uint8_t AppBenchmarkScript_AddRouteExpandedToken(char token,
                                                        uint8_t raw_index,
                                                        BenchmarkScriptStep_t *steps,
                                                        ManualRouteStepMeta_t *meta,
                                                        uint8_t *expanded_count)
{
    char normalized = token;

    if ((steps == NULL) || (meta == NULL) || (expanded_count == NULL))
    {
        return 0U;
    }

    if ((normalized >= 'A') && (normalized <= 'Z'))
    {
        normalized = (char)(normalized + ('a' - 'A'));
    }

    if ((normalized == 'l') || (normalized == 'r'))
    {
        if (AppBenchmarkScript_AddRoutePhase(normalized,
                                             ROUTE_PHASE_PRE_HALF,
                                             raw_index,
                                             steps,
                                             meta,
                                             expanded_count) == 0U)
        {
            return 0U;
        }

        if (AppBenchmarkScript_AddRoutePhase(normalized,
                                             ROUTE_PHASE_TURN,
                                             raw_index,
                                             steps,
                                             meta,
                                             expanded_count) == 0U)
        {
            return 0U;
        }

        return AppBenchmarkScript_AddRoutePhase(normalized,
                                                ROUTE_PHASE_POST_HALF,
                                                raw_index,
                                                steps,
                                                meta,
                                                expanded_count);
    }

    return AppBenchmarkScript_AddRoutePhase(normalized,
                                            ROUTE_PHASE_SINGLE,
                                            raw_index,
                                            steps,
                                            meta,
                                            expanded_count);
}

static uint8_t AppBenchmarkScript_AddRoutePhase(char token,
                                                ManualRoutePhase_t phase,
                                                uint8_t raw_index,
                                                BenchmarkScriptStep_t *steps,
                                                ManualRouteStepMeta_t *meta,
                                                uint8_t *expanded_count)
{
    if ((steps == NULL) || (meta == NULL) || (expanded_count == NULL))
    {
        return 0U;
    }

    if (*expanded_count >= ROUTE_MAX_STEPS)
    {
        APP_LOG("ROUTE: set rejected reason=too_many_expanded_steps max=%u",
                (unsigned int)ROUTE_MAX_STEPS);
        return 0U;
    }

    if (AppBenchmarkScript_RouteTokenToStep(token, phase, &steps[*expanded_count]) == 0U)
    {
        return 0U;
    }

    meta[*expanded_count].token = token;
    meta[*expanded_count].phase = phase;
    meta[*expanded_count].raw_index = raw_index;
    (*expanded_count)++;

    return 1U;
}

static uint8_t AppBenchmarkScript_RouteTokenToStep(char token,
                                                   ManualRoutePhase_t phase,
                                                   BenchmarkScriptStep_t *step)
{
    char normalized = token;

    if (step == NULL)
    {
        return 0U;
    }

    if ((normalized >= 'A') && (normalized <= 'Z'))
    {
        normalized = (char)(normalized + ('a' - 'A'));
    }

    switch (normalized)
    {
        case 'f':
            if (phase != ROUTE_PHASE_SINGLE)
            {
                return 0U;
            }

            step->action = SCRIPT_ACT_FORWARD;
            step->duty = BENCH_FORWARD_RIGHT_DUTY;
            step->duration_ms = ROUTE_FORWARD_MS;
            step->note = "manual route forward";
            return 1U;

        case 'h':
            if ((phase != ROUTE_PHASE_SINGLE) &&
                (phase != ROUTE_PHASE_PRE_HALF) &&
                (phase != ROUTE_PHASE_POST_HALF))
            {
                return 0U;
            }

            step->action = SCRIPT_ACT_FORWARD;
            step->duty = BENCH_FORWARD_RIGHT_DUTY;
            step->duration_ms = ROUTE_HALF_FORWARD_MS;
            step->note = "manual route half forward";
            return 1U;

        case 'l':
            if (phase == ROUTE_PHASE_PRE_HALF)
            {
                return AppBenchmarkScript_RouteTokenToStep('h', phase, step);
            }

            if (phase == ROUTE_PHASE_POST_HALF)
            {
                return AppBenchmarkScript_RouteTokenToStep('h', phase, step);
            }

            if (phase != ROUTE_PHASE_TURN)
            {
                return 0U;
            }

            step->action = SCRIPT_ACT_TURN_LEFT;
            step->duty = AUTO_TURN_DUTY;
            step->duration_ms = ROUTE_TURN_LEFT_MS;
            step->note = "manual route left";
            return 1U;

        case 'r':
            if (phase == ROUTE_PHASE_PRE_HALF)
            {
                return AppBenchmarkScript_RouteTokenToStep('h', phase, step);
            }

            if (phase == ROUTE_PHASE_POST_HALF)
            {
                return AppBenchmarkScript_RouteTokenToStep('h', phase, step);
            }

            if (phase != ROUTE_PHASE_TURN)
            {
                return 0U;
            }

            step->action = SCRIPT_ACT_TURN_RIGHT;
            step->duty = AUTO_TURN_DUTY;
            step->duration_ms = ROUTE_TURN_RIGHT_MS;
            step->note = "manual route right";
            return 1U;

        case 'u':
            if (phase != ROUTE_PHASE_SINGLE)
            {
                return 0U;
            }

            step->action = SCRIPT_ACT_TURN_AROUND;
            step->duty = AUTO_TURN_DUTY;
            step->duration_ms = RETURN_AUTO_TURN_AROUND_MS;
            step->note = "manual route turnaround";
            return 1U;

        case 'w':
            if (phase != ROUTE_PHASE_SINGLE)
            {
                return 0U;
            }

            step->action = SCRIPT_ACT_WAIT;
            step->duty = 0;
            step->duration_ms = ROUTE_WAIT_MS;
            step->note = "manual route wait";
            return 1U;

        case 's':
            if (phase != ROUTE_PHASE_SINGLE)
            {
                return 0U;
            }

            step->action = SCRIPT_ACT_STOP;
            step->duty = 0;
            step->duration_ms = ROUTE_STOP_MS;
            step->note = "manual route stop";
            return 1U;

        default:
            return 0U;
    }
}

static uint8_t AppBenchmarkScript_IsRouteScript(const BenchmarkScriptDef_t *script)
{
    return ((script == &route_exit_script) || (script == &route_return_script)) ? 1U : 0U;
}

static uint8_t AppBenchmarkScript_IsBenchScript(const BenchmarkScriptDef_t *script)
{
    return (script == &bench_forward_script) ? 1U : 0U;
}

static char AppBenchmarkScript_CurrentRouteToken(void)
{
    const ManualRouteStepMeta_t *meta = AppBenchmarkScript_CurrentRouteMeta();

    if (meta != NULL)
    {
        return (char)(meta->token - ('a' - 'A'));
    }

    return '-';
}

static uint8_t AppBenchmarkScript_CurrentRouteRawIndex(void)
{
    const ManualRouteStepMeta_t *meta = AppBenchmarkScript_CurrentRouteMeta();

    if (meta != NULL)
    {
        return (uint8_t)(meta->raw_index + 1U);
    }

    return 0U;
}

static uint8_t AppBenchmarkScript_CurrentRouteRawCount(void)
{
    if (script_current == &route_exit_script)
    {
        return route_exit_token_count;
    }

    if (script_current == &route_return_script)
    {
        return route_return_token_count;
    }

    return 0U;
}

static ManualRoutePhase_t AppBenchmarkScript_CurrentRoutePhase(void)
{
    const ManualRouteStepMeta_t *meta = AppBenchmarkScript_CurrentRouteMeta();

    if (meta != NULL)
    {
        return meta->phase;
    }

    return ROUTE_PHASE_SINGLE;
}

static const ManualRouteStepMeta_t *AppBenchmarkScript_CurrentRouteMeta(void)
{
    if ((script_current == &route_exit_script) && (script_step_index < route_exit_script.count))
    {
        return &route_exit_meta[script_step_index];
    }

    if ((script_current == &route_return_script) && (script_step_index < route_return_script.count))
    {
        return &route_return_meta[script_step_index];
    }

    return NULL;
}

static const char *AppBenchmarkScript_RoutePhaseName(ManualRoutePhase_t phase)
{
    switch (phase)
    {
        case ROUTE_PHASE_SINGLE:
            return "single";
        case ROUTE_PHASE_PRE_HALF:
            return "pre_half";
        case ROUTE_PHASE_TURN:
            return "turn";
        case ROUTE_PHASE_POST_HALF:
            return "post_half";
        default:
            return "unknown";
    }
}

static const char *AppBenchmarkScript_RouteExpandedName(char token)
{
    char normalized = token;

    if ((normalized >= 'A') && (normalized <= 'Z'))
    {
        normalized = (char)(normalized + ('a' - 'A'));
    }

    switch (normalized)
    {
        case 'l':
            return "H,L,H";
        case 'r':
            return "H,R,H";
        case 'f':
            return "F";
        case 'h':
            return "H";
        case 'u':
            return "U";
        case 'w':
            return "W";
        case 's':
            return "S";
        default:
            return "?";
    }
}

static void AppBenchmarkScript_LogRouteStepStart(const BenchmarkScriptStep_t *step)
{
    const ManualRouteStepMeta_t *meta = AppBenchmarkScript_CurrentRouteMeta();

    if ((meta == NULL) || (step == NULL))
    {
        return;
    }

    APP_LOG("ROUTE: token=%c expanded=%s phase=%s action=%s dur=%lums",
            (char)(meta->token - ('a' - 'A')),
            AppBenchmarkScript_RouteExpandedName(meta->token),
            AppBenchmarkScript_RoutePhaseName(meta->phase),
            AppBenchmarkScript_ActionName(step->action),
            (unsigned long)step->duration_ms);
}

static void AppBenchmarkScript_FormatRouteTokens(const char *tokens,
                                                 uint8_t count,
                                                 char *buffer,
                                                 uint32_t buffer_len)
{
    uint32_t pos = 0U;
    uint8_t i;

    if ((buffer == NULL) || (buffer_len == 0U))
    {
        return;
    }

    if ((tokens == NULL) || (count == 0U))
    {
        (void)snprintf(buffer, buffer_len, "(empty)");
        return;
    }

    for (i = 0U; i < count; i++)
    {
        char token = (char)(tokens[i] - ('a' - 'A'));

        if ((i != 0U) && (pos < (buffer_len - 1U)))
        {
            buffer[pos] = ',';
            pos++;
        }

        if (pos < (buffer_len - 1U))
        {
            buffer[pos] = token;
            pos++;
        }
    }

    buffer[pos] = '\0';
}

static uint8_t AppBenchmarkScript_BenchFrontSafeToStart(uint32_t now_ms)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();

    if (AppBenchmarkScript_LidarFrontStale(now_ms))
    {
        APP_LOG("BENCH: fwd rejected reason=lidar_invalid_or_stale");
        return 0U;
    }

    if ((lidar != NULL) && (lidar->front_valid != 0U) && (lidar->front_min_mm < BENCH_FWD_REJECT_FRONT_MM))
    {
        APP_LOG("BENCH: fwd rejected reason=front_too_close front=%u min=%u",
                (unsigned int)lidar->front_min_mm,
                (unsigned int)BENCH_FWD_REJECT_FRONT_MM);
        return 0U;
    }

    return 1U;
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

static bool AppBenchmarkScript_IsReturnAutoState(BenchmarkScriptState_t state)
{
    return ((state == SCRIPT_RETURN_AUTO_TURN_AROUND) ||
            (state == SCRIPT_RETURN_AUTO_FORWARD) ||
            (state == SCRIPT_RETURN_AUTO_TURN_LEFT) ||
            (state == SCRIPT_RETURN_AUTO_WAIT_CLEAR));
}

static bool AppBenchmarkScript_IsActiveState(BenchmarkScriptState_t state)
{
    return ((state == SCRIPT_RUNNING) ||
            (state == SCRIPT_WAIT_OBSTACLE_CLEAR) ||
            AppBenchmarkScript_IsAutoState(state) ||
            AppBenchmarkScript_IsReturnAutoState(state));
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

    if (AppBenchmarkScript_IsReturnAutoState(script_state))
    {
        return "script_return_auto";
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
