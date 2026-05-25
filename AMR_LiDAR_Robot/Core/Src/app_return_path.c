#include "app_return_path.h"

#include "amr_system.h"
#include "bringup_log.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"

#include <stddef.h>

#define RETURN_EXEC_LINEAR_PWM 320
#define RETURN_EXEC_TURN_PWM 360
#define RETURN_EXEC_STOP_BETWEEN_MS 100U

typedef struct
{
    PathAction_t action;
    uint32_t duration_ms;
    int16_t left_pwm;
    int16_t right_pwm;
} ReturnPathEntry_t;

typedef enum
{
    RETURN_PHASE_IDLE = 0,
    RETURN_PHASE_ACTION,
    RETURN_PHASE_STOP_GAP
} ReturnPhase_t;

static ReturnPathEntry_t return_path_stack[PATH_STACK_MAX];
static uint16_t return_path_count = 0U;
static uint8_t return_path_initialized = 0U;

static ReturnExecState_t return_exec_state = RETURN_EXEC_IDLE;
static ReturnPhase_t return_exec_phase = RETURN_PHASE_IDLE;
static PathAction_t return_exec_action = PATH_ACT_NONE;
static uint32_t return_exec_duration_ms = 0U;
static uint32_t return_exec_phase_start_ms = 0U;
static int16_t return_exec_left_pwm = 0;
static int16_t return_exec_right_pwm = 0;

static void ReturnPath_EnsureInitialized(void);
static uint32_t ReturnPath_ElapsedMs(uint32_t now_ms, uint32_t then_ms);
static PathAction_t ReturnPath_InverseAction(PathAction_t action);
static void ReturnPath_GetInversePwm(PathAction_t action, int16_t *left_pwm, int16_t *right_pwm);
static void ReturnExecutor_StartNextAction(uint32_t now_ms);
static void ReturnExecutor_ApplyCurrentAction(void);

void ReturnPath_Init(void)
{
    return_path_count = 0U;
    return_path_initialized = 1U;

    return_exec_state = RETURN_EXEC_IDLE;
    return_exec_phase = RETURN_PHASE_IDLE;
    return_exec_action = PATH_ACT_NONE;
    return_exec_duration_ms = 0U;
    return_exec_phase_start_ms = 0U;
    return_exec_left_pwm = 0;
    return_exec_right_pwm = 0;

    APP_LOG("[PATH] init max=%u", (unsigned int)PATH_STACK_MAX);
}

void ReturnPath_Reset(void)
{
    ReturnPath_EnsureInitialized();
    return_path_count = 0U;
    APP_LOG("[PATH] reset");
}

void ReturnPath_Record(PathAction_t action, uint32_t duration_ms, int16_t left_pwm, int16_t right_pwm)
{
    ReturnPathEntry_t *entry;

    ReturnPath_EnsureInitialized();

    if ((action == PATH_ACT_NONE) || (duration_ms == 0U))
    {
        return;
    }

    if (return_path_count >= PATH_STACK_MAX)
    {
        APP_LOG("[PATH] full drop action=%s", ReturnPath_ActionName(action));
        return;
    }

    entry = &return_path_stack[return_path_count];
    entry->action = action;
    entry->duration_ms = duration_ms;
    entry->left_pwm = left_pwm;
    entry->right_pwm = right_pwm;
    return_path_count++;

    APP_LOG("[PATH] record action=%s dur=%lu pwmL=%d pwmR=%d count=%u",
            ReturnPath_ActionName(action),
            (unsigned long)duration_ms,
            (int)left_pwm,
            (int)right_pwm,
            (unsigned int)return_path_count);
}

bool ReturnPath_HasActions(void)
{
    ReturnPath_EnsureInitialized();

    return (return_path_count > 0U);
}

bool ReturnPath_PopInverse(PathAction_t *action,
                           uint32_t *duration_ms,
                           int16_t *left_pwm,
                           int16_t *right_pwm)
{
    ReturnPathEntry_t entry;
    PathAction_t inverse_action;

    ReturnPath_EnsureInitialized();

    if ((action == NULL) || (duration_ms == NULL) || (left_pwm == NULL) || (right_pwm == NULL))
    {
        return false;
    }

    if (return_path_count == 0U)
    {
        return false;
    }

    return_path_count--;
    entry = return_path_stack[return_path_count];
    inverse_action = ReturnPath_InverseAction(entry.action);

    *action = inverse_action;
    *duration_ms = entry.duration_ms;
    ReturnPath_GetInversePwm(inverse_action, left_pwm, right_pwm);

    return true;
}

uint16_t ReturnPath_Count(void)
{
    ReturnPath_EnsureInitialized();

    return return_path_count;
}

void ReturnPath_Dump(void)
{
    uint16_t i;

    ReturnPath_EnsureInitialized();

    APP_LOG("[PATH] dump count=%u", (unsigned int)return_path_count);
    for (i = 0U; i < return_path_count; i++)
    {
        const ReturnPathEntry_t *entry = &return_path_stack[i];
        APP_LOG("[PATH] idx=%u action=%s dur=%lu pwmL=%d pwmR=%d",
                (unsigned int)i,
                ReturnPath_ActionName(entry->action),
                (unsigned long)entry->duration_ms,
                (int)entry->left_pwm,
                (int)entry->right_pwm);
    }
}

const char *ReturnPath_ActionName(PathAction_t action)
{
    switch (action)
    {
        case PATH_ACT_FORWARD:
            return "FORWARD";

        case PATH_ACT_BACKUP:
            return "BACKUP";

        case PATH_ACT_TURN_LEFT:
            return "TURN_LEFT";

        case PATH_ACT_TURN_RIGHT:
            return "TURN_RIGHT";

        case PATH_ACT_TURN_AROUND:
            return "TURN_AROUND";

        case PATH_ACT_STOP:
            return "STOP";

        case PATH_ACT_NONE:
        default:
            return "NONE";
    }
}

void ReturnExecutor_Start(void)
{
    ReturnPath_EnsureInitialized();

    Chassis_Stop();
    return_exec_action = PATH_ACT_NONE;
    return_exec_duration_ms = 0U;
    return_exec_left_pwm = 0;
    return_exec_right_pwm = 0;
    return_exec_phase = RETURN_PHASE_IDLE;
    return_exec_phase_start_ms = HAL_GetTick();

    if (return_path_count == 0U)
    {
        return_exec_state = RETURN_EXEC_EMPTY;
        APP_LOG("[RETURN] empty");
        if (AMR_GetState() == AMR_STATE_RETURN)
        {
            (void)AMR_SetState(AMR_STATE_IDLE, "return_empty");
        }
        return;
    }

    return_exec_state = RETURN_EXEC_RUNNING;
    APP_LOG("[RETURN] start count=%u", (unsigned int)return_path_count);
}

void ReturnExecutor_Stop(const char *reason)
{
    uint8_t was_active;

    ReturnPath_EnsureInitialized();

    if (reason == NULL)
    {
        reason = "unspecified";
    }

    was_active = ((return_exec_state == RETURN_EXEC_RUNNING) ||
                  (return_exec_phase != RETURN_PHASE_IDLE) ||
                  (return_exec_action != PATH_ACT_NONE)) ? 1U : 0U;

    return_exec_state = RETURN_EXEC_IDLE;
    return_exec_phase = RETURN_PHASE_IDLE;
    return_exec_action = PATH_ACT_NONE;
    return_exec_duration_ms = 0U;
    return_exec_left_pwm = 0;
    return_exec_right_pwm = 0;
    return_exec_phase_start_ms = 0U;

    Chassis_Stop();

    if (was_active != 0U)
    {
        APP_LOG("[RETURN] stopped reason=%s", reason);
    }
}

void ReturnExecutor_Update(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;

    ReturnPath_EnsureInitialized();

    if (AMR_GetState() != AMR_STATE_RETURN)
    {
        ReturnExecutor_Stop("state_not_return");
        return;
    }

    if (return_exec_state != RETURN_EXEC_RUNNING)
    {
        return;
    }

    now_ms = HAL_GetTick();
    elapsed_ms = ReturnPath_ElapsedMs(now_ms, return_exec_phase_start_ms);

    if (return_exec_phase == RETURN_PHASE_ACTION)
    {
        if (elapsed_ms >= return_exec_duration_ms)
        {
            Chassis_Stop();
            return_exec_phase = RETURN_PHASE_STOP_GAP;
            return_exec_phase_start_ms = now_ms;
            return;
        }

        ReturnExecutor_ApplyCurrentAction();
        return;
    }

    if (return_exec_phase == RETURN_PHASE_STOP_GAP)
    {
        if (elapsed_ms < RETURN_EXEC_STOP_BETWEEN_MS)
        {
            return;
        }

        return_exec_phase = RETURN_PHASE_IDLE;
    }

    ReturnExecutor_StartNextAction(now_ms);
}

ReturnExecState_t ReturnExecutor_GetState(void)
{
    ReturnPath_EnsureInitialized();

    return return_exec_state;
}

const char *ReturnExecutor_StateName(ReturnExecState_t state)
{
    switch (state)
    {
        case RETURN_EXEC_RUNNING:
            return "RUNNING";

        case RETURN_EXEC_DONE:
            return "DONE";

        case RETURN_EXEC_EMPTY:
            return "EMPTY";

        case RETURN_EXEC_IDLE:
        default:
            return "IDLE";
    }
}

static void ReturnPath_EnsureInitialized(void)
{
    if (return_path_initialized == 0U)
    {
        ReturnPath_Init();
    }
}

static uint32_t ReturnPath_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    if (then_ms == 0U)
    {
        return 0U;
    }

    if (now_ms >= then_ms)
    {
        return now_ms - then_ms;
    }

    return 0U;
}

static PathAction_t ReturnPath_InverseAction(PathAction_t action)
{
    switch (action)
    {
        case PATH_ACT_FORWARD:
            return PATH_ACT_BACKUP;

        case PATH_ACT_BACKUP:
            return PATH_ACT_FORWARD;

        case PATH_ACT_TURN_LEFT:
            return PATH_ACT_TURN_RIGHT;

        case PATH_ACT_TURN_RIGHT:
            return PATH_ACT_TURN_LEFT;

        case PATH_ACT_TURN_AROUND:
            return PATH_ACT_TURN_AROUND;

        case PATH_ACT_STOP:
            return PATH_ACT_STOP;

        case PATH_ACT_NONE:
        default:
            return PATH_ACT_NONE;
    }
}

static void ReturnPath_GetInversePwm(PathAction_t action, int16_t *left_pwm, int16_t *right_pwm)
{
    if ((left_pwm == NULL) || (right_pwm == NULL))
    {
        return;
    }

    switch (action)
    {
        case PATH_ACT_FORWARD:
            *left_pwm = RETURN_EXEC_LINEAR_PWM;
            *right_pwm = RETURN_EXEC_LINEAR_PWM;
            break;

        case PATH_ACT_BACKUP:
            *left_pwm = (int16_t)-RETURN_EXEC_LINEAR_PWM;
            *right_pwm = (int16_t)-RETURN_EXEC_LINEAR_PWM;
            break;

        case PATH_ACT_TURN_LEFT:
            *left_pwm = (int16_t)-RETURN_EXEC_TURN_PWM;
            *right_pwm = RETURN_EXEC_TURN_PWM;
            break;

        case PATH_ACT_TURN_RIGHT:
            *left_pwm = RETURN_EXEC_TURN_PWM;
            *right_pwm = (int16_t)-RETURN_EXEC_TURN_PWM;
            break;

        case PATH_ACT_TURN_AROUND:
            *left_pwm = RETURN_EXEC_TURN_PWM;
            *right_pwm = (int16_t)-RETURN_EXEC_TURN_PWM;
            break;

        case PATH_ACT_STOP:
        case PATH_ACT_NONE:
        default:
            *left_pwm = 0;
            *right_pwm = 0;
            break;
    }
}

static void ReturnExecutor_StartNextAction(uint32_t now_ms)
{
    if (AMR_GetState() != AMR_STATE_RETURN)
    {
        ReturnExecutor_Stop("state_not_return");
        return;
    }

    if (ReturnPath_PopInverse(&return_exec_action,
                              &return_exec_duration_ms,
                              &return_exec_left_pwm,
                              &return_exec_right_pwm) == false)
    {
        Chassis_Stop();
        return_exec_state = RETURN_EXEC_DONE;
        return_exec_phase = RETURN_PHASE_IDLE;
        return_exec_action = PATH_ACT_NONE;
        APP_LOG("[RETURN] done actions=0");
        if (AMR_GetState() == AMR_STATE_RETURN)
        {
            (void)AMR_SetState(AMR_STATE_IDLE, "return_done");
        }
        return;
    }

    return_exec_phase = RETURN_PHASE_ACTION;
    return_exec_phase_start_ms = now_ms;
    ReturnExecutor_ApplyCurrentAction();

    if ((return_exec_state != RETURN_EXEC_RUNNING) || (AMR_GetState() != AMR_STATE_RETURN))
    {
        return;
    }

    APP_LOG("[RETURN] exec action=%s dur=%lu pwmL=%d pwmR=%d remaining=%u",
            ReturnPath_ActionName(return_exec_action),
            (unsigned long)return_exec_duration_ms,
            (int)return_exec_left_pwm,
            (int)return_exec_right_pwm,
            (unsigned int)ReturnPath_Count());
}

static void ReturnExecutor_ApplyCurrentAction(void)
{
    if (AMR_GetState() != AMR_STATE_RETURN)
    {
        ReturnExecutor_Stop("state_not_return");
        return;
    }

    if (return_exec_action == PATH_ACT_STOP)
    {
        Chassis_Stop();
    }
    else
    {
        Chassis_SetRaw(return_exec_left_pwm, return_exec_right_pwm);
    }

}
