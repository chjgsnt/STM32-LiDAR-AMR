#include "app_obstacle_motor.h"

#include "app_lidar.h"
#include "app_obstacle.h"
#include "app_test_config.h"
#include "bringup_log.h"

#include <stdint.h>
#include <stdio.h>

#if APP_OBSTACLE_MOTOR_ENABLE && APP_OBSTACLE_GROUND_TEST_ENABLE
#include "chassis.h"
#endif

#define APP_OBS_MOTOR_COMMAND_INTERVAL_MS 250U
#define APP_OBS_MOTOR_LOG_INTERVAL_MS 1000U
#define APP_OBS_MOTOR_GROUND_START_DELAY_MS 3000U
#define APP_OBS_MOTOR_GROUND_MAX_RUN_MS 10000U
#define APP_OBS_MOTOR_INVALID_DISTANCE_MM 0xFFFFU
#define APP_OBS_GROUND_FORWARD_HOLD_MS 300U
#define APP_OBS_GROUND_TURN_HOLD_MS 600U
#define APP_OBS_GROUND_FRONT_CLEAR_MM 500U
#define APP_OBS_GROUND_FRONT_RESUME_MM 450U
#define APP_OBS_GROUND_FRONT_BLOCK_MM 350U
#define APP_OBS_GROUND_FRONT_MID_HOLD_MAX_MS 800U
#define APP_OBS_GROUND_SIDE_CLEAR_MM 300U
#define APP_OBS_GROUND_SIDE_BLOCK_MM 180U
#define APP_OBS_GROUND_HARD_STOP_MM 150U
#define APP_OBS_GROUND_TURN_SWITCH_HYST_MM 150U
#define APP_OBS_GROUND_SIDE_OPEN_SCORE_MM 12000U

typedef enum
{
    APP_OBS_MOTOR_ACTION_STOP = 0,
    APP_OBS_MOTOR_ACTION_FORWARD_SLOW,
    APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW,
    APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW
} AppObstacleMotorAction;

typedef enum
{
    APP_OBS_GROUND_STOP = 0,
    APP_OBS_GROUND_FORWARD,
    APP_OBS_GROUND_TURN_LEFT,
    APP_OBS_GROUND_TURN_RIGHT,
    APP_OBS_GROUND_BLOCKED
} AppObstacleGroundState;

typedef enum
{
    APP_OBS_GROUND_REASON_INIT = 0,
    APP_OBS_GROUND_REASON_START_DELAY,
    APP_OBS_GROUND_REASON_GROUND_TIMEOUT,
    APP_OBS_GROUND_REASON_LIDAR_NOT_READY,
    APP_OBS_GROUND_REASON_INVALID_DECISION,
    APP_OBS_GROUND_REASON_FRONT_CLEAR_FORWARD,
    APP_OBS_GROUND_REASON_TURN_RESUME_FORWARD,
    APP_OBS_GROUND_REASON_FRONT_BLOCK_TURN,
    APP_OBS_GROUND_REASON_FRONT_MID_HOLD,
    APP_OBS_GROUND_REASON_FRONT_MID_HOLD_EXPIRED_TURN,
    APP_OBS_GROUND_REASON_FRONT_MID_TURN,
    APP_OBS_GROUND_REASON_HARD_STOP_TURN,
    APP_OBS_GROUND_REASON_HARD_STOP_ALL_BLOCKED,
    APP_OBS_GROUND_REASON_TURN_HOLD_MIN,
    APP_OBS_GROUND_REASON_MIN_HOLD
} AppObstacleGroundReason;

typedef struct
{
    int16_t left_duty;
    int16_t right_duty;
} AppObstacleMotorCommand;

static AppObstacleGroundState app_ground_state = APP_OBS_GROUND_STOP;
static uint32_t app_ground_state_enter_ms = 0U;

static uint8_t App_ObstacleMotor_OutputEnabled(void);
static int16_t App_ObstacleMotor_ConfiguredSpeed(void);
static uint8_t App_ObstacleMotor_StatusAllowsMotion(const AppLidarStatus *lidar);
static AppObstacleGroundState App_ObstacleMotor_UpdateGroundState(const AppLidarStatus *lidar,
                                                                  AppObstacleDecision decision,
                                                                  uint32_t now_ms,
                                                                  uint32_t *hold_ms,
                                                                  AppObstacleGroundReason *reason);
static AppObstacleGroundState App_ObstacleMotor_EvaluateGroundState(const AppLidarStatus *lidar,
                                                                    AppObstacleDecision decision,
                                                                    uint32_t now_ms,
                                                                    AppObstacleGroundReason *reason);
static AppObstacleGroundState App_ObstacleMotor_SelectTurnState(const AppLidarStatus *lidar,
                                                                AppObstacleDecision decision);
static void App_ObstacleMotor_ForceGroundState(AppObstacleGroundState state,
                                               uint32_t now_ms,
                                               uint32_t *hold_ms);
static AppObstacleMotorAction App_ObstacleMotor_ActionFromGroundState(AppObstacleGroundState state);
static AppObstacleMotorCommand App_ObstacleMotor_CommandFromAction(AppObstacleMotorAction action,
                                                                    int16_t speed);
static const char *App_ObstacleMotor_ActionName(AppObstacleMotorAction action);
static const char *App_ObstacleMotor_GroundStateName(AppObstacleGroundState state);
static const char *App_ObstacleMotor_GroundReasonName(AppObstacleGroundReason reason);
static const char *App_ObstacleMotor_DecisionName(AppObstacleDecision decision);
static const char *App_ObstacleMotor_LogSuffix(void);
static const char *App_ObstacleMotor_FormatDistance(uint8_t valid,
                                                     uint16_t distance_mm,
                                                     char *buffer,
                                                     uint32_t buffer_size);
static uint32_t App_ObstacleMotor_StateHoldMs(AppObstacleGroundState state);
static uint32_t App_ObstacleMotor_SideScore(uint8_t valid, uint16_t distance_mm);
static uint8_t App_ObstacleMotor_SideIsClear(uint8_t valid, uint16_t distance_mm);
static uint8_t App_ObstacleMotor_SideIsBlocked(uint8_t valid, uint16_t distance_mm);
static void App_ObstacleMotor_ApplyAction(AppObstacleMotorAction action,
                                           const AppObstacleMotorCommand *command);

void App_ObstacleMotor_Init(void)
{
    app_ground_state = APP_OBS_GROUND_STOP;
    app_ground_state_enter_ms = HAL_GetTick();

#if APP_OBSTACLE_MOTOR_ENABLE && APP_OBSTACLE_GROUND_TEST_ENABLE
    Chassis_Init();
    Chassis_Stop();

    APP_LOG("APP OBS MOTOR: WARNING motor output ENABLED, lift wheels before test");
#elif APP_OBSTACLE_MOTOR_ENABLE
    APP_LOG("APP OBS MOTOR: motor output disabled, ground-test disabled dry-run only");
#else
    APP_LOG("APP OBS MOTOR: motor output disabled, dry-run only");
#endif
}

void App_ObstacleMotor_Task(void)
{
    static uint32_t last_command_ms = 0U;
    static uint32_t last_log_ms = 0U;
    static uint32_t ground_run_start_ms = 0U;
    static AppObstacleMotorAction last_applied_action = APP_OBS_MOTOR_ACTION_STOP;
    uint32_t now_ms = HAL_GetTick();
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    AppObstacleDecision decision = App_Obstacle_GetDecision();
    uint8_t output_enabled = App_ObstacleMotor_OutputEnabled();
    uint8_t waiting_start_delay = 0U;
    uint8_t ground_timeout = 0U;
    uint8_t command_due = 0U;
    uint32_t ground_hold_ms = 0U;
    AppObstacleGroundReason ground_reason = APP_OBS_GROUND_REASON_INIT;
    int16_t configured_speed = App_ObstacleMotor_ConfiguredSpeed();
    char front_text[16];
    char left_text[16];
    char right_text[16];

    if (App_ObstacleMotor_StatusAllowsMotion(lidar) == 0U)
    {
        decision = APP_OBS_DECISION_NO_LIDAR;
    }

    AppObstacleGroundState ground_state = App_ObstacleMotor_UpdateGroundState(lidar,
                                                                              decision,
                                                                              now_ms,
                                                                              &ground_hold_ms,
                                                                              &ground_reason);

    if (output_enabled != 0U)
    {
        if (now_ms < APP_OBS_MOTOR_GROUND_START_DELAY_MS)
        {
            waiting_start_delay = 1U;
            App_ObstacleMotor_ForceGroundState(APP_OBS_GROUND_STOP, now_ms, &ground_hold_ms);
            ground_state = APP_OBS_GROUND_STOP;
            ground_reason = APP_OBS_GROUND_REASON_START_DELAY;
        }
        else if (decision == APP_OBS_DECISION_NO_LIDAR)
        {
            ground_state = APP_OBS_GROUND_STOP;
            ground_reason = APP_OBS_GROUND_REASON_LIDAR_NOT_READY;
        }
        else
        {
            if (ground_run_start_ms == 0U)
            {
                ground_run_start_ms = now_ms;
            }

            if ((now_ms - ground_run_start_ms) >= APP_OBS_MOTOR_GROUND_MAX_RUN_MS)
            {
                ground_timeout = 1U;
                App_ObstacleMotor_ForceGroundState(APP_OBS_GROUND_STOP, now_ms, &ground_hold_ms);
                ground_state = APP_OBS_GROUND_STOP;
                ground_reason = APP_OBS_GROUND_REASON_GROUND_TIMEOUT;
            }
        }
    }

    AppObstacleMotorAction action = App_ObstacleMotor_ActionFromGroundState(ground_state);
    AppObstacleMotorCommand command = App_ObstacleMotor_CommandFromAction(action, configured_speed);

    if ((now_ms - last_command_ms) >= APP_OBS_MOTOR_COMMAND_INTERVAL_MS)
    {
        command_due = 1U;
    }

    if ((action == APP_OBS_MOTOR_ACTION_STOP) && (last_applied_action != APP_OBS_MOTOR_ACTION_STOP))
    {
        command_due = 1U;
    }

    if (command_due != 0U)
    {
        App_ObstacleMotor_ApplyAction(action, &command);
        last_command_ms = now_ms;
        last_applied_action = action;
    }

    if ((now_ms - last_log_ms) < APP_OBS_MOTOR_LOG_INTERVAL_MS)
    {
        return;
    }

    last_log_ms = now_ms;

    if ((output_enabled != 0U) && (waiting_start_delay != 0U))
    {
        APP_LOG("APP OBS MOTOR: ground test waiting start_delay");
    }

    if ((output_enabled != 0U) && (ground_timeout != 0U))
    {
        APP_LOG("APP OBS MOTOR: ground test timeout, force STOP");
    }

    APP_LOG("APP OBS GROUND: state=%s reason=%s decision=%s front=%s left=%s right=%s hold_ms=%lu",
            App_ObstacleMotor_GroundStateName(ground_state),
            App_ObstacleMotor_GroundReasonName(ground_reason),
            App_ObstacleMotor_DecisionName(decision),
            App_ObstacleMotor_FormatDistance(((lidar != NULL) && (lidar->ready != 0U)) ? lidar->front_valid : 0U,
                                             (lidar != NULL) ? lidar->front_min_mm : 0U,
                                             front_text,
                                             sizeof(front_text)),
            App_ObstacleMotor_FormatDistance(((lidar != NULL) && (lidar->ready != 0U)) ? lidar->left_valid : 0U,
                                             (lidar != NULL) ? lidar->left_min_mm : 0U,
                                             left_text,
                                             sizeof(left_text)),
            App_ObstacleMotor_FormatDistance(((lidar != NULL) && (lidar->ready != 0U)) ? lidar->right_valid : 0U,
                                             (lidar != NULL) ? lidar->right_min_mm : 0U,
                                             right_text,
                                             sizeof(right_text)),
            (unsigned long)ground_hold_ms);

    APP_LOG("APP OBS MOTOR: enabled=%u ground=%u speed=%d left=%d right=%d action=%s%s",
            (unsigned int)APP_OBSTACLE_MOTOR_ENABLE,
            (unsigned int)APP_OBSTACLE_GROUND_TEST_ENABLE,
            (int)configured_speed,
            (int)command.left_duty,
            (int)command.right_duty,
            App_ObstacleMotor_ActionName(action),
            App_ObstacleMotor_LogSuffix());
}

static uint8_t App_ObstacleMotor_OutputEnabled(void)
{
#if APP_OBSTACLE_MOTOR_ENABLE && APP_OBSTACLE_GROUND_TEST_ENABLE
    return 1U;
#else
    return 0U;
#endif
}

static int16_t App_ObstacleMotor_ConfiguredSpeed(void)
{
#if APP_OBSTACLE_GROUND_TEST_ENABLE
    return APP_OBSTACLE_GROUND_TEST_SPEED;
#else
    return APP_OBSTACLE_AIR_TEST_SPEED;
#endif
}

static uint8_t App_ObstacleMotor_StatusAllowsMotion(const AppLidarStatus *lidar)
{
    if ((lidar == NULL) || (lidar->ready == 0U))
    {
        return 0U;
    }

    if (((lidar->front_valid != 0U) &&
         ((lidar->front_min_mm == 0U) || (lidar->front_min_mm == APP_OBS_MOTOR_INVALID_DISTANCE_MM))) ||
        ((lidar->left_valid != 0U) &&
         ((lidar->left_min_mm == 0U) || (lidar->left_min_mm == APP_OBS_MOTOR_INVALID_DISTANCE_MM))) ||
        ((lidar->right_valid != 0U) &&
         ((lidar->right_min_mm == 0U) || (lidar->right_min_mm == APP_OBS_MOTOR_INVALID_DISTANCE_MM))))
    {
        return 0U;
    }

    return 1U;
}

static AppObstacleGroundState App_ObstacleMotor_UpdateGroundState(const AppLidarStatus *lidar,
                                                                  AppObstacleDecision decision,
                                                                  uint32_t now_ms,
                                                                  uint32_t *hold_ms,
                                                                  AppObstacleGroundReason *reason)
{
    AppObstacleGroundReason desired_reason = APP_OBS_GROUND_REASON_INIT;
    AppObstacleGroundState desired_state = App_ObstacleMotor_EvaluateGroundState(lidar,
                                                                                 decision,
                                                                                 now_ms,
                                                                                 &desired_reason);
    uint32_t elapsed_ms = now_ms - app_ground_state_enter_ms;
    uint32_t required_hold_ms = App_ObstacleMotor_StateHoldMs(app_ground_state);
    uint8_t bypass_hold = ((desired_state == APP_OBS_GROUND_STOP) ||
                           (desired_state == APP_OBS_GROUND_BLOCKED) ||
                           (desired_reason == APP_OBS_GROUND_REASON_HARD_STOP_TURN)) ? 1U : 0U;

    if ((bypass_hold == 0U) &&
        (desired_state != app_ground_state) &&
        (required_hold_ms > 0U) &&
        (elapsed_ms < required_hold_ms))
    {
        desired_state = app_ground_state;
        if ((app_ground_state == APP_OBS_GROUND_TURN_LEFT) ||
            (app_ground_state == APP_OBS_GROUND_TURN_RIGHT))
        {
            desired_reason = APP_OBS_GROUND_REASON_TURN_HOLD_MIN;
        }
        else
        {
            desired_reason = APP_OBS_GROUND_REASON_MIN_HOLD;
        }
    }

    if (desired_state != app_ground_state)
    {
        app_ground_state = desired_state;
        app_ground_state_enter_ms = now_ms;
        elapsed_ms = 0U;
    }

    if (hold_ms != NULL)
    {
        *hold_ms = elapsed_ms;
    }

    if (reason != NULL)
    {
        *reason = desired_reason;
    }

    return app_ground_state;
}

static AppObstacleGroundState App_ObstacleMotor_EvaluateGroundState(const AppLidarStatus *lidar,
                                                                    AppObstacleDecision decision,
                                                                    uint32_t now_ms,
                                                                    AppObstacleGroundReason *reason)
{
    AppObstacleGroundReason local_reason = APP_OBS_GROUND_REASON_INIT;
    AppObstacleGroundState state = APP_OBS_GROUND_STOP;
    uint32_t current_hold_ms = now_ms - app_ground_state_enter_ms;

    if (decision > APP_OBS_DECISION_STOP_BLOCKED)
    {
        local_reason = APP_OBS_GROUND_REASON_INVALID_DECISION;
        state = APP_OBS_GROUND_BLOCKED;
        goto done;
    }

    if (decision == APP_OBS_DECISION_NO_LIDAR)
    {
        local_reason = APP_OBS_GROUND_REASON_LIDAR_NOT_READY;
        state = APP_OBS_GROUND_STOP;
        goto done;
    }

    if ((lidar == NULL) || (lidar->ready == 0U))
    {
        local_reason = APP_OBS_GROUND_REASON_LIDAR_NOT_READY;
        state = APP_OBS_GROUND_STOP;
        goto done;
    }

    if ((lidar->front_valid != 0U) && (lidar->front_min_mm <= APP_OBS_GROUND_HARD_STOP_MM))
    {
        if ((App_ObstacleMotor_SideIsBlocked(lidar->left_valid, lidar->left_min_mm) != 0U) &&
            (App_ObstacleMotor_SideIsBlocked(lidar->right_valid, lidar->right_min_mm) != 0U))
        {
            local_reason = APP_OBS_GROUND_REASON_HARD_STOP_ALL_BLOCKED;
            state = APP_OBS_GROUND_BLOCKED;
            goto done;
        }

        local_reason = APP_OBS_GROUND_REASON_HARD_STOP_TURN;
        state = App_ObstacleMotor_SelectTurnState(lidar, decision);
        goto done;
    }

    if ((decision == APP_OBS_DECISION_CLEAR_FORWARD) &&
        ((lidar->front_valid == 0U) || (lidar->front_min_mm >= APP_OBS_GROUND_FRONT_RESUME_MM)))
    {
        if ((app_ground_state == APP_OBS_GROUND_TURN_LEFT) ||
            (app_ground_state == APP_OBS_GROUND_TURN_RIGHT))
        {
            local_reason = APP_OBS_GROUND_REASON_TURN_RESUME_FORWARD;
        }
        else
        {
            local_reason = APP_OBS_GROUND_REASON_FRONT_CLEAR_FORWARD;
        }

        state = APP_OBS_GROUND_FORWARD;
        goto done;
    }

    if ((lidar->front_valid == 0U) || (lidar->front_min_mm >= APP_OBS_GROUND_FRONT_CLEAR_MM))
    {
        local_reason = APP_OBS_GROUND_REASON_FRONT_CLEAR_FORWARD;
        state = APP_OBS_GROUND_FORWARD;
        goto done;
    }

    if (lidar->front_min_mm <= APP_OBS_GROUND_FRONT_BLOCK_MM)
    {
        local_reason = APP_OBS_GROUND_REASON_FRONT_BLOCK_TURN;
        state = App_ObstacleMotor_SelectTurnState(lidar, decision);
        goto done;
    }

    if (app_ground_state == APP_OBS_GROUND_FORWARD)
    {
        if (((decision == APP_OBS_DECISION_TURN_LEFT) ||
             (decision == APP_OBS_DECISION_TURN_RIGHT)) &&
            (current_hold_ms >= APP_OBS_GROUND_FRONT_MID_HOLD_MAX_MS))
        {
            local_reason = APP_OBS_GROUND_REASON_FRONT_MID_HOLD_EXPIRED_TURN;
            state = App_ObstacleMotor_SelectTurnState(lidar, decision);
            goto done;
        }

        local_reason = APP_OBS_GROUND_REASON_FRONT_MID_HOLD;
        state = APP_OBS_GROUND_FORWARD;
        goto done;
    }

    if ((app_ground_state == APP_OBS_GROUND_TURN_LEFT) ||
        (app_ground_state == APP_OBS_GROUND_TURN_RIGHT))
    {
        local_reason = APP_OBS_GROUND_REASON_FRONT_MID_HOLD;
        state = app_ground_state;
        goto done;
    }

    local_reason = APP_OBS_GROUND_REASON_FRONT_MID_TURN;
    state = App_ObstacleMotor_SelectTurnState(lidar, decision);

done:
    if (reason != NULL)
    {
        *reason = local_reason;
    }

    return state;
}

static AppObstacleGroundState App_ObstacleMotor_SelectTurnState(const AppLidarStatus *lidar,
                                                                AppObstacleDecision decision)
{
    uint8_t left_clear = App_ObstacleMotor_SideIsClear(lidar->left_valid, lidar->left_min_mm);
    uint8_t right_clear = App_ObstacleMotor_SideIsClear(lidar->right_valid, lidar->right_min_mm);
    uint32_t left_score = App_ObstacleMotor_SideScore(lidar->left_valid, lidar->left_min_mm);
    uint32_t right_score = App_ObstacleMotor_SideScore(lidar->right_valid, lidar->right_min_mm);

    if (app_ground_state == APP_OBS_GROUND_TURN_LEFT)
    {
        if ((right_clear != 0U) && ((right_score > left_score) &&
            ((right_score - left_score) >= APP_OBS_GROUND_TURN_SWITCH_HYST_MM)))
        {
            return APP_OBS_GROUND_TURN_RIGHT;
        }

        if (left_clear != 0U)
        {
            return APP_OBS_GROUND_TURN_LEFT;
        }

        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if (app_ground_state == APP_OBS_GROUND_TURN_RIGHT)
    {
        if ((left_clear != 0U) && ((left_score > right_score) &&
            ((left_score - right_score) >= APP_OBS_GROUND_TURN_SWITCH_HYST_MM)))
        {
            return APP_OBS_GROUND_TURN_LEFT;
        }

        if (right_clear != 0U)
        {
            return APP_OBS_GROUND_TURN_RIGHT;
        }

        return APP_OBS_GROUND_TURN_LEFT;
    }

    if ((left_clear != 0U) && (right_clear != 0U))
    {
        if (left_score > right_score)
        {
            return APP_OBS_GROUND_TURN_LEFT;
        }

        if (right_score > left_score)
        {
            return APP_OBS_GROUND_TURN_RIGHT;
        }

        return (decision == APP_OBS_DECISION_TURN_LEFT) ? APP_OBS_GROUND_TURN_LEFT : APP_OBS_GROUND_TURN_RIGHT;
    }

    if (left_clear != 0U)
    {
        return APP_OBS_GROUND_TURN_LEFT;
    }

    if (right_clear != 0U)
    {
        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if (left_score > right_score)
    {
        return APP_OBS_GROUND_TURN_LEFT;
    }

    if (right_score > left_score)
    {
        return APP_OBS_GROUND_TURN_RIGHT;
    }

    return (decision == APP_OBS_DECISION_TURN_LEFT) ? APP_OBS_GROUND_TURN_LEFT : APP_OBS_GROUND_TURN_RIGHT;
}

static void App_ObstacleMotor_ForceGroundState(AppObstacleGroundState state,
                                               uint32_t now_ms,
                                               uint32_t *hold_ms)
{
    if (state != app_ground_state)
    {
        app_ground_state = state;
        app_ground_state_enter_ms = now_ms;
    }

    if (hold_ms != NULL)
    {
        *hold_ms = now_ms - app_ground_state_enter_ms;
    }
}

static AppObstacleMotorAction App_ObstacleMotor_ActionFromGroundState(AppObstacleGroundState state)
{
    switch (state)
    {
        case APP_OBS_GROUND_FORWARD:
            return APP_OBS_MOTOR_ACTION_FORWARD_SLOW;

        case APP_OBS_GROUND_TURN_LEFT:
            return APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW;

        case APP_OBS_GROUND_TURN_RIGHT:
            return APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW;

        case APP_OBS_GROUND_STOP:
        case APP_OBS_GROUND_BLOCKED:
        default:
            return APP_OBS_MOTOR_ACTION_STOP;
    }
}

static AppObstacleMotorCommand App_ObstacleMotor_CommandFromAction(AppObstacleMotorAction action,
                                                                    int16_t speed)
{
    AppObstacleMotorCommand command = {0, 0};

    switch (action)
    {
        case APP_OBS_MOTOR_ACTION_FORWARD_SLOW:
            command.left_duty = speed;
            command.right_duty = speed;
            break;

        case APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW:
            command.left_duty = (int16_t)-speed;
            command.right_duty = speed;
            break;

        case APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW:
            command.left_duty = speed;
            command.right_duty = (int16_t)-speed;
            break;

        case APP_OBS_MOTOR_ACTION_STOP:
        default:
            command.left_duty = 0;
            command.right_duty = 0;
            break;
    }

    return command;
}

static const char *App_ObstacleMotor_ActionName(AppObstacleMotorAction action)
{
    switch (action)
    {
        case APP_OBS_MOTOR_ACTION_FORWARD_SLOW:
            return "FORWARD_SLOW";

        case APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW:
            return "TURN_LEFT_SLOW";

        case APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW:
            return "TURN_RIGHT_SLOW";

        case APP_OBS_MOTOR_ACTION_STOP:
        default:
            return "STOP";
    }
}

static const char *App_ObstacleMotor_GroundStateName(AppObstacleGroundState state)
{
    switch (state)
    {
        case APP_OBS_GROUND_FORWARD:
            return "FORWARD";

        case APP_OBS_GROUND_TURN_LEFT:
            return "TURN_LEFT";

        case APP_OBS_GROUND_TURN_RIGHT:
            return "TURN_RIGHT";

        case APP_OBS_GROUND_BLOCKED:
            return "BLOCKED";

        case APP_OBS_GROUND_STOP:
        default:
            return "STOP";
    }
}

static const char *App_ObstacleMotor_GroundReasonName(AppObstacleGroundReason reason)
{
    switch (reason)
    {
        case APP_OBS_GROUND_REASON_START_DELAY:
            return "start_delay";

        case APP_OBS_GROUND_REASON_GROUND_TIMEOUT:
            return "ground_timeout";

        case APP_OBS_GROUND_REASON_LIDAR_NOT_READY:
            return "lidar_not_ready";

        case APP_OBS_GROUND_REASON_INVALID_DECISION:
            return "invalid_decision";

        case APP_OBS_GROUND_REASON_FRONT_CLEAR_FORWARD:
            return "front_clear_forward";

        case APP_OBS_GROUND_REASON_TURN_RESUME_FORWARD:
            return "turn_resume_forward";

        case APP_OBS_GROUND_REASON_FRONT_BLOCK_TURN:
            return "front_block_turn";

        case APP_OBS_GROUND_REASON_FRONT_MID_HOLD:
            return "front_mid_hold";

        case APP_OBS_GROUND_REASON_FRONT_MID_HOLD_EXPIRED_TURN:
            return "front_mid_hold_expired_turn";

        case APP_OBS_GROUND_REASON_FRONT_MID_TURN:
            return "front_mid_turn";

        case APP_OBS_GROUND_REASON_HARD_STOP_TURN:
            return "hard_stop_turn";

        case APP_OBS_GROUND_REASON_HARD_STOP_ALL_BLOCKED:
            return "hard_stop_all_blocked";

        case APP_OBS_GROUND_REASON_TURN_HOLD_MIN:
            return "turn_hold_min";

        case APP_OBS_GROUND_REASON_MIN_HOLD:
            return "min_hold";

        case APP_OBS_GROUND_REASON_INIT:
        default:
            return "init";
    }
}

static const char *App_ObstacleMotor_DecisionName(AppObstacleDecision decision)
{
    switch (decision)
    {
        case APP_OBS_DECISION_CLEAR_FORWARD:
            return "CLEAR_FORWARD";

        case APP_OBS_DECISION_TURN_LEFT:
            return "TURN_LEFT";

        case APP_OBS_DECISION_TURN_RIGHT:
            return "TURN_RIGHT";

        case APP_OBS_DECISION_STOP_BLOCKED:
            return "STOP_BLOCKED";

        case APP_OBS_DECISION_NO_LIDAR:
            return "NO_LIDAR";

        default:
            return "UNKNOWN";
    }
}

static const char *App_ObstacleMotor_LogSuffix(void)
{
#if APP_OBSTACLE_MOTOR_ENABLE && APP_OBSTACLE_GROUND_TEST_ENABLE
    return "";
#elif APP_OBSTACLE_MOTOR_ENABLE
    return " dry-run ground-test disabled";
#else
    return " dry-run";
#endif
}

static const char *App_ObstacleMotor_FormatDistance(uint8_t valid,
                                                     uint16_t distance_mm,
                                                     char *buffer,
                                                     uint32_t buffer_size)
{
    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return "";
    }

    if (valid == 0U)
    {
        (void)snprintf(buffer, buffer_size, "--");
        return buffer;
    }

    (void)snprintf(buffer, buffer_size, "%umm", (unsigned int)distance_mm);
    return buffer;
}

static uint32_t App_ObstacleMotor_StateHoldMs(AppObstacleGroundState state)
{
    switch (state)
    {
        case APP_OBS_GROUND_FORWARD:
            return APP_OBS_GROUND_FORWARD_HOLD_MS;

        case APP_OBS_GROUND_TURN_LEFT:
        case APP_OBS_GROUND_TURN_RIGHT:
            return APP_OBS_GROUND_TURN_HOLD_MS;

        case APP_OBS_GROUND_STOP:
        case APP_OBS_GROUND_BLOCKED:
        default:
            return 0U;
    }
}

static uint32_t App_ObstacleMotor_SideScore(uint8_t valid, uint16_t distance_mm)
{
    if (valid == 0U)
    {
        return APP_OBS_GROUND_SIDE_OPEN_SCORE_MM;
    }

    return (uint32_t)distance_mm;
}

static uint8_t App_ObstacleMotor_SideIsClear(uint8_t valid, uint16_t distance_mm)
{
    if (valid == 0U)
    {
        return 1U;
    }

    return (distance_mm >= APP_OBS_GROUND_SIDE_CLEAR_MM) ? 1U : 0U;
}

static uint8_t App_ObstacleMotor_SideIsBlocked(uint8_t valid, uint16_t distance_mm)
{
    if (valid == 0U)
    {
        return 0U;
    }

    return (distance_mm <= APP_OBS_GROUND_SIDE_BLOCK_MM) ? 1U : 0U;
}

static void App_ObstacleMotor_ApplyAction(AppObstacleMotorAction action,
                                           const AppObstacleMotorCommand *command)
{
#if APP_OBSTACLE_MOTOR_ENABLE && APP_OBSTACLE_GROUND_TEST_ENABLE
    switch (action)
    {
        case APP_OBS_MOTOR_ACTION_FORWARD_SLOW:
        case APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW:
        case APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW:
            Chassis_SetRaw(command->left_duty, command->right_duty);
            break;

        case APP_OBS_MOTOR_ACTION_STOP:
        default:
            Chassis_Stop();
            break;
    }
#else
    (void)action;
    (void)command;
#endif
}
