#include "app_obstacle_motor.h"

#include "app_lidar.h"
#include "app_obstacle.h"
#include "app_test_config.h"
#include "bringup_log.h"

#include <stdint.h>

#if APP_OBSTACLE_MOTOR_ENABLE
#include "chassis.h"
#endif

#define APP_OBS_MOTOR_COMMAND_INTERVAL_MS 250U
#define APP_OBS_MOTOR_LOG_INTERVAL_MS 1000U
#define APP_OBS_MOTOR_FORWARD_DUTY 300
#define APP_OBS_MOTOR_TURN_DUTY 300
#define APP_OBS_MOTOR_INVALID_DISTANCE_MM 0xFFFFU

typedef enum
{
    APP_OBS_MOTOR_ACTION_STOP = 0,
    APP_OBS_MOTOR_ACTION_FORWARD_SLOW,
    APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW,
    APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW
} AppObstacleMotorAction;

typedef struct
{
    int16_t left_duty;
    int16_t right_duty;
} AppObstacleMotorCommand;

static uint8_t App_ObstacleMotor_StatusAllowsMotion(const AppLidarStatus *lidar);
static AppObstacleMotorAction App_ObstacleMotor_ActionFromDecision(AppObstacleDecision decision);
static AppObstacleMotorCommand App_ObstacleMotor_CommandFromAction(AppObstacleMotorAction action);
static const char *App_ObstacleMotor_ActionName(AppObstacleMotorAction action);
static const char *App_ObstacleMotor_DecisionName(AppObstacleDecision decision);
static void App_ObstacleMotor_ApplyAction(AppObstacleMotorAction action,
                                           const AppObstacleMotorCommand *command);

void App_ObstacleMotor_Init(void)
{
#if APP_OBSTACLE_MOTOR_ENABLE
    Chassis_Init();
    Chassis_Stop();

    APP_LOG("APP OBS MOTOR: WARNING motor output ENABLED, lift wheels before test");
#else
    APP_LOG("APP OBS MOTOR: motor output disabled, dry-run only");
#endif
}

void App_ObstacleMotor_Task(void)
{
    static uint32_t last_command_ms = 0U;
    static uint32_t last_log_ms = 0U;
    static AppObstacleMotorAction last_applied_action = APP_OBS_MOTOR_ACTION_STOP;
    uint32_t now_ms = HAL_GetTick();
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    AppObstacleDecision decision = App_Obstacle_GetDecision();
    uint8_t command_due = 0U;

    if (App_ObstacleMotor_StatusAllowsMotion(lidar) == 0U)
    {
        decision = APP_OBS_DECISION_NO_LIDAR;
    }

    AppObstacleMotorAction action = App_ObstacleMotor_ActionFromDecision(decision);
    AppObstacleMotorCommand command = App_ObstacleMotor_CommandFromAction(action);

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

    APP_LOG("APP OBS MOTOR: enabled=%u decision=%s action=%s left=%d right=%d%s",
            (unsigned int)APP_OBSTACLE_MOTOR_ENABLE,
            App_ObstacleMotor_DecisionName(decision),
            App_ObstacleMotor_ActionName(action),
            (int)command.left_duty,
            (int)command.right_duty,
            (APP_OBSTACLE_MOTOR_ENABLE == 0) ? " dry-run" : "");
}

static uint8_t App_ObstacleMotor_StatusAllowsMotion(const AppLidarStatus *lidar)
{
    if ((lidar == NULL) || (lidar->ready == 0U))
    {
        return 0U;
    }

    if ((lidar->front_valid == 0U) && (lidar->left_valid == 0U) && (lidar->right_valid == 0U))
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

static AppObstacleMotorAction App_ObstacleMotor_ActionFromDecision(AppObstacleDecision decision)
{
    switch (decision)
    {
        case APP_OBS_DECISION_CLEAR_FORWARD:
            return APP_OBS_MOTOR_ACTION_FORWARD_SLOW;

        case APP_OBS_DECISION_TURN_LEFT:
            return APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW;

        case APP_OBS_DECISION_TURN_RIGHT:
            return APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW;

        case APP_OBS_DECISION_STOP_BLOCKED:
        case APP_OBS_DECISION_NO_LIDAR:
        default:
            return APP_OBS_MOTOR_ACTION_STOP;
    }
}

static AppObstacleMotorCommand App_ObstacleMotor_CommandFromAction(AppObstacleMotorAction action)
{
    AppObstacleMotorCommand command = {0, 0};

    switch (action)
    {
        case APP_OBS_MOTOR_ACTION_FORWARD_SLOW:
            command.left_duty = APP_OBS_MOTOR_FORWARD_DUTY;
            command.right_duty = APP_OBS_MOTOR_FORWARD_DUTY;
            break;

        case APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW:
            command.left_duty = (int16_t)-APP_OBS_MOTOR_TURN_DUTY;
            command.right_duty = APP_OBS_MOTOR_TURN_DUTY;
            break;

        case APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW:
            command.left_duty = APP_OBS_MOTOR_TURN_DUTY;
            command.right_duty = (int16_t)-APP_OBS_MOTOR_TURN_DUTY;
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

static void App_ObstacleMotor_ApplyAction(AppObstacleMotorAction action,
                                           const AppObstacleMotorCommand *command)
{
#if APP_OBSTACLE_MOTOR_ENABLE
    (void)command;

    switch (action)
    {
        case APP_OBS_MOTOR_ACTION_FORWARD_SLOW:
            Chassis_Forward(APP_OBS_MOTOR_FORWARD_DUTY);
            break;

        case APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW:
            Chassis_TurnLeft(APP_OBS_MOTOR_TURN_DUTY);
            break;

        case APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW:
            Chassis_TurnRight(APP_OBS_MOTOR_TURN_DUTY);
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
