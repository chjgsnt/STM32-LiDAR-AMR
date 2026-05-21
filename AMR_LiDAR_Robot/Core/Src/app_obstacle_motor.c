#include "app_obstacle_motor.h"

#include "app_config.h"
#include "app_obstacle.h"
#include "bringup_log.h"

#include <stdint.h>

#if APP_OBSTACLE_MOTOR_ENABLE
#include "chassis.h"
#endif

#define APP_OBS_MOTOR_LOG_INTERVAL_MS 1000U
#define APP_OBS_MOTOR_FORWARD_DUTY 180
#define APP_OBS_MOTOR_TURN_DUTY 160

typedef enum
{
    APP_OBS_MOTOR_ACTION_STOP = 0,
    APP_OBS_MOTOR_ACTION_FORWARD_SLOW,
    APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW,
    APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW
} AppObstacleMotorAction;

static AppObstacleMotorAction App_ObstacleMotor_ActionFromDecision(AppObstacleDecision decision);
static const char *App_ObstacleMotor_ActionName(AppObstacleMotorAction action);
static const char *App_ObstacleMotor_DecisionName(AppObstacleDecision decision);
static void App_ObstacleMotor_ApplyAction(AppObstacleMotorAction action);

void App_ObstacleMotor_Init(void)
{
#if APP_OBSTACLE_MOTOR_ENABLE
    Chassis_Init();
    Chassis_Stop();
#endif

    APP_LOG("APP OBS MOTOR: enabled=%u init safe low-speed mode",
            (unsigned int)APP_OBSTACLE_MOTOR_ENABLE);
}

void App_ObstacleMotor_Task(void)
{
    static uint32_t last_log_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    AppObstacleDecision decision = App_Obstacle_GetDecision();
    AppObstacleMotorAction action = App_ObstacleMotor_ActionFromDecision(decision);

    App_ObstacleMotor_ApplyAction(action);

    if ((now_ms - last_log_ms) < APP_OBS_MOTOR_LOG_INTERVAL_MS)
    {
        return;
    }

    last_log_ms = now_ms;

    APP_LOG("APP OBS MOTOR: enabled=%u decision=%s action=%s",
            (unsigned int)APP_OBSTACLE_MOTOR_ENABLE,
            App_ObstacleMotor_DecisionName(decision),
            App_ObstacleMotor_ActionName(action));
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
        default:
            return "NO_LIDAR";
    }
}

static void App_ObstacleMotor_ApplyAction(AppObstacleMotorAction action)
{
#if APP_OBSTACLE_MOTOR_ENABLE
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
#endif
}
