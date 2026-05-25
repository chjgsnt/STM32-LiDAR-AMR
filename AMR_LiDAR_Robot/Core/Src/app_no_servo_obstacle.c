#include "app_no_servo_obstacle.h"

#include "app_ultrasonic.h"
#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"

#include <stdint.h>

#define NO_SERVO_START_DELAY_MS 3000U
#define NO_SERVO_FORWARD_LEFT_CMD 498
#define NO_SERVO_FORWARD_RIGHT_CMD 500
#define NO_SERVO_STOP_CM 45U
#define NO_SERVO_OBSTACLE_STOP_MS 150U
#define NO_SERVO_BACKUP_MS 450U
#define NO_SERVO_TURN_MS 650U
#define NO_SERVO_RECOVER_MS 200U
#define NO_SERVO_BACKUP_DUTY 320
#define NO_SERVO_TURN_DUTY 360
#define NO_SERVO_FRONT_INVALID_LOG_MS 1000U

typedef enum
{
    NO_SERVO_STATE_START_DELAY = 0,
    NO_SERVO_STATE_DRIVE_FORWARD,
    NO_SERVO_STATE_OBSTACLE_STOP,
    NO_SERVO_STATE_BACKUP,
    NO_SERVO_STATE_TURN,
    NO_SERVO_STATE_RECOVER
} NoServoObstacleState;

typedef enum
{
    NO_SERVO_REASON_INIT = 0,
    NO_SERVO_REASON_START_DELAY,
    NO_SERVO_REASON_RECOVER_FORWARD,
    NO_SERVO_REASON_OBSTACLE_DETECTED,
    NO_SERVO_REASON_STOP_DONE,
    NO_SERVO_REASON_BACKUP_DONE,
    NO_SERVO_REASON_TURN_DONE
} NoServoObstacleReason;

static NoServoObstacleState no_servo_state = NO_SERVO_STATE_START_DELAY;
static uint32_t no_servo_start_ms = 0U;
static uint32_t no_servo_state_enter_ms = 0U;
static uint32_t no_servo_last_front_invalid_log_ms = 0U;

static void App_NoServoObstacle_EnterState(NoServoObstacleState state,
                                           NoServoObstacleReason reason,
                                           uint32_t now_ms);
static void App_NoServoObstacle_ApplyStop(void);
static void App_NoServoObstacle_ApplyForward(void);
static void App_NoServoObstacle_ApplyBackup(void);
static void App_NoServoObstacle_ApplyTurn(void);
static const char *App_NoServoObstacle_StateName(NoServoObstacleState state);
static const char *App_NoServoObstacle_ReasonName(NoServoObstacleReason reason);

void App_NoServoObstacle_Init(void)
{
    no_servo_start_ms = HAL_GetTick();
    no_servo_state_enter_ms = no_servo_start_ms;
    no_servo_last_front_invalid_log_ms = 0U;
    no_servo_state = NO_SERVO_STATE_START_DELAY;

    App_Ultrasonic_Init();
    Chassis_Init();
    App_NoServoObstacle_ApplyStop();

    APP_LOG("APP NO_SERVO: init start_delay_ms=%u stop_cm=%u backup_ms=%u turn_ms=%u recover_ms=%u left_cmd=%d right_cmd=%d backup_duty=%d turn_duty=%d",
            (unsigned int)NO_SERVO_START_DELAY_MS,
            (unsigned int)NO_SERVO_STOP_CM,
            (unsigned int)NO_SERVO_BACKUP_MS,
            (unsigned int)NO_SERVO_TURN_MS,
            (unsigned int)NO_SERVO_RECOVER_MS,
            NO_SERVO_FORWARD_LEFT_CMD,
            NO_SERVO_FORWARD_RIGHT_CMD,
            NO_SERVO_BACKUP_DUTY,
            NO_SERVO_TURN_DUTY);
    APP_LOG("APP NO_SERVO: state=%s reason=%s",
            App_NoServoObstacle_StateName(no_servo_state),
            App_NoServoObstacle_ReasonName(NO_SERVO_REASON_START_DELAY));
}

void App_NoServoObstacle_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t state_elapsed_ms = now_ms - no_servo_state_enter_ms;
    uint16_t front_cm = APP_ULTRASONIC_INVALID_CM;
    uint8_t front_valid = 0U;

    switch (no_servo_state)
    {
        case NO_SERVO_STATE_START_DELAY:
            App_NoServoObstacle_ApplyStop();
            if ((now_ms - no_servo_start_ms) >= NO_SERVO_START_DELAY_MS)
            {
                App_NoServoObstacle_EnterState(NO_SERVO_STATE_DRIVE_FORWARD,
                                               NO_SERVO_REASON_RECOVER_FORWARD,
                                               now_ms);
            }
            break;

        case NO_SERVO_STATE_DRIVE_FORWARD:
            front_valid = App_Ultrasonic_ReadDistanceCm(&front_cm);
            if (front_valid == 0U)
            {
                App_NoServoObstacle_ApplyStop();
                if ((now_ms - no_servo_last_front_invalid_log_ms) >= NO_SERVO_FRONT_INVALID_LOG_MS)
                {
                    no_servo_last_front_invalid_log_ms = now_ms;
                    APP_LOG("APP NO_SERVO: front invalid, stop for safety");
                }
                break;
            }

            if (front_cm <= NO_SERVO_STOP_CM)
            {
                APP_LOG("APP NO_SERVO: obstacle detected front=%ucm",
                        (unsigned int)front_cm);
                App_NoServoObstacle_EnterState(NO_SERVO_STATE_OBSTACLE_STOP,
                                               NO_SERVO_REASON_OBSTACLE_DETECTED,
                                               now_ms);
                break;
            }

            App_NoServoObstacle_ApplyForward();
            break;

        case NO_SERVO_STATE_OBSTACLE_STOP:
            App_NoServoObstacle_ApplyStop();
            if (state_elapsed_ms >= NO_SERVO_OBSTACLE_STOP_MS)
            {
                App_NoServoObstacle_EnterState(NO_SERVO_STATE_BACKUP,
                                               NO_SERVO_REASON_STOP_DONE,
                                               now_ms);
            }
            break;

        case NO_SERVO_STATE_BACKUP:
            App_NoServoObstacle_ApplyBackup();
            if (state_elapsed_ms >= NO_SERVO_BACKUP_MS)
            {
                App_NoServoObstacle_EnterState(NO_SERVO_STATE_TURN,
                                               NO_SERVO_REASON_BACKUP_DONE,
                                               now_ms);
            }
            break;

        case NO_SERVO_STATE_TURN:
            App_NoServoObstacle_ApplyTurn();
            if (state_elapsed_ms >= NO_SERVO_TURN_MS)
            {
                App_NoServoObstacle_EnterState(NO_SERVO_STATE_RECOVER,
                                               NO_SERVO_REASON_TURN_DONE,
                                               now_ms);
            }
            break;

        case NO_SERVO_STATE_RECOVER:
            App_NoServoObstacle_ApplyStop();
            if (state_elapsed_ms >= NO_SERVO_RECOVER_MS)
            {
                APP_LOG("APP NO_SERVO: recover forward");
                App_NoServoObstacle_EnterState(NO_SERVO_STATE_DRIVE_FORWARD,
                                               NO_SERVO_REASON_RECOVER_FORWARD,
                                               now_ms);
            }
            break;

        default:
            App_NoServoObstacle_EnterState(NO_SERVO_STATE_START_DELAY,
                                           NO_SERVO_REASON_INIT,
                                           now_ms);
            break;
    }
}

static void App_NoServoObstacle_EnterState(NoServoObstacleState state,
                                           NoServoObstacleReason reason,
                                           uint32_t now_ms)
{
    no_servo_state = state;
    no_servo_state_enter_ms = now_ms;

    switch (state)
    {
        case NO_SERVO_STATE_DRIVE_FORWARD:
            App_NoServoObstacle_ApplyForward();
            break;

        case NO_SERVO_STATE_BACKUP:
            App_NoServoObstacle_ApplyBackup();
            break;

        case NO_SERVO_STATE_TURN:
            App_NoServoObstacle_ApplyTurn();
            break;

        case NO_SERVO_STATE_START_DELAY:
        case NO_SERVO_STATE_OBSTACLE_STOP:
        case NO_SERVO_STATE_RECOVER:
        default:
            App_NoServoObstacle_ApplyStop();
            break;
    }

    APP_LOG("APP NO_SERVO: state=%s reason=%s",
            App_NoServoObstacle_StateName(state),
            App_NoServoObstacle_ReasonName(reason));
}

static void App_NoServoObstacle_ApplyStop(void)
{
    Chassis_Stop();
    MotorDriver_StopAll();
}

static void App_NoServoObstacle_ApplyForward(void)
{
    Chassis_SetRaw(NO_SERVO_FORWARD_LEFT_CMD, NO_SERVO_FORWARD_RIGHT_CMD);
}

static void App_NoServoObstacle_ApplyBackup(void)
{
    Chassis_SetRaw((int16_t)-NO_SERVO_BACKUP_DUTY, (int16_t)-NO_SERVO_BACKUP_DUTY);
}

static void App_NoServoObstacle_ApplyTurn(void)
{
    Chassis_SetRaw(NO_SERVO_TURN_DUTY, (int16_t)-NO_SERVO_TURN_DUTY);
}

static const char *App_NoServoObstacle_StateName(NoServoObstacleState state)
{
    switch (state)
    {
        case NO_SERVO_STATE_START_DELAY:
            return "START_DELAY";

        case NO_SERVO_STATE_DRIVE_FORWARD:
            return "DRIVE_FORWARD";

        case NO_SERVO_STATE_OBSTACLE_STOP:
            return "OBSTACLE_STOP";

        case NO_SERVO_STATE_BACKUP:
            return "BACKUP";

        case NO_SERVO_STATE_TURN:
            return "TURN";

        case NO_SERVO_STATE_RECOVER:
            return "RECOVER";

        default:
            return "UNKNOWN";
    }
}

static const char *App_NoServoObstacle_ReasonName(NoServoObstacleReason reason)
{
    switch (reason)
    {
        case NO_SERVO_REASON_START_DELAY:
            return "start_delay";

        case NO_SERVO_REASON_RECOVER_FORWARD:
            return "recover_forward";

        case NO_SERVO_REASON_OBSTACLE_DETECTED:
            return "obstacle_detected";

        case NO_SERVO_REASON_STOP_DONE:
            return "stop_done";

        case NO_SERVO_REASON_BACKUP_DONE:
            return "backup_done";

        case NO_SERVO_REASON_TURN_DONE:
            return "turn_done";

        case NO_SERVO_REASON_INIT:
        default:
            return "init";
    }
}
