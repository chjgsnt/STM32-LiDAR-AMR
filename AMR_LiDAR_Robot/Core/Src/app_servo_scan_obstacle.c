#include "app_servo_scan_obstacle.h"

#include "app_servo.h"
#include "app_test_config.h"
#include "app_ultrasonic.h"
#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"

#include <stdint.h>
#include <stdio.h>

#define APP_SERVO_SCAN_START_DELAY_MS 3000U
#define APP_SERVO_SCAN_FORWARD_LEFT_CMD 498
#define APP_SERVO_SCAN_FORWARD_RIGHT_CMD 500
#define APP_SERVO_SCAN_FRONT_STOP_CM 45U
#define APP_SERVO_SCAN_SIDE_BLOCKED_CM 25U
#define APP_SERVO_SCAN_SAMPLE_COUNT 3U
#define APP_SERVO_SCAN_SERVO_SETTLE_MS 450U
#define APP_SERVO_SCAN_STOP_SETTLE_MS 150U
#define APP_SERVO_SCAN_SAMPLE_INTERVAL_MS 80U
#define APP_SERVO_SCAN_TURN_MS 650U
#define APP_SERVO_SCAN_BACKUP_MS 450U
#define APP_SERVO_SCAN_RECOVER_MS 200U
#define APP_SERVO_SCAN_TURN_DUTY 360
#define APP_SERVO_SCAN_BACKUP_DUTY 320
#define APP_SERVO_SCAN_FRONT_INVALID_LOG_MS 1000U

typedef enum
{
    APP_SERVO_SCAN_STATE_START_DELAY = 0,
    APP_SERVO_SCAN_STATE_DRIVE_FORWARD,
    APP_SERVO_SCAN_STATE_OBSTACLE_STOP,
    APP_SERVO_SCAN_STATE_SCAN_LEFT,
    APP_SERVO_SCAN_STATE_SCAN_RIGHT,
    APP_SERVO_SCAN_STATE_DECIDE_TURN,
    APP_SERVO_SCAN_STATE_TURN_LEFT,
    APP_SERVO_SCAN_STATE_TURN_RIGHT,
    APP_SERVO_SCAN_STATE_BACKUP,
    APP_SERVO_SCAN_STATE_RECOVER
} AppServoScanState;

typedef enum
{
    APP_SERVO_SCAN_REASON_INIT = 0,
    APP_SERVO_SCAN_REASON_START_DELAY,
    APP_SERVO_SCAN_REASON_RECOVER_FORWARD,
    APP_SERVO_SCAN_REASON_OBSTACLE_DETECTED,
    APP_SERVO_SCAN_REASON_SCAN_LEFT,
    APP_SERVO_SCAN_REASON_SCAN_RIGHT,
    APP_SERVO_SCAN_REASON_DECIDE,
    APP_SERVO_SCAN_REASON_CHOOSE_LEFT,
    APP_SERVO_SCAN_REASON_CHOOSE_RIGHT,
    APP_SERVO_SCAN_REASON_BOTH_BLOCKED_BACKUP,
    APP_SERVO_SCAN_REASON_BACKUP_DONE,
    APP_SERVO_SCAN_REASON_TURN_DONE
} AppServoScanReason;

static AppServoScanState app_scan_state = APP_SERVO_SCAN_STATE_START_DELAY;
static AppServoScanState app_scan_after_backup_turn = APP_SERVO_SCAN_STATE_TURN_RIGHT;
static uint32_t app_scan_start_ms = 0U;
static uint32_t app_scan_state_enter_ms = 0U;
static uint32_t app_scan_next_sample_ms = 0U;
static uint32_t app_scan_last_front_invalid_log_ms = 0U;
static uint16_t app_scan_left_cm = APP_ULTRASONIC_INVALID_CM;
static uint16_t app_scan_right_cm = APP_ULTRASONIC_INVALID_CM;
static uint16_t app_scan_samples[APP_SERVO_SCAN_SAMPLE_COUNT];
static uint8_t app_scan_sample_count = 0U;

static void App_ServoScanObstacle_EnterState(AppServoScanState state,
                                             AppServoScanReason reason,
                                             uint32_t now_ms);
static void App_ServoScanObstacle_ResetSamples(void);
static uint8_t App_ServoScanObstacle_TakeSample(void);
static uint8_t App_ServoScanObstacle_FinishSamples(uint16_t *distance_cm);
static uint16_t App_ServoScanObstacle_SideScore(uint16_t distance_cm);
static uint8_t App_ServoScanObstacle_SideBlocked(uint16_t distance_cm);
static void App_ServoScanObstacle_ApplyStop(void);
static void App_ServoScanObstacle_ApplyForward(void);
static void App_ServoScanObstacle_ApplyTurnLeft(void);
static void App_ServoScanObstacle_ApplyTurnRight(void);
static void App_ServoScanObstacle_ApplyBackup(void);
static const char *App_ServoScanObstacle_StateName(AppServoScanState state);
static const char *App_ServoScanObstacle_ReasonName(AppServoScanReason reason);
static const char *App_ServoScanObstacle_FormatDistance(uint16_t distance_cm,
                                                        char *buffer,
                                                        uint32_t buffer_size);

void App_ServoScanObstacle_Init(void)
{
    app_scan_start_ms = HAL_GetTick();
    app_scan_state = APP_SERVO_SCAN_STATE_START_DELAY;
    app_scan_after_backup_turn = APP_SERVO_SCAN_STATE_TURN_RIGHT;
    app_scan_state_enter_ms = app_scan_start_ms;
    app_scan_next_sample_ms = app_scan_start_ms;
    app_scan_last_front_invalid_log_ms = 0U;
    app_scan_left_cm = APP_ULTRASONIC_INVALID_CM;
    app_scan_right_cm = APP_ULTRASONIC_INVALID_CM;
    App_ServoScanObstacle_ResetSamples();

    App_Ultrasonic_Init();
    App_Servo_Init();
    App_Servo_Center();
    Chassis_Init();
    App_ServoScanObstacle_ApplyStop();

    APP_LOG("APP SERVO_SCAN: init start_delay_ms=%u front_stop_cm=%u side_blocked_cm=%u sample_count=%u servo_settle_ms=%u left_cmd=%d right_cmd=%d turn_ms=%u backup_ms=%u",
            (unsigned int)APP_SERVO_SCAN_START_DELAY_MS,
            (unsigned int)APP_SERVO_SCAN_FRONT_STOP_CM,
            (unsigned int)APP_SERVO_SCAN_SIDE_BLOCKED_CM,
            (unsigned int)APP_SERVO_SCAN_SAMPLE_COUNT,
            (unsigned int)APP_SERVO_SCAN_SERVO_SETTLE_MS,
            APP_SERVO_SCAN_FORWARD_LEFT_CMD,
            APP_SERVO_SCAN_FORWARD_RIGHT_CMD,
            (unsigned int)APP_SERVO_SCAN_TURN_MS,
            (unsigned int)APP_SERVO_SCAN_BACKUP_MS);
    APP_LOG("APP SERVO_SCAN: state=%s reason=%s",
            App_ServoScanObstacle_StateName(app_scan_state),
            App_ServoScanObstacle_ReasonName(APP_SERVO_SCAN_REASON_START_DELAY));
}

void App_ServoScanObstacle_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t state_elapsed_ms = now_ms - app_scan_state_enter_ms;
    uint16_t front_cm = APP_ULTRASONIC_INVALID_CM;
    uint8_t front_valid = 0U;
    char left_text[16];
    char right_text[16];

    switch (app_scan_state)
    {
        case APP_SERVO_SCAN_STATE_START_DELAY:
            App_ServoScanObstacle_ApplyStop();
            if ((now_ms - app_scan_start_ms) >= APP_SERVO_SCAN_START_DELAY_MS)
            {
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_DRIVE_FORWARD,
                                                 APP_SERVO_SCAN_REASON_RECOVER_FORWARD,
                                                 now_ms);
            }
            break;

        case APP_SERVO_SCAN_STATE_DRIVE_FORWARD:
            front_valid = App_Ultrasonic_ReadDistanceCm(&front_cm);
            if (front_valid == 0U)
            {
                App_ServoScanObstacle_ApplyStop();
                if ((now_ms - app_scan_last_front_invalid_log_ms) >= APP_SERVO_SCAN_FRONT_INVALID_LOG_MS)
                {
                    app_scan_last_front_invalid_log_ms = now_ms;
                    APP_LOG("APP SERVO_SCAN: front invalid, stop for safety");
                }
                break;
            }

            if (front_cm <= APP_SERVO_SCAN_FRONT_STOP_CM)
            {
                APP_LOG("APP SERVO_SCAN: obstacle detected front=%ucm",
                        (unsigned int)front_cm);
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_OBSTACLE_STOP,
                                                 APP_SERVO_SCAN_REASON_OBSTACLE_DETECTED,
                                                 now_ms);
                break;
            }

            App_ServoScanObstacle_ApplyForward();
            break;

        case APP_SERVO_SCAN_STATE_OBSTACLE_STOP:
            App_ServoScanObstacle_ApplyStop();
            if (state_elapsed_ms >= APP_SERVO_SCAN_STOP_SETTLE_MS)
            {
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_SCAN_LEFT,
                                                 APP_SERVO_SCAN_REASON_SCAN_LEFT,
                                                 now_ms);
            }
            break;

        case APP_SERVO_SCAN_STATE_SCAN_LEFT:
            App_ServoScanObstacle_ApplyStop();
            if (now_ms < app_scan_next_sample_ms)
            {
                break;
            }

            if (App_ServoScanObstacle_TakeSample() != 0U)
            {
                (void)App_ServoScanObstacle_FinishSamples(&app_scan_left_cm);
                APP_LOG("APP SERVO_SCAN: scan left distance=%s",
                        App_ServoScanObstacle_FormatDistance(app_scan_left_cm,
                                                             left_text,
                                                             sizeof(left_text)));
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_SCAN_RIGHT,
                                                 APP_SERVO_SCAN_REASON_SCAN_RIGHT,
                                                 now_ms);
            }
            else
            {
                app_scan_next_sample_ms = now_ms + APP_SERVO_SCAN_SAMPLE_INTERVAL_MS;
            }
            break;

        case APP_SERVO_SCAN_STATE_SCAN_RIGHT:
            App_ServoScanObstacle_ApplyStop();
            if (now_ms < app_scan_next_sample_ms)
            {
                break;
            }

            if (App_ServoScanObstacle_TakeSample() != 0U)
            {
                (void)App_ServoScanObstacle_FinishSamples(&app_scan_right_cm);
                APP_LOG("APP SERVO_SCAN: scan right distance=%s",
                        App_ServoScanObstacle_FormatDistance(app_scan_right_cm,
                                                             right_text,
                                                             sizeof(right_text)));
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_DECIDE_TURN,
                                                 APP_SERVO_SCAN_REASON_DECIDE,
                                                 now_ms);
            }
            else
            {
                app_scan_next_sample_ms = now_ms + APP_SERVO_SCAN_SAMPLE_INTERVAL_MS;
            }
            break;

        case APP_SERVO_SCAN_STATE_DECIDE_TURN:
            App_ServoScanObstacle_ApplyStop();
            if ((App_ServoScanObstacle_SideBlocked(app_scan_left_cm) != 0U) &&
                (App_ServoScanObstacle_SideBlocked(app_scan_right_cm) != 0U))
            {
                app_scan_after_backup_turn =
                    (App_ServoScanObstacle_SideScore(app_scan_left_cm) >= App_ServoScanObstacle_SideScore(app_scan_right_cm)) ?
                    APP_SERVO_SCAN_STATE_TURN_LEFT :
                    APP_SERVO_SCAN_STATE_TURN_RIGHT;
                APP_LOG("APP SERVO_SCAN: backup because both blocked left=%s right=%s next=%s",
                        App_ServoScanObstacle_FormatDistance(app_scan_left_cm,
                                                             left_text,
                                                             sizeof(left_text)),
                        App_ServoScanObstacle_FormatDistance(app_scan_right_cm,
                                                             right_text,
                                                             sizeof(right_text)),
                        App_ServoScanObstacle_StateName(app_scan_after_backup_turn));
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_BACKUP,
                                                 APP_SERVO_SCAN_REASON_BOTH_BLOCKED_BACKUP,
                                                 now_ms);
            }
            else if (App_ServoScanObstacle_SideScore(app_scan_left_cm) >= App_ServoScanObstacle_SideScore(app_scan_right_cm))
            {
                APP_LOG("APP SERVO_SCAN: choose left left=%s right=%s",
                        App_ServoScanObstacle_FormatDistance(app_scan_left_cm,
                                                             left_text,
                                                             sizeof(left_text)),
                        App_ServoScanObstacle_FormatDistance(app_scan_right_cm,
                                                             right_text,
                                                             sizeof(right_text)));
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_TURN_LEFT,
                                                 APP_SERVO_SCAN_REASON_CHOOSE_LEFT,
                                                 now_ms);
            }
            else
            {
                APP_LOG("APP SERVO_SCAN: choose right left=%s right=%s",
                        App_ServoScanObstacle_FormatDistance(app_scan_left_cm,
                                                             left_text,
                                                             sizeof(left_text)),
                        App_ServoScanObstacle_FormatDistance(app_scan_right_cm,
                                                             right_text,
                                                             sizeof(right_text)));
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_TURN_RIGHT,
                                                 APP_SERVO_SCAN_REASON_CHOOSE_RIGHT,
                                                 now_ms);
            }
            break;

        case APP_SERVO_SCAN_STATE_TURN_LEFT:
            App_ServoScanObstacle_ApplyTurnLeft();
            if (state_elapsed_ms >= APP_SERVO_SCAN_TURN_MS)
            {
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_RECOVER,
                                                 APP_SERVO_SCAN_REASON_TURN_DONE,
                                                 now_ms);
            }
            break;

        case APP_SERVO_SCAN_STATE_TURN_RIGHT:
            App_ServoScanObstacle_ApplyTurnRight();
            if (state_elapsed_ms >= APP_SERVO_SCAN_TURN_MS)
            {
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_RECOVER,
                                                 APP_SERVO_SCAN_REASON_TURN_DONE,
                                                 now_ms);
            }
            break;

        case APP_SERVO_SCAN_STATE_BACKUP:
            App_ServoScanObstacle_ApplyBackup();
            if (state_elapsed_ms >= APP_SERVO_SCAN_BACKUP_MS)
            {
                App_ServoScanObstacle_EnterState(app_scan_after_backup_turn,
                                                 APP_SERVO_SCAN_REASON_BACKUP_DONE,
                                                 now_ms);
            }
            break;

        case APP_SERVO_SCAN_STATE_RECOVER:
            App_ServoScanObstacle_ApplyStop();
            if (state_elapsed_ms >= APP_SERVO_SCAN_RECOVER_MS)
            {
                APP_LOG("APP SERVO_SCAN: recover forward");
                App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_DRIVE_FORWARD,
                                                 APP_SERVO_SCAN_REASON_RECOVER_FORWARD,
                                                 now_ms);
            }
            break;

        default:
            App_ServoScanObstacle_EnterState(APP_SERVO_SCAN_STATE_START_DELAY,
                                             APP_SERVO_SCAN_REASON_INIT,
                                             now_ms);
            break;
    }
}

static void App_ServoScanObstacle_EnterState(AppServoScanState state,
                                             AppServoScanReason reason,
                                             uint32_t now_ms)
{
    app_scan_state = state;
    app_scan_state_enter_ms = now_ms;

    switch (state)
    {
        case APP_SERVO_SCAN_STATE_START_DELAY:
        case APP_SERVO_SCAN_STATE_OBSTACLE_STOP:
        case APP_SERVO_SCAN_STATE_DECIDE_TURN:
        case APP_SERVO_SCAN_STATE_RECOVER:
            App_Servo_Center();
            App_ServoScanObstacle_ApplyStop();
            break;

        case APP_SERVO_SCAN_STATE_DRIVE_FORWARD:
            App_Servo_Center();
            break;

        case APP_SERVO_SCAN_STATE_SCAN_LEFT:
            App_ServoScanObstacle_ResetSamples();
            App_Servo_Left();
            app_scan_next_sample_ms = now_ms + APP_SERVO_SCAN_SERVO_SETTLE_MS;
            App_ServoScanObstacle_ApplyStop();
            break;

        case APP_SERVO_SCAN_STATE_SCAN_RIGHT:
            App_ServoScanObstacle_ResetSamples();
            App_Servo_Right();
            app_scan_next_sample_ms = now_ms + APP_SERVO_SCAN_SERVO_SETTLE_MS;
            App_ServoScanObstacle_ApplyStop();
            break;

        case APP_SERVO_SCAN_STATE_TURN_LEFT:
            App_Servo_Center();
            App_ServoScanObstacle_ApplyTurnLeft();
            break;

        case APP_SERVO_SCAN_STATE_TURN_RIGHT:
            App_Servo_Center();
            App_ServoScanObstacle_ApplyTurnRight();
            break;

        case APP_SERVO_SCAN_STATE_BACKUP:
            App_Servo_Center();
            App_ServoScanObstacle_ApplyBackup();
            break;

        default:
            break;
    }

    APP_LOG("APP SERVO_SCAN: state=%s reason=%s",
            App_ServoScanObstacle_StateName(state),
            App_ServoScanObstacle_ReasonName(reason));
}

static void App_ServoScanObstacle_ResetSamples(void)
{
    uint8_t i;

    for (i = 0U; i < APP_SERVO_SCAN_SAMPLE_COUNT; i++)
    {
        app_scan_samples[i] = APP_ULTRASONIC_INVALID_CM;
    }

    app_scan_sample_count = 0U;
}

static uint8_t App_ServoScanObstacle_TakeSample(void)
{
    uint16_t sample_cm = APP_ULTRASONIC_INVALID_CM;

    if (app_scan_sample_count >= APP_SERVO_SCAN_SAMPLE_COUNT)
    {
        return 1U;
    }

    if (App_Ultrasonic_ReadDistanceCm(&sample_cm) == 0U)
    {
        sample_cm = APP_ULTRASONIC_INVALID_CM;
    }

    app_scan_samples[app_scan_sample_count] = sample_cm;
    app_scan_sample_count++;

    return (app_scan_sample_count >= APP_SERVO_SCAN_SAMPLE_COUNT) ? 1U : 0U;
}

static uint8_t App_ServoScanObstacle_FinishSamples(uint16_t *distance_cm)
{
    uint16_t valid_samples[APP_SERVO_SCAN_SAMPLE_COUNT];
    uint8_t valid_count = 0U;
    uint8_t i;

    if (distance_cm == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < APP_SERVO_SCAN_SAMPLE_COUNT; i++)
    {
        if (app_scan_samples[i] != APP_ULTRASONIC_INVALID_CM)
        {
            valid_samples[valid_count] = app_scan_samples[i];
            valid_count++;
        }
    }

    if (valid_count == 0U)
    {
        *distance_cm = APP_ULTRASONIC_INVALID_CM;
        return 0U;
    }

    if (valid_count >= 2U)
    {
        uint8_t j;
        for (i = 0U; i < valid_count - 1U; i++)
        {
            for (j = i + 1U; j < valid_count; j++)
            {
                if (valid_samples[j] < valid_samples[i])
                {
                    uint16_t tmp = valid_samples[i];
                    valid_samples[i] = valid_samples[j];
                    valid_samples[j] = tmp;
                }
            }
        }
    }

    if (valid_count == 2U)
    {
        *distance_cm = (uint16_t)(((uint32_t)valid_samples[0] + (uint32_t)valid_samples[1]) / 2U);
    }
    else
    {
        *distance_cm = valid_samples[valid_count / 2U];
    }

    return 1U;
}

static uint16_t App_ServoScanObstacle_SideScore(uint16_t distance_cm)
{
    if (distance_cm == APP_ULTRASONIC_INVALID_CM)
    {
        return 0U;
    }

    return distance_cm;
}

static uint8_t App_ServoScanObstacle_SideBlocked(uint16_t distance_cm)
{
    if (distance_cm == APP_ULTRASONIC_INVALID_CM)
    {
        return 1U;
    }

    return (distance_cm <= APP_SERVO_SCAN_SIDE_BLOCKED_CM) ? 1U : 0U;
}

static void App_ServoScanObstacle_ApplyStop(void)
{
    Chassis_Stop();
    MotorDriver_StopAll();
}

static void App_ServoScanObstacle_ApplyForward(void)
{
    Chassis_SetRaw(APP_SERVO_SCAN_FORWARD_LEFT_CMD, APP_SERVO_SCAN_FORWARD_RIGHT_CMD);
}

static void App_ServoScanObstacle_ApplyTurnLeft(void)
{
    Chassis_SetRaw((int16_t)-APP_SERVO_SCAN_TURN_DUTY, APP_SERVO_SCAN_TURN_DUTY);
}

static void App_ServoScanObstacle_ApplyTurnRight(void)
{
    Chassis_SetRaw(APP_SERVO_SCAN_TURN_DUTY, (int16_t)-APP_SERVO_SCAN_TURN_DUTY);
}

static void App_ServoScanObstacle_ApplyBackup(void)
{
    Chassis_SetRaw((int16_t)-APP_SERVO_SCAN_BACKUP_DUTY, (int16_t)-APP_SERVO_SCAN_BACKUP_DUTY);
}

static const char *App_ServoScanObstacle_StateName(AppServoScanState state)
{
    switch (state)
    {
        case APP_SERVO_SCAN_STATE_START_DELAY:
            return "START_DELAY";

        case APP_SERVO_SCAN_STATE_DRIVE_FORWARD:
            return "DRIVE_FORWARD";

        case APP_SERVO_SCAN_STATE_OBSTACLE_STOP:
            return "OBSTACLE_STOP";

        case APP_SERVO_SCAN_STATE_SCAN_LEFT:
            return "SCAN_LEFT";

        case APP_SERVO_SCAN_STATE_SCAN_RIGHT:
            return "SCAN_RIGHT";

        case APP_SERVO_SCAN_STATE_DECIDE_TURN:
            return "DECIDE_TURN";

        case APP_SERVO_SCAN_STATE_TURN_LEFT:
            return "TURN_LEFT";

        case APP_SERVO_SCAN_STATE_TURN_RIGHT:
            return "TURN_RIGHT";

        case APP_SERVO_SCAN_STATE_BACKUP:
            return "BACKUP";

        case APP_SERVO_SCAN_STATE_RECOVER:
            return "RECOVER";

        default:
            return "UNKNOWN";
    }
}

static const char *App_ServoScanObstacle_ReasonName(AppServoScanReason reason)
{
    switch (reason)
    {
        case APP_SERVO_SCAN_REASON_START_DELAY:
            return "start_delay";

        case APP_SERVO_SCAN_REASON_RECOVER_FORWARD:
            return "recover_forward";

        case APP_SERVO_SCAN_REASON_OBSTACLE_DETECTED:
            return "obstacle_detected";

        case APP_SERVO_SCAN_REASON_SCAN_LEFT:
            return "scan_left";

        case APP_SERVO_SCAN_REASON_SCAN_RIGHT:
            return "scan_right";

        case APP_SERVO_SCAN_REASON_DECIDE:
            return "decide_turn";

        case APP_SERVO_SCAN_REASON_CHOOSE_LEFT:
            return "choose_left";

        case APP_SERVO_SCAN_REASON_CHOOSE_RIGHT:
            return "choose_right";

        case APP_SERVO_SCAN_REASON_BOTH_BLOCKED_BACKUP:
            return "both_blocked_backup";

        case APP_SERVO_SCAN_REASON_BACKUP_DONE:
            return "backup_done";

        case APP_SERVO_SCAN_REASON_TURN_DONE:
            return "turn_done";

        case APP_SERVO_SCAN_REASON_INIT:
        default:
            return "init";
    }
}

static const char *App_ServoScanObstacle_FormatDistance(uint16_t distance_cm,
                                                        char *buffer,
                                                        uint32_t buffer_size)
{
    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return "";
    }

    if (distance_cm == APP_ULTRASONIC_INVALID_CM)
    {
        (void)snprintf(buffer, buffer_size, "--");
        return buffer;
    }

    (void)snprintf(buffer, buffer_size, "%ucm", (unsigned int)distance_cm);
    return buffer;
}
