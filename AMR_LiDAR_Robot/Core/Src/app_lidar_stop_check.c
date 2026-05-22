#define APP_LIDAR_STOP_CHECK_MOTOR_OWNER

#include "app_lidar_stop_check.h"

#include "app_lidar.h"
#include "app_test_config.h"
#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define APP_LIDAR_STOP_CHECK_LOG_INTERVAL_MS 500U
#define APP_LIDAR_STOP_CHECK_START_DELAY_MS 3000U
#define APP_LIDAR_STOP_CHECK_FRONT_STOP_MM 350U
#define APP_LIDAR_STOP_CHECK_FRONT_RESUME_MM 450U
#define APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD 220
#define APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD 260
#define APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM 0xFFFFU

#if (APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD > 220) || (APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD > 260)
#error "LidarObstacleStopCheck motor command must stay at or below left=220 right=260"
#endif

typedef enum
{
    APP_LIDAR_STOP_REASON_INIT = 0,
    APP_LIDAR_STOP_REASON_START_DELAY,
    APP_LIDAR_STOP_REASON_LIDAR_NOT_READY,
    APP_LIDAR_STOP_REASON_FRONT_INVALID,
    APP_LIDAR_STOP_REASON_FRONT_LT_350_STOP,
    APP_LIDAR_STOP_REASON_FRONT_GT_450_RESUME,
    APP_LIDAR_STOP_REASON_FRONT_GT_450_FORWARD,
    APP_LIDAR_STOP_REASON_HYSTERESIS_HOLD_STOP,
    APP_LIDAR_STOP_REASON_HYSTERESIS_HOLD_FORWARD
} AppLidarStopReason;

static const char *App_LidarStopCheck_ReasonName(AppLidarStopReason reason);
static void App_LidarStopCheck_LogFinal(uint32_t now_ms,
                                        uint32_t control_seq,
                                        uint8_t lidar_ready,
                                        uint8_t front_valid,
                                        uint16_t front_min_mm,
                                        uint8_t would_clear_forward,
                                        uint8_t drive_latched,
                                        uint8_t obstacle_stop,
                                        int16_t left_cmd,
                                        int16_t right_cmd,
                                        AppLidarStopReason reason);

static uint32_t app_lidar_stop_check_start_ms = 0U;
static uint32_t app_lidar_stop_check_control_seq = 0U;
static uint8_t app_lidar_stop_check_drive_latched = 0U;

void App_LidarStopCheck_Init(void)
{
    app_lidar_stop_check_start_ms = HAL_GetTick();
    app_lidar_stop_check_control_seq = 0U;
    app_lidar_stop_check_drive_latched = 0U;

    Chassis_Init();
    Chassis_Stop();
    MotorDriver_StopAll();

    APP_LOG("APP LIDAR STOP: init start_delay_ms=%u stop_mm=%u resume_mm=%u output_enable=%u left_cmd=%d right_cmd=%d",
            (unsigned int)APP_LIDAR_STOP_CHECK_START_DELAY_MS,
            (unsigned int)APP_LIDAR_STOP_CHECK_FRONT_STOP_MM,
            (unsigned int)APP_LIDAR_STOP_CHECK_FRONT_RESUME_MM,
            (unsigned int)APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE,
            APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD,
            APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD);
}

void App_LidarStopCheck_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t control_seq = ++app_lidar_stop_check_control_seq;
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    uint8_t lidar_ready = 0U;
    uint8_t front_valid = 0U;
    uint16_t front_min_mm = APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM;
    uint8_t front_distance_valid = 0U;
    uint8_t obstacle_stop = 1U;
    uint8_t would_clear_forward = 0U;
    int16_t left_cmd = 0;
    int16_t right_cmd = 0;
    AppLidarStopReason reason = APP_LIDAR_STOP_REASON_LIDAR_NOT_READY;
    uint8_t previous_drive_latched = app_lidar_stop_check_drive_latched;
    uint8_t start_delay_active = ((now_ms - app_lidar_stop_check_start_ms) <
                                  APP_LIDAR_STOP_CHECK_START_DELAY_MS) ? 1U : 0U;

    if (lidar != NULL)
    {
        lidar_ready = lidar->ready;
        front_valid = lidar->front_valid;
        front_min_mm = lidar->front_min_mm;

        front_distance_valid = ((lidar_ready != 0U) &&
                                (front_valid != 0U) &&
                                (front_min_mm != 0U) &&
                                (front_min_mm != APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM)) ? 1U : 0U;

        if ((lidar_ready != 0U) &&
            (front_valid != 0U) &&
            (front_distance_valid != 0U) &&
            (front_min_mm >= APP_LIDAR_STOP_CHECK_FRONT_STOP_MM))
        {
            would_clear_forward = 1U;
        }
    }

    if (start_delay_active != 0U)
    {
        app_lidar_stop_check_drive_latched = 0U;
        reason = APP_LIDAR_STOP_REASON_START_DELAY;
    }
    else if (lidar != NULL)
    {
        if (lidar_ready == 0U)
        {
            app_lidar_stop_check_drive_latched = 0U;
            reason = APP_LIDAR_STOP_REASON_LIDAR_NOT_READY;
        }
        else if ((front_valid == 0U) || (front_distance_valid == 0U))
        {
            app_lidar_stop_check_drive_latched = 0U;
            reason = APP_LIDAR_STOP_REASON_FRONT_INVALID;
        }
        else if (front_min_mm < APP_LIDAR_STOP_CHECK_FRONT_STOP_MM)
        {
            app_lidar_stop_check_drive_latched = 0U;
            reason = APP_LIDAR_STOP_REASON_FRONT_LT_350_STOP;
        }
        else if (front_min_mm > APP_LIDAR_STOP_CHECK_FRONT_RESUME_MM)
        {
            reason = (previous_drive_latched != 0U) ?
                     APP_LIDAR_STOP_REASON_FRONT_GT_450_FORWARD :
                     APP_LIDAR_STOP_REASON_FRONT_GT_450_RESUME;
            app_lidar_stop_check_drive_latched = 1U;
        }
        else
        {
            reason = (app_lidar_stop_check_drive_latched != 0U) ?
                     APP_LIDAR_STOP_REASON_HYSTERESIS_HOLD_FORWARD :
                     APP_LIDAR_STOP_REASON_HYSTERESIS_HOLD_STOP;
        }
    }
    else
    {
        app_lidar_stop_check_drive_latched = 0U;
        reason = APP_LIDAR_STOP_REASON_LIDAR_NOT_READY;
    }

    if (app_lidar_stop_check_drive_latched == 0U)
    {
        left_cmd = 0;
        right_cmd = 0;
        obstacle_stop = 1U;

        APP_LOG("APP LIDAR CMD: seq=%lu action=STOP",
                (unsigned long)control_seq);
        Chassis_Stop();
        MotorDriver_StopAll();
        App_LidarStopCheck_LogFinal(now_ms,
                                    control_seq,
                                    lidar_ready,
                                    front_valid,
                                    front_min_mm,
                                    would_clear_forward,
                                    app_lidar_stop_check_drive_latched,
                                    obstacle_stop,
                                    left_cmd,
                                    right_cmd,
                                    reason);
        return;
    }

#if (APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE != 0)
    obstacle_stop = 0U;
    left_cmd = APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD;
    right_cmd = APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD;

    APP_LOG("APP LIDAR CMD: seq=%lu action=FORWARD left=%d right=%d",
            (unsigned long)control_seq,
            (int)left_cmd,
            (int)right_cmd);
    Chassis_SetRaw(left_cmd, right_cmd);
    App_LidarStopCheck_LogFinal(now_ms,
                                control_seq,
                                lidar_ready,
                                front_valid,
                                front_min_mm,
                                would_clear_forward,
                                app_lidar_stop_check_drive_latched,
                                obstacle_stop,
                                left_cmd,
                                right_cmd,
                                reason);
    return;
#else
    left_cmd = 0;
    right_cmd = 0;
    obstacle_stop = 1U;

    APP_LOG("APP LIDAR CMD: seq=%lu action=STOP",
            (unsigned long)control_seq);
    Chassis_Stop();
    MotorDriver_StopAll();
    App_LidarStopCheck_LogFinal(now_ms,
                                control_seq,
                                lidar_ready,
                                front_valid,
                                front_min_mm,
                                would_clear_forward,
                                app_lidar_stop_check_drive_latched,
                                obstacle_stop,
                                left_cmd,
                                right_cmd,
                                reason);
    return;
#endif
}

static void App_LidarStopCheck_LogFinal(uint32_t now_ms,
                                        uint32_t control_seq,
                                        uint8_t lidar_ready,
                                        uint8_t front_valid,
                                        uint16_t front_min_mm,
                                        uint8_t would_clear_forward,
                                        uint8_t drive_latched,
                                        uint8_t obstacle_stop,
                                        int16_t left_cmd,
                                        int16_t right_cmd,
                                        AppLidarStopReason reason)
{
    static uint32_t last_log_ms = 0U;
    static uint8_t have_last_final = 0U;
    static uint8_t last_drive_latched = 0U;
    static uint8_t last_obstacle_stop = 0U;
    static int16_t last_left_cmd = 0;
    static int16_t last_right_cmd = 0;
    static AppLidarStopReason last_reason = APP_LIDAR_STOP_REASON_INIT;
    uint8_t final_changed = ((have_last_final == 0U) ||
                             (last_drive_latched != drive_latched) ||
                             (last_obstacle_stop != obstacle_stop) ||
                             (last_left_cmd != left_cmd) ||
                             (last_right_cmd != right_cmd) ||
                             (last_reason != reason)) ? 1U : 0U;

    if (((now_ms - last_log_ms) < APP_LIDAR_STOP_CHECK_LOG_INTERVAL_MS) &&
        (final_changed == 0U))
    {
        return;
    }

    last_log_ms = now_ms;
    have_last_final = 1U;
    last_drive_latched = drive_latched;
    last_obstacle_stop = obstacle_stop;
    last_left_cmd = left_cmd;
    last_right_cmd = right_cmd;
    last_reason = reason;

    APP_LOG("APP LIDAR STOP: seq=%lu ready=%u front_valid=%u front_min_mm=%u would_clear_forward=%u drive_latched=%u obstacle_stop=%u left_cmd=%d right_cmd=%d reason=%s",
            (unsigned long)control_seq,
            (unsigned int)lidar_ready,
            (unsigned int)front_valid,
            (unsigned int)front_min_mm,
            (unsigned int)would_clear_forward,
            (unsigned int)drive_latched,
            (unsigned int)obstacle_stop,
            (int)left_cmd,
            (int)right_cmd,
            App_LidarStopCheck_ReasonName(reason));
}

static const char *App_LidarStopCheck_ReasonName(AppLidarStopReason reason)
{
    switch (reason)
    {
        case APP_LIDAR_STOP_REASON_START_DELAY:
            return "start_delay";

        case APP_LIDAR_STOP_REASON_LIDAR_NOT_READY:
            return "lidar_not_ready";

        case APP_LIDAR_STOP_REASON_FRONT_INVALID:
            return "front_invalid";

        case APP_LIDAR_STOP_REASON_FRONT_LT_350_STOP:
            return "front_lt_350mm_stop";

        case APP_LIDAR_STOP_REASON_FRONT_GT_450_RESUME:
            return "front_gt_450mm_resume";

        case APP_LIDAR_STOP_REASON_FRONT_GT_450_FORWARD:
            return "front_gt_450mm_forward";

        case APP_LIDAR_STOP_REASON_HYSTERESIS_HOLD_STOP:
            return "hysteresis_hold_stop";

        case APP_LIDAR_STOP_REASON_HYSTERESIS_HOLD_FORWARD:
            return "hysteresis_hold_forward";

        case APP_LIDAR_STOP_REASON_INIT:
        default:
            return "init";
    }
}
