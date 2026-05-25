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
#define APP_LIDAR_STOP_CHECK_FRONT_STOP_MM 850U
#define APP_LIDAR_STOP_CHECK_CLEAR_MM 900U
#define APP_LIDAR_STOP_CHECK_CLEAR_COUNT_REQUIRED 5U
#define APP_LIDAR_STOP_CHECK_STOP_HOLD_MS 500U
#define APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD 498
#define APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD 500
#define APP_LIDAR_STOP_CHECK_USE_FRONT_WIDE_STOP 1U
#define APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM 0xFFFFU

#if (APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD < -500) || (APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD > 500) || \
    (APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD < -500) || (APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD > 500)
#error "LidarObstacleStopCheck motor command absolute value must stay at or below 500"
#endif

typedef enum
{
    APP_LIDAR_STOP_REASON_INIT = 0,
    APP_LIDAR_STOP_REASON_START_DELAY,
    APP_LIDAR_STOP_REASON_LIDAR_INVALID,
    APP_LIDAR_STOP_REASON_OBSTACLE_LT_STOP,
    APP_LIDAR_STOP_REASON_CLEAR_COUNT_WAIT,
    APP_LIDAR_STOP_REASON_CLEAR_COUNT_RESUME,
    APP_LIDAR_STOP_REASON_HYSTERESIS_FORWARD
} AppLidarStopReason;

typedef enum
{
    APP_LIDAR_STOP_OBSTACLE_SOURCE_NONE = 0,
    APP_LIDAR_STOP_OBSTACLE_SOURCE_FRONT,
    APP_LIDAR_STOP_OBSTACLE_SOURCE_FRONT_WIDE,
    APP_LIDAR_STOP_OBSTACLE_SOURCE_MIN_FRONT_FRONT_WIDE
} AppLidarStopObstacleSource;

static const char *App_LidarStopCheck_ReasonName(AppLidarStopReason reason);
static const char *App_LidarStopCheck_ObstacleSourceName(AppLidarStopObstacleSource source);
static void App_LidarStopCheck_LogFinal(uint32_t now_ms,
                                        uint32_t control_seq,
                                        uint8_t lidar_ready,
                                        uint8_t front_valid,
                                        uint16_t front_min_mm,
                                        uint8_t front_wide_valid,
                                        uint16_t front_wide_min_mm,
                                        uint8_t obstacle_stop_valid,
                                        uint16_t obstacle_stop_min_mm,
                                        AppLidarStopObstacleSource obstacle_stop_source,
                                        uint8_t clear_count,
                                        uint8_t drive_latched,
                                        uint8_t obstacle_stop,
                                        uint32_t stop_hold_elapsed_ms,
                                        int16_t left_cmd,
                                        int16_t right_cmd,
                                        AppLidarStopReason reason);

static uint32_t app_lidar_stop_check_start_ms = 0U;
static uint32_t app_lidar_stop_check_last_stop_ms = 0U;
static uint32_t app_lidar_stop_check_control_seq = 0U;
static uint8_t app_lidar_stop_check_drive_latched = 0U;
static uint8_t app_lidar_stop_check_clear_count = 0U;

void App_LidarStopCheck_Init(void)
{
    app_lidar_stop_check_start_ms = HAL_GetTick();
    app_lidar_stop_check_last_stop_ms = app_lidar_stop_check_start_ms;
    app_lidar_stop_check_control_seq = 0U;
    app_lidar_stop_check_drive_latched = 0U;
    app_lidar_stop_check_clear_count = 0U;

    Chassis_Init();
    Chassis_Stop();
    MotorDriver_StopAll();

    APP_LOG("APP LIDAR STOP: init start_delay_ms=%u stop_mm=%u clear_mm=%u clear_count_required=%u stop_hold_ms=%u output_enable=%u left_cmd=%d right_cmd=%d use_front_wide_stop=%u",
            (unsigned int)APP_LIDAR_STOP_CHECK_START_DELAY_MS,
            (unsigned int)APP_LIDAR_STOP_CHECK_FRONT_STOP_MM,
            (unsigned int)APP_LIDAR_STOP_CHECK_CLEAR_MM,
            (unsigned int)APP_LIDAR_STOP_CHECK_CLEAR_COUNT_REQUIRED,
            (unsigned int)APP_LIDAR_STOP_CHECK_STOP_HOLD_MS,
            (unsigned int)APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE,
            APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD,
            APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD,
            (unsigned int)APP_LIDAR_STOP_CHECK_USE_FRONT_WIDE_STOP);
}

void App_LidarStopCheck_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t control_seq = ++app_lidar_stop_check_control_seq;
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    uint8_t lidar_ready = 0U;
    uint8_t front_valid = 0U;
    uint16_t front_min_mm = APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM;
    uint8_t front_wide_valid = 0U;
    uint16_t front_wide_min_mm = APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM;
    uint8_t obstacle_stop_valid = 0U;
    uint16_t obstacle_stop_min_mm = APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM;
    AppLidarStopObstacleSource obstacle_stop_source = APP_LIDAR_STOP_OBSTACLE_SOURCE_NONE;
    uint8_t obstacle_stop = 1U;
    uint32_t stop_hold_elapsed_ms = now_ms - app_lidar_stop_check_last_stop_ms;
    int16_t left_cmd = 0;
    int16_t right_cmd = 0;
    AppLidarStopReason reason = APP_LIDAR_STOP_REASON_LIDAR_INVALID;
    uint8_t start_delay_active = ((now_ms - app_lidar_stop_check_start_ms) <
                                  APP_LIDAR_STOP_CHECK_START_DELAY_MS) ? 1U : 0U;

    if (lidar != NULL)
    {
        lidar_ready = lidar->ready;
        front_valid = lidar->front_valid;
        front_min_mm = lidar->front_min_mm;
        front_wide_valid = lidar->front_wide_valid;
        front_wide_min_mm = lidar->front_wide_min_mm;

        if (lidar_ready != 0U)
        {
            uint8_t front_candidate_valid =
                ((front_valid != 0U) &&
                 (front_min_mm != 0U) &&
                 (front_min_mm != APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM)) ? 1U : 0U;
            uint8_t front_wide_candidate_valid =
                ((front_wide_valid != 0U) &&
                 (front_wide_min_mm != 0U) &&
                 (front_wide_min_mm != APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM)) ? 1U : 0U;

#if (APP_LIDAR_STOP_CHECK_USE_FRONT_WIDE_STOP != 0)
            if ((front_candidate_valid != 0U) && (front_wide_candidate_valid != 0U))
            {
                obstacle_stop_min_mm = (front_min_mm < front_wide_min_mm) ? front_min_mm : front_wide_min_mm;
                obstacle_stop_valid = 1U;
                obstacle_stop_source = APP_LIDAR_STOP_OBSTACLE_SOURCE_MIN_FRONT_FRONT_WIDE;
            }
            else if (front_candidate_valid != 0U)
            {
                obstacle_stop_min_mm = front_min_mm;
                obstacle_stop_valid = 1U;
                obstacle_stop_source = APP_LIDAR_STOP_OBSTACLE_SOURCE_FRONT;
            }
            else if (front_wide_candidate_valid != 0U)
            {
                obstacle_stop_min_mm = front_wide_min_mm;
                obstacle_stop_valid = 1U;
                obstacle_stop_source = APP_LIDAR_STOP_OBSTACLE_SOURCE_FRONT_WIDE;
            }
#else
            if (front_candidate_valid != 0U)
            {
                obstacle_stop_min_mm = front_min_mm;
                obstacle_stop_valid = 1U;
                obstacle_stop_source = APP_LIDAR_STOP_OBSTACLE_SOURCE_FRONT;
            }
#endif

        }
    }

    if (start_delay_active != 0U)
    {
        app_lidar_stop_check_last_stop_ms = now_ms;
        stop_hold_elapsed_ms = 0U;
        app_lidar_stop_check_drive_latched = 0U;
        app_lidar_stop_check_clear_count = 0U;
        reason = APP_LIDAR_STOP_REASON_START_DELAY;
    }
    else if (lidar != NULL)
    {
        if (lidar_ready == 0U)
        {
            app_lidar_stop_check_last_stop_ms = now_ms;
            stop_hold_elapsed_ms = 0U;
            app_lidar_stop_check_drive_latched = 0U;
            app_lidar_stop_check_clear_count = 0U;
            reason = APP_LIDAR_STOP_REASON_LIDAR_INVALID;
        }
        else if (obstacle_stop_valid == 0U)
        {
            app_lidar_stop_check_last_stop_ms = now_ms;
            stop_hold_elapsed_ms = 0U;
            app_lidar_stop_check_drive_latched = 0U;
            app_lidar_stop_check_clear_count = 0U;
            reason = APP_LIDAR_STOP_REASON_LIDAR_INVALID;
        }
        else if (obstacle_stop_min_mm < APP_LIDAR_STOP_CHECK_FRONT_STOP_MM)
        {
            app_lidar_stop_check_last_stop_ms = now_ms;
            stop_hold_elapsed_ms = 0U;
            app_lidar_stop_check_drive_latched = 0U;
            app_lidar_stop_check_clear_count = 0U;
            reason = APP_LIDAR_STOP_REASON_OBSTACLE_LT_STOP;
        }
        else if (app_lidar_stop_check_drive_latched == 0U)
        {
            if (obstacle_stop_min_mm >= APP_LIDAR_STOP_CHECK_CLEAR_MM)
            {
                if (app_lidar_stop_check_clear_count < APP_LIDAR_STOP_CHECK_CLEAR_COUNT_REQUIRED)
                {
                    app_lidar_stop_check_clear_count++;
                }
            }
            else
            {
                app_lidar_stop_check_clear_count = 0U;
            }

            if ((app_lidar_stop_check_clear_count >= APP_LIDAR_STOP_CHECK_CLEAR_COUNT_REQUIRED) &&
                (stop_hold_elapsed_ms >= APP_LIDAR_STOP_CHECK_STOP_HOLD_MS))
            {
                app_lidar_stop_check_drive_latched = 1U;
                reason = APP_LIDAR_STOP_REASON_CLEAR_COUNT_RESUME;
            }
            else
            {
                reason = APP_LIDAR_STOP_REASON_CLEAR_COUNT_WAIT;
            }
        }
        else
        {
            reason = APP_LIDAR_STOP_REASON_HYSTERESIS_FORWARD;
        }
    }
    else
    {
        app_lidar_stop_check_last_stop_ms = now_ms;
        stop_hold_elapsed_ms = 0U;
        app_lidar_stop_check_drive_latched = 0U;
        app_lidar_stop_check_clear_count = 0U;
        reason = APP_LIDAR_STOP_REASON_LIDAR_INVALID;
    }

    if (app_lidar_stop_check_drive_latched == 0U)
    {
        left_cmd = 0;
        right_cmd = 0;
        obstacle_stop = 1U;

        APP_LOG("APP LIDAR CMD: seq=%lu action=STOP left=%d right=%d",
                (unsigned long)control_seq,
                (int)left_cmd,
                (int)right_cmd);
        Chassis_Stop();
        MotorDriver_StopAll();
        App_LidarStopCheck_LogFinal(now_ms,
                                    control_seq,
                                    lidar_ready,
                                    front_valid,
                                    front_min_mm,
                                    front_wide_valid,
                                    front_wide_min_mm,
                                    obstacle_stop_valid,
                                    obstacle_stop_min_mm,
                                    obstacle_stop_source,
                                    app_lidar_stop_check_clear_count,
                                    app_lidar_stop_check_drive_latched,
                                    obstacle_stop,
                                    stop_hold_elapsed_ms,
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
                                front_wide_valid,
                                front_wide_min_mm,
                                obstacle_stop_valid,
                                obstacle_stop_min_mm,
                                obstacle_stop_source,
                                app_lidar_stop_check_clear_count,
                                app_lidar_stop_check_drive_latched,
                                obstacle_stop,
                                stop_hold_elapsed_ms,
                                left_cmd,
                                right_cmd,
                                reason);
    return;
#else
    left_cmd = 0;
    right_cmd = 0;
    obstacle_stop = 1U;

    APP_LOG("APP LIDAR CMD: seq=%lu action=STOP left=%d right=%d",
            (unsigned long)control_seq,
            (int)left_cmd,
            (int)right_cmd);
    Chassis_Stop();
    MotorDriver_StopAll();
    App_LidarStopCheck_LogFinal(now_ms,
                                control_seq,
                                lidar_ready,
                                front_valid,
                                front_min_mm,
                                front_wide_valid,
                                front_wide_min_mm,
                                obstacle_stop_valid,
                                obstacle_stop_min_mm,
                                obstacle_stop_source,
                                app_lidar_stop_check_clear_count,
                                app_lidar_stop_check_drive_latched,
                                obstacle_stop,
                                stop_hold_elapsed_ms,
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
                                        uint8_t front_wide_valid,
                                        uint16_t front_wide_min_mm,
                                        uint8_t obstacle_stop_valid,
                                        uint16_t obstacle_stop_min_mm,
                                        AppLidarStopObstacleSource obstacle_stop_source,
                                        uint8_t clear_count,
                                        uint8_t drive_latched,
                                        uint8_t obstacle_stop,
                                        uint32_t stop_hold_elapsed_ms,
                                        int16_t left_cmd,
                                        int16_t right_cmd,
                                        AppLidarStopReason reason)
{
    static uint32_t last_log_ms = 0U;
    static uint8_t have_last_final = 0U;
    static uint8_t last_drive_latched = 0U;
    static uint8_t last_obstacle_stop = 0U;
    static uint8_t last_clear_count = 0U;
    static int16_t last_left_cmd = 0;
    static int16_t last_right_cmd = 0;
    static AppLidarStopReason last_reason = APP_LIDAR_STOP_REASON_INIT;
    uint8_t final_changed = ((have_last_final == 0U) ||
                             (last_drive_latched != drive_latched) ||
                             (last_obstacle_stop != obstacle_stop) ||
                             (last_clear_count != clear_count) ||
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
    last_clear_count = clear_count;
    last_left_cmd = left_cmd;
    last_right_cmd = right_cmd;
    last_reason = reason;

    APP_LOG("APP LIDAR STOP: seq=%lu ready=%u front_valid=%u front_min_mm=%u front_wide_valid=%u front_wide_min_mm=%u obstacle_stop_valid=%u obstacle_stop_min_mm=%u obstacle_stop_source=%s clear_count=%u clear_count_required=%u drive_latched=%u obstacle_stop=%u stop_hold_elapsed_ms=%lu left_cmd=%d right_cmd=%d reason=%s",
            (unsigned long)control_seq,
            (unsigned int)lidar_ready,
            (unsigned int)front_valid,
            (unsigned int)front_min_mm,
            (unsigned int)front_wide_valid,
            (unsigned int)front_wide_min_mm,
            (unsigned int)obstacle_stop_valid,
            (unsigned int)obstacle_stop_min_mm,
            App_LidarStopCheck_ObstacleSourceName(obstacle_stop_source),
            (unsigned int)clear_count,
            (unsigned int)APP_LIDAR_STOP_CHECK_CLEAR_COUNT_REQUIRED,
            (unsigned int)drive_latched,
            (unsigned int)obstacle_stop,
            (unsigned long)stop_hold_elapsed_ms,
            (int)left_cmd,
            (int)right_cmd,
            App_LidarStopCheck_ReasonName(reason));
}

static const char *App_LidarStopCheck_ObstacleSourceName(AppLidarStopObstacleSource source)
{
    switch (source)
    {
        case APP_LIDAR_STOP_OBSTACLE_SOURCE_FRONT:
            return "front";

        case APP_LIDAR_STOP_OBSTACLE_SOURCE_FRONT_WIDE:
            return "front_wide";

        case APP_LIDAR_STOP_OBSTACLE_SOURCE_MIN_FRONT_FRONT_WIDE:
            return "min_front_front_wide";

        case APP_LIDAR_STOP_OBSTACLE_SOURCE_NONE:
        default:
            return "none";
    }
}

static const char *App_LidarStopCheck_ReasonName(AppLidarStopReason reason)
{
    switch (reason)
    {
        case APP_LIDAR_STOP_REASON_START_DELAY:
            return "start_delay";

        case APP_LIDAR_STOP_REASON_LIDAR_INVALID:
            return "lidar_invalid";

        case APP_LIDAR_STOP_REASON_OBSTACLE_LT_STOP:
            return "obstacle_lt_850_stop";

        case APP_LIDAR_STOP_REASON_CLEAR_COUNT_WAIT:
            return "clear_count_wait";

        case APP_LIDAR_STOP_REASON_CLEAR_COUNT_RESUME:
            return "clear_count_resume";

        case APP_LIDAR_STOP_REASON_HYSTERESIS_FORWARD:
            return "hysteresis_forward";

        case APP_LIDAR_STOP_REASON_INIT:
        default:
            return "init";
    }
}
