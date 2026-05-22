#define APP_LIDAR_STOP_CHECK_MOTOR_OWNER

#include "app_lidar_stop_check.h"

#include "app_lidar.h"
#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define APP_LIDAR_STOP_CHECK_LOG_INTERVAL_MS 500U
#define APP_LIDAR_STOP_CHECK_START_DELAY_MS 3000U
#define APP_LIDAR_STOP_CHECK_FRONT_STOP_MM 350U
#define APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE 0
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
    APP_LIDAR_STOP_REASON_NO_STATUS,
    APP_LIDAR_STOP_REASON_LIDAR_NOT_READY,
    APP_LIDAR_STOP_REASON_FRONT_INVALID,
    APP_LIDAR_STOP_REASON_FRONT_INVALID_DISTANCE,
    APP_LIDAR_STOP_REASON_FRONT_OBSTACLE,
    APP_LIDAR_STOP_REASON_CLEAR_FORWARD
} AppLidarStopReason;

static const char *App_LidarStopCheck_ReasonName(AppLidarStopReason reason);

static uint32_t app_lidar_stop_check_start_ms = 0U;

void App_LidarStopCheck_Init(void)
{
    app_lidar_stop_check_start_ms = HAL_GetTick();

    Chassis_Init();
    Chassis_Stop();
    MotorDriver_StopAll();

    APP_LOG("APP LIDAR STOP: init start_delay_ms=%u stop_mm=%u left_cmd=%d right_cmd=%d",
            (unsigned int)APP_LIDAR_STOP_CHECK_START_DELAY_MS,
            (unsigned int)APP_LIDAR_STOP_CHECK_FRONT_STOP_MM,
            APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD,
            APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD);
}

void App_LidarStopCheck_Task(void)
{
    static uint32_t last_log_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    uint8_t ready = 0U;
    uint8_t front_valid = 0U;
    uint16_t front_min_mm = APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM;
    uint8_t obstacle_stop = 1U;
    uint8_t would_clear_forward = 0U;
    int16_t left_cmd = 0;
    int16_t right_cmd = 0;
    AppLidarStopReason reason = APP_LIDAR_STOP_REASON_NO_STATUS;
    uint8_t start_delay_active = ((now_ms - app_lidar_stop_check_start_ms) <
                                  APP_LIDAR_STOP_CHECK_START_DELAY_MS) ? 1U : 0U;

    if (lidar != NULL)
    {
        ready = lidar->ready;
        front_valid = lidar->front_valid;
        front_min_mm = lidar->front_min_mm;

        if ((ready != 0U) &&
            (front_valid != 0U) &&
            (front_min_mm != 0U) &&
            (front_min_mm != APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM) &&
            (front_min_mm >= APP_LIDAR_STOP_CHECK_FRONT_STOP_MM))
        {
            would_clear_forward = 1U;
        }
    }

    if (start_delay_active != 0U)
    {
        reason = APP_LIDAR_STOP_REASON_START_DELAY;
    }
    else if (lidar != NULL)
    {
        if (ready == 0U)
        {
            reason = APP_LIDAR_STOP_REASON_LIDAR_NOT_READY;
        }
        else if (front_valid == 0U)
        {
            reason = APP_LIDAR_STOP_REASON_FRONT_INVALID;
        }
        else if ((front_min_mm == 0U) ||
                 (front_min_mm == APP_LIDAR_STOP_CHECK_INVALID_DISTANCE_MM))
        {
            reason = APP_LIDAR_STOP_REASON_FRONT_INVALID_DISTANCE;
        }
        else if (front_min_mm < APP_LIDAR_STOP_CHECK_FRONT_STOP_MM)
        {
            reason = APP_LIDAR_STOP_REASON_FRONT_OBSTACLE;
        }
        else
        {
#if APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE
            obstacle_stop = 0U;
            left_cmd = APP_LIDAR_STOP_CHECK_FORWARD_LEFT_CMD;
            right_cmd = APP_LIDAR_STOP_CHECK_FORWARD_RIGHT_CMD;
#endif
            reason = APP_LIDAR_STOP_REASON_CLEAR_FORWARD;
        }
    }

    if (obstacle_stop != 0U)
    {
        Chassis_Stop();
        MotorDriver_StopAll();
    }
    else
    {
#if APP_LIDAR_STOP_CHECK_MOTOR_OUTPUT_ENABLE
        Chassis_SetRaw(left_cmd, right_cmd);
#else
        Chassis_Stop();
        MotorDriver_StopAll();
#endif
    }

    if ((now_ms - last_log_ms) >= APP_LIDAR_STOP_CHECK_LOG_INTERVAL_MS)
    {
        last_log_ms = now_ms;

        APP_LOG("APP LIDAR STOP: ready=%u front_valid=%u front_min_mm=%u would_clear_forward=%u obstacle_stop=%u left_cmd=%d right_cmd=%d reason=%s",
                (unsigned int)ready,
                (unsigned int)front_valid,
                (unsigned int)front_min_mm,
                (unsigned int)would_clear_forward,
                (unsigned int)obstacle_stop,
                (int)left_cmd,
                (int)right_cmd,
                App_LidarStopCheck_ReasonName(reason));
    }
}

static const char *App_LidarStopCheck_ReasonName(AppLidarStopReason reason)
{
    switch (reason)
    {
        case APP_LIDAR_STOP_REASON_START_DELAY:
            return "start_delay";

        case APP_LIDAR_STOP_REASON_NO_STATUS:
            return "no_status";

        case APP_LIDAR_STOP_REASON_LIDAR_NOT_READY:
            return "lidar_not_ready";

        case APP_LIDAR_STOP_REASON_FRONT_INVALID:
            return "front_invalid";

        case APP_LIDAR_STOP_REASON_FRONT_INVALID_DISTANCE:
            return "front_invalid_distance";

        case APP_LIDAR_STOP_REASON_FRONT_OBSTACLE:
            return "front_lt_350mm";

        case APP_LIDAR_STOP_REASON_CLEAR_FORWARD:
            return "clear_forward";

        case APP_LIDAR_STOP_REASON_INIT:
        default:
            return "init";
    }
}
