#include "app_obstacle.h"

#include "app_lidar.h"
#include "bringup_log.h"

#include <stdint.h>
#include <stdio.h>

#define APP_OBS_LOG_INTERVAL_MS 1000U
#define APP_OBS_FRONT_CLEAR_MM 450U
#define APP_OBS_STOP_FRONT_MM 220U
#define APP_OBS_SIDE_BLOCKED_MM 300U

static AppObstacleDecision app_obstacle_decision = APP_OBS_DECISION_NO_LIDAR;

static AppObstacleDecision App_Obstacle_Evaluate(const AppLidarStatus *lidar);
static const char *App_Obstacle_DecisionName(AppObstacleDecision decision);
static const char *App_Obstacle_FormatDistance(uint8_t valid,
                                                uint16_t distance_mm,
                                                char *buffer,
                                                uint32_t buffer_size);

void App_Obstacle_Init(void)
{
    app_obstacle_decision = APP_OBS_DECISION_NO_LIDAR;
    APP_LOG("APP OBS: init log-only mode");
}

void App_Obstacle_Task(void)
{
    static uint32_t last_log_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    uint8_t front_valid = 0U;
    uint8_t left_valid = 0U;
    uint8_t right_valid = 0U;
    uint16_t front_mm = 0U;
    uint16_t left_mm = 0U;
    uint16_t right_mm = 0U;
    char front_text[16];
    char left_text[16];
    char right_text[16];

    app_obstacle_decision = App_Obstacle_Evaluate(lidar);

    if ((now_ms - last_log_ms) < APP_OBS_LOG_INTERVAL_MS)
    {
        return;
    }

    last_log_ms = now_ms;

    if ((lidar != NULL) && (lidar->ready != 0U))
    {
        front_valid = lidar->front_valid;
        left_valid = lidar->left_valid;
        right_valid = lidar->right_valid;
        front_mm = lidar->front_min_mm;
        left_mm = lidar->left_min_mm;
        right_mm = lidar->right_min_mm;
    }

    APP_LOG("APP OBS: decision=%s front=%s left=%s right=%s",
            App_Obstacle_DecisionName(app_obstacle_decision),
            App_Obstacle_FormatDistance(front_valid, front_mm, front_text, sizeof(front_text)),
            App_Obstacle_FormatDistance(left_valid, left_mm, left_text, sizeof(left_text)),
            App_Obstacle_FormatDistance(right_valid, right_mm, right_text, sizeof(right_text)));
}

AppObstacleDecision App_Obstacle_GetDecision(void)
{
    return app_obstacle_decision;
}

static AppObstacleDecision App_Obstacle_Evaluate(const AppLidarStatus *lidar)
{
    if ((lidar == NULL) || (lidar->ready == 0U))
    {
        return APP_OBS_DECISION_NO_LIDAR;
    }

    if (lidar->front_valid == 0U)
    {
        return APP_OBS_DECISION_CLEAR_FORWARD;
    }

    if (lidar->front_min_mm > APP_OBS_FRONT_CLEAR_MM)
    {
        return APP_OBS_DECISION_CLEAR_FORWARD;
    }

    if (lidar->front_min_mm <= APP_OBS_STOP_FRONT_MM)
    {
        uint8_t left_blocked = ((lidar->left_valid != 0U) &&
                                (lidar->left_min_mm < APP_OBS_SIDE_BLOCKED_MM)) ? 1U : 0U;
        uint8_t right_blocked = ((lidar->right_valid != 0U) &&
                                 (lidar->right_min_mm < APP_OBS_SIDE_BLOCKED_MM)) ? 1U : 0U;

        if ((left_blocked != 0U) && (right_blocked != 0U))
        {
            return APP_OBS_DECISION_STOP_BLOCKED;
        }
    }

    if ((lidar->left_valid != 0U) && (lidar->right_valid != 0U))
    {
        if (lidar->left_min_mm >= lidar->right_min_mm)
        {
            return APP_OBS_DECISION_TURN_LEFT;
        }

        return APP_OBS_DECISION_TURN_RIGHT;
    }

    if (lidar->left_valid != 0U)
    {
        return APP_OBS_DECISION_TURN_LEFT;
    }

    if (lidar->right_valid != 0U)
    {
        return APP_OBS_DECISION_TURN_RIGHT;
    }

    return APP_OBS_DECISION_STOP_BLOCKED;
}

static const char *App_Obstacle_DecisionName(AppObstacleDecision decision)
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

static const char *App_Obstacle_FormatDistance(uint8_t valid,
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
