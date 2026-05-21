#ifndef APP_OBSTACLE_H
#define APP_OBSTACLE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_OBS_DECISION_NO_LIDAR = 0,
    APP_OBS_DECISION_CLEAR_FORWARD,
    APP_OBS_DECISION_TURN_LEFT,
    APP_OBS_DECISION_TURN_RIGHT,
    APP_OBS_DECISION_STOP_BLOCKED
} AppObstacleDecision;

void App_Obstacle_Init(void);
void App_Obstacle_Task(void);
AppObstacleDecision App_Obstacle_GetDecision(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_OBSTACLE_H */
