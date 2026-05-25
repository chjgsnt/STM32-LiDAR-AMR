#include "app_lidar_obstacle_avoidance.h"

#include "amr_system.h"
#include "app_lidar.h"
#include "bringup_log.h"
#include "chassis.h"
#include "motor_driver.h"

#include <stdint.h>

#define LIDAR_OBS_START_DELAY_MS 3000U
#define LIDAR_OBS_FRONT_MIN_MM 120U
#define LIDAR_OBS_STOP_MM 400U
#define LIDAR_OBS_FRONT_TIMEOUT_MS 600U
#define LIDAR_OBS_FRONT_INVALID_LIMIT 3U
#define LIDAR_OBS_LOG_INTERVAL_MS 500U
#define LIDAR_OBS_NO_FRONT_LOG_MS 1000U
#define LIDAR_OBS_BACKUP_MS 450U
#define LIDAR_OBS_TURN_MS 1000U
#define LIDAR_OBS_RECOVER_MS 200U
#define LIDAR_OBS_FORWARD_LEFT_DUTY 482
#define LIDAR_OBS_FORWARD_RIGHT_DUTY 500
#define LIDAR_OBS_TURN_DUTY 360
#define LIDAR_OBS_BACKUP_DUTY 320

typedef enum
{
    LIDAR_OBS_STATE_START_DELAY = 0,
    LIDAR_OBS_STATE_WAIT_LIDAR,
    LIDAR_OBS_STATE_DRIVE_FORWARD,
    LIDAR_OBS_STATE_OBSTACLE_STOP,
    LIDAR_OBS_STATE_BACKUP,
    LIDAR_OBS_STATE_TURN,
    LIDAR_OBS_STATE_RECOVER
} LidarObsState;

typedef enum
{
    LIDAR_OBS_REASON_INIT = 0,
    LIDAR_OBS_REASON_START_DELAY_DONE,
    LIDAR_OBS_REASON_LIDAR_READY,
    LIDAR_OBS_REASON_LIDAR_NO_VALID_FRONT,
    LIDAR_OBS_REASON_LIDAR_TIMEOUT,
    LIDAR_OBS_REASON_OBSTACLE_DETECTED,
    LIDAR_OBS_REASON_STOP_DONE,
    LIDAR_OBS_REASON_BACKUP_DONE,
    LIDAR_OBS_REASON_TURN_DONE,
    LIDAR_OBS_REASON_RECOVER_FORWARD
} LidarObsReason;

typedef struct
{
    uint8_t lidar_ready;
    uint8_t front_valid;
    uint16_t front_min_mm;
    uint32_t valid_points;
} LidarObsFront;

static LidarObsState lidar_obs_state = LIDAR_OBS_STATE_START_DELAY;
static uint32_t lidar_obs_start_ms = 0U;
static uint32_t lidar_obs_state_enter_ms = 0U;
static uint32_t lidar_obs_last_front_valid_ms = 0U;
static uint32_t lidar_obs_last_log_ms = 0U;
static uint32_t lidar_obs_last_no_front_log_ms = 0U;
static uint32_t lidar_obs_last_lidar_data_ms = 0U;
static uint32_t lidar_obs_last_valid_points = 0U;
static uint8_t lidar_obs_front_invalid_count = 0U;
static uint8_t lidar_obs_initialized = 0U;

static void App_LidarObstacleAvoidance_EnterState(LidarObsState state,
                                                  LidarObsReason reason,
                                                  uint32_t now_ms);
static LidarObsFront App_LidarObstacleAvoidance_ReadFront(uint32_t now_ms);
static void App_LidarObstacleAvoidance_LogFront(const LidarObsFront *front,
                                                uint32_t now_ms);
static void App_LidarObstacleAvoidance_ApplyStop(void);
static void App_LidarObstacleAvoidance_ApplyForward(void);
static void App_LidarObstacleAvoidance_ApplyBackup(void);
static void App_LidarObstacleAvoidance_ApplyTurn(void);
static const char *App_LidarObstacleAvoidance_StateName(LidarObsState state);
static const char *App_LidarObstacleAvoidance_ReasonName(LidarObsReason reason);
static uint32_t App_LidarObstacleAvoidance_ElapsedMs(uint32_t now_ms, uint32_t then_ms);

void App_LidarObstacleAvoidance_Init(void)
{
    lidar_obs_start_ms = HAL_GetTick();
    lidar_obs_state_enter_ms = lidar_obs_start_ms;
    lidar_obs_last_front_valid_ms = 0U;
    lidar_obs_last_log_ms = 0U;
    lidar_obs_last_no_front_log_ms = 0U;
    lidar_obs_last_lidar_data_ms = 0U;
    lidar_obs_last_valid_points = 0U;
    lidar_obs_front_invalid_count = 0U;
    lidar_obs_state = LIDAR_OBS_STATE_START_DELAY;
    lidar_obs_initialized = 1U;

    Chassis_Init();
    App_LidarObstacleAvoidance_ApplyStop();

    APP_LOG("APP LIDAR_OBS: init start_delay_ms=%u front_min_mm=%u stop_mm=%u front_timeout_ms=%u front_invalid_limit=%u backup_ms=%u turn_ms=%u recover_ms=%u left_duty=%d right_duty=%d backup_duty=%d turn_duty=%d",
            (unsigned int)LIDAR_OBS_START_DELAY_MS,
            (unsigned int)LIDAR_OBS_FRONT_MIN_MM,
            (unsigned int)LIDAR_OBS_STOP_MM,
            (unsigned int)LIDAR_OBS_FRONT_TIMEOUT_MS,
            (unsigned int)LIDAR_OBS_FRONT_INVALID_LIMIT,
            (unsigned int)LIDAR_OBS_BACKUP_MS,
            (unsigned int)LIDAR_OBS_TURN_MS,
            (unsigned int)LIDAR_OBS_RECOVER_MS,
            LIDAR_OBS_FORWARD_LEFT_DUTY,
            LIDAR_OBS_FORWARD_RIGHT_DUTY,
            LIDAR_OBS_BACKUP_DUTY,
            LIDAR_OBS_TURN_DUTY);
    APP_LOG("APP LIDAR_OBS: state=%s reason=%s",
            App_LidarObstacleAvoidance_StateName(lidar_obs_state),
            App_LidarObstacleAvoidance_ReasonName(LIDAR_OBS_REASON_INIT));
}

void App_LidarObstacleAvoidance_Start(void)
{
    uint32_t now_ms = HAL_GetTick();
    AMR_State_t amr_state = AMR_GetState();
    LidarObsFront front;

    if (lidar_obs_initialized == 0U)
    {
        App_LidarObstacleAvoidance_Init();
    }

    lidar_obs_start_ms = now_ms;
    lidar_obs_last_front_valid_ms = 0U;
    lidar_obs_last_log_ms = 0U;
    lidar_obs_last_no_front_log_ms = 0U;
    lidar_obs_last_lidar_data_ms = 0U;
    lidar_obs_last_valid_points = 0U;
    lidar_obs_front_invalid_count = 0U;
    lidar_obs_state_enter_ms = now_ms;

    front = App_LidarObstacleAvoidance_ReadFront(now_ms);

    APP_LOG("APP LIDAR_OBS: start called state=%s amr=%s ready=%u front_valid=%u front=%u valid_points=%lu",
            App_LidarObstacleAvoidance_StateName(lidar_obs_state),
            AMR_StateName(amr_state),
            (unsigned int)front.lidar_ready,
            (unsigned int)front.front_valid,
            (unsigned int)front.front_min_mm,
            (unsigned long)front.valid_points);

    if ((amr_state != AMR_STATE_EXPLORE) && (amr_state != AMR_STATE_AVOID))
    {
        APP_LOG("APP LIDAR_OBS: hold reason=AMR state not EXPLORE/AVOID amr=%s",
                AMR_StateName(amr_state));
        App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_WAIT_LIDAR,
                                              LIDAR_OBS_REASON_LIDAR_NO_VALID_FRONT,
                                              now_ms);
    }
    else if (front.lidar_ready == 0U)
    {
        APP_LOG("APP LIDAR_OBS: hold reason=lidar not ready");
        App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_WAIT_LIDAR,
                                              LIDAR_OBS_REASON_LIDAR_NO_VALID_FRONT,
                                              now_ms);
    }
    else if (front.front_valid == 0U)
    {
        APP_LOG("APP LIDAR_OBS: hold reason=front invalid ready=%u valid_points=%lu",
                (unsigned int)front.lidar_ready,
                (unsigned long)front.valid_points);
        App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_WAIT_LIDAR,
                                              LIDAR_OBS_REASON_LIDAR_NO_VALID_FRONT,
                                              now_ms);
    }
    else if (front.front_min_mm < LIDAR_OBS_STOP_MM)
    {
        APP_LOG("[OBS] hold reason=front_blocked front=%u stop=%u",
                (unsigned int)front.front_min_mm,
                (unsigned int)LIDAR_OBS_STOP_MM);
        App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_BACKUP,
                                              LIDAR_OBS_REASON_OBSTACLE_DETECTED,
                                              now_ms);
    }
    else
    {
        App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_DRIVE_FORWARD,
                                              LIDAR_OBS_REASON_LIDAR_READY,
                                              now_ms);
    }
}

void App_LidarObstacleAvoidance_Update(void)
{
    App_LidarObstacleAvoidance_Task();
}

void App_LidarObstacleAvoidance_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t state_elapsed_ms = App_LidarObstacleAvoidance_ElapsedMs(now_ms, lidar_obs_state_enter_ms);

    if (lidar_obs_initialized == 0U)
    {
        return;
    }

    LidarObsFront front = App_LidarObstacleAvoidance_ReadFront(now_ms);

    App_LidarObstacleAvoidance_LogFront(&front, now_ms);

    switch (lidar_obs_state)
    {
        case LIDAR_OBS_STATE_START_DELAY:
            App_LidarObstacleAvoidance_ApplyStop();
            if (App_LidarObstacleAvoidance_ElapsedMs(now_ms, lidar_obs_start_ms) >= LIDAR_OBS_START_DELAY_MS)
            {
                App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_WAIT_LIDAR,
                                                      LIDAR_OBS_REASON_START_DELAY_DONE,
                                                      now_ms);
            }
            break;

        case LIDAR_OBS_STATE_WAIT_LIDAR:
            App_LidarObstacleAvoidance_ApplyStop();
            if (front.front_valid != 0U)
            {
                App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_DRIVE_FORWARD,
                                                      LIDAR_OBS_REASON_LIDAR_READY,
                                                      now_ms);
            }
            else if (App_LidarObstacleAvoidance_ElapsedMs(now_ms, lidar_obs_last_no_front_log_ms) >= LIDAR_OBS_NO_FRONT_LOG_MS)
            {
                lidar_obs_last_no_front_log_ms = now_ms;
                APP_LOG("APP LIDAR_OBS: waiting_lidar ready=%u valid_points=%lu",
                        (unsigned int)front.lidar_ready,
                        (unsigned long)front.valid_points);
            }
            break;

        case LIDAR_OBS_STATE_DRIVE_FORWARD:
            if (front.front_valid == 0U)
            {
                uint32_t front_age_ms = App_LidarObstacleAvoidance_ElapsedMs(now_ms, lidar_obs_last_front_valid_ms);
                if (lidar_obs_front_invalid_count < LIDAR_OBS_FRONT_INVALID_LIMIT)
                {
                    lidar_obs_front_invalid_count++;
                }

                if ((lidar_obs_front_invalid_count >= LIDAR_OBS_FRONT_INVALID_LIMIT) ||
                    (front_age_ms >= LIDAR_OBS_FRONT_TIMEOUT_MS))
                {
                    LidarObsReason reason = (front_age_ms >= LIDAR_OBS_FRONT_TIMEOUT_MS)
                        ? LIDAR_OBS_REASON_LIDAR_TIMEOUT
                        : LIDAR_OBS_REASON_LIDAR_NO_VALID_FRONT;

                    APP_LOG("APP LIDAR_OBS: lidar_no_valid_front_distance invalid_count=%u front_age_ms=%lu ready=%u valid_points=%lu",
                            (unsigned int)lidar_obs_front_invalid_count,
                            (unsigned long)front_age_ms,
                            (unsigned int)front.lidar_ready,
                            (unsigned long)front.valid_points);
                    App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_WAIT_LIDAR,
                                                          reason,
                                                          now_ms);
                    break;
                }

                App_LidarObstacleAvoidance_ApplyForward();
                break;
            }

            lidar_obs_front_invalid_count = 0U;

            if (front.front_min_mm < LIDAR_OBS_STOP_MM)
            {
                APP_LOG("[OBS] hold reason=front_blocked front=%u stop=%u",
                        (unsigned int)front.front_min_mm,
                        (unsigned int)LIDAR_OBS_STOP_MM);
                APP_LOG("APP LIDAR_OBS: obstacle detected front_min_mm=%u valid_points=%lu",
                        (unsigned int)front.front_min_mm,
                        (unsigned long)front.valid_points);
                App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_OBSTACLE_STOP,
                                                      LIDAR_OBS_REASON_OBSTACLE_DETECTED,
                                                      now_ms);
                break;
            }

            App_LidarObstacleAvoidance_ApplyForward();
            break;

        case LIDAR_OBS_STATE_OBSTACLE_STOP:
            App_LidarObstacleAvoidance_ApplyStop();
            if (state_elapsed_ms >= LIDAR_OBS_RECOVER_MS)
            {
                App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_BACKUP,
                                                      LIDAR_OBS_REASON_STOP_DONE,
                                                      now_ms);
            }
            break;

        case LIDAR_OBS_STATE_BACKUP:
            if (state_elapsed_ms >= LIDAR_OBS_BACKUP_MS)
            {
                App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_TURN,
                                                      LIDAR_OBS_REASON_BACKUP_DONE,
                                                      now_ms);
                break;
            }

            App_LidarObstacleAvoidance_ApplyBackup();
            break;

        case LIDAR_OBS_STATE_TURN:
            if (state_elapsed_ms >= LIDAR_OBS_TURN_MS)
            {
                App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_RECOVER,
                                                      LIDAR_OBS_REASON_TURN_DONE,
                                                      now_ms);
                break;
            }

            App_LidarObstacleAvoidance_ApplyTurn();
            break;

        case LIDAR_OBS_STATE_RECOVER:
            App_LidarObstacleAvoidance_ApplyStop();
            if (state_elapsed_ms >= LIDAR_OBS_RECOVER_MS)
            {
                APP_LOG("APP LIDAR_OBS: recover forward");
                if (front.front_valid != 0U)
                {
                    App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_DRIVE_FORWARD,
                                                          LIDAR_OBS_REASON_RECOVER_FORWARD,
                                                          now_ms);
                }
                else
                {
                    App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_WAIT_LIDAR,
                                                          LIDAR_OBS_REASON_LIDAR_NO_VALID_FRONT,
                                                          now_ms);
                }
            }
            break;

        default:
            App_LidarObstacleAvoidance_EnterState(LIDAR_OBS_STATE_START_DELAY,
                                                  LIDAR_OBS_REASON_INIT,
                                                  now_ms);
            break;
    }
}

static void App_LidarObstacleAvoidance_EnterState(LidarObsState state,
                                                  LidarObsReason reason,
                                                  uint32_t now_ms)
{
    uint32_t elapsed_ms = App_LidarObstacleAvoidance_ElapsedMs(now_ms, lidar_obs_state_enter_ms);

    lidar_obs_state = state;
    lidar_obs_state_enter_ms = now_ms;

    switch (state)
    {
        case LIDAR_OBS_STATE_DRIVE_FORWARD:
            lidar_obs_front_invalid_count = 0U;
            App_LidarObstacleAvoidance_ApplyForward();
            break;

        case LIDAR_OBS_STATE_BACKUP:
            App_LidarObstacleAvoidance_ApplyBackup();
            break;

        case LIDAR_OBS_STATE_TURN:
            App_LidarObstacleAvoidance_ApplyTurn();
            break;

        case LIDAR_OBS_STATE_START_DELAY:
        case LIDAR_OBS_STATE_WAIT_LIDAR:
            lidar_obs_front_invalid_count = 0U;
            App_LidarObstacleAvoidance_ApplyStop();
            break;

        case LIDAR_OBS_STATE_OBSTACLE_STOP:
        case LIDAR_OBS_STATE_RECOVER:
        default:
            App_LidarObstacleAvoidance_ApplyStop();
            break;
    }

    APP_LOG("APP LIDAR_OBS: state=%s reason=%s elapsed=%lu",
            App_LidarObstacleAvoidance_StateName(state),
            App_LidarObstacleAvoidance_ReasonName(reason),
            (unsigned long)elapsed_ms);
}

static LidarObsFront App_LidarObstacleAvoidance_ReadFront(uint32_t now_ms)
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    LidarObsFront front = {0U, 0U, 0xFFFFU, 0U};

    if (lidar == NULL)
    {
        return front;
    }

    front.lidar_ready = lidar->ready;
    front.valid_points = lidar->valid_points;

    if (lidar->valid_points != lidar_obs_last_valid_points)
    {
        lidar_obs_last_valid_points = lidar->valid_points;
        lidar_obs_last_lidar_data_ms = now_ms;
    }

    if ((lidar->ready != 0U) &&
        (lidar_obs_last_lidar_data_ms != 0U) &&
        (App_LidarObstacleAvoidance_ElapsedMs(now_ms, lidar_obs_last_lidar_data_ms) < LIDAR_OBS_FRONT_TIMEOUT_MS) &&
        (lidar->front_valid != 0U) &&
        (lidar->front_min_mm >= LIDAR_OBS_FRONT_MIN_MM) &&
        (lidar->front_min_mm != 0xFFFFU))
    {
        front.front_valid = 1U;
        front.front_min_mm = lidar->front_min_mm;
        lidar_obs_last_front_valid_ms = now_ms;
    }

    return front;
}

static void App_LidarObstacleAvoidance_LogFront(const LidarObsFront *front,
                                                uint32_t now_ms)
{
    if ((front == NULL) ||
        (App_LidarObstacleAvoidance_ElapsedMs(now_ms, lidar_obs_last_log_ms) < LIDAR_OBS_LOG_INTERVAL_MS))
    {
        return;
    }

    lidar_obs_last_log_ms = now_ms;

    if (front->front_valid != 0U)
    {
        APP_LOG("APP LIDAR_OBS: front_min_mm=%u valid_points=%lu",
                (unsigned int)front->front_min_mm,
                (unsigned long)front->valid_points);
    }
}

static void App_LidarObstacleAvoidance_ApplyStop(void)
{
    Chassis_Stop();
    MotorDriver_StopAll();
}

static void App_LidarObstacleAvoidance_ApplyForward(void)
{
    Chassis_SetRaw(LIDAR_OBS_FORWARD_LEFT_DUTY, LIDAR_OBS_FORWARD_RIGHT_DUTY);
}

static void App_LidarObstacleAvoidance_ApplyBackup(void)
{
    Chassis_SetRaw((int16_t)-LIDAR_OBS_BACKUP_DUTY, (int16_t)-LIDAR_OBS_BACKUP_DUTY);
}

static void App_LidarObstacleAvoidance_ApplyTurn(void)
{
    Chassis_SetRaw(LIDAR_OBS_TURN_DUTY, (int16_t)-LIDAR_OBS_TURN_DUTY);
}

static const char *App_LidarObstacleAvoidance_StateName(LidarObsState state)
{
    switch (state)
    {
        case LIDAR_OBS_STATE_START_DELAY:
            return "START_DELAY";

        case LIDAR_OBS_STATE_WAIT_LIDAR:
            return "WAIT_LIDAR";

        case LIDAR_OBS_STATE_DRIVE_FORWARD:
            return "DRIVE_FORWARD";

        case LIDAR_OBS_STATE_OBSTACLE_STOP:
            return "OBSTACLE_STOP";

        case LIDAR_OBS_STATE_BACKUP:
            return "BACKUP";

        case LIDAR_OBS_STATE_TURN:
            return "TURN";

        case LIDAR_OBS_STATE_RECOVER:
            return "RECOVER";

        default:
            return "UNKNOWN";
    }
}

static const char *App_LidarObstacleAvoidance_ReasonName(LidarObsReason reason)
{
    switch (reason)
    {
        case LIDAR_OBS_REASON_START_DELAY_DONE:
            return "start_delay_done";

        case LIDAR_OBS_REASON_LIDAR_READY:
            return "lidar_ready";

        case LIDAR_OBS_REASON_LIDAR_NO_VALID_FRONT:
            return "lidar_no_valid_front_distance";

        case LIDAR_OBS_REASON_LIDAR_TIMEOUT:
            return "lidar_front_timeout";

        case LIDAR_OBS_REASON_OBSTACLE_DETECTED:
            return "obstacle_detected";

        case LIDAR_OBS_REASON_STOP_DONE:
            return "stop_done";

        case LIDAR_OBS_REASON_BACKUP_DONE:
            return "backup_done";

        case LIDAR_OBS_REASON_TURN_DONE:
            return "turn_done";

        case LIDAR_OBS_REASON_RECOVER_FORWARD:
            return "recover_forward";

        case LIDAR_OBS_REASON_INIT:
        default:
            return "init";
    }
}

static uint32_t App_LidarObstacleAvoidance_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    if (then_ms == 0U)
    {
        return UINT32_MAX;
    }

    if (now_ms >= then_ms)
    {
        return now_ms - then_ms;
    }

    return 0U;
}
