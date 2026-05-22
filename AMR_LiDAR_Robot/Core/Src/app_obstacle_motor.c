#include "app_obstacle_motor.h"

#include "app_lidar.h"
#include "app_obstacle.h"
#include "app_test_config.h"
#include "bringup_log.h"
#include "mpu6500.h"

#include <stdint.h>
#include <stdio.h>

#if APP_OBSTACLE_MOTOR_ENABLE
#include "chassis.h"
#endif

#define APP_OBS_MOTOR_COMMAND_INTERVAL_MS 250U
#define APP_OBS_MOTOR_LOG_INTERVAL_MS 1000U
#define APP_IMU_HEADING_TEST_LOG_INTERVAL_MS 250U
#define APP_OBS_MOTOR_GROUND_START_DELAY_MS 3000U
#define APP_OBS_MOTOR_GROUND_MAX_RUN_MS 10000U
#define APP_OBS_MOTOR_INVALID_DISTANCE_MM 0xFFFFU
#define APP_OBS_GROUND_FORWARD_HOLD_MS 300U
#define APP_OBS_GROUND_TURN_HOLD_MS APP_GROUND_TURN_MIN_HOLD_MS
#define APP_OBS_GROUND_FRONT_CLEAR_MM 750U
#define APP_OBS_GROUND_FRONT_RESUME_MM 650U
#define APP_OBS_GROUND_FRONT_BLOCK_MM 550U
#define APP_OBS_GROUND_FRONT_MID_HOLD_MAX_MS 300U
#define APP_OBS_GROUND_EMERGENCY_STOP_MM 320U
#define APP_OBS_GROUND_CONTACT_STOP_MM 180U
#define APP_OBS_GROUND_SIDE_CLEAR_MM 350U
#define APP_OBS_GROUND_SIDE_BLOCK_MM 220U
#define APP_OBS_GROUND_HARD_STOP_MM 180U
#define APP_OBS_GROUND_APPROACHING_TURN_MM 800U
#define APP_OBS_GROUND_TURN_SWITCH_HYST_MM APP_GROUND_TURN_SWITCH_MARGIN_MM
#define APP_OBS_GROUND_SIDE_OPEN_SCORE_MM 12000U
#define APP_OBS_GROUND_FRONT_HISTORY_MASK 0x07U
#define APP_OBS_GROUND_FRONT_CONFIRM_COUNT 2U
#define APP_OBS_GROUND_FRONT_HISTORY_SIZE 3U
#define APP_OBS_GROUND_OBS_FRONT_HISTORY_SIZE 3U
#define APP_OBS_GROUND_BACKUP_HISTORY_SIZE 3U
#define APP_OBS_GROUND_BACKUP_WINDOW_MS 5000U
#define APP_OBS_GROUND_FRONT_NEAR_MS 3000U

typedef enum
{
    APP_OBS_MOTOR_ACTION_STOP = 0,
    APP_OBS_MOTOR_ACTION_FORWARD_SLOW,
    APP_OBS_MOTOR_ACTION_FORWARD_CAUTION,
    APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW,
    APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW,
    APP_OBS_MOTOR_ACTION_BACKUP_SLOW,
    APP_OBS_MOTOR_ACTION_CORNER_BACKUP
} AppObstacleMotorAction;

typedef enum
{
    APP_OBS_GROUND_STOP = 0,
    APP_OBS_GROUND_FORWARD,
    APP_OBS_GROUND_CAUTION,
    APP_OBS_GROUND_TURN_LEFT,
    APP_OBS_GROUND_TURN_RIGHT,
    APP_OBS_GROUND_BACKUP,
    APP_OBS_GROUND_CORNER_BACKUP,
    APP_OBS_GROUND_CORNER_TURN,
    APP_OBS_GROUND_BLOCKED
} AppObstacleGroundState;

typedef enum
{
    APP_OBS_GROUND_REASON_INIT = 0,
    APP_OBS_GROUND_REASON_START_DELAY,
    APP_OBS_GROUND_REASON_GROUND_TIMEOUT,
    APP_OBS_GROUND_REASON_LIDAR_NOT_READY,
    APP_OBS_GROUND_REASON_INVALID_DECISION,
    APP_OBS_GROUND_REASON_FRONT_CLEAR_FORWARD,
    APP_OBS_GROUND_REASON_CLEAR_FORWARD_RESUME,
    APP_OBS_GROUND_REASON_FRONT_CONFIRMED_CLEAR,
    APP_OBS_GROUND_REASON_TURN_RESUME_FORWARD,
    APP_OBS_GROUND_REASON_DECISION_TURN_LEFT,
    APP_OBS_GROUND_REASON_DECISION_TURN_RIGHT,
    APP_OBS_GROUND_REASON_FRONT_BLOCK_TURN,
    APP_OBS_GROUND_REASON_FRONT_CONFIRMED_BLOCK,
    APP_OBS_GROUND_REASON_FRONT_MID_HOLD,
    APP_OBS_GROUND_REASON_FRONT_MID_HOLD_EXPIRED_TURN,
    APP_OBS_GROUND_REASON_FRONT_MID_TURN,
    APP_OBS_GROUND_REASON_FRONT_CAUTION_FORWARD,
    APP_OBS_GROUND_REASON_FRONT_APPROACHING_TURN,
    APP_OBS_GROUND_REASON_CONTACT_BACKUP,
    APP_OBS_GROUND_REASON_BACKUP_COMPLETE_TURN_LEFT,
    APP_OBS_GROUND_REASON_BACKUP_COMPLETE_TURN_RIGHT,
    APP_OBS_GROUND_REASON_BACKUP_COMPLETE_BLOCKED,
    APP_OBS_GROUND_REASON_ESCAPE_TURN_HOLD,
    APP_OBS_GROUND_REASON_BACKUP_COOLDOWN_TURN,
    APP_OBS_GROUND_REASON_EMERGENCY_FRONT_TURN,
    APP_OBS_GROUND_REASON_HARD_STOP_TURN,
    APP_OBS_GROUND_REASON_BLOCKED_ALL_SIDES,
    APP_OBS_GROUND_REASON_STOP_BLOCKED_PRIORITY,
    APP_OBS_GROUND_REASON_TURN_HOLD_KEEP_LEFT,
    APP_OBS_GROUND_REASON_TURN_HOLD_KEEP_RIGHT,
    APP_OBS_GROUND_REASON_TURN_SWITCH_MARGIN_KEEP_LEFT,
    APP_OBS_GROUND_REASON_TURN_SWITCH_MARGIN_KEEP_RIGHT,
    APP_OBS_GROUND_REASON_TURN_SWITCH_ALLOWED_LEFT,
    APP_OBS_GROUND_REASON_TURN_SWITCH_ALLOWED_RIGHT,
    APP_OBS_GROUND_REASON_ESCAPE_LOCK_HOLD_LEFT,
    APP_OBS_GROUND_REASON_ESCAPE_LOCK_HOLD_RIGHT,
    APP_OBS_GROUND_REASON_ESCAPE_LOCK_CLEAR_FORWARD_EXIT,
    APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_RESELECT_LEFT,
    APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_RESELECT_RIGHT,
    APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_MARGIN_SWITCH_LEFT,
    APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_MARGIN_SWITCH_RIGHT,
    APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_KEEP_LEFT,
    APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_KEEP_RIGHT,
    APP_OBS_GROUND_REASON_CORNER_ESCAPE_ENTER,
    APP_OBS_GROUND_REASON_CORNER_BACKUP,
    APP_OBS_GROUND_REASON_CORNER_TURN_LEFT,
    APP_OBS_GROUND_REASON_CORNER_TURN_RIGHT,
    APP_OBS_GROUND_REASON_CORNER_ESCAPE_EXIT,
    APP_OBS_GROUND_REASON_CORNER_COOLDOWN,
    APP_OBS_GROUND_REASON_TURN_HOLD_MIN_KEEP_LEFT,
    APP_OBS_GROUND_REASON_TURN_HOLD_MIN_KEEP_RIGHT,
    APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT,
    APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT,
    APP_OBS_GROUND_REASON_TURN_HOLD_MIN,
    APP_OBS_GROUND_REASON_MIN_HOLD
} AppObstacleGroundReason;

typedef struct
{
    int16_t left_duty;
    int16_t right_duty;
} AppObstacleMotorCommand;

typedef struct
{
    uint8_t valid;
    uint16_t distance_mm;
    const char *source;
} AppObstacleFrontDistance;

typedef struct
{
    uint8_t ready;
    uint8_t active;
    uint8_t target_valid;
    uint8_t apply_active;
    float yaw_deg;
    float target_deg;
    float current_deg;
    float error_deg;
    int16_t correction;
    uint32_t last_imu_update_ms;
} AppImuHeadingAssistState;

static AppObstacleGroundState app_ground_state = APP_OBS_GROUND_STOP;
static uint32_t app_ground_state_enter_ms = 0U;
static uint32_t app_forward_enter_ms = 0U;
static uint32_t app_backup_complete_ms = 0U;
static AppObstacleGroundState app_escape_turn_state = APP_OBS_GROUND_TURN_RIGHT;
static uint8_t app_escape_lock_active = 0U;
static AppObstacleGroundState app_escape_lock_state = APP_OBS_GROUND_STOP;
static uint32_t app_escape_lock_start_ms = 0U;
static AppObstacleGroundState app_last_turn_state = APP_OBS_GROUND_TURN_RIGHT;
static uint8_t app_corner_escape_active = 0U;
static uint32_t app_corner_escape_start_ms = 0U;
static uint32_t app_corner_cooldown_start_ms = 0U;
static AppObstacleGroundState app_corner_turn_state = APP_OBS_GROUND_TURN_RIGHT;
static uint32_t app_recovery_start_ms = 0U;
static uint32_t app_front_near_start_ms = 0U;
static uint32_t app_backup_event_ms[APP_OBS_GROUND_BACKUP_HISTORY_SIZE];
static uint8_t app_backup_event_count = 0U;
static uint8_t app_front_block_history = 0U;
static uint8_t app_front_clear_history = 0U;
static uint8_t app_front_history_count = 0U;
static uint16_t app_obs_front_history[APP_OBS_GROUND_OBS_FRONT_HISTORY_SIZE];
static uint8_t app_obs_front_history_count = 0U;
static AppImuHeadingAssistState app_imu_heading = {0};

static uint8_t App_ObstacleMotor_OutputEnabled(void);
static int16_t App_ObstacleMotor_ConfiguredSpeed(void);
#if APP_IMU_HEADING_ASSIST_DRY_RUN_ENABLE
static void App_ObstacleMotor_RunHeadingAssistDryRun(uint32_t now_ms);
#endif
static void App_ObstacleMotor_ResetHeadingAssist(void);
static uint8_t App_ObstacleMotor_ActionAllowsHeadingAssist(AppObstacleMotorAction action);
static uint8_t App_ObstacleMotor_UpdateHeadingYaw(void);
static int16_t App_ObstacleMotor_UpdateHeadingAssist(AppObstacleGroundState ground_state,
                                                     AppObstacleMotorAction action,
                                                     AppObstacleMotorCommand *command);
static int32_t App_ObstacleMotor_HeadingKp(void);
static int32_t App_ObstacleMotor_HeadingCorrectionMax(void);
static int32_t App_ObstacleMotor_HeadingDeadbandDeg(void);
static float App_ObstacleMotor_NormalizeAngleDeg(float angle_deg);
static void App_ObstacleMotor_LogHeadingAssist(void);
#if APP_IMU_HEADING_ASSIST_DRY_RUN_ENABLE
static void App_ObstacleMotor_LogHeadingAssistTest(int16_t heading_correction);
#endif
static const char *App_ObstacleMotor_FormatHeadingDeg(uint8_t valid,
                                                       float value_deg,
                                                       char *buffer,
                                                       uint32_t buffer_size);
static int32_t App_ObstacleMotor_ScaleFloatRounded(float value, float multiplier);
static const char *App_ObstacleMotor_FixedSign(int32_t value);
static uint32_t App_ObstacleMotor_FixedWhole(int32_t value, int32_t decimal_scale);
static uint32_t App_ObstacleMotor_FixedFraction(int32_t value, int32_t decimal_scale);
static void App_ObstacleMotor_ResetFrontHistory(void);
static AppObstacleFrontDistance App_ObstacleMotor_SelectObstacleFront(const AppLidarStatus *lidar);
static void App_ObstacleMotor_UpdateFrontHistory(const AppObstacleFrontDistance *obs_front);
static void App_ObstacleMotor_UpdateObsFrontTrend(const AppObstacleFrontDistance *obs_front);
static uint8_t App_ObstacleMotor_FrontConfirmedBlocked(void);
static uint8_t App_ObstacleMotor_FrontConfirmedClear(void);
static uint8_t App_ObstacleMotor_FrontApproaching(void);
static uint8_t App_ObstacleMotor_CountHistoryBits(uint8_t value);
static uint8_t App_ObstacleMotor_StatusAllowsMotion(const AppLidarStatus *lidar);
static uint8_t App_ObstacleMotor_IsTurnState(AppObstacleGroundState state);
static AppObstacleGroundReason App_ObstacleMotor_EscapeHoldReason(AppObstacleGroundState state);
static AppObstacleGroundReason App_ObstacleMotor_CornerTurnReason(AppObstacleGroundState state);
static void App_ObstacleMotor_SetEscapeLock(AppObstacleGroundState state,
                                            uint32_t now_ms,
                                            AppObstacleGroundReason reason);
static void App_ObstacleMotor_ClearEscapeLock(const char *exit_reason);
static uint8_t App_ObstacleMotor_EscapeLockShouldExit(const AppObstacleFrontDistance *obs_front,
                                                      uint32_t now_ms);
static AppObstacleGroundState App_ObstacleMotor_SelectEscapeTurnState(const AppLidarStatus *lidar,
                                                                      AppObstacleDecision decision,
                                                                      AppObstacleGroundReason *reason);
static void App_ObstacleMotor_ResetCornerTracking(void);
static void App_ObstacleMotor_UpdateCornerTracking(const AppObstacleFrontDistance *obs_front,
                                                   uint32_t now_ms);
static void App_ObstacleMotor_RecordBackupEvent(uint32_t now_ms);
static uint8_t App_ObstacleMotor_BackupCount5s(uint32_t now_ms);
static uint8_t App_ObstacleMotor_CornerCooldownActive(uint32_t now_ms);
static uint8_t App_ObstacleMotor_CornerShouldEnter(uint32_t now_ms);
static AppObstacleGroundState App_ObstacleMotor_SelectCornerTurnState(const AppLidarStatus *lidar);
static void App_ObstacleMotor_StartCornerEscape(const AppLidarStatus *lidar, uint32_t now_ms);
static void App_ObstacleMotor_ClearCornerEscape(const char *exit_reason, uint32_t now_ms);
static AppObstacleGroundState App_ObstacleMotor_UpdateGroundState(const AppLidarStatus *lidar,
                                                                  const AppObstacleFrontDistance *obs_front,
                                                                  AppObstacleDecision decision,
                                                                  uint32_t now_ms,
                                                                  uint32_t *hold_ms,
                                                                  AppObstacleGroundReason *reason);
static AppObstacleGroundState App_ObstacleMotor_EvaluateGroundState(const AppLidarStatus *lidar,
                                                                    const AppObstacleFrontDistance *obs_front,
                                                                    AppObstacleDecision decision,
                                                                    uint32_t now_ms,
                                                                    AppObstacleGroundReason *reason);
static AppObstacleGroundState App_ObstacleMotor_SelectTurnState(const AppLidarStatus *lidar,
                                                                AppObstacleDecision decision,
                                                                AppObstacleGroundReason *reason);
static uint8_t App_ObstacleMotor_TurnSwitchAllowed(const AppLidarStatus *lidar,
                                                   AppObstacleGroundState current_state,
                                                   AppObstacleGroundState desired_state,
                                                   AppObstacleGroundReason *reason);
static AppObstacleGroundState App_ObstacleMotor_SelectBackupTurnState(const AppLidarStatus *lidar,
                                                                      AppObstacleGroundReason *reason);
static void App_ObstacleMotor_ForceGroundState(AppObstacleGroundState state,
                                               uint32_t now_ms,
                                               uint32_t *hold_ms);
static AppObstacleMotorAction App_ObstacleMotor_ActionFromGroundState(AppObstacleGroundState state);
static int16_t App_ObstacleMotor_ActionSpeed(AppObstacleMotorAction action, int16_t configured_speed);
static AppObstacleMotorCommand App_ObstacleMotor_CommandFromAction(AppObstacleMotorAction action,
                                                                    int16_t speed,
                                                                    uint8_t start_boost);
static const char *App_ObstacleMotor_ActionName(AppObstacleMotorAction action);
static const char *App_ObstacleMotor_GroundStateName(AppObstacleGroundState state);
static const char *App_ObstacleMotor_GroundReasonName(AppObstacleGroundReason reason);
static const char *App_ObstacleMotor_DecisionName(AppObstacleDecision decision);
static const char *App_ObstacleMotor_LogSuffix(void);
static const char *App_ObstacleMotor_FormatDistance(uint8_t valid,
                                                     uint16_t distance_mm,
                                                     char *buffer,
                                                     uint32_t buffer_size);
static const char *App_ObstacleMotor_FrontSourceName(const AppObstacleFrontDistance *obs_front);
static const char *App_ObstacleMotor_EscapeDirName(AppObstacleGroundState state);
static const char *App_ObstacleMotor_CornerPhaseName(AppObstacleGroundState state);
static uint32_t App_ObstacleMotor_StateHoldMs(AppObstacleGroundState state);
static uint32_t App_ObstacleMotor_SideScore(uint8_t valid, uint16_t distance_mm);
static uint8_t App_ObstacleMotor_SideIsClear(uint8_t valid, uint16_t distance_mm);
static uint8_t App_ObstacleMotor_SideIsBlocked(uint8_t valid, uint16_t distance_mm);
static int16_t App_ObstacleMotor_ClampDuty(int32_t duty);
static void App_ObstacleMotor_ApplyAction(AppObstacleMotorAction action,
                                           const AppObstacleMotorCommand *command);

void App_ObstacleMotor_Init(void)
{
    app_ground_state = APP_OBS_GROUND_STOP;
    app_ground_state_enter_ms = HAL_GetTick();
    app_forward_enter_ms = 0U;
    app_backup_complete_ms = 0U;
    app_escape_turn_state = APP_OBS_GROUND_TURN_RIGHT;
    app_escape_lock_active = 0U;
    app_escape_lock_state = APP_OBS_GROUND_STOP;
    app_escape_lock_start_ms = 0U;
    app_last_turn_state = APP_OBS_GROUND_TURN_RIGHT;
    app_corner_escape_active = 0U;
    app_corner_escape_start_ms = 0U;
    app_corner_cooldown_start_ms = 0U;
    app_corner_turn_state = APP_OBS_GROUND_TURN_RIGHT;
    app_recovery_start_ms = 0U;
    app_front_near_start_ms = 0U;
    App_ObstacleMotor_ResetHeadingAssist();
    App_ObstacleMotor_ResetCornerTracking();
    App_ObstacleMotor_ResetFrontHistory();

#if APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_OUTPUT_ENABLE
    Chassis_Init();
    Chassis_Stop();

    APP_LOG("APP OBS MOTOR: WARNING lifted-wheel-test output ENABLED, ground=0, keep wheels lifted");
#elif APP_OBSTACLE_MOTOR_ENABLE && APP_OBSTACLE_GROUND_TEST_ENABLE
    Chassis_Init();
    Chassis_Stop();

    APP_LOG("APP OBS MOTOR: WARNING motor output ENABLED, lift wheels before test");
#elif APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ACTIVE
    APP_LOG("APP OBS MOTOR: lifted-wheel-test selected, apply=0, motor output disabled, ground=0");
#elif APP_OBSTACLE_MOTOR_ENABLE
    APP_LOG("APP OBS MOTOR: motor output disabled, ground-test disabled dry-run only");
#else
    APP_LOG("APP OBS MOTOR: motor output disabled, dry-run only");
#endif
}

void App_ObstacleMotor_Task(void)
{
    static uint32_t last_command_ms = 0U;
    static uint32_t last_log_ms = 0U;
    static uint32_t ground_run_start_ms = 0U;
    static AppObstacleMotorAction last_applied_action = APP_OBS_MOTOR_ACTION_STOP;
    static uint8_t last_applied_start_boost = 0U;
    static AppObstacleMotorAction last_logged_action = APP_OBS_MOTOR_ACTION_STOP;
    static AppObstacleGroundReason last_logged_ground_reason = APP_OBS_GROUND_REASON_INIT;
    static uint8_t last_logged_start_boost = 0xFFU;
    static uint8_t last_logged_heading_ready = 0xFFU;
    uint32_t now_ms = HAL_GetTick();
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    AppObstacleDecision decision = App_Obstacle_GetDecision();
    uint8_t output_enabled = App_ObstacleMotor_OutputEnabled();
    uint8_t waiting_start_delay = 0U;
    uint8_t ground_timeout = 0U;
    uint8_t command_due = 0U;
    uint8_t log_due = 0U;
    uint32_t ground_hold_ms = 0U;
    uint32_t forward_elapsed_ms = 0U;
    uint32_t backup_cooldown_ms = 0U;
    uint32_t escape_elapsed_ms = 0U;
    uint32_t corner_elapsed_ms = 0U;
    uint8_t backup_count_5s = 0U;
    int16_t heading_correction = 0;
    AppObstacleGroundReason ground_reason = APP_OBS_GROUND_REASON_INIT;
    int16_t configured_speed = App_ObstacleMotor_ConfiguredSpeed();
    char front_text[16];
    char front_wide_text[16];
    char obs_front_text[16];
    char left_text[16];
    char right_text[16];

#if APP_IMU_HEADING_ASSIST_DRY_RUN_ENABLE
    (void)last_command_ms;
    (void)ground_run_start_ms;
    (void)last_applied_action;
    (void)last_applied_start_boost;
    (void)last_logged_action;
    (void)last_logged_ground_reason;
    (void)last_logged_start_boost;
    (void)last_logged_heading_ready;
    (void)lidar;
    (void)decision;
    (void)output_enabled;
    (void)waiting_start_delay;
    (void)ground_timeout;
    (void)command_due;
    (void)log_due;
    (void)ground_hold_ms;
    (void)forward_elapsed_ms;
    (void)backup_cooldown_ms;
    (void)escape_elapsed_ms;
    (void)corner_elapsed_ms;
    (void)backup_count_5s;
    (void)heading_correction;
    (void)ground_reason;
    (void)configured_speed;
    (void)front_text;
    (void)front_wide_text;
    (void)obs_front_text;
    (void)left_text;
    (void)right_text;
    App_ObstacleMotor_RunHeadingAssistDryRun(now_ms);
    return;
#endif

    AppObstacleFrontDistance obs_front = App_ObstacleMotor_SelectObstacleFront(lidar);
    App_ObstacleMotor_UpdateFrontHistory(&obs_front);
    App_ObstacleMotor_UpdateObsFrontTrend(&obs_front);

    if (App_ObstacleMotor_StatusAllowsMotion(lidar) == 0U)
    {
        decision = APP_OBS_DECISION_NO_LIDAR;
    }

    AppObstacleGroundState ground_state = App_ObstacleMotor_UpdateGroundState(lidar,
                                                                              &obs_front,
                                                                              decision,
                                                                              now_ms,
                                                                              &ground_hold_ms,
                                                                              &ground_reason);

    if (output_enabled != 0U)
    {
        if (now_ms < APP_OBS_MOTOR_GROUND_START_DELAY_MS)
        {
            waiting_start_delay = 1U;
            App_ObstacleMotor_ForceGroundState(APP_OBS_GROUND_STOP, now_ms, &ground_hold_ms);
            ground_state = APP_OBS_GROUND_STOP;
            ground_reason = APP_OBS_GROUND_REASON_START_DELAY;
        }
        else if (decision == APP_OBS_DECISION_NO_LIDAR)
        {
            ground_state = APP_OBS_GROUND_STOP;
            ground_reason = APP_OBS_GROUND_REASON_LIDAR_NOT_READY;
        }
        else
        {
            if (ground_run_start_ms == 0U)
            {
                ground_run_start_ms = now_ms;
            }

            if ((now_ms - ground_run_start_ms) >= APP_OBS_MOTOR_GROUND_MAX_RUN_MS)
            {
                ground_timeout = 1U;
                App_ObstacleMotor_ForceGroundState(APP_OBS_GROUND_STOP, now_ms, &ground_hold_ms);
                ground_state = APP_OBS_GROUND_STOP;
                ground_reason = APP_OBS_GROUND_REASON_GROUND_TIMEOUT;
            }
        }
    }

    AppObstacleMotorAction action = App_ObstacleMotor_ActionFromGroundState(ground_state);
    int16_t action_speed = App_ObstacleMotor_ActionSpeed(action, configured_speed);
    if (app_backup_complete_ms != 0U)
    {
        uint32_t backup_elapsed_ms = now_ms - app_backup_complete_ms;

        if (backup_elapsed_ms < APP_GROUND_BACKUP_COOLDOWN_MS)
        {
            backup_cooldown_ms = backup_elapsed_ms;
        }

    }

    if (app_escape_lock_active != 0U)
    {
        escape_elapsed_ms = now_ms - app_escape_lock_start_ms;
    }

    if (app_corner_escape_active != 0U)
    {
        corner_elapsed_ms = now_ms - app_corner_escape_start_ms;
    }

    backup_count_5s = App_ObstacleMotor_BackupCount5s(now_ms);

    if ((action == APP_OBS_MOTOR_ACTION_FORWARD_SLOW) &&
        (ground_state == APP_OBS_GROUND_FORWARD))
    {
        forward_elapsed_ms = (app_forward_enter_ms != 0U) ?
                             (now_ms - app_forward_enter_ms) :
                             ground_hold_ms;
    }

    uint8_t forward_start_boost = ((action == APP_OBS_MOTOR_ACTION_FORWARD_SLOW) &&
                                   (ground_state == APP_OBS_GROUND_FORWARD) &&
                                   (forward_elapsed_ms < APP_GROUND_FORWARD_START_BOOST_MS)) ? 1U : 0U;
    AppObstacleMotorCommand command = App_ObstacleMotor_CommandFromAction(action,
                                                                          action_speed,
                                                                          forward_start_boost);
    heading_correction = App_ObstacleMotor_UpdateHeadingAssist(ground_state, action, &command);

    if ((now_ms - last_command_ms) >= APP_OBS_MOTOR_COMMAND_INTERVAL_MS)
    {
        command_due = 1U;
    }

    if ((action == APP_OBS_MOTOR_ACTION_STOP) && (last_applied_action != APP_OBS_MOTOR_ACTION_STOP))
    {
        command_due = 1U;
    }

    if ((action != last_applied_action) ||
        (forward_start_boost != last_applied_start_boost))
    {
        command_due = 1U;
    }

    if (command_due != 0U)
    {
        App_ObstacleMotor_ApplyAction(action, &command);
        last_command_ms = now_ms;
        last_applied_action = action;
        last_applied_start_boost = forward_start_boost;
    }

    if (((now_ms - last_log_ms) >= APP_OBS_MOTOR_LOG_INTERVAL_MS) ||
        (action != last_logged_action) ||
        (ground_reason != last_logged_ground_reason) ||
        (forward_start_boost != last_logged_start_boost) ||
        (app_imu_heading.ready != last_logged_heading_ready))
    {
        log_due = 1U;
    }

    if (log_due == 0U)
    {
        return;
    }

    last_log_ms = now_ms;
    last_logged_action = action;
    last_logged_ground_reason = ground_reason;
    last_logged_start_boost = forward_start_boost;
    last_logged_heading_ready = app_imu_heading.ready;

    if ((output_enabled != 0U) && (waiting_start_delay != 0U))
    {
        APP_LOG("APP OBS MOTOR: ground test waiting start_delay");
    }

    if ((output_enabled != 0U) && (ground_timeout != 0U))
    {
        APP_LOG("APP OBS MOTOR: ground test timeout, force STOP");
    }

    APP_LOG("APP OBS GROUND: state=%s reason=%s decision=%s escape_lock=%u escape_dir=%s corner_escape=%u corner_phase=%s front=%s front_wide=%s obs_front=%s source=%s left=%s right=%s hold_ms=%lu backup_cooldown_ms=%lu escape_elapsed_ms=%lu corner_elapsed_ms=%lu backup_count_5s=%u",
            App_ObstacleMotor_GroundStateName(ground_state),
            App_ObstacleMotor_GroundReasonName(ground_reason),
            App_ObstacleMotor_DecisionName(decision),
            (unsigned int)app_escape_lock_active,
            App_ObstacleMotor_EscapeDirName(app_escape_lock_state),
            (unsigned int)app_corner_escape_active,
            App_ObstacleMotor_CornerPhaseName(ground_state),
            App_ObstacleMotor_FormatDistance(((lidar != NULL) && (lidar->ready != 0U)) ? lidar->front_valid : 0U,
                                             (lidar != NULL) ? lidar->front_min_mm : 0U,
                                             front_text,
                                             sizeof(front_text)),
            App_ObstacleMotor_FormatDistance(((lidar != NULL) && (lidar->ready != 0U)) ? lidar->front_wide_valid : 0U,
                                             (lidar != NULL) ? lidar->front_wide_min_mm : 0U,
                                             front_wide_text,
                                             sizeof(front_wide_text)),
            App_ObstacleMotor_FormatDistance(obs_front.valid,
                                             obs_front.distance_mm,
                                             obs_front_text,
                                             sizeof(obs_front_text)),
            App_ObstacleMotor_FrontSourceName(&obs_front),
            App_ObstacleMotor_FormatDistance(((lidar != NULL) && (lidar->ready != 0U)) ? lidar->left_valid : 0U,
                                             (lidar != NULL) ? lidar->left_min_mm : 0U,
                                             left_text,
                                             sizeof(left_text)),
            App_ObstacleMotor_FormatDistance(((lidar != NULL) && (lidar->ready != 0U)) ? lidar->right_valid : 0U,
                                             (lidar != NULL) ? lidar->right_min_mm : 0U,
                                             right_text,
                                             sizeof(right_text)),
            (unsigned long)ground_hold_ms,
            (unsigned long)backup_cooldown_ms,
            (unsigned long)escape_elapsed_ms,
            (unsigned long)corner_elapsed_ms,
            (unsigned int)backup_count_5s);

    App_ObstacleMotor_LogHeadingAssist();

    APP_LOG("APP OBS MOTOR: enabled=%u ground=%u speed=%d trimL=%d trimR=%d action=%s start_boost=%u forward_elapsed_ms=%lu heading_corr=%d heading_apply=%u left=%d right=%d%s",
            (unsigned int)APP_OBSTACLE_MOTOR_ENABLE,
            (unsigned int)APP_OBSTACLE_GROUND_TEST_ENABLE,
            (int)action_speed,
            (int)APP_GROUND_LEFT_TRIM,
            (int)APP_GROUND_RIGHT_TRIM,
            App_ObstacleMotor_ActionName(action),
            (unsigned int)forward_start_boost,
            (unsigned long)forward_elapsed_ms,
            (int)heading_correction,
            (unsigned int)app_imu_heading.apply_active,
            (int)command.left_duty,
            (int)command.right_duty,
            App_ObstacleMotor_LogSuffix());
}

static uint8_t App_ObstacleMotor_OutputEnabled(void)
{
#if APP_OBSTACLE_MOTOR_ENABLE
    return 1U;
#else
    return 0U;
#endif
}

static int16_t App_ObstacleMotor_ConfiguredSpeed(void)
{
    return APP_OBSTACLE_GROUND_TEST_SPEED;
}

#if APP_IMU_HEADING_ASSIST_DRY_RUN_ENABLE
static void App_ObstacleMotor_RunHeadingAssistDryRun(uint32_t now_ms)
{
    static uint32_t last_command_ms = 0U;
    static uint32_t last_log_ms = 0U;
    uint32_t hold_ms = 0U;
    AppObstacleGroundState ground_state = APP_OBS_GROUND_FORWARD;
    AppObstacleMotorAction action = APP_OBS_MOTOR_ACTION_FORWARD_SLOW;
    int16_t action_speed = App_ObstacleMotor_ActionSpeed(action, App_ObstacleMotor_ConfiguredSpeed());
    AppObstacleMotorCommand command = App_ObstacleMotor_CommandFromAction(action, action_speed, 0U);
    AppObstacleMotorCommand *apply_command = NULL;
    int16_t heading_correction = 0;
    uint8_t output_enabled = App_ObstacleMotor_OutputEnabled();

    App_ObstacleMotor_ClearEscapeLock("imu_heading_test");
    App_ObstacleMotor_ClearCornerEscape("imu_heading_test", now_ms);
    App_ObstacleMotor_ForceGroundState(APP_OBS_GROUND_FORWARD, now_ms, &hold_ms);
    app_recovery_start_ms = 0U;
    app_front_near_start_ms = 0U;

#if APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_OUTPUT_ENABLE
    apply_command = &command;
#endif

    heading_correction = App_ObstacleMotor_UpdateHeadingAssist(ground_state, action, apply_command);

    if ((output_enabled != 0U) &&
        ((now_ms - last_command_ms) >= APP_OBS_MOTOR_COMMAND_INTERVAL_MS))
    {
        App_ObstacleMotor_ApplyAction(action, &command);
        last_command_ms = now_ms;
    }

    if ((now_ms - last_log_ms) < APP_IMU_HEADING_TEST_LOG_INTERVAL_MS)
    {
        return;
    }

    last_log_ms = now_ms;

    App_ObstacleMotor_LogHeadingAssistTest(heading_correction);
    APP_LOG("APP OBS MOTOR: enabled=%u ground=0 action=%s heading_corr=%d heading_apply=%u lifted_wheel=%u left=%d right=%d speed=%d trimL=%d trimR=%d start_boost=0 forward_elapsed_ms=%lu%s",
            (unsigned int)output_enabled,
            App_ObstacleMotor_ActionName(action),
            (int)heading_correction,
            (unsigned int)app_imu_heading.apply_active,
            (unsigned int)APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE,
            (int)command.left_duty,
            (int)command.right_duty,
            (int)action_speed,
            (int)APP_GROUND_LEFT_TRIM,
            (int)APP_GROUND_RIGHT_TRIM,
            (unsigned long)hold_ms,
            App_ObstacleMotor_LogSuffix());
}
#endif

static void App_ObstacleMotor_ResetHeadingAssist(void)
{
    app_imu_heading.ready = 0U;
    app_imu_heading.active = 0U;
    app_imu_heading.target_valid = 0U;
    app_imu_heading.apply_active = 0U;
    app_imu_heading.yaw_deg = 0.0f;
    app_imu_heading.target_deg = 0.0f;
    app_imu_heading.current_deg = 0.0f;
    app_imu_heading.error_deg = 0.0f;
    app_imu_heading.correction = 0;
    app_imu_heading.last_imu_update_ms = 0U;
}

static uint8_t App_ObstacleMotor_ActionAllowsHeadingAssist(AppObstacleMotorAction action)
{
    return ((action == APP_OBS_MOTOR_ACTION_FORWARD_SLOW) ||
            (action == APP_OBS_MOTOR_ACTION_FORWARD_CAUTION)) ? 1U : 0U;
}

static uint8_t App_ObstacleMotor_UpdateHeadingYaw(void)
{
    MPU6500_Data_t imu;

    if (MPU6500_GetLatest(&imu) == false)
    {
        app_imu_heading.ready = 0U;
        return 0U;
    }

    if (imu.is_ready == 0U)
    {
        app_imu_heading.ready = 0U;
        return 0U;
    }

    app_imu_heading.ready = 1U;

    if (app_imu_heading.last_imu_update_ms == 0U)
    {
        app_imu_heading.last_imu_update_ms = imu.last_update_ms;
        app_imu_heading.current_deg = app_imu_heading.yaw_deg;
        return 1U;
    }

    if (imu.last_update_ms != app_imu_heading.last_imu_update_ms)
    {
        uint32_t dt_ms = imu.last_update_ms - app_imu_heading.last_imu_update_ms;

        app_imu_heading.last_imu_update_ms = imu.last_update_ms;

        if ((dt_ms != 0U) && (dt_ms <= 500U))
        {
            app_imu_heading.yaw_deg += imu.gz_dps * ((float)dt_ms / 1000.0f);
            app_imu_heading.yaw_deg = App_ObstacleMotor_NormalizeAngleDeg(app_imu_heading.yaw_deg);
        }
    }

    app_imu_heading.current_deg = app_imu_heading.yaw_deg;
    return 1U;
}

static int16_t App_ObstacleMotor_UpdateHeadingAssist(AppObstacleGroundState ground_state,
                                                     AppObstacleMotorAction action,
                                                     AppObstacleMotorCommand *command)
{
    uint8_t assist_allowed = App_ObstacleMotor_ActionAllowsHeadingAssist(action);

#if APP_IMU_HEADING_ASSIST_ENABLE
    uint8_t imu_ready = App_ObstacleMotor_UpdateHeadingYaw();
    int32_t heading_kp = App_ObstacleMotor_HeadingKp();
    int32_t heading_correction_max = App_ObstacleMotor_HeadingCorrectionMax();
    int32_t heading_deadband_deg = App_ObstacleMotor_HeadingDeadbandDeg();
    int32_t correction = 0;

    app_imu_heading.apply_active = 0U;

    if (assist_allowed == 0U)
    {
        if (app_imu_heading.active != 0U)
        {
            APP_LOG("APP IMU HEADING: paused state=%s",
                    App_ObstacleMotor_GroundStateName(ground_state));
        }

        app_imu_heading.active = 0U;
        app_imu_heading.target_valid = 0U;
        app_imu_heading.error_deg = 0.0f;
        app_imu_heading.correction = 0;
        return 0;
    }

    app_imu_heading.active = 1U;

    if (imu_ready == 0U)
    {
        app_imu_heading.error_deg = 0.0f;
        app_imu_heading.correction = 0;
        return 0;
    }

    if (app_imu_heading.target_valid == 0U)
    {
        char target_text[20];

        app_imu_heading.target_deg = app_imu_heading.current_deg;
        app_imu_heading.target_valid = 1U;
        APP_LOG("APP IMU HEADING: target captured target=%s",
                App_ObstacleMotor_FormatHeadingDeg(1U,
                                                   app_imu_heading.target_deg,
                                                   target_text,
                                                   sizeof(target_text)));
    }

    app_imu_heading.error_deg = App_ObstacleMotor_NormalizeAngleDeg(app_imu_heading.current_deg -
                                                                    app_imu_heading.target_deg);

    if (((app_imu_heading.error_deg >= 0.0f) ? app_imu_heading.error_deg : -app_imu_heading.error_deg) <
        (float)heading_deadband_deg)
    {
        correction = 0;
    }
    else
    {
        correction = App_ObstacleMotor_ScaleFloatRounded(-((float)heading_kp * app_imu_heading.error_deg),
                                                         1.0f);
    }

    if (correction > heading_correction_max)
    {
        correction = heading_correction_max;
    }
    else if (correction < -heading_correction_max)
    {
        correction = -heading_correction_max;
    }

    app_imu_heading.correction = (int16_t)correction;

#if APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_OUTPUT_ENABLE
    if ((command != NULL) && (action == APP_OBS_MOTOR_ACTION_FORWARD_SLOW))
    {
        command->left_duty = App_ObstacleMotor_ClampDuty((int32_t)command->left_duty -
                                                         app_imu_heading.correction);
        command->right_duty = App_ObstacleMotor_ClampDuty((int32_t)command->right_duty +
                                                          app_imu_heading.correction);
        app_imu_heading.apply_active = 1U;
    }
#elif APP_IMU_HEADING_ASSIST_GROUND_OUTPUT_ENABLE
    if ((command != NULL) && (assist_allowed != 0U))
    {
        command->left_duty = App_ObstacleMotor_ClampDuty((int32_t)command->left_duty -
                                                         app_imu_heading.correction);
        command->right_duty = App_ObstacleMotor_ClampDuty((int32_t)command->right_duty +
                                                          app_imu_heading.correction);
        app_imu_heading.apply_active = 1U;
    }
#else
    (void)command;
#endif

    return app_imu_heading.correction;
#else
    (void)assist_allowed;
    (void)ground_state;
    (void)command;
    app_imu_heading.ready = 0U;
    app_imu_heading.active = 0U;
    app_imu_heading.target_valid = 0U;
    app_imu_heading.apply_active = 0U;
    app_imu_heading.correction = 0;
    return 0;
#endif
}

static int32_t App_ObstacleMotor_HeadingKp(void)
{
#if APP_IMU_HEADING_ASSIST_GROUND_TEST_ACTIVE
    return APP_IMU_HEADING_GROUND_KP;
#else
    return APP_IMU_HEADING_KP;
#endif
}

static int32_t App_ObstacleMotor_HeadingCorrectionMax(void)
{
#if APP_IMU_HEADING_ASSIST_GROUND_TEST_ACTIVE
    return APP_IMU_HEADING_GROUND_CORRECTION_MAX;
#else
    return APP_IMU_HEADING_CORRECTION_MAX;
#endif
}

static int32_t App_ObstacleMotor_HeadingDeadbandDeg(void)
{
#if APP_IMU_HEADING_ASSIST_GROUND_TEST_ACTIVE
    return APP_IMU_HEADING_GROUND_DEADBAND_DEG;
#else
    return APP_IMU_HEADING_DEADBAND_DEG;
#endif
}

static float App_ObstacleMotor_NormalizeAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static void App_ObstacleMotor_LogHeadingAssist(void)
{
    char target_text[20];
    char current_text[20];
    char error_text[20];

    APP_LOG("APP IMU HEADING: ground_test=%u apply=%u kp=%ld max=%ld deadband=%ld target=%s current=%s error=%s corr=%d",
            (unsigned int)APP_IMU_HEADING_ASSIST_GROUND_TEST_ACTIVE,
            (unsigned int)app_imu_heading.apply_active,
            (long)App_ObstacleMotor_HeadingKp(),
            (long)App_ObstacleMotor_HeadingCorrectionMax(),
            (long)App_ObstacleMotor_HeadingDeadbandDeg(),
            App_ObstacleMotor_FormatHeadingDeg(app_imu_heading.target_valid,
                                               app_imu_heading.target_deg,
                                               target_text,
                                               sizeof(target_text)),
            App_ObstacleMotor_FormatHeadingDeg(app_imu_heading.ready,
                                               app_imu_heading.current_deg,
                                               current_text,
                                               sizeof(current_text)),
            App_ObstacleMotor_FormatHeadingDeg((app_imu_heading.ready != 0U) &&
                                               (app_imu_heading.target_valid != 0U),
                                               app_imu_heading.error_deg,
                                               error_text,
                                               sizeof(error_text)),
            (int)app_imu_heading.correction);
}

#if APP_IMU_HEADING_ASSIST_DRY_RUN_ENABLE
static void App_ObstacleMotor_LogHeadingAssistTest(int16_t heading_correction)
{
    char target_text[20];
    char current_text[20];
    char error_text[20];

    APP_LOG("APP IMU HEADING TEST: lifted_wheel=%u apply=%u target=%s current=%s error=%s corr=%d",
            (unsigned int)APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ENABLE,
            (unsigned int)APP_IMU_HEADING_ASSIST_APPLY_TO_MOTOR,
            App_ObstacleMotor_FormatHeadingDeg(app_imu_heading.target_valid,
                                               app_imu_heading.target_deg,
                                               target_text,
                                               sizeof(target_text)),
            App_ObstacleMotor_FormatHeadingDeg(app_imu_heading.ready,
                                               app_imu_heading.current_deg,
                                               current_text,
                                               sizeof(current_text)),
            App_ObstacleMotor_FormatHeadingDeg((app_imu_heading.ready != 0U) &&
                                               (app_imu_heading.target_valid != 0U),
                                               app_imu_heading.error_deg,
                                               error_text,
                                               sizeof(error_text)),
            (int)heading_correction);
}
#endif

static const char *App_ObstacleMotor_FormatHeadingDeg(uint8_t valid,
                                                       float value_deg,
                                                       char *buffer,
                                                       uint32_t buffer_size)
{
    int32_t tenth_deg = 0;

    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return "";
    }

    if (valid == 0U)
    {
        (void)snprintf(buffer, buffer_size, "--");
        return buffer;
    }

    tenth_deg = App_ObstacleMotor_ScaleFloatRounded(value_deg, 10.0f);
    (void)snprintf(buffer,
                   buffer_size,
                   "%s%lu.%01ludeg",
                   App_ObstacleMotor_FixedSign(tenth_deg),
                   (unsigned long)App_ObstacleMotor_FixedWhole(tenth_deg, 10),
                   (unsigned long)App_ObstacleMotor_FixedFraction(tenth_deg, 10));

    return buffer;
}

static int32_t App_ObstacleMotor_ScaleFloatRounded(float value, float multiplier)
{
    float scaled = value * multiplier;

    if (scaled < 0.0f)
    {
        return (int32_t)(scaled - 0.5f);
    }

    return (int32_t)(scaled + 0.5f);
}

static const char *App_ObstacleMotor_FixedSign(int32_t value)
{
    return (value < 0) ? "-" : "";
}

static uint32_t App_ObstacleMotor_FixedWhole(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;
    return (uint32_t)(abs_value / decimal_scale);
}

static uint32_t App_ObstacleMotor_FixedFraction(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;
    return (uint32_t)(abs_value % decimal_scale);
}

static void App_ObstacleMotor_ResetFrontHistory(void)
{
    app_front_block_history = 0U;
    app_front_clear_history = 0U;
    app_front_history_count = 0U;
    app_obs_front_history_count = 0U;

    for (uint8_t i = 0U; i < APP_OBS_GROUND_OBS_FRONT_HISTORY_SIZE; i++)
    {
        app_obs_front_history[i] = APP_OBS_MOTOR_INVALID_DISTANCE_MM;
    }
}

static AppObstacleFrontDistance App_ObstacleMotor_SelectObstacleFront(const AppLidarStatus *lidar)
{
    AppObstacleFrontDistance obs_front = {0U, APP_OBS_MOTOR_INVALID_DISTANCE_MM, "none"};

    if ((lidar == NULL) || (lidar->ready == 0U))
    {
        return obs_front;
    }

    if ((lidar->front_valid != 0U) &&
        (lidar->front_min_mm != 0U) &&
        (lidar->front_min_mm != APP_OBS_MOTOR_INVALID_DISTANCE_MM))
    {
        obs_front.valid = 1U;
        obs_front.distance_mm = lidar->front_min_mm;
        obs_front.source = "front";
    }

    if ((lidar->front_wide_valid != 0U) &&
        (lidar->front_wide_min_mm != 0U) &&
        (lidar->front_wide_min_mm != APP_OBS_MOTOR_INVALID_DISTANCE_MM) &&
        ((obs_front.valid == 0U) || (lidar->front_wide_min_mm < obs_front.distance_mm)))
    {
        obs_front.valid = 1U;
        obs_front.distance_mm = lidar->front_wide_min_mm;
        obs_front.source = "front_wide";
    }

    return obs_front;
}

static void App_ObstacleMotor_UpdateFrontHistory(const AppObstacleFrontDistance *obs_front)
{
    uint8_t block_sample = 0U;
    uint8_t clear_sample = 0U;

    if (obs_front == NULL)
    {
        App_ObstacleMotor_ResetFrontHistory();
        return;
    }

    if (obs_front->valid == 0U)
    {
        clear_sample = 1U;
    }
    else if (obs_front->distance_mm <= APP_OBS_GROUND_FRONT_BLOCK_MM)
    {
        block_sample = 1U;
    }
    else if (obs_front->distance_mm >= APP_OBS_GROUND_FRONT_CLEAR_MM)
    {
        clear_sample = 1U;
    }

    app_front_block_history = (uint8_t)(((app_front_block_history << 1U) | block_sample) &
                                        APP_OBS_GROUND_FRONT_HISTORY_MASK);
    app_front_clear_history = (uint8_t)(((app_front_clear_history << 1U) | clear_sample) &
                                        APP_OBS_GROUND_FRONT_HISTORY_MASK);

    if (app_front_history_count < APP_OBS_GROUND_FRONT_HISTORY_SIZE)
    {
        app_front_history_count++;
    }
}

static void App_ObstacleMotor_UpdateObsFrontTrend(const AppObstacleFrontDistance *obs_front)
{
    if ((obs_front == NULL) || (obs_front->valid == 0U))
    {
        app_obs_front_history_count = 0U;
        return;
    }

    app_obs_front_history[0] = app_obs_front_history[1];
    app_obs_front_history[1] = app_obs_front_history[2];
    app_obs_front_history[2] = obs_front->distance_mm;

    if (app_obs_front_history_count < APP_OBS_GROUND_OBS_FRONT_HISTORY_SIZE)
    {
        app_obs_front_history_count++;
    }
}

static uint8_t App_ObstacleMotor_FrontConfirmedBlocked(void)
{
    if (app_front_history_count < APP_OBS_GROUND_FRONT_HISTORY_SIZE)
    {
        return 0U;
    }

    return (App_ObstacleMotor_CountHistoryBits(app_front_block_history) >=
            APP_OBS_GROUND_FRONT_CONFIRM_COUNT) ? 1U : 0U;
}

static uint8_t App_ObstacleMotor_FrontConfirmedClear(void)
{
    if (app_front_history_count < APP_OBS_GROUND_FRONT_HISTORY_SIZE)
    {
        return 0U;
    }

    return (App_ObstacleMotor_CountHistoryBits(app_front_clear_history) >=
            APP_OBS_GROUND_FRONT_CONFIRM_COUNT) ? 1U : 0U;
}

static uint8_t App_ObstacleMotor_FrontApproaching(void)
{
    if (app_obs_front_history_count < APP_OBS_GROUND_OBS_FRONT_HISTORY_SIZE)
    {
        return 0U;
    }

    if (app_obs_front_history[2] >= APP_OBS_GROUND_APPROACHING_TURN_MM)
    {
        return 0U;
    }

    return ((app_obs_front_history[0] > app_obs_front_history[1]) &&
            (app_obs_front_history[1] > app_obs_front_history[2])) ? 1U : 0U;
}

static uint8_t App_ObstacleMotor_CountHistoryBits(uint8_t value)
{
    uint8_t count = 0U;

    value &= APP_OBS_GROUND_FRONT_HISTORY_MASK;

    while (value != 0U)
    {
        count = (uint8_t)(count + (value & 0x01U));
        value >>= 1U;
    }

    return count;
}

static uint8_t App_ObstacleMotor_StatusAllowsMotion(const AppLidarStatus *lidar)
{
    if ((lidar == NULL) || (lidar->ready == 0U))
    {
        return 0U;
    }

    if (((lidar->front_valid != 0U) &&
          ((lidar->front_min_mm == 0U) || (lidar->front_min_mm == APP_OBS_MOTOR_INVALID_DISTANCE_MM))) ||
        ((lidar->front_wide_valid != 0U) &&
         ((lidar->front_wide_min_mm == 0U) || (lidar->front_wide_min_mm == APP_OBS_MOTOR_INVALID_DISTANCE_MM))) ||
        ((lidar->left_valid != 0U) &&
         ((lidar->left_min_mm == 0U) || (lidar->left_min_mm == APP_OBS_MOTOR_INVALID_DISTANCE_MM))) ||
        ((lidar->right_valid != 0U) &&
         ((lidar->right_min_mm == 0U) || (lidar->right_min_mm == APP_OBS_MOTOR_INVALID_DISTANCE_MM))))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t App_ObstacleMotor_IsTurnState(AppObstacleGroundState state)
{
    return ((state == APP_OBS_GROUND_TURN_LEFT) ||
            (state == APP_OBS_GROUND_TURN_RIGHT)) ? 1U : 0U;
}

static AppObstacleGroundReason App_ObstacleMotor_EscapeHoldReason(AppObstacleGroundState state)
{
    return (state == APP_OBS_GROUND_TURN_LEFT) ?
           APP_OBS_GROUND_REASON_ESCAPE_LOCK_HOLD_LEFT :
           APP_OBS_GROUND_REASON_ESCAPE_LOCK_HOLD_RIGHT;
}

static AppObstacleGroundReason App_ObstacleMotor_CornerTurnReason(AppObstacleGroundState state)
{
    return (state == APP_OBS_GROUND_TURN_LEFT) ?
           APP_OBS_GROUND_REASON_CORNER_TURN_LEFT :
           APP_OBS_GROUND_REASON_CORNER_TURN_RIGHT;
}

static void App_ObstacleMotor_SetEscapeLock(AppObstacleGroundState state,
                                            uint32_t now_ms,
                                            AppObstacleGroundReason reason)
{
    uint8_t should_log = 0U;

    if (App_ObstacleMotor_IsTurnState(state) == 0U)
    {
        return;
    }

    if ((app_escape_lock_active == 0U) || (app_escape_lock_state != state) ||
        (reason == APP_OBS_GROUND_REASON_BACKUP_COMPLETE_TURN_LEFT) ||
        (reason == APP_OBS_GROUND_REASON_BACKUP_COMPLETE_TURN_RIGHT))
    {
        should_log = 1U;
    }

    app_escape_lock_active = 1U;
    app_escape_lock_state = state;
    app_escape_lock_start_ms = now_ms;
    app_last_turn_state = state;

    if (should_log != 0U)
    {
        APP_LOG("APP OBS GROUND: escape_lock enter dir=%s reason=%s",
                App_ObstacleMotor_EscapeDirName(state),
                App_ObstacleMotor_GroundReasonName(reason));
    }
}

static void App_ObstacleMotor_ClearEscapeLock(const char *exit_reason)
{
    if (app_escape_lock_active == 0U)
    {
        return;
    }

    APP_LOG("APP OBS GROUND: escape_lock exit reason=%s",
            (exit_reason != NULL) ? exit_reason : "unknown");
    app_escape_lock_active = 0U;
    app_escape_lock_state = APP_OBS_GROUND_STOP;
    app_escape_lock_start_ms = 0U;
}

static uint8_t App_ObstacleMotor_EscapeLockShouldExit(const AppObstacleFrontDistance *obs_front,
                                                      uint32_t now_ms)
{
    uint32_t elapsed_ms = 0U;
    uint8_t front_exit_clear = 0U;

    if (app_escape_lock_active == 0U)
    {
        return 0U;
    }

    elapsed_ms = now_ms - app_escape_lock_start_ms;
    front_exit_clear = ((obs_front == NULL) || (obs_front->valid == 0U) ||
                        (obs_front->distance_mm >= APP_GROUND_ESCAPE_EXIT_MM)) ? 1U : 0U;

    if ((app_ground_state == APP_OBS_GROUND_FORWARD) && (front_exit_clear != 0U))
    {
        return 1U;
    }

    if ((elapsed_ms >= APP_GROUND_ESCAPE_LOCK_MS) && (front_exit_clear != 0U))
    {
        return 1U;
    }

    return 0U;
}

static AppObstacleGroundState App_ObstacleMotor_SelectEscapeTurnState(const AppLidarStatus *lidar,
                                                                      AppObstacleDecision decision,
                                                                      AppObstacleGroundReason *reason)
{
    uint8_t left_valid = 0U;
    uint8_t right_valid = 0U;
    uint32_t left_score = 0U;
    uint32_t right_score = 0U;

    if (decision == APP_OBS_DECISION_TURN_LEFT)
    {
        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_DECISION_TURN_LEFT;
        }

        return APP_OBS_GROUND_TURN_LEFT;
    }

    if (decision == APP_OBS_DECISION_TURN_RIGHT)
    {
        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_DECISION_TURN_RIGHT;
        }

        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if ((lidar != NULL) && (lidar->ready != 0U))
    {
        left_valid = lidar->left_valid;
        right_valid = lidar->right_valid;
        left_score = App_ObstacleMotor_SideScore(lidar->left_valid, lidar->left_min_mm);
        right_score = App_ObstacleMotor_SideScore(lidar->right_valid, lidar->right_min_mm);
    }

    if ((left_valid == 0U) && (right_valid == 0U))
    {
        if (App_ObstacleMotor_IsTurnState(app_last_turn_state) != 0U)
        {
            if (reason != NULL)
            {
                *reason = (app_last_turn_state == APP_OBS_GROUND_TURN_LEFT) ?
                          APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT :
                          APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
            }

            return app_last_turn_state;
        }

        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
        }

        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if (left_score > right_score)
    {
        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT;
        }

        return APP_OBS_GROUND_TURN_LEFT;
    }

    if (reason != NULL)
    {
        *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
    }

    return APP_OBS_GROUND_TURN_RIGHT;
}

static void App_ObstacleMotor_ResetCornerTracking(void)
{
    app_recovery_start_ms = 0U;
    app_front_near_start_ms = 0U;
    app_backup_event_count = 0U;

    for (uint8_t i = 0U; i < APP_OBS_GROUND_BACKUP_HISTORY_SIZE; i++)
    {
        app_backup_event_ms[i] = 0U;
    }
}

static void App_ObstacleMotor_UpdateCornerTracking(const AppObstacleFrontDistance *obs_front,
                                                   uint32_t now_ms)
{
    uint8_t recovery_state = ((App_ObstacleMotor_IsTurnState(app_ground_state) != 0U) ||
                              (app_ground_state == APP_OBS_GROUND_BACKUP) ||
                              (app_escape_lock_active != 0U)) ? 1U : 0U;
    uint8_t front_near = ((obs_front != NULL) &&
                          (obs_front->valid != 0U) &&
                          (obs_front->distance_mm < APP_GROUND_ESCAPE_ENTER_MM) &&
                          (app_ground_state != APP_OBS_GROUND_FORWARD)) ? 1U : 0U;

    if ((app_ground_state == APP_OBS_GROUND_FORWARD) ||
        (app_ground_state == APP_OBS_GROUND_CAUTION))
    {
        app_recovery_start_ms = 0U;
    }
    else if (recovery_state != 0U)
    {
        if (app_recovery_start_ms == 0U)
        {
            app_recovery_start_ms = now_ms;
        }
    }
    else
    {
        app_recovery_start_ms = 0U;
    }

    if (front_near != 0U)
    {
        if (app_front_near_start_ms == 0U)
        {
            app_front_near_start_ms = now_ms;
        }
    }
    else
    {
        app_front_near_start_ms = 0U;
    }
}

static void App_ObstacleMotor_RecordBackupEvent(uint32_t now_ms)
{
    if (app_backup_event_count < APP_OBS_GROUND_BACKUP_HISTORY_SIZE)
    {
        app_backup_event_ms[app_backup_event_count] = now_ms;
        app_backup_event_count++;
        return;
    }

    for (uint8_t i = 1U; i < APP_OBS_GROUND_BACKUP_HISTORY_SIZE; i++)
    {
        app_backup_event_ms[i - 1U] = app_backup_event_ms[i];
    }

    app_backup_event_ms[APP_OBS_GROUND_BACKUP_HISTORY_SIZE - 1U] = now_ms;
}

static uint8_t App_ObstacleMotor_BackupCount5s(uint32_t now_ms)
{
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < app_backup_event_count; i++)
    {
        if ((now_ms - app_backup_event_ms[i]) <= APP_OBS_GROUND_BACKUP_WINDOW_MS)
        {
            count++;
        }
    }

    return count;
}

static uint8_t App_ObstacleMotor_CornerCooldownActive(uint32_t now_ms)
{
    return ((app_corner_cooldown_start_ms != 0U) &&
            ((now_ms - app_corner_cooldown_start_ms) < APP_GROUND_CORNER_COOLDOWN_MS)) ? 1U : 0U;
}

static uint8_t App_ObstacleMotor_CornerShouldEnter(uint32_t now_ms)
{
    if ((app_corner_escape_active != 0U) ||
        (App_ObstacleMotor_CornerCooldownActive(now_ms) != 0U))
    {
        return 0U;
    }

    if ((app_recovery_start_ms != 0U) &&
        ((now_ms - app_recovery_start_ms) >= APP_GROUND_CORNER_DETECT_MS))
    {
        return 1U;
    }

    if (App_ObstacleMotor_BackupCount5s(now_ms) >= APP_OBS_GROUND_BACKUP_HISTORY_SIZE)
    {
        return 1U;
    }

    if ((app_front_near_start_ms != 0U) &&
        ((now_ms - app_front_near_start_ms) >= APP_OBS_GROUND_FRONT_NEAR_MS))
    {
        return 1U;
    }

    return 0U;
}

static AppObstacleGroundState App_ObstacleMotor_SelectCornerTurnState(const AppLidarStatus *lidar)
{
    uint32_t left_score = 0U;
    uint32_t right_score = 0U;

    if ((lidar != NULL) && (lidar->ready != 0U))
    {
        left_score = App_ObstacleMotor_SideScore(lidar->left_valid, lidar->left_min_mm);
        right_score = App_ObstacleMotor_SideScore(lidar->right_valid, lidar->right_min_mm);

        if (left_score > right_score)
        {
            return APP_OBS_GROUND_TURN_LEFT;
        }

        if (right_score > left_score)
        {
            return APP_OBS_GROUND_TURN_RIGHT;
        }
    }

    if (app_last_turn_state == APP_OBS_GROUND_TURN_LEFT)
    {
        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if (app_last_turn_state == APP_OBS_GROUND_TURN_RIGHT)
    {
        return APP_OBS_GROUND_TURN_LEFT;
    }

    return APP_OBS_GROUND_TURN_RIGHT;
}

static void App_ObstacleMotor_StartCornerEscape(const AppLidarStatus *lidar, uint32_t now_ms)
{
    App_ObstacleMotor_ClearEscapeLock("corner_escape");
    app_corner_escape_active = 1U;
    app_corner_escape_start_ms = now_ms;
    app_corner_turn_state = App_ObstacleMotor_SelectCornerTurnState(lidar);
    app_recovery_start_ms = 0U;
    app_front_near_start_ms = 0U;

    APP_LOG("APP OBS GROUND: state=CORNER_BACKUP reason=corner_escape_enter");
}

static void App_ObstacleMotor_ClearCornerEscape(const char *exit_reason, uint32_t now_ms)
{
    if (app_corner_escape_active == 0U)
    {
        return;
    }

    APP_LOG("APP OBS GROUND: corner_escape exit reason=%s",
            (exit_reason != NULL) ? exit_reason : "unknown");
    app_corner_escape_active = 0U;
    app_corner_escape_start_ms = 0U;
    app_corner_cooldown_start_ms = now_ms;
    app_recovery_start_ms = 0U;
    app_front_near_start_ms = 0U;
}

static AppObstacleGroundState App_ObstacleMotor_UpdateGroundState(const AppLidarStatus *lidar,
                                                                  const AppObstacleFrontDistance *obs_front,
                                                                  AppObstacleDecision decision,
                                                                  uint32_t now_ms,
                                                                  uint32_t *hold_ms,
                                                                  AppObstacleGroundReason *reason)
{
    AppObstacleGroundReason desired_reason = APP_OBS_GROUND_REASON_INIT;
    AppObstacleGroundState desired_state = App_ObstacleMotor_EvaluateGroundState(lidar,
                                                                                 obs_front,
                                                                                 decision,
                                                                                 now_ms,
                                                                                 &desired_reason);
    uint32_t elapsed_ms = now_ms - app_ground_state_enter_ms;
    uint32_t required_hold_ms = App_ObstacleMotor_StateHoldMs(app_ground_state);
    uint8_t opposite_turn = (((app_ground_state == APP_OBS_GROUND_TURN_LEFT) &&
                              (desired_state == APP_OBS_GROUND_TURN_RIGHT)) ||
                             ((app_ground_state == APP_OBS_GROUND_TURN_RIGHT) &&
                              (desired_state == APP_OBS_GROUND_TURN_LEFT))) ? 1U : 0U;
    uint8_t escape_lock_reason = ((desired_reason == APP_OBS_GROUND_REASON_ESCAPE_LOCK_HOLD_LEFT) ||
                                  (desired_reason == APP_OBS_GROUND_REASON_ESCAPE_LOCK_HOLD_RIGHT) ||
                                  (desired_reason == APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_RESELECT_LEFT) ||
                                  (desired_reason == APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_RESELECT_RIGHT) ||
                                  (desired_reason == APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_MARGIN_SWITCH_LEFT) ||
                                  (desired_reason == APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_MARGIN_SWITCH_RIGHT) ||
                                  (desired_reason == APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_KEEP_LEFT) ||
                                  (desired_reason == APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_KEEP_RIGHT)) ? 1U : 0U;
    uint8_t bypass_hold = ((desired_state == APP_OBS_GROUND_STOP) ||
                           (desired_state == APP_OBS_GROUND_BACKUP) ||
                           (desired_state == APP_OBS_GROUND_CORNER_BACKUP) ||
                           (desired_state == APP_OBS_GROUND_CORNER_TURN) ||
                           (desired_state == APP_OBS_GROUND_BLOCKED) ||
                           (desired_reason == APP_OBS_GROUND_REASON_DECISION_TURN_LEFT) ||
                           (desired_reason == APP_OBS_GROUND_REASON_DECISION_TURN_RIGHT) ||
                           (desired_reason == APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT) ||
                           (desired_reason == APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT) ||
                           (desired_reason == APP_OBS_GROUND_REASON_EMERGENCY_FRONT_TURN) ||
                           (desired_reason == APP_OBS_GROUND_REASON_FRONT_APPROACHING_TURN) ||
                           (desired_reason == APP_OBS_GROUND_REASON_FRONT_CONFIRMED_BLOCK) ||
                           (desired_reason == APP_OBS_GROUND_REASON_FRONT_BLOCK_TURN) ||
                           (desired_reason == APP_OBS_GROUND_REASON_ESCAPE_LOCK_CLEAR_FORWARD_EXIT) ||
                           (desired_reason == APP_OBS_GROUND_REASON_CORNER_ESCAPE_ENTER) ||
                           (desired_reason == APP_OBS_GROUND_REASON_CORNER_BACKUP) ||
                           (desired_reason == APP_OBS_GROUND_REASON_CORNER_TURN_LEFT) ||
                           (desired_reason == APP_OBS_GROUND_REASON_CORNER_TURN_RIGHT) ||
                           (desired_reason == APP_OBS_GROUND_REASON_CORNER_ESCAPE_EXIT) ||
                           (desired_reason == APP_OBS_GROUND_REASON_HARD_STOP_TURN)) ? 1U : 0U;

    if (escape_lock_reason != 0U)
    {
        /* Escape lock has priority over normal turn hold/switch hysteresis. */
    }
    else if (opposite_turn != 0U)
    {
        AppObstacleGroundReason switch_reason = APP_OBS_GROUND_REASON_INIT;

        if (elapsed_ms < APP_OBS_GROUND_TURN_HOLD_MS)
        {
            desired_state = app_ground_state;
            desired_reason = (app_ground_state == APP_OBS_GROUND_TURN_LEFT) ?
                             APP_OBS_GROUND_REASON_TURN_HOLD_KEEP_LEFT :
                             APP_OBS_GROUND_REASON_TURN_HOLD_KEEP_RIGHT;
        }
        else if (App_ObstacleMotor_TurnSwitchAllowed(lidar,
                                                     app_ground_state,
                                                     desired_state,
                                                     &switch_reason) == 0U)
        {
            desired_state = app_ground_state;
            desired_reason = switch_reason;
        }
        else
        {
            desired_reason = switch_reason;
        }
    }
    else if ((bypass_hold == 0U) &&
             (desired_state != app_ground_state) &&
             (required_hold_ms > 0U) &&
             (elapsed_ms < required_hold_ms))
    {
        desired_state = app_ground_state;
        if ((app_ground_state == APP_OBS_GROUND_TURN_LEFT) ||
            (app_ground_state == APP_OBS_GROUND_TURN_RIGHT))
        {
            desired_reason = (app_ground_state == APP_OBS_GROUND_TURN_LEFT) ?
                             APP_OBS_GROUND_REASON_TURN_HOLD_MIN_KEEP_LEFT :
                             APP_OBS_GROUND_REASON_TURN_HOLD_MIN_KEEP_RIGHT;
        }
        else
        {
            desired_reason = APP_OBS_GROUND_REASON_MIN_HOLD;
        }
    }

    if (desired_state != app_ground_state)
    {
        AppObstacleGroundState previous_state = app_ground_state;

        app_ground_state = desired_state;
        app_ground_state_enter_ms = now_ms;
        app_forward_enter_ms = (desired_state == APP_OBS_GROUND_FORWARD) ? now_ms : 0U;
        if ((previous_state == APP_OBS_GROUND_BACKUP) &&
            ((desired_state == APP_OBS_GROUND_TURN_LEFT) ||
             (desired_state == APP_OBS_GROUND_TURN_RIGHT)))
        {
            app_backup_complete_ms = now_ms;
            app_escape_turn_state = desired_state;
        }

        if ((previous_state != APP_OBS_GROUND_BACKUP) &&
            (desired_state == APP_OBS_GROUND_BACKUP))
        {
            App_ObstacleMotor_RecordBackupEvent(now_ms);
        }

        elapsed_ms = 0U;
    }

    if (app_ground_state == APP_OBS_GROUND_FORWARD)
    {
        app_recovery_start_ms = 0U;
        app_front_near_start_ms = 0U;
    }

    if (App_ObstacleMotor_IsTurnState(app_ground_state) != 0U)
    {
        app_last_turn_state = app_ground_state;
    }

    if (hold_ms != NULL)
    {
        *hold_ms = elapsed_ms;
    }

    if (reason != NULL)
    {
        *reason = desired_reason;
    }

    return app_ground_state;
}

static AppObstacleGroundState App_ObstacleMotor_EvaluateGroundState(const AppLidarStatus *lidar,
                                                                    const AppObstacleFrontDistance *obs_front,
                                                                    AppObstacleDecision decision,
                                                                    uint32_t now_ms,
                                                                    AppObstacleGroundReason *reason)
{
    AppObstacleGroundReason local_reason = APP_OBS_GROUND_REASON_INIT;
    AppObstacleGroundState state = APP_OBS_GROUND_STOP;
    uint32_t current_hold_ms = now_ms - app_ground_state_enter_ms;
    uint32_t backup_elapsed_ms = (app_backup_complete_ms != 0U) ?
                                 (now_ms - app_backup_complete_ms) :
                                 0U;
    uint8_t backup_cooldown_active = ((app_backup_complete_ms != 0U) &&
                                      (backup_elapsed_ms < APP_GROUND_BACKUP_COOLDOWN_MS)) ? 1U : 0U;
    uint8_t escape_turn_active = ((app_backup_complete_ms != 0U) &&
                                  (backup_elapsed_ms < APP_GROUND_ESCAPE_TURN_MS)) ? 1U : 0U;
    uint8_t front_contact = 0U;
    uint8_t all_sides_blocked = 0U;
    uint8_t front_confirmed_blocked = App_ObstacleMotor_FrontConfirmedBlocked();
    uint8_t front_confirmed_clear = App_ObstacleMotor_FrontConfirmedClear();

    if (decision > APP_OBS_DECISION_STOP_BLOCKED)
    {
        App_ObstacleMotor_ClearEscapeLock("safety");
        App_ObstacleMotor_ClearCornerEscape("safety", now_ms);
        local_reason = APP_OBS_GROUND_REASON_INVALID_DECISION;
        state = APP_OBS_GROUND_BLOCKED;
        goto done;
    }

    if (decision == APP_OBS_DECISION_NO_LIDAR)
    {
        App_ObstacleMotor_ClearEscapeLock("safety");
        App_ObstacleMotor_ClearCornerEscape("safety", now_ms);
        local_reason = APP_OBS_GROUND_REASON_LIDAR_NOT_READY;
        state = APP_OBS_GROUND_STOP;
        goto done;
    }

    if ((lidar == NULL) || (lidar->ready == 0U))
    {
        App_ObstacleMotor_ClearEscapeLock("safety");
        App_ObstacleMotor_ClearCornerEscape("safety", now_ms);
        local_reason = APP_OBS_GROUND_REASON_LIDAR_NOT_READY;
        state = APP_OBS_GROUND_STOP;
        goto done;
    }

    front_contact = ((obs_front != NULL) &&
                     (obs_front->valid != 0U) &&
                     (obs_front->distance_mm <= APP_OBS_GROUND_CONTACT_STOP_MM)) ? 1U : 0U;
    all_sides_blocked = ((front_contact != 0U) &&
                         (App_ObstacleMotor_SideIsBlocked(lidar->left_valid, lidar->left_min_mm) != 0U) &&
                         (App_ObstacleMotor_SideIsBlocked(lidar->right_valid, lidar->right_min_mm) != 0U)) ? 1U : 0U;
    App_ObstacleMotor_UpdateCornerTracking(obs_front, now_ms);

    if (app_ground_state == APP_OBS_GROUND_CORNER_BACKUP)
    {
        if (current_hold_ms < APP_GROUND_CORNER_BACKUP_MS)
        {
            local_reason = APP_OBS_GROUND_REASON_CORNER_BACKUP;
            state = APP_OBS_GROUND_CORNER_BACKUP;
            goto done;
        }

        local_reason = App_ObstacleMotor_CornerTurnReason(app_corner_turn_state);
        state = APP_OBS_GROUND_CORNER_TURN;
        goto done;
    }

    if (app_ground_state == APP_OBS_GROUND_BACKUP)
    {
        if (current_hold_ms < APP_GROUND_BACKUP_MS)
        {
            local_reason = APP_OBS_GROUND_REASON_CONTACT_BACKUP;
            state = APP_OBS_GROUND_BACKUP;
            goto done;
        }

        if (decision == APP_OBS_DECISION_STOP_BLOCKED)
        {
            App_ObstacleMotor_ClearEscapeLock("safety");
            App_ObstacleMotor_ClearCornerEscape("safety", now_ms);
            local_reason = APP_OBS_GROUND_REASON_BACKUP_COMPLETE_BLOCKED;
            state = APP_OBS_GROUND_BLOCKED;
            goto done;
        }

        state = App_ObstacleMotor_SelectBackupTurnState(lidar, &local_reason);
        App_ObstacleMotor_SetEscapeLock(state, now_ms, local_reason);
        goto done;
    }

    if (decision == APP_OBS_DECISION_STOP_BLOCKED)
    {
        App_ObstacleMotor_ClearEscapeLock("safety");
        App_ObstacleMotor_ClearCornerEscape("safety", now_ms);
        local_reason = APP_OBS_GROUND_REASON_STOP_BLOCKED_PRIORITY;
        state = APP_OBS_GROUND_BLOCKED;
        goto done;
    }

    if (all_sides_blocked != 0U)
    {
        App_ObstacleMotor_ClearEscapeLock("safety");
        App_ObstacleMotor_ClearCornerEscape("safety", now_ms);
        local_reason = APP_OBS_GROUND_REASON_BLOCKED_ALL_SIDES;
        state = APP_OBS_GROUND_BLOCKED;
        goto done;
    }

    if (app_ground_state == APP_OBS_GROUND_CORNER_TURN)
    {
        if (current_hold_ms < APP_GROUND_CORNER_TURN_MS)
        {
            local_reason = App_ObstacleMotor_CornerTurnReason(app_corner_turn_state);
            state = APP_OBS_GROUND_CORNER_TURN;
            goto done;
        }

        App_ObstacleMotor_ClearCornerEscape("corner_escape_exit", now_ms);
        local_reason = APP_OBS_GROUND_REASON_CORNER_ESCAPE_EXIT;
    }

    if (App_ObstacleMotor_CornerShouldEnter(now_ms) != 0U)
    {
        App_ObstacleMotor_StartCornerEscape(lidar, now_ms);
        local_reason = APP_OBS_GROUND_REASON_CORNER_ESCAPE_ENTER;
        state = APP_OBS_GROUND_CORNER_BACKUP;
        goto done;
    }
    else if ((App_ObstacleMotor_CornerCooldownActive(now_ms) != 0U) &&
             ((app_recovery_start_ms != 0U) || (app_front_near_start_ms != 0U) ||
              (App_ObstacleMotor_BackupCount5s(now_ms) >= APP_OBS_GROUND_BACKUP_HISTORY_SIZE)))
    {
        local_reason = APP_OBS_GROUND_REASON_CORNER_COOLDOWN;
    }

    if (front_contact != 0U)
    {
        if (backup_cooldown_active != 0U)
        {
            local_reason = APP_OBS_GROUND_REASON_BACKUP_COOLDOWN_TURN;
            if (app_escape_lock_active != 0U)
            {
                local_reason = App_ObstacleMotor_EscapeHoldReason(app_escape_lock_state);
                state = app_escape_lock_state;
            }
            else if ((app_ground_state == APP_OBS_GROUND_TURN_LEFT) ||
                     (app_ground_state == APP_OBS_GROUND_TURN_RIGHT))
            {
                state = app_ground_state;
            }
            else
            {
                state = App_ObstacleMotor_SelectBackupTurnState(lidar, NULL);
            }

            goto done;
        }

        local_reason = APP_OBS_GROUND_REASON_CONTACT_BACKUP;
        state = APP_OBS_GROUND_BACKUP;
        goto done;
    }

    if ((app_escape_lock_active != 0U) &&
        (decision == APP_OBS_DECISION_CLEAR_FORWARD) &&
        ((obs_front == NULL) || (obs_front->valid == 0U) ||
         (obs_front->distance_mm >= APP_GROUND_ESCAPE_EXIT_MM)))
    {
        App_ObstacleMotor_ClearEscapeLock("clear_forward");
        local_reason = APP_OBS_GROUND_REASON_ESCAPE_LOCK_CLEAR_FORWARD_EXIT;
        state = APP_OBS_GROUND_FORWARD;
        goto done;
    }

    if (App_ObstacleMotor_EscapeLockShouldExit(obs_front, now_ms) != 0U)
    {
        App_ObstacleMotor_ClearEscapeLock("front_recovered");
    }

    if (app_escape_lock_active != 0U)
    {
        uint32_t escape_elapsed_ms = now_ms - app_escape_lock_start_ms;
        uint8_t front_exit_clear = ((obs_front == NULL) || (obs_front->valid == 0U) ||
                                    (obs_front->distance_mm >= APP_GROUND_ESCAPE_EXIT_MM)) ? 1U : 0U;

        if (escape_elapsed_ms >= APP_GROUND_ESCAPE_LOCK_MS)
        {
            uint32_t left_score = App_ObstacleMotor_SideScore(lidar->left_valid, lidar->left_min_mm);
            uint32_t right_score = App_ObstacleMotor_SideScore(lidar->right_valid, lidar->right_min_mm);

            if (decision == APP_OBS_DECISION_CLEAR_FORWARD)
            {
                App_ObstacleMotor_ClearEscapeLock("clear_forward");
                local_reason = APP_OBS_GROUND_REASON_ESCAPE_LOCK_CLEAR_FORWARD_EXIT;
                state = APP_OBS_GROUND_FORWARD;
                goto done;
            }

            if (front_exit_clear != 0U)
            {
                App_ObstacleMotor_ClearEscapeLock("front_recovered");
            }
            else
            {
                if ((decision == APP_OBS_DECISION_TURN_LEFT) &&
                    (app_escape_lock_state == APP_OBS_GROUND_TURN_RIGHT))
                {
                    local_reason = APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_RESELECT_LEFT;
                    state = APP_OBS_GROUND_TURN_LEFT;
                    App_ObstacleMotor_SetEscapeLock(state, now_ms, local_reason);
                    goto done;
                }

                if ((decision == APP_OBS_DECISION_TURN_RIGHT) &&
                    (app_escape_lock_state == APP_OBS_GROUND_TURN_LEFT))
                {
                    local_reason = APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_RESELECT_RIGHT;
                    state = APP_OBS_GROUND_TURN_RIGHT;
                    App_ObstacleMotor_SetEscapeLock(state, now_ms, local_reason);
                    goto done;
                }

                if ((app_escape_lock_state == APP_OBS_GROUND_TURN_LEFT) &&
                    (right_score > left_score) &&
                    ((right_score - left_score) >= APP_GROUND_ESCAPE_TIMEOUT_SWITCH_MARGIN_MM))
                {
                    local_reason = APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_MARGIN_SWITCH_RIGHT;
                    state = APP_OBS_GROUND_TURN_RIGHT;
                    App_ObstacleMotor_SetEscapeLock(state, now_ms, local_reason);
                    goto done;
                }

                if ((app_escape_lock_state == APP_OBS_GROUND_TURN_RIGHT) &&
                    (left_score > right_score) &&
                    ((left_score - right_score) >= APP_GROUND_ESCAPE_TIMEOUT_SWITCH_MARGIN_MM))
                {
                    local_reason = APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_MARGIN_SWITCH_LEFT;
                    state = APP_OBS_GROUND_TURN_LEFT;
                    App_ObstacleMotor_SetEscapeLock(state, now_ms, local_reason);
                    goto done;
                }

                state = app_escape_lock_state;
                local_reason = (state == APP_OBS_GROUND_TURN_LEFT) ?
                               APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_KEEP_LEFT :
                               APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_KEEP_RIGHT;
                App_ObstacleMotor_SetEscapeLock(state, now_ms, local_reason);
                goto done;
            }
        }

        if (app_escape_lock_active != 0U)
        {
            local_reason = App_ObstacleMotor_EscapeHoldReason(app_escape_lock_state);
            state = app_escape_lock_state;
            goto done;
        }
    }

    if ((obs_front != NULL) &&
        (obs_front->valid != 0U) &&
        (obs_front->distance_mm <= APP_GROUND_ESCAPE_ENTER_MM))
    {
        state = App_ObstacleMotor_SelectEscapeTurnState(lidar, decision, &local_reason);
        App_ObstacleMotor_SetEscapeLock(state, now_ms, local_reason);
        local_reason = App_ObstacleMotor_EscapeHoldReason(state);
        goto done;
    }

    if (escape_turn_active != 0U)
    {
        local_reason = APP_OBS_GROUND_REASON_ESCAPE_TURN_HOLD;
        state = app_escape_turn_state;
        goto done;
    }

    if ((obs_front != NULL) &&
        (obs_front->valid != 0U) &&
        (obs_front->distance_mm <= APP_OBS_GROUND_EMERGENCY_STOP_MM))
    {
        local_reason = APP_OBS_GROUND_REASON_EMERGENCY_FRONT_TURN;
        state = App_ObstacleMotor_SelectTurnState(lidar, decision, NULL);
        goto done;
    }

    if (((obs_front != NULL) && (obs_front->valid != 0U)) &&
        (App_ObstacleMotor_FrontApproaching() != 0U))
    {
        local_reason = APP_OBS_GROUND_REASON_FRONT_APPROACHING_TURN;
        state = App_ObstacleMotor_SelectTurnState(lidar, decision, NULL);
        goto done;
    }

    /*
     * CLEAR_FORWARD has priority over fallback turn selection. Once the high
     * level obstacle decision reports clear, do not invent a new left/right
     * fallback turn unless a hard front safety threshold above already fired.
     */
    if ((decision == APP_OBS_DECISION_CLEAR_FORWARD) &&
        ((obs_front == NULL) || (obs_front->valid == 0U) ||
         (obs_front->distance_mm >= APP_OBS_GROUND_FRONT_CLEAR_MM)))
    {
        local_reason = APP_OBS_GROUND_REASON_CLEAR_FORWARD_RESUME;
        state = APP_OBS_GROUND_FORWARD;
        goto done;
    }

    if ((front_confirmed_blocked != 0U) ||
        ((obs_front != NULL) &&
         (obs_front->valid != 0U) &&
         (obs_front->distance_mm <= APP_OBS_GROUND_FRONT_BLOCK_MM)))
    {
        local_reason = (front_confirmed_blocked != 0U) ?
                       APP_OBS_GROUND_REASON_FRONT_CONFIRMED_BLOCK :
                       APP_OBS_GROUND_REASON_FRONT_BLOCK_TURN;
        state = App_ObstacleMotor_SelectTurnState(lidar,
                                                  decision,
                                                  (decision == APP_OBS_DECISION_CLEAR_FORWARD) ? NULL : &local_reason);
        goto done;
    }

    if ((decision == APP_OBS_DECISION_CLEAR_FORWARD) &&
        (obs_front != NULL) &&
        (obs_front->valid != 0U) &&
        (obs_front->distance_mm > APP_OBS_GROUND_FRONT_BLOCK_MM) &&
        (obs_front->distance_mm < APP_OBS_GROUND_FRONT_CLEAR_MM))
    {
        local_reason = APP_OBS_GROUND_REASON_FRONT_CAUTION_FORWARD;
        state = APP_OBS_GROUND_CAUTION;
        goto done;
    }

    if (front_confirmed_clear != 0U)
    {
        local_reason = APP_OBS_GROUND_REASON_FRONT_CONFIRMED_CLEAR;
        state = APP_OBS_GROUND_FORWARD;
        goto done;
    }

    if ((app_front_history_count < APP_OBS_GROUND_FRONT_HISTORY_SIZE) &&
        ((obs_front == NULL) || (obs_front->valid == 0U) ||
         (obs_front->distance_mm >= APP_OBS_GROUND_FRONT_CLEAR_MM)))
    {
        local_reason = APP_OBS_GROUND_REASON_FRONT_CLEAR_FORWARD;
        state = APP_OBS_GROUND_FORWARD;
        goto done;
    }

    if ((app_ground_state == APP_OBS_GROUND_FORWARD) ||
        (app_ground_state == APP_OBS_GROUND_CAUTION))
    {
        if (current_hold_ms >= APP_OBS_GROUND_FRONT_MID_HOLD_MAX_MS)
        {
            local_reason = APP_OBS_GROUND_REASON_FRONT_MID_HOLD_EXPIRED_TURN;
            state = App_ObstacleMotor_SelectTurnState(lidar, decision, &local_reason);
            goto done;
        }

        local_reason = APP_OBS_GROUND_REASON_FRONT_MID_HOLD;
        state = APP_OBS_GROUND_FORWARD;
        goto done;
    }

    if ((app_ground_state == APP_OBS_GROUND_TURN_LEFT) ||
        (app_ground_state == APP_OBS_GROUND_TURN_RIGHT))
    {
        local_reason = APP_OBS_GROUND_REASON_FRONT_MID_TURN;
        state = App_ObstacleMotor_SelectTurnState(lidar, decision, &local_reason);
        goto done;
    }

    local_reason = APP_OBS_GROUND_REASON_FRONT_MID_TURN;
    state = App_ObstacleMotor_SelectTurnState(lidar, decision, &local_reason);

done:
    if (reason != NULL)
    {
        *reason = local_reason;
    }

    return state;
}

static AppObstacleGroundState App_ObstacleMotor_SelectTurnState(const AppLidarStatus *lidar,
                                                                AppObstacleDecision decision,
                                                                AppObstacleGroundReason *reason)
{
    uint8_t left_clear = App_ObstacleMotor_SideIsClear(lidar->left_valid, lidar->left_min_mm);
    uint8_t right_clear = App_ObstacleMotor_SideIsClear(lidar->right_valid, lidar->right_min_mm);
    uint32_t left_score = App_ObstacleMotor_SideScore(lidar->left_valid, lidar->left_min_mm);
    uint32_t right_score = App_ObstacleMotor_SideScore(lidar->right_valid, lidar->right_min_mm);

    if (decision == APP_OBS_DECISION_TURN_LEFT)
    {
        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_DECISION_TURN_LEFT;
        }

        return APP_OBS_GROUND_TURN_LEFT;
    }

    if (decision == APP_OBS_DECISION_TURN_RIGHT)
    {
        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_DECISION_TURN_RIGHT;
        }

        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if (app_ground_state == APP_OBS_GROUND_TURN_LEFT)
    {
        if ((right_clear != 0U) && ((right_score > left_score) &&
            ((right_score - left_score) >= APP_OBS_GROUND_TURN_SWITCH_HYST_MM)))
        {
            if (reason != NULL)
            {
                *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
            }

            return APP_OBS_GROUND_TURN_RIGHT;
        }

        if (left_clear != 0U)
        {
            if (reason != NULL)
            {
                *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT;
            }

            return APP_OBS_GROUND_TURN_LEFT;
        }

        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
        }

        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if (app_ground_state == APP_OBS_GROUND_TURN_RIGHT)
    {
        if ((left_clear != 0U) && ((left_score > right_score) &&
            ((left_score - right_score) >= APP_OBS_GROUND_TURN_SWITCH_HYST_MM)))
        {
            if (reason != NULL)
            {
                *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT;
            }

            return APP_OBS_GROUND_TURN_LEFT;
        }

        if (right_clear != 0U)
        {
            if (reason != NULL)
            {
                *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
            }

            return APP_OBS_GROUND_TURN_RIGHT;
        }

        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT;
        }

        return APP_OBS_GROUND_TURN_LEFT;
    }

    if ((left_clear != 0U) && (right_clear != 0U))
    {
        if (left_score > right_score)
        {
            if (reason != NULL)
            {
                *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT;
            }

            return APP_OBS_GROUND_TURN_LEFT;
        }

        if (right_score > left_score)
        {
            if (reason != NULL)
            {
                *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
            }

            return APP_OBS_GROUND_TURN_RIGHT;
        }

        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
        }

        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if (left_clear != 0U)
    {
        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT;
        }

        return APP_OBS_GROUND_TURN_LEFT;
    }

    if (right_clear != 0U)
    {
        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
        }

        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if (left_score > right_score)
    {
        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT;
        }

        return APP_OBS_GROUND_TURN_LEFT;
    }

    if (right_score > left_score)
    {
        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
        }

        return APP_OBS_GROUND_TURN_RIGHT;
    }

    if (reason != NULL)
    {
        *reason = APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT;
    }

    return APP_OBS_GROUND_TURN_RIGHT;
}

static uint8_t App_ObstacleMotor_TurnSwitchAllowed(const AppLidarStatus *lidar,
                                                   AppObstacleGroundState current_state,
                                                   AppObstacleGroundState desired_state,
                                                   AppObstacleGroundReason *reason)
{
    uint32_t left_score = 0U;
    uint32_t right_score = 0U;

    if ((lidar == NULL) || (lidar->ready == 0U))
    {
        if (reason != NULL)
        {
            *reason = (current_state == APP_OBS_GROUND_TURN_LEFT) ?
                      APP_OBS_GROUND_REASON_TURN_SWITCH_MARGIN_KEEP_LEFT :
                      APP_OBS_GROUND_REASON_TURN_SWITCH_MARGIN_KEEP_RIGHT;
        }

        return 0U;
    }

    left_score = App_ObstacleMotor_SideScore(lidar->left_valid, lidar->left_min_mm);
    right_score = App_ObstacleMotor_SideScore(lidar->right_valid, lidar->right_min_mm);

    if ((current_state == APP_OBS_GROUND_TURN_LEFT) &&
        (desired_state == APP_OBS_GROUND_TURN_RIGHT))
    {
        if ((right_score > left_score) &&
            ((right_score - left_score) >= APP_OBS_GROUND_TURN_SWITCH_HYST_MM))
        {
            if (reason != NULL)
            {
                *reason = APP_OBS_GROUND_REASON_TURN_SWITCH_ALLOWED_RIGHT;
            }

            return 1U;
        }

        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_TURN_SWITCH_MARGIN_KEEP_LEFT;
        }

        return 0U;
    }

    if ((current_state == APP_OBS_GROUND_TURN_RIGHT) &&
        (desired_state == APP_OBS_GROUND_TURN_LEFT))
    {
        if ((left_score > right_score) &&
            ((left_score - right_score) >= APP_OBS_GROUND_TURN_SWITCH_HYST_MM))
        {
            if (reason != NULL)
            {
                *reason = APP_OBS_GROUND_REASON_TURN_SWITCH_ALLOWED_LEFT;
            }

            return 1U;
        }

        if (reason != NULL)
        {
            *reason = APP_OBS_GROUND_REASON_TURN_SWITCH_MARGIN_KEEP_RIGHT;
        }

        return 0U;
    }

    return 1U;
}

static AppObstacleGroundState App_ObstacleMotor_SelectBackupTurnState(const AppLidarStatus *lidar,
                                                                      AppObstacleGroundReason *reason)
{
    uint8_t left_blocked = App_ObstacleMotor_SideIsBlocked(lidar->left_valid, lidar->left_min_mm);
    uint8_t right_blocked = App_ObstacleMotor_SideIsBlocked(lidar->right_valid, lidar->right_min_mm);
    uint32_t left_score = App_ObstacleMotor_SideScore(lidar->left_valid, lidar->left_min_mm);
    uint32_t right_score = App_ObstacleMotor_SideScore(lidar->right_valid, lidar->right_min_mm);
    AppObstacleGroundState state = APP_OBS_GROUND_TURN_RIGHT;

    if ((left_blocked == 0U) && (right_blocked != 0U))
    {
        state = APP_OBS_GROUND_TURN_LEFT;
    }
    else if ((right_blocked == 0U) && (left_blocked != 0U))
    {
        state = APP_OBS_GROUND_TURN_RIGHT;
    }
    else if (left_score > right_score)
    {
        state = APP_OBS_GROUND_TURN_LEFT;
    }
    else
    {
        state = APP_OBS_GROUND_TURN_RIGHT;
    }

    if (reason != NULL)
    {
        *reason = (state == APP_OBS_GROUND_TURN_LEFT) ?
                  APP_OBS_GROUND_REASON_BACKUP_COMPLETE_TURN_LEFT :
                  APP_OBS_GROUND_REASON_BACKUP_COMPLETE_TURN_RIGHT;
    }

    return state;
}

static void App_ObstacleMotor_ForceGroundState(AppObstacleGroundState state,
                                               uint32_t now_ms,
                                               uint32_t *hold_ms)
{
    if (state != app_ground_state)
    {
        app_ground_state = state;
        app_ground_state_enter_ms = now_ms;
        app_forward_enter_ms = (state == APP_OBS_GROUND_FORWARD) ? now_ms : 0U;
    }

    if ((state == APP_OBS_GROUND_STOP) || (state == APP_OBS_GROUND_BLOCKED))
    {
        App_ObstacleMotor_ClearEscapeLock("safety");
        App_ObstacleMotor_ClearCornerEscape("safety", now_ms);
    }

    if (App_ObstacleMotor_IsTurnState(state) != 0U)
    {
        app_last_turn_state = state;
    }

    if (hold_ms != NULL)
    {
        *hold_ms = now_ms - app_ground_state_enter_ms;
    }
}

static AppObstacleMotorAction App_ObstacleMotor_ActionFromGroundState(AppObstacleGroundState state)
{
    switch (state)
    {
        case APP_OBS_GROUND_FORWARD:
            return APP_OBS_MOTOR_ACTION_FORWARD_SLOW;

        case APP_OBS_GROUND_CAUTION:
            return APP_OBS_MOTOR_ACTION_FORWARD_CAUTION;

        case APP_OBS_GROUND_TURN_LEFT:
            return APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW;

        case APP_OBS_GROUND_TURN_RIGHT:
            return APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW;

        case APP_OBS_GROUND_BACKUP:
            return APP_OBS_MOTOR_ACTION_BACKUP_SLOW;

        case APP_OBS_GROUND_CORNER_BACKUP:
            return APP_OBS_MOTOR_ACTION_CORNER_BACKUP;

        case APP_OBS_GROUND_CORNER_TURN:
            return (app_corner_turn_state == APP_OBS_GROUND_TURN_LEFT) ?
                   APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW :
                   APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW;

        case APP_OBS_GROUND_STOP:
        case APP_OBS_GROUND_BLOCKED:
        default:
            return APP_OBS_MOTOR_ACTION_STOP;
    }
}

static int16_t App_ObstacleMotor_ActionSpeed(AppObstacleMotorAction action, int16_t configured_speed)
{
    switch (action)
    {
        case APP_OBS_MOTOR_ACTION_FORWARD_CAUTION:
            return APP_GROUND_CAUTION_SPEED;

        case APP_OBS_MOTOR_ACTION_BACKUP_SLOW:
            return APP_GROUND_BACKUP_SPEED;

        case APP_OBS_MOTOR_ACTION_CORNER_BACKUP:
            return APP_GROUND_CORNER_BACKUP_SPEED;

        case APP_OBS_MOTOR_ACTION_FORWARD_SLOW:
        case APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW:
        case APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW:
            return configured_speed;

        case APP_OBS_MOTOR_ACTION_STOP:
        default:
            return 0;
    }
}

static AppObstacleMotorCommand App_ObstacleMotor_CommandFromAction(AppObstacleMotorAction action,
                                                                    int16_t speed,
                                                                    uint8_t start_boost)
{
    AppObstacleMotorCommand command = {0, 0};
    int32_t left_duty = 0;
    int32_t right_duty = 0;

    switch (action)
    {
        case APP_OBS_MOTOR_ACTION_FORWARD_SLOW:
        case APP_OBS_MOTOR_ACTION_FORWARD_CAUTION:
            left_duty = (int32_t)speed + APP_GROUND_LEFT_TRIM;
            right_duty = (int32_t)speed + APP_GROUND_RIGHT_TRIM;
            if ((action == APP_OBS_MOTOR_ACTION_FORWARD_SLOW) && (start_boost != 0U))
            {
                left_duty += APP_GROUND_FORWARD_START_LEFT_EXTRA;
                right_duty += APP_GROUND_FORWARD_START_RIGHT_EXTRA;
            }

            command.left_duty = App_ObstacleMotor_ClampDuty(left_duty);
            command.right_duty = App_ObstacleMotor_ClampDuty(right_duty);
            break;

        case APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW:
            command.left_duty = App_ObstacleMotor_ClampDuty((int32_t)-speed);
            command.right_duty = App_ObstacleMotor_ClampDuty(speed);
            break;

        case APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW:
            command.left_duty = App_ObstacleMotor_ClampDuty(speed);
            command.right_duty = App_ObstacleMotor_ClampDuty((int32_t)-speed);
            break;

        case APP_OBS_MOTOR_ACTION_BACKUP_SLOW:
        case APP_OBS_MOTOR_ACTION_CORNER_BACKUP:
            command.left_duty = App_ObstacleMotor_ClampDuty((int32_t)-speed);
            command.right_duty = App_ObstacleMotor_ClampDuty((int32_t)-speed);
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

        case APP_OBS_MOTOR_ACTION_FORWARD_CAUTION:
            return "FORWARD_CAUTION";

        case APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW:
            return "TURN_LEFT_SLOW";

        case APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW:
            return "TURN_RIGHT_SLOW";

        case APP_OBS_MOTOR_ACTION_BACKUP_SLOW:
            return "BACKUP_SLOW";

        case APP_OBS_MOTOR_ACTION_CORNER_BACKUP:
            return "CORNER_BACKUP";

        case APP_OBS_MOTOR_ACTION_STOP:
        default:
            return "STOP";
    }
}

static const char *App_ObstacleMotor_GroundStateName(AppObstacleGroundState state)
{
    switch (state)
    {
        case APP_OBS_GROUND_FORWARD:
            return "FORWARD";

        case APP_OBS_GROUND_CAUTION:
            return "CAUTION";

        case APP_OBS_GROUND_TURN_LEFT:
            return "TURN_LEFT";

        case APP_OBS_GROUND_TURN_RIGHT:
            return "TURN_RIGHT";

        case APP_OBS_GROUND_BACKUP:
            return "BACKUP";

        case APP_OBS_GROUND_CORNER_BACKUP:
            return "CORNER_BACKUP";

        case APP_OBS_GROUND_CORNER_TURN:
            return "CORNER_TURN";

        case APP_OBS_GROUND_BLOCKED:
            return "BLOCKED";

        case APP_OBS_GROUND_STOP:
        default:
            return "STOP";
    }
}

static const char *App_ObstacleMotor_GroundReasonName(AppObstacleGroundReason reason)
{
    switch (reason)
    {
        case APP_OBS_GROUND_REASON_START_DELAY:
            return "start_delay";

        case APP_OBS_GROUND_REASON_GROUND_TIMEOUT:
            return "ground_timeout";

        case APP_OBS_GROUND_REASON_LIDAR_NOT_READY:
            return "lidar_not_ready";

        case APP_OBS_GROUND_REASON_INVALID_DECISION:
            return "invalid_decision";

        case APP_OBS_GROUND_REASON_FRONT_CLEAR_FORWARD:
            return "front_clear_forward";

        case APP_OBS_GROUND_REASON_CLEAR_FORWARD_RESUME:
            return "clear_forward_resume";

        case APP_OBS_GROUND_REASON_FRONT_CONFIRMED_CLEAR:
            return "front_confirmed_clear";

        case APP_OBS_GROUND_REASON_TURN_RESUME_FORWARD:
            return "turn_resume_forward";

        case APP_OBS_GROUND_REASON_DECISION_TURN_LEFT:
            return "decision_turn_left";

        case APP_OBS_GROUND_REASON_DECISION_TURN_RIGHT:
            return "decision_turn_right";

        case APP_OBS_GROUND_REASON_FRONT_BLOCK_TURN:
            return "front_block_turn";

        case APP_OBS_GROUND_REASON_FRONT_CONFIRMED_BLOCK:
            return "front_confirmed_block";

        case APP_OBS_GROUND_REASON_FRONT_MID_HOLD:
            return "front_mid_hold";

        case APP_OBS_GROUND_REASON_FRONT_MID_HOLD_EXPIRED_TURN:
            return "front_mid_hold_expired_turn";

        case APP_OBS_GROUND_REASON_FRONT_MID_TURN:
            return "front_mid_turn";

        case APP_OBS_GROUND_REASON_FRONT_CAUTION_FORWARD:
            return "front_caution_forward";

        case APP_OBS_GROUND_REASON_FRONT_APPROACHING_TURN:
            return "front_approaching_turn";

        case APP_OBS_GROUND_REASON_CONTACT_BACKUP:
            return "contact_backup";

        case APP_OBS_GROUND_REASON_BACKUP_COMPLETE_TURN_LEFT:
            return "backup_complete_turn_left";

        case APP_OBS_GROUND_REASON_BACKUP_COMPLETE_TURN_RIGHT:
            return "backup_complete_turn_right";

        case APP_OBS_GROUND_REASON_BACKUP_COMPLETE_BLOCKED:
            return "backup_complete_blocked";

        case APP_OBS_GROUND_REASON_ESCAPE_TURN_HOLD:
            return "escape_turn_hold";

        case APP_OBS_GROUND_REASON_BACKUP_COOLDOWN_TURN:
            return "backup_cooldown_turn";

        case APP_OBS_GROUND_REASON_EMERGENCY_FRONT_TURN:
            return "emergency_front_turn";

        case APP_OBS_GROUND_REASON_HARD_STOP_TURN:
            return "hard_stop_turn";

        case APP_OBS_GROUND_REASON_BLOCKED_ALL_SIDES:
            return "blocked_all_sides";

        case APP_OBS_GROUND_REASON_STOP_BLOCKED_PRIORITY:
            return "stop_blocked_priority";

        case APP_OBS_GROUND_REASON_TURN_HOLD_KEEP_LEFT:
            return "turn_hold_keep_left";

        case APP_OBS_GROUND_REASON_TURN_HOLD_KEEP_RIGHT:
            return "turn_hold_keep_right";

        case APP_OBS_GROUND_REASON_TURN_SWITCH_MARGIN_KEEP_LEFT:
            return "turn_switch_margin_keep_left";

        case APP_OBS_GROUND_REASON_TURN_SWITCH_MARGIN_KEEP_RIGHT:
            return "turn_switch_margin_keep_right";

        case APP_OBS_GROUND_REASON_TURN_SWITCH_ALLOWED_LEFT:
            return "turn_switch_allowed_left";

        case APP_OBS_GROUND_REASON_TURN_SWITCH_ALLOWED_RIGHT:
            return "turn_switch_allowed_right";

        case APP_OBS_GROUND_REASON_ESCAPE_LOCK_HOLD_LEFT:
            return "escape_lock_hold_left";

        case APP_OBS_GROUND_REASON_ESCAPE_LOCK_HOLD_RIGHT:
            return "escape_lock_hold_right";

        case APP_OBS_GROUND_REASON_ESCAPE_LOCK_CLEAR_FORWARD_EXIT:
            return "escape_lock_clear_forward_exit";

        case APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_RESELECT_LEFT:
            return "escape_lock_timeout_reselect_left";

        case APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_RESELECT_RIGHT:
            return "escape_lock_timeout_reselect_right";

        case APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_MARGIN_SWITCH_LEFT:
            return "escape_lock_timeout_margin_switch_left";

        case APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_MARGIN_SWITCH_RIGHT:
            return "escape_lock_timeout_margin_switch_right";

        case APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_KEEP_LEFT:
            return "escape_lock_timeout_keep_left";

        case APP_OBS_GROUND_REASON_ESCAPE_LOCK_TIMEOUT_KEEP_RIGHT:
            return "escape_lock_timeout_keep_right";

        case APP_OBS_GROUND_REASON_CORNER_ESCAPE_ENTER:
            return "corner_escape_enter";

        case APP_OBS_GROUND_REASON_CORNER_BACKUP:
            return "corner_backup";

        case APP_OBS_GROUND_REASON_CORNER_TURN_LEFT:
            return "corner_turn_left";

        case APP_OBS_GROUND_REASON_CORNER_TURN_RIGHT:
            return "corner_turn_right";

        case APP_OBS_GROUND_REASON_CORNER_ESCAPE_EXIT:
            return "corner_escape_exit";

        case APP_OBS_GROUND_REASON_CORNER_COOLDOWN:
            return "corner_cooldown";

        case APP_OBS_GROUND_REASON_TURN_HOLD_MIN_KEEP_LEFT:
            return "turn_hold_min_keep_left";

        case APP_OBS_GROUND_REASON_TURN_HOLD_MIN_KEEP_RIGHT:
            return "turn_hold_min_keep_right";

        case APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_LEFT:
            return "fallback_choose_left";

        case APP_OBS_GROUND_REASON_FALLBACK_CHOOSE_RIGHT:
            return "fallback_choose_right";

        case APP_OBS_GROUND_REASON_TURN_HOLD_MIN:
            return "turn_hold_min";

        case APP_OBS_GROUND_REASON_MIN_HOLD:
            return "min_hold";

        case APP_OBS_GROUND_REASON_INIT:
        default:
            return "init";
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

static const char *App_ObstacleMotor_LogSuffix(void)
{
#if APP_IMU_HEADING_ASSIST_LIFTED_WHEEL_TEST_ACTIVE
    return " lifted-wheel-test";
#elif APP_OBSTACLE_MOTOR_ENABLE && APP_OBSTACLE_GROUND_TEST_ENABLE
    return "";
#elif APP_OBSTACLE_MOTOR_ENABLE
    return " dry-run ground-test disabled";
#else
    return " dry-run";
#endif
}

static const char *App_ObstacleMotor_FrontSourceName(const AppObstacleFrontDistance *obs_front)
{
    if ((obs_front == NULL) || (obs_front->source == NULL))
    {
        return "none";
    }

    return obs_front->source;
}

static const char *App_ObstacleMotor_EscapeDirName(AppObstacleGroundState state)
{
    switch (state)
    {
        case APP_OBS_GROUND_TURN_LEFT:
            return "LEFT";

        case APP_OBS_GROUND_TURN_RIGHT:
            return "RIGHT";

        default:
            return "NONE";
    }
}

static const char *App_ObstacleMotor_CornerPhaseName(AppObstacleGroundState state)
{
    if (app_corner_escape_active == 0U)
    {
        return "NONE";
    }

    switch (state)
    {
        case APP_OBS_GROUND_CORNER_BACKUP:
            return "BACKUP";

        case APP_OBS_GROUND_CORNER_TURN:
            return "TURN";

        default:
            return "NONE";
    }
}

static const char *App_ObstacleMotor_FormatDistance(uint8_t valid,
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

static uint32_t App_ObstacleMotor_StateHoldMs(AppObstacleGroundState state)
{
    switch (state)
    {
        case APP_OBS_GROUND_FORWARD:
        case APP_OBS_GROUND_CAUTION:
            return APP_OBS_GROUND_FORWARD_HOLD_MS;

        case APP_OBS_GROUND_TURN_LEFT:
        case APP_OBS_GROUND_TURN_RIGHT:
            return APP_OBS_GROUND_TURN_HOLD_MS;

        case APP_OBS_GROUND_BACKUP:
            return APP_GROUND_BACKUP_MS;

        case APP_OBS_GROUND_STOP:
        case APP_OBS_GROUND_BLOCKED:
        default:
            return 0U;
    }
}

static uint32_t App_ObstacleMotor_SideScore(uint8_t valid, uint16_t distance_mm)
{
    if (valid == 0U)
    {
        return APP_OBS_GROUND_SIDE_OPEN_SCORE_MM;
    }

    return (uint32_t)distance_mm;
}

static uint8_t App_ObstacleMotor_SideIsClear(uint8_t valid, uint16_t distance_mm)
{
    if (valid == 0U)
    {
        return 1U;
    }

    return (distance_mm >= APP_OBS_GROUND_SIDE_CLEAR_MM) ? 1U : 0U;
}

static uint8_t App_ObstacleMotor_SideIsBlocked(uint8_t valid, uint16_t distance_mm)
{
    if (valid == 0U)
    {
        return 0U;
    }

    return (distance_mm <= APP_OBS_GROUND_SIDE_BLOCK_MM) ? 1U : 0U;
}

static int16_t App_ObstacleMotor_ClampDuty(int32_t duty)
{
    if (duty > APP_GROUND_MOTOR_MAX_ABS)
    {
        return APP_GROUND_MOTOR_MAX_ABS;
    }

    if (duty < -APP_GROUND_MOTOR_MAX_ABS)
    {
        return (int16_t)-APP_GROUND_MOTOR_MAX_ABS;
    }

    return (int16_t)duty;
}

static void App_ObstacleMotor_ApplyAction(AppObstacleMotorAction action,
                                           const AppObstacleMotorCommand *command)
{
#if APP_OBSTACLE_MOTOR_ENABLE
    switch (action)
    {
        case APP_OBS_MOTOR_ACTION_FORWARD_SLOW:
        case APP_OBS_MOTOR_ACTION_FORWARD_CAUTION:
        case APP_OBS_MOTOR_ACTION_TURN_LEFT_SLOW:
        case APP_OBS_MOTOR_ACTION_TURN_RIGHT_SLOW:
        case APP_OBS_MOTOR_ACTION_BACKUP_SLOW:
        case APP_OBS_MOTOR_ACTION_CORNER_BACKUP:
            Chassis_SetRaw(command->left_duty, command->right_duty);
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
