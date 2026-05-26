#include "app_serial_command.h"

#include "amr_system.h"
#include "app_explorer.h"
#include "app_fault.h"
#include "app_lidar.h"
#include "app_map.h"
#include "app_odometry.h"
#include "app_button_control.h"
#include "app_return_path.h"
#include "app_safety.h"
#include "app_ui.h"
#include "bringup_log.h"
#include "chassis.h"
#include "cmsis_os2.h"
#include "usart.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define APP_CMD_LINE_MAX 40U
#define APP_CMD_RX_QUEUE_SIZE 64U
#define APP_CMD_PENDING_QUEUE_SIZE 4U
#define APP_CMD_NO_NEWLINE_TIMEOUT_MS 250U
#define APP_CMD_STATUS_RATE_LIMIT_MS 500U
#define APP_CMD_MOTOR_TEST_MAX_DUTY 350
#define APP_CMD_MOTOR_TEST_MAX_MS 1500U
#ifndef APP_CMD_RX_CHAR_DEBUG
#define APP_CMD_RX_CHAR_DEBUG 0
#endif

static char app_cmd_line[APP_CMD_LINE_MAX];
static char app_cmd_pending_lines[APP_CMD_PENDING_QUEUE_SIZE][APP_CMD_LINE_MAX];
static uint8_t app_cmd_len = 0U;
static uint8_t app_cmd_initialized = 0U;
static uint8_t app_cmd_rx_byte = 0U;
static volatile uint8_t app_cmd_rx_queue[APP_CMD_RX_QUEUE_SIZE];
static volatile uint8_t app_cmd_rx_head = 0U;
static volatile uint8_t app_cmd_rx_tail = 0U;
static volatile uint8_t app_cmd_rx_overflow = 0U;
static uint8_t app_cmd_pending_head = 0U;
static uint8_t app_cmd_pending_tail = 0U;
static uint8_t app_cmd_pending_count = 0U;
static uint32_t app_cmd_last_char_ms = 0U;
static uint32_t app_cmd_last_status_ms = 0U;

static HAL_StatusTypeDef App_SerialCommand_StartRx(void);
static void App_SerialCommand_DrainRxQueue(void);
static void App_SerialCommand_ProcessChar(char ch);
static void App_SerialCommand_CheckNoNewlineCommand(uint32_t now_ms);
static void App_SerialCommand_QueueLine(const char *line);
static void App_SerialCommand_ProcessPendingCommand(void);
static void App_SerialCommand_HandleLine(const char *line);
static uint8_t App_SerialCommand_IsKnownCommand(const char *line);
static uint8_t App_SerialCommand_ParsePageCommand(const char *line, uint8_t *page);
static uint8_t App_SerialCommand_ParseOdoFreezeCommand(const char *line, uint8_t *freeze);
static uint8_t App_SerialCommand_ParseMotorTestCommand(const char *line,
                                                       char *mode,
                                                       int16_t *duty,
                                                       uint32_t *duration_ms);
static void App_SerialCommand_RunMotorTest(char mode, int16_t duty, uint32_t duration_ms);
static char App_SerialCommand_ToLower(char ch);
static void App_SerialCommand_LogRxChar(char ch);
static void App_SerialCommand_LogStatus(void);
static int32_t App_SerialCommand_ScaleFloatRounded(float value, float multiplier);
static const char *App_SerialCommand_FixedSign(int32_t value);
static uint32_t App_SerialCommand_FixedWhole(int32_t value, int32_t decimal_scale);
static uint32_t App_SerialCommand_FixedFraction(int32_t value, int32_t decimal_scale);
static uint32_t App_SerialCommand_ElapsedMs(uint32_t now_ms, uint32_t then_ms);

void App_SerialCommand_Init(void)
{
    HAL_StatusTypeDef rx_status;

    app_cmd_len = 0U;
    app_cmd_rx_head = 0U;
    app_cmd_rx_tail = 0U;
    app_cmd_rx_overflow = 0U;
    app_cmd_pending_head = 0U;
    app_cmd_pending_tail = 0U;
    app_cmd_pending_count = 0U;
    app_cmd_last_char_ms = 0U;
    app_cmd_last_status_ms = 0U;
    app_cmd_initialized = 1U;

    rx_status = App_SerialCommand_StartRx();
    if ((rx_status == HAL_OK) || (rx_status == HAL_BUSY))
    {
        APP_LOG("[CMD] ready uart=huart2 commands=start stop explore return estop reset_fault odo_reset odom_reset enc_dbg odom_dbg odo_freeze map_reset motor_test status map grid exp ui page tel");
        APP_LOG("[CMD] use newline: start<Enter> or no-newline command timeout=%u ms",
                (unsigned int)APP_CMD_NO_NEWLINE_TIMEOUT_MS);
    }
    else
    {
        APP_LOG("[CMD] huart2 rx start failed status=%u", (unsigned int)rx_status);
    }
}

void App_SerialCommand_Task(void)
{
    if (app_cmd_initialized == 0U)
    {
        App_SerialCommand_Init();
    }

    App_SerialCommand_Process();

    if (huart2.RxState != HAL_UART_STATE_BUSY_RX)
    {
        (void)App_SerialCommand_StartRx();
    }
}

void App_SerialCommand_Process(void)
{
    App_SerialCommand_DrainRxQueue();
    App_SerialCommand_CheckNoNewlineCommand(HAL_GetTick());
    App_SerialCommand_ProcessPendingCommand();
}

void App_SerialCommand_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t next_head;

    if ((huart == NULL) || (huart->Instance != USART2))
    {
        return;
    }

    next_head = (uint8_t)((app_cmd_rx_head + 1U) % APP_CMD_RX_QUEUE_SIZE);
    if (next_head == app_cmd_rx_tail)
    {
        app_cmd_rx_overflow = 1U;
    }
    else
    {
        app_cmd_rx_queue[app_cmd_rx_head] = app_cmd_rx_byte;
        app_cmd_rx_head = next_head;
    }

    (void)App_SerialCommand_StartRx();
}

static HAL_StatusTypeDef App_SerialCommand_StartRx(void)
{
    return HAL_UART_Receive_IT(&huart2, &app_cmd_rx_byte, 1U);
}

static void App_SerialCommand_DrainRxQueue(void)
{
    if (app_cmd_rx_overflow != 0U)
    {
        app_cmd_rx_overflow = 0U;
        app_cmd_len = 0U;
        APP_LOG("[CMD_RX] overflow, line discarded");
    }

    while (app_cmd_rx_tail != app_cmd_rx_head)
    {
        char ch = (char)app_cmd_rx_queue[app_cmd_rx_tail];
        app_cmd_rx_tail = (uint8_t)((app_cmd_rx_tail + 1U) % APP_CMD_RX_QUEUE_SIZE);
        App_SerialCommand_LogRxChar(ch);
        App_SerialCommand_ProcessChar(ch);
    }
}

static void App_SerialCommand_ProcessChar(char ch)
{
    if ((ch == '\r') || (ch == '\n'))
    {
        if (app_cmd_len > 0U)
        {
            app_cmd_line[app_cmd_len] = '\0';
            App_SerialCommand_QueueLine(app_cmd_line);
            app_cmd_len = 0U;
            app_cmd_last_char_ms = 0U;
        }

        return;
    }

    if ((ch == '\b') || (ch == 0x7F))
    {
        if (app_cmd_len > 0U)
        {
            app_cmd_len--;
        }

        return;
    }

    if ((ch < 0x20) || (ch > 0x7E))
    {
        return;
    }

    if (app_cmd_len < (APP_CMD_LINE_MAX - 1U))
    {
        app_cmd_line[app_cmd_len] = App_SerialCommand_ToLower(ch);
        app_cmd_len++;
        app_cmd_last_char_ms = HAL_GetTick();
    }
    else
    {
        APP_LOG("[CMD] line too long len=%u max=%u",
                (unsigned int)app_cmd_len,
                (unsigned int)(APP_CMD_LINE_MAX - 1U));
        app_cmd_len = 0U;
        app_cmd_last_char_ms = 0U;
    }
}

static void App_SerialCommand_CheckNoNewlineCommand(uint32_t now_ms)
{
    if ((app_cmd_len == 0U) ||
        (app_cmd_last_char_ms == 0U) ||
        ((now_ms - app_cmd_last_char_ms) < APP_CMD_NO_NEWLINE_TIMEOUT_MS))
    {
        return;
    }

    app_cmd_line[app_cmd_len] = '\0';
    if (App_SerialCommand_IsKnownCommand(app_cmd_line) != 0U)
    {
        App_SerialCommand_QueueLine(app_cmd_line);
        app_cmd_len = 0U;
        app_cmd_last_char_ms = 0U;
    }
}

static void App_SerialCommand_QueueLine(const char *line)
{
    const char *start;
    size_t len;

    if ((line == NULL) || (line[0] == '\0'))
    {
        return;
    }

    start = line;
    while (*start == ' ')
    {
        start++;
    }

    len = strlen(start);
    while ((len > 0U) && (start[len - 1U] == ' '))
    {
        len--;
    }

    if (len == 0U)
    {
        return;
    }

    if (app_cmd_pending_count >= APP_CMD_PENDING_QUEUE_SIZE)
    {
        APP_LOG("[CMD] command queue full, dropped=%.*s", (int)len, start);
        return;
    }

    APP_LOG("[CMD_RX] line=%.*s", (int)len, start);

    (void)snprintf(app_cmd_pending_lines[app_cmd_pending_head],
                   APP_CMD_LINE_MAX,
                   "%.*s",
                   (int)len,
                   start);
    app_cmd_pending_head = (uint8_t)((app_cmd_pending_head + 1U) % APP_CMD_PENDING_QUEUE_SIZE);
    app_cmd_pending_count++;
}

static void App_SerialCommand_ProcessPendingCommand(void)
{
    const char *line;

    if (app_cmd_pending_count == 0U)
    {
        return;
    }

    line = app_cmd_pending_lines[app_cmd_pending_tail];
    App_SerialCommand_HandleLine(line);
    app_cmd_pending_tail = (uint8_t)((app_cmd_pending_tail + 1U) % APP_CMD_PENDING_QUEUE_SIZE);
    app_cmd_pending_count--;
}

static void App_SerialCommand_HandleLine(const char *line)
{
    uint8_t page = 0U;
    uint8_t freeze = 0U;
    char motor_mode = '\0';
    int16_t motor_duty = 0;
    uint32_t motor_duration_ms = 0U;

    if (line == NULL)
    {
        return;
    }

    if (strcmp(line, "start") == 0)
    {
        AMR_RequestStart("serial_start");
    }
    else if (strcmp(line, "explore") == 0)
    {
        AMR_RequestStart("serial_explore");
        AppExplorer_StartExplore();
    }
    else if (strcmp(line, "stop") == 0)
    {
        AppExplorer_Stop();
        if (AMR_GetState() == AMR_STATE_RETURN)
        {
            ReturnExecutor_Stop("serial_stop");
        }
        AMR_RequestStop("serial_stop");
        AppOdo_SyncBaseline();
    }
    else if (strcmp(line, "return") == 0)
    {
        AppExplorer_StartReturn();
        AMR_RequestReturn("serial_return");
        if (AMR_GetState() == AMR_STATE_RETURN)
        {
            ReturnExecutor_Start();
        }
        else
        {
            APP_LOG("[RETURN] ignored state=%s", AMR_StateName(AMR_GetState()));
        }
    }
    else if (strcmp(line, "estop") == 0)
    {
        AppExplorer_Stop();
        ReturnExecutor_Stop("serial_estop");
        AMR_RequestEStop("serial_estop");
        AppOdo_SyncBaseline();
    }
    else if (strcmp(line, "reset_fault") == 0)
    {
        App_Safety_ClearFault();
        AppExplorer_Reset();
        AMR_RequestResetFault("serial_reset_fault");
        AppOdo_SyncBaseline();
    }
    else if ((strcmp(line, "odom_reset") == 0) || (strcmp(line, "odo_reset") == 0))
    {
        Odom_Reset();
        AppOdo_SyncBaseline();
        AppMap_Reset();
    }
    else if ((strcmp(line, "odom_dbg") == 0) || (strcmp(line, "enc_dbg") == 0))
    {
        AppOdo_PrintDebug();
    }
    else if (App_SerialCommand_ParseOdoFreezeCommand(line, &freeze) != 0U)
    {
        AppOdo_SetFreeze(freeze);
    }
    else if (strcmp(line, "map_reset") == 0)
    {
        AppMap_Reset();
    }
    else if (App_SerialCommand_ParseMotorTestCommand(line, &motor_mode, &motor_duty, &motor_duration_ms) != 0U)
    {
        App_SerialCommand_RunMotorTest(motor_mode, motor_duty, motor_duration_ms);
    }
    else if (strcmp(line, "status") == 0)
    {
        uint32_t now_ms = HAL_GetTick();
        if ((app_cmd_last_status_ms != 0U) &&
            (App_SerialCommand_ElapsedMs(now_ms, app_cmd_last_status_ms) < APP_CMD_STATUS_RATE_LIMIT_MS))
        {
            APP_LOG("[CMD] status ignored rate_limit");
            return;
        }

        app_cmd_last_status_ms = now_ms;
        App_SerialCommand_LogStatus();
        AppTelemetry_Print();
    }
    else if ((strcmp(line, "map") == 0) || (strcmp(line, "grid") == 0))
    {
        AppMap_PrintSummary();
        AppMap_PrintGrid();
    }
    else if (strcmp(line, "exp") == 0)
    {
        AppExplorer_PrintStatus();
    }
    else if (strcmp(line, "ui") == 0)
    {
        AppUI_PrintStatus();
    }
    else if (strcmp(line, "tel") == 0)
    {
        AppTelemetry_Print();
    }
    else if (App_SerialCommand_ParsePageCommand(line, &page) != 0U)
    {
        AppUI_SetPage(page);
        AppUI_PrintStatus();
    }
    else
    {
        APP_LOG("[CMD] unknown=%s", line);
    }
}

static uint8_t App_SerialCommand_IsKnownCommand(const char *line)
{
    if (line == NULL)
    {
        return 0U;
    }

    return ((strcmp(line, "start") == 0) ||
            (strcmp(line, "explore") == 0) ||
            (strcmp(line, "stop") == 0) ||
            (strcmp(line, "return") == 0) ||
            (strcmp(line, "estop") == 0) ||
            (strcmp(line, "reset_fault") == 0) ||
            (strcmp(line, "odom_reset") == 0) ||
            (strcmp(line, "odo_reset") == 0) ||
            (strcmp(line, "odom_dbg") == 0) ||
            (strcmp(line, "enc_dbg") == 0) ||
            (strcmp(line, "map_reset") == 0) ||
            (App_SerialCommand_ParseOdoFreezeCommand(line, NULL) != 0U) ||
            (App_SerialCommand_ParseMotorTestCommand(line, NULL, NULL, NULL) != 0U) ||
            (strcmp(line, "status") == 0) ||
            (strcmp(line, "map") == 0) ||
            (strcmp(line, "grid") == 0) ||
            (strcmp(line, "exp") == 0) ||
            (strcmp(line, "ui") == 0) ||
            (strcmp(line, "tel") == 0) ||
            (App_SerialCommand_ParsePageCommand(line, NULL) != 0U)) ? 1U : 0U;
}

static uint8_t App_SerialCommand_ParsePageCommand(const char *line, uint8_t *page)
{
    uint8_t parsed_page;

    if (line == NULL)
    {
        return 0U;
    }

    if ((strcmp(line, "page 0") == 0) || (strcmp(line, "page0") == 0))
    {
        parsed_page = 0U;
    }
    else if ((strcmp(line, "page 1") == 0) || (strcmp(line, "page1") == 0))
    {
        parsed_page = 1U;
    }
    else if ((strcmp(line, "page 2") == 0) || (strcmp(line, "page2") == 0))
    {
        parsed_page = 2U;
    }
    else
    {
        return 0U;
    }

    if (page != NULL)
    {
        *page = parsed_page;
    }

    return 1U;
}

static uint8_t App_SerialCommand_ParseOdoFreezeCommand(const char *line, uint8_t *freeze)
{
    uint8_t parsed;

    if (line == NULL)
    {
        return 0U;
    }

    if (strcmp(line, "odo_freeze 0") == 0)
    {
        parsed = 0U;
    }
    else if (strcmp(line, "odo_freeze 1") == 0)
    {
        parsed = 1U;
    }
    else
    {
        return 0U;
    }

    if (freeze != NULL)
    {
        *freeze = parsed;
    }

    return 1U;
}

static uint8_t App_SerialCommand_ParseMotorTestCommand(const char *line,
                                                       char *mode,
                                                       int16_t *duty,
                                                       uint32_t *duration_ms)
{
    char parsed_mode;
    int parsed_duty;
    unsigned long parsed_duration_ms;

    if (line == NULL)
    {
        return 0U;
    }

    if (sscanf(line, "motor_test %c %d %lu", &parsed_mode, &parsed_duty, &parsed_duration_ms) != 3)
    {
        return 0U;
    }

    if ((parsed_mode != 'l') && (parsed_mode != 'r') && (parsed_mode != 'f') && (parsed_mode != 'b'))
    {
        return 0U;
    }

    if (parsed_duty < 0)
    {
        parsed_duty = -parsed_duty;
    }

    if ((parsed_duty == 0) ||
        (parsed_duty > APP_CMD_MOTOR_TEST_MAX_DUTY) ||
        (parsed_duration_ms == 0UL) ||
        (parsed_duration_ms > APP_CMD_MOTOR_TEST_MAX_MS))
    {
        return 0U;
    }

    if (mode != NULL)
    {
        *mode = parsed_mode;
    }

    if (duty != NULL)
    {
        *duty = (int16_t)parsed_duty;
    }

    if (duration_ms != NULL)
    {
        *duration_ms = (uint32_t)parsed_duration_ms;
    }

    return 1U;
}

static void App_SerialCommand_RunMotorTest(char mode, int16_t duty, uint32_t duration_ms)
{
    uint32_t start_ms;

    if (AppFault_IsActive())
    {
        APP_LOG("[MOTOR_TEST] rejected fault=%s", AppFault_Name(AppFault_Get()));
        return;
    }

    if (AMR_GetState() != AMR_STATE_IDLE)
    {
        APP_LOG("[MOTOR_TEST] rejected state=%s", AMR_StateName(AMR_GetState()));
        return;
    }

    APP_LOG("[MOTOR_TEST] start mode=%c duty=%d duration=%lums", mode, (int)duty, (unsigned long)duration_ms);

    switch (mode)
    {
        case 'l':
            Chassis_SetRaw(duty, 0);
            break;

        case 'r':
            Chassis_SetRaw(0, duty);
            break;

        case 'f':
            Chassis_Forward(duty);
            break;

        case 'b':
            Chassis_Backward(duty);
            break;

        default:
            APP_LOG("[MOTOR_TEST] rejected mode=%c", mode);
            return;
    }

    start_ms = HAL_GetTick();
    while (App_SerialCommand_ElapsedMs(HAL_GetTick(), start_ms) < duration_ms)
    {
        if (AppFault_IsActive() || (AMR_GetState() != AMR_STATE_IDLE))
        {
            APP_LOG("[MOTOR_TEST] interrupted state=%s fault=%s",
                    AMR_StateName(AMR_GetState()),
                    AppFault_IsActive() ? AppFault_Name(AppFault_Get()) : "NONE");
            break;
        }

        osDelay(20U);
    }

    Chassis_Stop();
    APP_LOG("[MOTOR_TEST] done");
}

static char App_SerialCommand_ToLower(char ch)
{
    if ((ch >= 'A') && (ch <= 'Z'))
    {
        return (char)(ch + ('a' - 'A'));
    }

    return ch;
}

static void App_SerialCommand_LogRxChar(char ch)
{
#if APP_CMD_RX_CHAR_DEBUG
    if (ch == '\r')
    {
        APP_LOG("[CMD_RX] c=\\r");
    }
    else if (ch == '\n')
    {
        APP_LOG("[CMD_RX] c=\\n");
    }
    else if ((ch >= 0x20) && (ch <= 0x7E))
    {
        APP_LOG("[CMD_RX] c=%c", ch);
    }
    else
    {
        APP_LOG("[CMD_RX] c=0x%02X", (unsigned int)((uint8_t)ch));
    }
#else
    (void)ch;
#endif
}

static void App_SerialCommand_LogStatus(void)
{
    AMR_State_t state = AMR_GetState();
    AppSafetyStatus_t safety = {APP_FAULT_NONE, 0U, 0U, 0, 0, 0, 0};
    OdomPose_t pose = {0.0f, 0.0f, 0.0f};
    OdomSample_t sample = {0};
    ChassisCommandStatus_t command = {0, 0, 0U};
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    int32_t x_mm;
    int32_t y_mm;
    int32_t theta_tenth_deg;
    uint32_t now_ms;
    uint32_t lidar_last_rx_tick_ms;
    uint32_t lidar_last_valid_update_ms;
    uint32_t lidar_rx_age_ms;
    uint32_t lidar_valid_age_ms;
    uint16_t path_count;
    ReturnExecState_t return_state;
    AppFaultCode fault_code;
    const char *fault_name;
    AppMapSummary_t map_summary = {0, 0, APP_MAP_DIR_EAST, 0U, 0U, 0U};
    AppExplorerStatus_t exp_status = {EXP_IDLE, 0, 0, 0, 0, APP_MAP_DIR_EAST, APP_MAP_DIR_EAST, 0U, 0U, 1U};
    int32_t odom_ds_mm;
    int32_t odom_dtheta_mrad;
    int32_t odom_dt_ms;

    (void)App_Safety_GetStatus(&safety);
    (void)Odom_GetPose(&pose);
    (void)Odom_GetLastSample(&sample);
    Chassis_GetLastCommand(&command);

    now_ms = HAL_GetTick();
    lidar_last_rx_tick_ms = (lidar != NULL) ? lidar->last_rx_tick_ms : 0U;
    lidar_last_valid_update_ms = (lidar != NULL) ? lidar->last_valid_update_ms : 0U;
    lidar_rx_age_ms = App_SerialCommand_ElapsedMs(now_ms, lidar_last_rx_tick_ms);
    lidar_valid_age_ms = App_SerialCommand_ElapsedMs(now_ms, lidar_last_valid_update_ms);
    path_count = ReturnPath_Count();
    return_state = ReturnExecutor_GetState();
    fault_code = AppFault_Get();
    fault_name = AppFault_IsActive() ? AppFault_Name(fault_code) : App_Safety_FaultName(safety.fault_code);
    (void)AppMap_GetSummary(&map_summary);
    (void)AppExplorer_GetStatus(&exp_status);

    x_mm = App_SerialCommand_ScaleFloatRounded(pose.x_m, 1000.0f);
    y_mm = App_SerialCommand_ScaleFloatRounded(pose.y_m, 1000.0f);
    theta_tenth_deg = App_SerialCommand_ScaleFloatRounded(pose.theta_rad * 57.2957795f, 10.0f);
    odom_ds_mm = App_SerialCommand_ScaleFloatRounded(sample.delta_center_m, 1000.0f);
    odom_dtheta_mrad = App_SerialCommand_ScaleFloatRounded(sample.delta_theta_rad, 1000.0f);
    odom_dt_ms = App_SerialCommand_ScaleFloatRounded(sample.dt_s, 1000.0f);

    APP_LOG("[STATUS] state=%s fault=%s",
            AMR_StateName(state),
            fault_name);
    APP_LOG("[STATUS] lidar ready=%u front=%u rx_age=%lu valid_age=%lu err=%lu",
            (unsigned int)((lidar != NULL) ? lidar->ready : 0U),
            (unsigned int)(((lidar != NULL) && (lidar->front_valid != 0U)) ? lidar->front_min_mm : 0U),
            (unsigned long)lidar_rx_age_ms,
            (unsigned long)lidar_valid_age_ms,
            (unsigned long)((lidar != NULL) ? lidar->uart4_error_count : 0U));
    APP_LOG("[STATUS] pwmL=%d pwmR=%d rawL=%ld rawR=%ld",
            (int)command.left_duty,
            (int)command.right_duty,
            (long)sample.raw_left_delta,
            (long)sample.raw_right_delta);
    APP_LOG("[STATUS] odom_dbg ds_mm=%s%lu dth_mrad=%s%lu dt_ms=%lu skipped=%u reason=%s frozen=%u",
            App_SerialCommand_FixedSign(odom_ds_mm),
            (unsigned long)App_SerialCommand_FixedWhole(odom_ds_mm, 1),
            App_SerialCommand_FixedSign(odom_dtheta_mrad),
            (unsigned long)App_SerialCommand_FixedWhole(odom_dtheta_mrad, 1),
            (unsigned long)App_SerialCommand_FixedWhole(odom_dt_ms, 1),
            (unsigned int)sample.step_skipped,
            AppOdo_SkipReasonName(sample.skip_reason),
            (unsigned int)AppOdo_IsFrozen());
    APP_LOG("[STATUS] pose x=%s%lu.%03lu y=%s%lu.%03lu th=%s%lu.%01lu odo_frozen=%u",
            App_SerialCommand_FixedSign(x_mm),
            (unsigned long)App_SerialCommand_FixedWhole(x_mm, 1000),
            (unsigned long)App_SerialCommand_FixedFraction(x_mm, 1000),
            App_SerialCommand_FixedSign(y_mm),
            (unsigned long)App_SerialCommand_FixedWhole(y_mm, 1000),
            (unsigned long)App_SerialCommand_FixedFraction(y_mm, 1000),
            App_SerialCommand_FixedSign(theta_tenth_deg),
            (unsigned long)App_SerialCommand_FixedWhole(theta_tenth_deg, 10),
            (unsigned long)App_SerialCommand_FixedFraction(theta_tenth_deg, 10),
            (unsigned int)AppOdo_IsFrozen());
    APP_LOG("[STATUS] path count=%u return_state=%s button=%s",
            (unsigned int)path_count,
            ReturnExecutor_StateName(return_state),
            (App_ButtonControl_IsReady() != 0U) ? "ready" : "init");
    APP_LOG("[STATUS] map cell=(%d,%d) heading=%c visited=%u known_edges=%u walls=%u",
            map_summary.robot_cx,
            map_summary.robot_cy,
            AppMap_DirChar(map_summary.heading),
            (unsigned int)map_summary.visited_count,
            (unsigned int)map_summary.known_edges,
            (unsigned int)map_summary.walls);
    APP_LOG("[STATUS] explorer state=%s cell=(%d,%d) target=(%d,%d) path_len=%u mode=%s",
            AppExplorer_StateName(exp_status.state),
            exp_status.current_cx,
            exp_status.current_cy,
            exp_status.target_cx,
            exp_status.target_cy,
            (unsigned int)exp_status.path_len,
            (exp_status.skeleton_only != 0U) ? "skeleton" : "drive");
}

static int32_t App_SerialCommand_ScaleFloatRounded(float value, float multiplier)
{
    float scaled = value * multiplier;

    if (scaled < 0.0f)
    {
        return (int32_t)(scaled - 0.5f);
    }

    return (int32_t)(scaled + 0.5f);
}

static const char *App_SerialCommand_FixedSign(int32_t value)
{
    return (value < 0) ? "-" : "";
}

static uint32_t App_SerialCommand_FixedWhole(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;

    return (uint32_t)(abs_value / decimal_scale);
}

static uint32_t App_SerialCommand_FixedFraction(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;

    return (uint32_t)(abs_value % decimal_scale);
}

static uint32_t App_SerialCommand_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
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
