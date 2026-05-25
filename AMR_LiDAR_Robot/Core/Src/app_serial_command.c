#include "app_serial_command.h"

#include "amr_system.h"
#include "app_lidar.h"
#include "app_odometry.h"
#include "app_safety.h"
#include "bringup_log.h"
#include "chassis.h"
#include "usart.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define APP_CMD_LINE_MAX 40U
#define APP_CMD_RX_QUEUE_SIZE 64U
#define APP_CMD_NO_NEWLINE_TIMEOUT_MS 250U

static char app_cmd_line[APP_CMD_LINE_MAX];
static uint8_t app_cmd_len = 0U;
static uint8_t app_cmd_initialized = 0U;
static uint8_t app_cmd_rx_byte = 0U;
static volatile uint8_t app_cmd_rx_queue[APP_CMD_RX_QUEUE_SIZE];
static volatile uint8_t app_cmd_rx_head = 0U;
static volatile uint8_t app_cmd_rx_tail = 0U;
static volatile uint8_t app_cmd_rx_overflow = 0U;
static uint32_t app_cmd_last_char_ms = 0U;

static HAL_StatusTypeDef App_SerialCommand_StartRx(void);
static void App_SerialCommand_DrainRxQueue(void);
static void App_SerialCommand_ProcessChar(char ch);
static void App_SerialCommand_CheckNoNewlineCommand(uint32_t now_ms);
static void App_SerialCommand_HandleLine(const char *line);
static uint8_t App_SerialCommand_IsKnownCommand(const char *line);
static char App_SerialCommand_ToLower(char ch);
static void App_SerialCommand_LogRxChar(char ch);
static void App_SerialCommand_LogStatus(void);
static int32_t App_SerialCommand_ScaleFloatRounded(float value, float multiplier);
static const char *App_SerialCommand_FixedSign(int32_t value);
static uint32_t App_SerialCommand_FixedWhole(int32_t value, int32_t decimal_scale);
static uint32_t App_SerialCommand_FixedFraction(int32_t value, int32_t decimal_scale);

void App_SerialCommand_Init(void)
{
    HAL_StatusTypeDef rx_status;

    app_cmd_len = 0U;
    app_cmd_rx_head = 0U;
    app_cmd_rx_tail = 0U;
    app_cmd_rx_overflow = 0U;
    app_cmd_last_char_ms = 0U;
    app_cmd_initialized = 1U;

    rx_status = App_SerialCommand_StartRx();
    if ((rx_status == HAL_OK) || (rx_status == HAL_BUSY))
    {
        APP_LOG("[CMD] ready uart=huart2 commands=start stop return estop reset_fault odom_reset status");
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

    App_SerialCommand_DrainRxQueue();
    App_SerialCommand_CheckNoNewlineCommand(HAL_GetTick());

    if (huart2.RxState != HAL_UART_STATE_BUSY_RX)
    {
        (void)App_SerialCommand_StartRx();
    }
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
            APP_LOG("[CMD_RX] line=%s", app_cmd_line);
            App_SerialCommand_HandleLine(app_cmd_line);
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
        app_cmd_len = 0U;
        app_cmd_last_char_ms = 0U;
        APP_LOG("[CMD] line too long, discarded");
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
        APP_LOG("[CMD_RX] line=%s", app_cmd_line);
        App_SerialCommand_HandleLine(app_cmd_line);
        app_cmd_len = 0U;
        app_cmd_last_char_ms = 0U;
    }
}

static void App_SerialCommand_HandleLine(const char *line)
{
    if (line == NULL)
    {
        return;
    }

    if (strcmp(line, "start") == 0)
    {
        AMR_RequestStart("serial_start");
    }
    else if (strcmp(line, "stop") == 0)
    {
        AMR_RequestStop("serial_stop");
    }
    else if (strcmp(line, "return") == 0)
    {
        AMR_RequestReturn("serial_return");
    }
    else if (strcmp(line, "estop") == 0)
    {
        AMR_RequestEStop("serial_estop");
    }
    else if (strcmp(line, "reset_fault") == 0)
    {
        App_Safety_ClearFault();
        AMR_RequestResetFault("serial_reset_fault");
    }
    else if (strcmp(line, "odom_reset") == 0)
    {
        Odom_Reset();
    }
    else if (strcmp(line, "status") == 0)
    {
        App_SerialCommand_LogStatus();
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
            (strcmp(line, "stop") == 0) ||
            (strcmp(line, "return") == 0) ||
            (strcmp(line, "estop") == 0) ||
            (strcmp(line, "reset_fault") == 0) ||
            (strcmp(line, "odom_reset") == 0) ||
            (strcmp(line, "status") == 0)) ? 1U : 0U;
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
    uint32_t lidar_last_update_ms;
    uint32_t lidar_age_ms;

    (void)App_Safety_GetStatus(&safety);
    (void)Odom_GetPose(&pose);
    (void)Odom_GetLastSample(&sample);
    Chassis_GetLastCommand(&command);

    now_ms = HAL_GetTick();
    lidar_last_update_ms = (lidar != NULL) ? lidar->last_update_ms : 0U;
    lidar_age_ms = (lidar != NULL) ? (now_ms - lidar_last_update_ms) : 0U;

    x_mm = App_SerialCommand_ScaleFloatRounded(pose.x_m, 1000.0f);
    y_mm = App_SerialCommand_ScaleFloatRounded(pose.y_m, 1000.0f);
    theta_tenth_deg = App_SerialCommand_ScaleFloatRounded(pose.theta_rad * 57.2957795f, 10.0f);

    APP_LOG("[STATUS] state=%s fault=%s pose=%s%lu.%03lu,%s%lu.%03lu,%s%lu.%01ludeg lidar_ready=%u front_valid=%u front_mm=%u lidar_last_update=%lu lidar_age=%lu pwmL=%d pwmR=%d rawL=%ld rawR=%ld",
            AMR_StateName(state),
            App_Safety_FaultName(safety.fault_code),
            App_SerialCommand_FixedSign(x_mm),
            (unsigned long)App_SerialCommand_FixedWhole(x_mm, 1000),
            (unsigned long)App_SerialCommand_FixedFraction(x_mm, 1000),
            App_SerialCommand_FixedSign(y_mm),
            (unsigned long)App_SerialCommand_FixedWhole(y_mm, 1000),
            (unsigned long)App_SerialCommand_FixedFraction(y_mm, 1000),
            App_SerialCommand_FixedSign(theta_tenth_deg),
            (unsigned long)App_SerialCommand_FixedWhole(theta_tenth_deg, 10),
            (unsigned long)App_SerialCommand_FixedFraction(theta_tenth_deg, 10),
            (unsigned int)((lidar != NULL) ? lidar->ready : 0U),
            (unsigned int)((lidar != NULL) ? lidar->front_valid : 0U),
            (unsigned int)((lidar != NULL) ? lidar->front_min_mm : 0U),
            (unsigned long)lidar_last_update_ms,
            (unsigned long)lidar_age_ms,
            (int)command.left_duty,
            (int)command.right_duty,
            (long)sample.raw_left_delta,
            (long)sample.raw_right_delta);
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
