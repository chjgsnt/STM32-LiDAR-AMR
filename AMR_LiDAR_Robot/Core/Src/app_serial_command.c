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

static char app_cmd_line[APP_CMD_LINE_MAX];
static uint8_t app_cmd_len = 0U;
static uint8_t app_cmd_initialized = 0U;

static void App_SerialCommand_PollRx(void);
static void App_SerialCommand_ProcessChar(char ch);
static void App_SerialCommand_HandleLine(const char *line);
static char App_SerialCommand_ToLower(char ch);
static void App_SerialCommand_LogStatus(void);
static int32_t App_SerialCommand_ScaleFloatRounded(float value, float multiplier);
static const char *App_SerialCommand_FixedSign(int32_t value);
static uint32_t App_SerialCommand_FixedWhole(int32_t value, int32_t decimal_scale);
static uint32_t App_SerialCommand_FixedFraction(int32_t value, int32_t decimal_scale);

void App_SerialCommand_Init(void)
{
    app_cmd_len = 0U;
    app_cmd_initialized = 1U;
    APP_LOG("[CMD] ready commands=start stop return estop reset_fault odom_reset status");
}

void App_SerialCommand_Task(void)
{
    if (app_cmd_initialized == 0U)
    {
        App_SerialCommand_Init();
    }

    App_SerialCommand_PollRx();
}

static void App_SerialCommand_PollRx(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET)
    {
        volatile uint32_t clear_sr = huart2.Instance->SR;
        volatile uint32_t clear_dr = huart2.Instance->DR;
        (void)clear_sr;
        (void)clear_dr;
    }

    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
    {
        char ch = (char)(huart2.Instance->DR & 0xFFU);
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
            App_SerialCommand_HandleLine(app_cmd_line);
            app_cmd_len = 0U;
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
    }
    else
    {
        app_cmd_len = 0U;
        APP_LOG("[CMD] line too long, discarded");
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

static char App_SerialCommand_ToLower(char ch)
{
    if ((ch >= 'A') && (ch <= 'Z'))
    {
        return (char)(ch + ('a' - 'A'));
    }

    return ch;
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

    (void)App_Safety_GetStatus(&safety);
    (void)Odom_GetPose(&pose);
    (void)Odom_GetLastSample(&sample);
    Chassis_GetLastCommand(&command);

    x_mm = App_SerialCommand_ScaleFloatRounded(pose.x_m, 1000.0f);
    y_mm = App_SerialCommand_ScaleFloatRounded(pose.y_m, 1000.0f);
    theta_tenth_deg = App_SerialCommand_ScaleFloatRounded(pose.theta_rad * 57.2957795f, 10.0f);

    APP_LOG("[STATUS] state=%s fault=%s pose=%s%lu.%03lu,%s%lu.%03lu,%s%lu.%01ludeg lidar_ready=%u front_valid=%u front_mm=%u last_lidar=%lu pwmL=%d pwmR=%d rawL=%ld rawR=%ld",
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
            (unsigned long)((lidar != NULL) ? lidar->last_update_ms : 0U),
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
