#include "app_ui.h"

#include "amr_system.h"
#include "app_lidar.h"
#include "app_odometry.h"
#include "app_return_path.h"
#include "app_safety.h"
#include "bringup_log.h"
#include "chassis.h"
#include "i2c.h"
#include "ssd1306.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>

#ifndef AMR_UI_ENABLE_OLED
#define AMR_UI_ENABLE_OLED 1
#endif

#define AMR_UI_UPDATE_MS 200U
#define AMR_UI_SERIAL_LOG_MS 2000U
#define AMR_UI_OLED_RETRY_MS 2000U

static uint32_t app_ui_last_update_ms = 0U;
static uint32_t app_ui_last_serial_log_ms = 0U;
static uint32_t app_ui_last_oled_retry_ms = 0U;
static uint8_t app_ui_oled_ready = 0U;

static void App_UI_BuildLines(char line1[22], char line2[22], char line3[22], char line4[22]);
static void App_UI_TryOledInit(uint32_t now_ms);
static void App_UI_WriteOled(const char *line1, const char *line2, const char *line3, const char *line4);

void App_UI_Init(void)
{
    app_ui_last_update_ms = 0U;
    app_ui_last_serial_log_ms = 0U;
    app_ui_last_oled_retry_ms = 0U;
    app_ui_oled_ready = 0U;

    APP_LOG("[UI] init oled_enable=%u update_ms=%u",
            (unsigned int)AMR_UI_ENABLE_OLED,
            (unsigned int)AMR_UI_UPDATE_MS);
}

void App_UI_Update(void)
{
    uint32_t now_ms = HAL_GetTick();
    char line1[22];
    char line2[22];
    char line3[22];
    char line4[22];

    if ((app_ui_last_update_ms != 0U) &&
        ((now_ms - app_ui_last_update_ms) < AMR_UI_UPDATE_MS))
    {
        return;
    }

    app_ui_last_update_ms = now_ms;
    App_UI_BuildLines(line1, line2, line3, line4);

#if AMR_UI_ENABLE_OLED
    App_UI_TryOledInit(now_ms);

    if (app_ui_oled_ready != 0U)
    {
        App_UI_WriteOled(line1, line2, line3, line4);
    }
#endif

    if ((app_ui_oled_ready == 0U) &&
        ((app_ui_last_serial_log_ms == 0U) ||
         ((now_ms - app_ui_last_serial_log_ms) >= AMR_UI_SERIAL_LOG_MS)))
    {
        app_ui_last_serial_log_ms = now_ms;
        APP_LOG("[UI] %s | %s | %s | %s", line1, line2, line3, line4);
    }
}

static void App_UI_BuildLines(char line1[22], char line2[22], char line3[22], char line4[22])
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    AppSafetyStatus_t safety = {APP_FAULT_NONE, 0U, 0U, 0, 0, 0, 0};
    OdomSample_t sample = {0};
    ChassisCommandStatus_t command = {0, 0, 0U};
    AMR_State_t state = AMR_GetState();

    (void)App_Safety_GetStatus(&safety);
    (void)Odom_GetLastSample(&sample);
    Chassis_GetLastCommand(&command);

    (void)snprintf(line1, 22U, "MODE:%s", AMR_StateShortName(state));

    if (safety.fault_code == APP_FAULT_LIDAR_TIMEOUT)
    {
        (void)snprintf(line2, 22U, "LIDAR:TO");
    }
    else if ((lidar != NULL) && (lidar->front_valid != 0U))
    {
        (void)snprintf(line2, 22U, "FRONT:%uMM", (unsigned int)lidar->front_min_mm);
    }
    else
    {
        (void)snprintf(line2, 22U, "LIDAR:%s", ((lidar != NULL) && (lidar->ready != 0U)) ? "OK" : "--");
    }

    (void)snprintf(line3, 22U, "ENC:L%ld R%ld",
                   (long)sample.raw_left_delta,
                   (long)sample.raw_right_delta);
    if (state == AMR_STATE_RETURN)
    {
        (void)snprintf(line4, 22U, "RET:%u",
                       (unsigned int)ReturnPath_Count());
    }
    else
    {
        (void)snprintf(line4, 22U, "PWM:L%d R%d",
                       (int)command.left_duty,
                       (int)command.right_duty);
    }
}

static void App_UI_TryOledInit(uint32_t now_ms)
{
    if (app_ui_oled_ready != 0U)
    {
        return;
    }

    if ((app_ui_last_oled_retry_ms != 0U) &&
        ((now_ms - app_ui_last_oled_retry_ms) < AMR_UI_OLED_RETRY_MS))
    {
        return;
    }

    app_ui_last_oled_retry_ms = now_ms;

    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
        return;
    }

    if (SSD1306_Init(&hi2c1, SSD1306_I2C_ADDR_7BIT) == true)
    {
        app_ui_oled_ready = 1U;
        APP_LOG("[UI] OLED ready");
    }
}

static void App_UI_WriteOled(const char *line1, const char *line2, const char *line3, const char *line4)
{
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
        return;
    }

    if ((SSD1306_Clear() == false) ||
        (SSD1306_WriteString(0U, 0U, line1) == false) ||
        (SSD1306_WriteString(0U, 2U, line2) == false) ||
        (SSD1306_WriteString(0U, 4U, line3) == false) ||
        (SSD1306_WriteString(0U, 6U, line4) == false) ||
        (SSD1306_UpdateScreen() == false))
    {
        app_ui_oled_ready = 0U;
        APP_LOG("[UI] OLED update failed, serial fallback active");
    }
}
