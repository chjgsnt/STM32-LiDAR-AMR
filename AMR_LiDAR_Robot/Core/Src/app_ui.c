#include "app_ui.h"

#include "amr_system.h"
#include "app_explorer.h"
#include "app_fault.h"
#include "app_lidar.h"
#include "app_map.h"
#include "app_odometry.h"
#include "app_return_path.h"
#include "app_safety.h"
#include "bringup_log.h"
#include "chassis.h"
#if APP_UI_OLED_ENABLE
#include "i2c.h"
#include "ssd1306.h"
#endif
#include "stm32f4xx_hal.h"

#include <stdio.h>

#ifndef APP_UI_OLED_ENABLE
#define APP_UI_OLED_ENABLE 0
#endif

#ifndef APP_UI_SERIAL_FALLBACK_ENABLE
#define APP_UI_SERIAL_FALLBACK_ENABLE 1
#endif

#ifndef APP_UI_SERIAL_AUTO_PRINT_ENABLE
#define APP_UI_SERIAL_AUTO_PRINT_ENABLE 0
#endif

#ifndef APP_UI_SERIAL_PRINT_PERIOD_MS
#define APP_UI_SERIAL_PRINT_PERIOD_MS 2000U
#endif

#define AMR_UI_UPDATE_MS 200U
#define AMR_UI_OLED_RETRY_MS 2000U
#define AMR_UI_PAGE_COUNT 3U
#define AMR_UI_ADC_SAMPLE_MS 200U
#define AMR_UI_LIDAR_STALE_MS 1000U

#ifndef APP_UI_ADC_ENABLE
#define APP_UI_ADC_ENABLE 0
#endif

#ifndef APP_UI_SPEED_LIMIT_DEFAULT
#define APP_UI_SPEED_LIMIT_DEFAULT 500U
#endif

#ifndef APP_UI_SPEED_LIMIT_MIN
#define APP_UI_SPEED_LIMIT_MIN 200U
#endif

#ifndef APP_UI_SPEED_LIMIT_MAX
#define APP_UI_SPEED_LIMIT_MAX 700U
#endif

#ifndef APP_UI_OBS_THRESHOLD_DEFAULT_MM
#define APP_UI_OBS_THRESHOLD_DEFAULT_MM 450U
#endif

#ifndef APP_UI_OBS_THRESHOLD_MIN_MM
#define APP_UI_OBS_THRESHOLD_MIN_MM 250U
#endif

#ifndef APP_UI_OBS_THRESHOLD_MAX_MM
#define APP_UI_OBS_THRESHOLD_MAX_MM 700U
#endif

#ifndef APP_UI_WALL_THRESHOLD_MIN_MM
#define APP_UI_WALL_THRESHOLD_MIN_MM 300U
#endif

#ifndef APP_UI_WALL_THRESHOLD_MAX_MM
#define APP_UI_WALL_THRESHOLD_MAX_MM 600U
#endif

#define APP_UI_ADC_MAX_RAW 4095U

static uint32_t app_ui_last_update_ms = 0U;
static uint32_t app_ui_last_serial_log_ms = 0U;
static uint32_t app_ui_last_oled_retry_ms = 0U;
static uint32_t app_ui_last_adc_ms = 0U;
static uint32_t app_ui_last_telemetry_ms = 0U;
static uint16_t app_ui_adc_raw = 0U;
static uint16_t app_ui_adc_filtered = 0U;
static uint16_t app_ui_speed_limit = APP_UI_SPEED_LIMIT_DEFAULT;
static uint16_t app_ui_obstacle_threshold_mm = APP_UI_OBS_THRESHOLD_DEFAULT_MM;
static uint16_t app_ui_wall_threshold_mm = (uint16_t)(APP_MAP_WALL_THRESHOLD_M * 1000.0f);
static uint8_t app_ui_page = 0U;
static uint8_t app_ui_oled_ready = 0U;
static uint8_t app_ui_adc_valid = 0U;

static void App_UI_BuildLines(char line1[22], char line2[22], char line3[22], char line4[22]);
static void App_UI_BuildSystemPage(char line1[22], char line2[22], char line3[22], char line4[22]);
static void App_UI_BuildOdomPage(char line1[22], char line2[22], char line3[22], char line4[22]);
static void App_UI_BuildMapPage(char line1[22], char line2[22], char line3[22], char line4[22]);
#if APP_UI_OLED_ENABLE
static void App_UI_TryOledInit(uint32_t now_ms);
static void App_UI_WriteOled(const char *line1, const char *line2, const char *line3, const char *line4);
#endif
static void App_UI_UpdateParameters(uint32_t now_ms);
static uint8_t App_UI_ReadAdcRaw(uint16_t *raw);
static uint16_t App_UI_MapRawToRange(uint16_t raw, uint16_t min_value, uint16_t max_value);
static uint32_t App_UI_ElapsedMs(uint32_t now_ms, uint32_t then_ms);
static int32_t App_UI_ScaleFloatRounded(float value, float multiplier);
static const char *App_UI_FixedSign(int32_t value);
static uint32_t App_UI_FixedWhole(int32_t value, int32_t decimal_scale);
static uint32_t App_UI_FixedFraction(int32_t value, int32_t decimal_scale);
static const char *App_UI_FaultName(void);
static const char *App_UI_LidarState(const AppLidarStatus *lidar, uint32_t now_ms);
static const char *App_UI_ExplorerShortName(AppExplorerState_t state);

void App_UI_Init(void)
{
    app_ui_last_update_ms = 0U;
    app_ui_last_serial_log_ms = 0U;
    app_ui_last_oled_retry_ms = 0U;
    app_ui_last_adc_ms = 0U;
    app_ui_last_telemetry_ms = 0U;
    app_ui_adc_raw = 0U;
    app_ui_adc_filtered = 0U;
    app_ui_speed_limit = APP_UI_SPEED_LIMIT_DEFAULT;
    app_ui_obstacle_threshold_mm = APP_UI_OBS_THRESHOLD_DEFAULT_MM;
    app_ui_wall_threshold_mm = (uint16_t)(APP_MAP_WALL_THRESHOLD_M * 1000.0f);
    app_ui_page = 0U;
    app_ui_oled_ready = 0U;
    app_ui_adc_valid = 0U;

    APP_LOG("[UI] init oled_enable=%u update_ms=%u pages=%u adc_enable=%u",
            (unsigned int)APP_UI_OLED_ENABLE,
            (unsigned int)AMR_UI_UPDATE_MS,
            (unsigned int)AMR_UI_PAGE_COUNT,
            (unsigned int)APP_UI_ADC_ENABLE);
}

void App_UI_Update(void)
{
    uint32_t now_ms = HAL_GetTick();
#if (APP_UI_OLED_ENABLE || (APP_UI_SERIAL_FALLBACK_ENABLE && APP_UI_SERIAL_AUTO_PRINT_ENABLE))
    char line1[22];
    char line2[22];
    char line3[22];
    char line4[22];
#endif

    if ((app_ui_last_update_ms != 0U) &&
        ((now_ms - app_ui_last_update_ms) < AMR_UI_UPDATE_MS))
    {
        return;
    }

    app_ui_last_update_ms = now_ms;
    App_UI_UpdateParameters(now_ms);
#if (APP_UI_OLED_ENABLE || (APP_UI_SERIAL_FALLBACK_ENABLE && APP_UI_SERIAL_AUTO_PRINT_ENABLE))
    App_UI_BuildLines(line1, line2, line3, line4);
#endif

#if APP_UI_OLED_ENABLE
    App_UI_TryOledInit(now_ms);

    if (app_ui_oled_ready != 0U)
    {
        App_UI_WriteOled(line1, line2, line3, line4);
    }
#endif

#if (APP_UI_SERIAL_FALLBACK_ENABLE && APP_UI_SERIAL_AUTO_PRINT_ENABLE)
    if ((app_ui_oled_ready == 0U) &&
        ((app_ui_last_serial_log_ms == 0U) ||
         (App_UI_ElapsedMs(now_ms, app_ui_last_serial_log_ms) >= APP_UI_SERIAL_PRINT_PERIOD_MS)))
    {
        app_ui_last_serial_log_ms = now_ms;
        APP_LOG("[UI] %s | %s | %s | %s", line1, line2, line3, line4);
    }
#endif

#if APP_TELEMETRY_AUTO_ENABLE
    if ((app_ui_last_telemetry_ms == 0U) ||
        (App_UI_ElapsedMs(now_ms, app_ui_last_telemetry_ms) >= APP_TELEMETRY_PERIOD_MS))
    {
        app_ui_last_telemetry_ms = now_ms;
        AppTelemetry_Print();
    }
#endif
}

void AppUI_Init(void)
{
    App_UI_Init();
}

void AppUI_Update(void)
{
    App_UI_Update();
}

void AppUI_SetPage(uint8_t page)
{
    if (page >= AMR_UI_PAGE_COUNT)
    {
        APP_LOG("[UI] page invalid=%u max=%u",
                (unsigned int)page,
                (unsigned int)(AMR_UI_PAGE_COUNT - 1U));
        return;
    }

    app_ui_page = page;
    APP_LOG("[UI] page=%u", (unsigned int)app_ui_page);
}

void AppUI_NextPage(void)
{
    app_ui_page = (uint8_t)((app_ui_page + 1U) % AMR_UI_PAGE_COUNT);
    APP_LOG("[UI] page=%u", (unsigned int)app_ui_page);
}

void AppUI_PrintStatus(void)
{
    char line1[22];
    char line2[22];
    char line3[22];
    char line4[22];

    App_UI_BuildLines(line1, line2, line3, line4);

    APP_LOG("[UI] page=%u oled=%s serial_ui=%s adc=%s raw=%u speed_lim=%u obs_th=%u wall_th=%u",
            (unsigned int)app_ui_page,
            (APP_UI_OLED_ENABLE != 0) ? ((app_ui_oled_ready != 0U) ? "ready" : "init") : "disabled",
            (APP_UI_SERIAL_FALLBACK_ENABLE != 0) ? "enabled" : "disabled",
            (app_ui_adc_valid != 0U) ? "valid" : "default",
            (unsigned int)app_ui_adc_raw,
            (unsigned int)app_ui_speed_limit,
            (unsigned int)app_ui_obstacle_threshold_mm,
            (unsigned int)app_ui_wall_threshold_mm);
    APP_LOG("[UI_PAGE] %s | %s | %s | %s", line1, line2, line3, line4);
}

uint16_t AppUI_GetSpeedLimit(void)
{
    return app_ui_speed_limit;
}

uint16_t AppUI_GetObstacleThreshold(void)
{
    return app_ui_obstacle_threshold_mm;
}

uint16_t AppUI_GetWallThreshold(void)
{
    return app_ui_wall_threshold_mm;
}

uint8_t AppUI_GetPage(void)
{
    return app_ui_page;
}

uint8_t AppUI_IsOledReady(void)
{
    return app_ui_oled_ready;
}

void AppTelemetry_Print(void)
{
    OdomPose_t pose = {0.0f, 0.0f, 0.0f};
    AppMapSummary_t map = {0, 0, APP_MAP_DIR_EAST, 0U, 0U, 0U};
    AppExplorerStatus_t exp = {EXP_IDLE, 0, 0, 0, 0, APP_MAP_DIR_EAST, APP_MAP_DIR_EAST, 0U, 0U, 1U};
    int32_t x_cm;
    int32_t y_cm;
    int32_t th_mrad;

    (void)Odom_GetPose(&pose);
    (void)AppMap_GetSummary(&map);
    (void)AppExplorer_GetStatus(&exp);

    x_cm = App_UI_ScaleFloatRounded(pose.x_m, 100.0f);
    y_cm = App_UI_ScaleFloatRounded(pose.y_m, 100.0f);
    th_mrad = App_UI_ScaleFloatRounded(pose.theta_rad, 1000.0f);

    APP_LOG("TEL: mode=%s fault=%s x=%s%lu.%02lu y=%s%lu.%02lu th=%s%lu.%03lu cell=(%d,%d) exp=%s map_v=%u speed_lim=%u obs_th=%lu.%02lu wall_th=%lu.%02lu",
            AMR_StateShortName(AMR_GetState()),
            App_UI_FaultName(),
            App_UI_FixedSign(x_cm),
            (unsigned long)App_UI_FixedWhole(x_cm, 100),
            (unsigned long)App_UI_FixedFraction(x_cm, 100),
            App_UI_FixedSign(y_cm),
            (unsigned long)App_UI_FixedWhole(y_cm, 100),
            (unsigned long)App_UI_FixedFraction(y_cm, 100),
            App_UI_FixedSign(th_mrad),
            (unsigned long)App_UI_FixedWhole(th_mrad, 1000),
            (unsigned long)App_UI_FixedFraction(th_mrad, 1000),
            map.robot_cx,
            map.robot_cy,
            App_UI_ExplorerShortName(exp.state),
            (unsigned int)map.visited_count,
            (unsigned int)app_ui_speed_limit,
            (unsigned long)(app_ui_obstacle_threshold_mm / 1000U),
            (unsigned long)((app_ui_obstacle_threshold_mm % 1000U) / 10U),
            (unsigned long)(app_ui_wall_threshold_mm / 1000U),
            (unsigned long)((app_ui_wall_threshold_mm % 1000U) / 10U));
}

static void App_UI_BuildLines(char line1[22], char line2[22], char line3[22], char line4[22])
{
    switch (app_ui_page)
    {
        case 1U:
            App_UI_BuildOdomPage(line1, line2, line3, line4);
            break;

        case 2U:
            App_UI_BuildMapPage(line1, line2, line3, line4);
            break;

        case 0U:
        default:
            App_UI_BuildSystemPage(line1, line2, line3, line4);
            break;
    }
}

static void App_UI_BuildSystemPage(char line1[22], char line2[22], char line3[22], char line4[22])
{
    const AppLidarStatus *lidar = App_Lidar_GetStatus();
    AppSafetyStatus_t safety = {APP_FAULT_NONE, 0U, 0U, 0, 0, 0, 0};
    ChassisCommandStatus_t command = {0, 0, 0U};
    AMR_State_t state = AMR_GetState();
    uint32_t now_ms = HAL_GetTick();

    (void)App_Safety_GetStatus(&safety);
    Chassis_GetLastCommand(&command);

    (void)safety;
    if (AppFault_IsActive())
    {
        (void)snprintf(line1, 22U, "FAULT:%s", App_UI_FaultName());
    }
    else
    {
        (void)snprintf(line1, 22U, "MODE:%s F:%s", AMR_StateShortName(state), App_UI_FaultName());
    }

    if ((lidar != NULL) && (lidar->front_valid != 0U))
    {
        (void)snprintf(line2, 22U, "LIDAR:%s F:%u",
                       App_UI_LidarState(lidar, now_ms),
                       (unsigned int)lidar->front_min_mm);
    }
    else
    {
        (void)snprintf(line2, 22U, "LIDAR:%s ADC:%u",
                       App_UI_LidarState(lidar, now_ms),
                       (unsigned int)app_ui_adc_raw);
    }

    (void)snprintf(line3, 22U, "SPD:%u OBS:%umm",
                   (unsigned int)app_ui_speed_limit,
                   (unsigned int)app_ui_obstacle_threshold_mm);

    if (state == AMR_STATE_RETURN)
    {
        (void)snprintf(line4, 22U, "RET:%u PWM:%d",
                       (unsigned int)ReturnPath_Count(),
                       (int)command.left_duty);
    }
    else
    {
        (void)snprintf(line4, 22U, "PWM:L%d R%d",
                       (int)command.left_duty,
                       (int)command.right_duty);
    }
}

static void App_UI_BuildOdomPage(char line1[22], char line2[22], char line3[22], char line4[22])
{
    OdomPose_t pose = {0.0f, 0.0f, 0.0f};
    float v = 0.0f;
    float w = 0.0f;
    int32_t x_cm;
    int32_t y_cm;
    int32_t th_deg;
    int32_t v_cmps;
    int32_t w_mradps;

    (void)Odom_GetPose(&pose);
    AppOdo_GetVelocity(&v, &w);

    x_cm = App_UI_ScaleFloatRounded(pose.x_m, 100.0f);
    y_cm = App_UI_ScaleFloatRounded(pose.y_m, 100.0f);
    th_deg = App_UI_ScaleFloatRounded(pose.theta_rad * 57.2957795f, 1.0f);
    v_cmps = App_UI_ScaleFloatRounded(v, 100.0f);
    w_mradps = App_UI_ScaleFloatRounded(w, 1000.0f);

    (void)snprintf(line1, 22U, "ODO PAGE");
    (void)snprintf(line2, 22U, "X:%s%lu.%02lu Y:%s%lu.%02lu",
                   App_UI_FixedSign(x_cm),
                   (unsigned long)App_UI_FixedWhole(x_cm, 100),
                   (unsigned long)App_UI_FixedFraction(x_cm, 100),
                   App_UI_FixedSign(y_cm),
                   (unsigned long)App_UI_FixedWhole(y_cm, 100),
                   (unsigned long)App_UI_FixedFraction(y_cm, 100));
    (void)snprintf(line3, 22U, "TH:%s%luDEG",
                   App_UI_FixedSign(th_deg),
                   (unsigned long)App_UI_FixedWhole(th_deg, 1));
    (void)snprintf(line4, 22U, "V:%s%lu.%02lu W:%s%lu",
                   App_UI_FixedSign(v_cmps),
                   (unsigned long)App_UI_FixedWhole(v_cmps, 100),
                   (unsigned long)App_UI_FixedFraction(v_cmps, 100),
                   App_UI_FixedSign(w_mradps),
                   (unsigned long)App_UI_FixedWhole(w_mradps, 1));
}

static void App_UI_BuildMapPage(char line1[22], char line2[22], char line3[22], char line4[22])
{
    AppMapSummary_t map = {0, 0, APP_MAP_DIR_EAST, 0U, 0U, 0U};
    AppExplorerStatus_t exp = {EXP_IDLE, 0, 0, 0, 0, APP_MAP_DIR_EAST, APP_MAP_DIR_EAST, 0U, 0U, 1U};

    (void)AppMap_GetSummary(&map);
    (void)AppExplorer_GetStatus(&exp);

    (void)snprintf(line1, 22U, "MAP CELL:(%d,%d)", map.robot_cx, map.robot_cy);
    (void)snprintf(line2, 22U, "HEAD:%c VIS:%u",
                   AppMap_DirChar(map.heading),
                   (unsigned int)map.visited_count);
    (void)snprintf(line3, 22U, "EXP:%s", App_UI_ExplorerShortName(exp.state));
    (void)snprintf(line4, 22U, "PATH:%u WALL:%u",
                   (unsigned int)exp.path_len,
                   (unsigned int)map.walls);
}

#if APP_UI_OLED_ENABLE
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
#endif

static void App_UI_UpdateParameters(uint32_t now_ms)
{
    uint16_t raw;

    if ((app_ui_last_adc_ms != 0U) &&
        (App_UI_ElapsedMs(now_ms, app_ui_last_adc_ms) < AMR_UI_ADC_SAMPLE_MS))
    {
        return;
    }

    app_ui_last_adc_ms = now_ms;
    if (App_UI_ReadAdcRaw(&raw) == 0U)
    {
        app_ui_adc_valid = 0U;
        return;
    }

    app_ui_adc_raw = raw;
    if (app_ui_adc_valid == 0U)
    {
        app_ui_adc_filtered = raw;
        app_ui_adc_valid = 1U;
    }
    else
    {
        app_ui_adc_filtered = (uint16_t)((((uint32_t)app_ui_adc_filtered * 3U) + raw) / 4U);
    }

    app_ui_speed_limit = App_UI_MapRawToRange(app_ui_adc_filtered,
                                             APP_UI_SPEED_LIMIT_MIN,
                                             APP_UI_SPEED_LIMIT_MAX);
    app_ui_obstacle_threshold_mm = App_UI_MapRawToRange(app_ui_adc_filtered,
                                                       APP_UI_OBS_THRESHOLD_MIN_MM,
                                                       APP_UI_OBS_THRESHOLD_MAX_MM);
    app_ui_wall_threshold_mm = App_UI_MapRawToRange(app_ui_adc_filtered,
                                                   APP_UI_WALL_THRESHOLD_MIN_MM,
                                                   APP_UI_WALL_THRESHOLD_MAX_MM);
}

static uint8_t App_UI_ReadAdcRaw(uint16_t *raw)
{
#if APP_UI_ADC_ENABLE
    uint32_t timeout;

    if (raw == NULL)
    {
        return 0U;
    }

    if ((RCC->APB2ENR & RCC_APB2ENR_ADC1EN) == 0U)
    {
        return 0U;
    }

    if ((ADC1->CR2 & ADC_CR2_ADON) == 0U)
    {
        return 0U;
    }

    ADC1->SR &= ~(ADC_SR_EOC | ADC_SR_OVR);
    ADC1->CR2 |= ADC_CR2_SWSTART;

    for (timeout = 0U; timeout < 10000U; timeout++)
    {
        if ((ADC1->SR & ADC_SR_EOC) != 0U)
        {
            *raw = (uint16_t)(ADC1->DR & 0x0FFFU);
            return 1U;
        }
    }
#else
    (void)raw;
#endif

    return 0U;
}

static uint16_t App_UI_MapRawToRange(uint16_t raw, uint16_t min_value, uint16_t max_value)
{
    uint32_t span;

    if (max_value <= min_value)
    {
        return min_value;
    }

    span = (uint32_t)max_value - (uint32_t)min_value;
    return (uint16_t)((uint32_t)min_value + (((uint32_t)raw * span) / APP_UI_ADC_MAX_RAW));
}

static uint32_t App_UI_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
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

static int32_t App_UI_ScaleFloatRounded(float value, float multiplier)
{
    float scaled = value * multiplier;

    if (scaled < 0.0f)
    {
        return (int32_t)(scaled - 0.5f);
    }

    return (int32_t)(scaled + 0.5f);
}

static const char *App_UI_FixedSign(int32_t value)
{
    return (value < 0) ? "-" : "";
}

static uint32_t App_UI_FixedWhole(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;

    return (uint32_t)(abs_value / decimal_scale);
}

static uint32_t App_UI_FixedFraction(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;

    return (uint32_t)(abs_value % decimal_scale);
}

static const char *App_UI_FaultName(void)
{
    AppSafetyStatus_t safety = {APP_FAULT_NONE, 0U, 0U, 0, 0, 0, 0};

    if (AppFault_IsActive())
    {
        return AppFault_Name(AppFault_Get());
    }

    (void)App_Safety_GetStatus(&safety);
    return App_Safety_FaultName(safety.fault_code);
}

static const char *App_UI_LidarState(const AppLidarStatus *lidar, uint32_t now_ms)
{
    if ((lidar == NULL) || (lidar->ready == 0U))
    {
        return "--";
    }

    if (App_UI_ElapsedMs(now_ms, lidar->last_valid_update_ms) > AMR_UI_LIDAR_STALE_MS)
    {
        return "STALE";
    }

    return "OK";
}

static const char *App_UI_ExplorerShortName(AppExplorerState_t state)
{
    switch (state)
    {
        case EXP_IDLE:
            return "IDLE";

        case EXP_EXPLORE:
            return "EXP";

        case EXP_TURNING:
            return "TURN";

        case EXP_MOVING_TO_CELL:
            return "MOVE";

        case EXP_EXIT_FOUND:
            return "EXIT";

        case EXP_RETURNING:
            return "RET";

        case EXP_DONE:
            return "DONE";

        case EXP_FAULT:
            return "FAULT";

        default:
            return "?";
    }
}

#if APP_UI_OLED_ENABLE
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
#endif
