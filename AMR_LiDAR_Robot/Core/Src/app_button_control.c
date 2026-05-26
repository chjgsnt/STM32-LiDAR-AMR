#include "app_button_control.h"

#include "amr_system.h"
#include "app_explorer.h"
#include "app_fault.h"
#include "app_map.h"
#include "app_odometry.h"
#include "app_return_path.h"
#include "app_safety.h"
#include "bringup_log.h"
#include "chassis.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define APP_BUTTON_DEBOUNCE_MS 50U
#define APP_BUTTON_LONG_PRESS_MS 2000U

#if defined(B1_Pin) && defined(B1_GPIO_Port)
#define APP_BUTTON_GPIO_PORT B1_GPIO_Port
#define APP_BUTTON_GPIO_PIN B1_Pin
#define APP_BUTTON_PIN_NAME "PC13"
#else
#define APP_BUTTON_GPIO_PORT GPIOC
#define APP_BUTTON_GPIO_PIN GPIO_PIN_13
#define APP_BUTTON_PIN_NAME "PC13"
#endif

#define APP_BUTTON_ACTIVE_LEVEL GPIO_PIN_RESET

static uint8_t app_button_initialized = 0U;
static uint8_t app_button_ready = 0U;
static uint8_t app_button_last_raw_active = 0U;
static uint8_t app_button_stable_active = 0U;
static uint8_t app_button_long_reported = 0U;
static uint32_t app_button_raw_changed_ms = 0U;
static uint32_t app_button_press_start_ms = 0U;

static uint8_t App_ButtonControl_ReadActive(void);
static void App_ButtonControl_HandleShortPress(void);
static void App_ButtonControl_HandleLongPress(void);
static uint32_t App_ButtonControl_ElapsedMs(uint32_t now_ms, uint32_t then_ms);

void App_ButtonControl_Init(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t active = App_ButtonControl_ReadActive();

    app_button_last_raw_active = active;
    app_button_stable_active = active;
    app_button_long_reported = 0U;
    app_button_raw_changed_ms = now_ms;
    app_button_press_start_ms = active ? now_ms : 0U;
    app_button_initialized = 1U;
    app_button_ready = 1U;

    APP_LOG("[BTN] init pin=%s active=RESET raw=%u debounce=%u long=%u",
            APP_BUTTON_PIN_NAME,
            (unsigned int)HAL_GPIO_ReadPin(APP_BUTTON_GPIO_PORT, APP_BUTTON_GPIO_PIN),
            (unsigned int)APP_BUTTON_DEBOUNCE_MS,
            (unsigned int)APP_BUTTON_LONG_PRESS_MS);
}

void App_ButtonControl_Update(void)
{
    uint32_t now_ms;
    uint8_t raw_active;

    if (app_button_initialized == 0U)
    {
        App_ButtonControl_Init();
    }

    now_ms = HAL_GetTick();
    raw_active = App_ButtonControl_ReadActive();

    if (raw_active != app_button_last_raw_active)
    {
        app_button_last_raw_active = raw_active;
        app_button_raw_changed_ms = now_ms;
    }

    if ((raw_active != app_button_stable_active) &&
        (App_ButtonControl_ElapsedMs(now_ms, app_button_raw_changed_ms) >= APP_BUTTON_DEBOUNCE_MS))
    {
        app_button_stable_active = raw_active;

        if (app_button_stable_active != 0U)
        {
            app_button_press_start_ms = now_ms;
            app_button_long_reported = 0U;
            APP_LOG("[BTN] pressed");
        }
        else
        {
            uint32_t held_ms = App_ButtonControl_ElapsedMs(now_ms, app_button_press_start_ms);
            if ((app_button_long_reported == 0U) && (held_ms >= APP_BUTTON_DEBOUNCE_MS))
            {
                APP_LOG("[BTN] short_press");
                App_ButtonControl_HandleShortPress();
            }

            app_button_press_start_ms = 0U;
            app_button_long_reported = 0U;
        }
    }

    if ((app_button_stable_active != 0U) &&
        (app_button_long_reported == 0U) &&
        (App_ButtonControl_ElapsedMs(now_ms, app_button_press_start_ms) >= APP_BUTTON_LONG_PRESS_MS))
    {
        app_button_long_reported = 1U;
        APP_LOG("[BTN] long_press");
        App_ButtonControl_HandleLongPress();
    }
}

uint8_t App_ButtonControl_IsReady(void)
{
    return app_button_ready;
}

static uint8_t App_ButtonControl_ReadActive(void)
{
    return (HAL_GPIO_ReadPin(APP_BUTTON_GPIO_PORT, APP_BUTTON_GPIO_PIN) == APP_BUTTON_ACTIVE_LEVEL) ? 1U : 0U;
}

static void App_ButtonControl_HandleShortPress(void)
{
    AMR_State_t state = AMR_GetState();

    switch (state)
    {
        case AMR_STATE_IDLE:
            APP_LOG("[BTN] action=start");
            AMR_RequestStart("button_start");
            AppExplorer_StartExplore();
            break;

        case AMR_STATE_EXPLORE:
        case AMR_STATE_AVOID:
            APP_LOG("[BTN] action=return");
            AppExplorer_StartReturn();
            AMR_RequestReturn("button_return");
            if (AMR_GetState() == AMR_STATE_RETURN)
            {
                ReturnExecutor_Start();
            }
            break;

        case AMR_STATE_RETURN:
            APP_LOG("[BTN] action=stop");
            AppExplorer_Stop();
            ReturnExecutor_Stop("button_stop");
            (void)AMR_SetState(AMR_STATE_IDLE, "button_stop");
            Chassis_Stop();
            break;

        case AMR_STATE_FAULT:
            APP_LOG("[BTN] action=reset_fault");
            App_Safety_ClearFault();
            AppExplorer_Reset();
            AMR_RequestResetFault("button_reset_fault");
            break;

        case AMR_STATE_ESTOP:
            APP_LOG("[BTN] action=reset_estop");
            App_Safety_ClearFault();
            AppExplorer_Reset();
            AMR_RequestResetFault("button_reset_estop");
            break;

        default:
            break;
    }
}

static void App_ButtonControl_HandleLongPress(void)
{
    AMR_State_t state = AMR_GetState();

    if (AppFault_IsActive() ||
        (state == AMR_STATE_FAULT) ||
        (state == AMR_STATE_ESTOP))
    {
        APP_LOG("[BTN] action=clear_fault_reset_odom");
        App_Safety_ClearFault();
        AppExplorer_Reset();
        Odom_Reset();
        AppMap_Reset();
        AMR_RequestResetFault("button_long_clear_fault");
        return;
    }

    APP_LOG("[BTN] action=estop");
    AppExplorer_Stop();
    ReturnExecutor_Stop("button_estop");
    Chassis_Stop();
    AppFault_Set(FAULT_USER_ESTOP);
}

static uint32_t App_ButtonControl_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    if (then_ms == 0U)
    {
        return 0U;
    }

    if (now_ms >= then_ms)
    {
        return now_ms - then_ms;
    }

    return 0U;
}
