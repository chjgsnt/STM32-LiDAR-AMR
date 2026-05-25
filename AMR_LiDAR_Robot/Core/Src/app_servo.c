#include "app_servo.h"

#include "bringup_log.h"

#include <stdint.h>

static TIM_HandleTypeDef app_servo_htim1;
static uint8_t app_servo_initialized = 0U;

static uint32_t App_Servo_GetTim1ClockHz(void);
static uint16_t App_Servo_ClampPulseUs(uint16_t pulse_us);

void App_Servo_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};
    TIM_MasterConfigTypeDef master = {0};
    uint32_t tim_clock_hz = App_Servo_GetTim1ClockHz();

    APP_SERVO_PWM_GPIO_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    HAL_GPIO_WritePin(APP_SERVO_PWM_GPIO_PORT, APP_SERVO_PWM_GPIO_PIN, GPIO_PIN_RESET);

    gpio.Pin = APP_SERVO_PWM_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = APP_SERVO_PWM_GPIO_AF;
    HAL_GPIO_Init(APP_SERVO_PWM_GPIO_PORT, &gpio);

    app_servo_htim1.Instance = TIM1;
    app_servo_htim1.Init.Prescaler = (tim_clock_hz / 1000000U) - 1U;
    app_servo_htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    app_servo_htim1.Init.Period = APP_SERVO_PERIOD_US - 1U;
    app_servo_htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    app_servo_htim1.Init.RepetitionCounter = 0U;
    app_servo_htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&app_servo_htim1) != HAL_OK)
    {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&app_servo_htim1, &master) != HAL_OK)
    {
        Error_Handler();
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = APP_SERVO_CENTER_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&app_servo_htim1, &oc, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&app_servo_htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    app_servo_initialized = 1U;
    App_Servo_Center();

    APP_LOG("APP SERVO: init pwm=%s period_us=%u center_us=%u left_us=%u right_us=%u",
            APP_SERVO_PWM_TIMER_NAME,
            (unsigned int)APP_SERVO_PERIOD_US,
            (unsigned int)APP_SERVO_CENTER_US,
            (unsigned int)APP_SERVO_LEFT_US,
            (unsigned int)APP_SERVO_RIGHT_US);
}

void App_Servo_Center(void)
{
    App_Servo_SetPulseUs(APP_SERVO_CENTER_US);
}

void App_Servo_Left(void)
{
    App_Servo_SetPulseUs(APP_SERVO_LEFT_US);
}

void App_Servo_Right(void)
{
    App_Servo_SetPulseUs(APP_SERVO_RIGHT_US);
}

void App_Servo_SetPulseUs(uint16_t pulse_us)
{
    if (app_servo_initialized == 0U)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(&app_servo_htim1, TIM_CHANNEL_1, App_Servo_ClampPulseUs(pulse_us));
}

static uint32_t App_Servo_GetTim1ClockHz(void)
{
    uint32_t pclk2_hz = HAL_RCC_GetPCLK2Freq();
    uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2);

    if (ppre2 != RCC_CFGR_PPRE2_DIV1)
    {
        return pclk2_hz * 2U;
    }

    return pclk2_hz;
}

static uint16_t App_Servo_ClampPulseUs(uint16_t pulse_us)
{
    if (pulse_us < APP_SERVO_MIN_US)
    {
        return APP_SERVO_MIN_US;
    }

    if (pulse_us > APP_SERVO_MAX_US)
    {
        return APP_SERVO_MAX_US;
    }

    return pulse_us;
}
