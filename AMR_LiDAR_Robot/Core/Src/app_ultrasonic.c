#include "app_ultrasonic.h"

#include "bringup_log.h"

#include <stdint.h>

#define APP_ULTRASONIC_TRIGGER_PULSE_US 12U
#define APP_ULTRASONIC_ECHO_START_TIMEOUT_US 5000U
#define APP_ULTRASONIC_ECHO_HIGH_TIMEOUT_US 30000U
#define APP_ULTRASONIC_CM_DIVISOR_US 58U

static void App_Ultrasonic_EnableCycleCounter(void);
static uint32_t App_Ultrasonic_Micros(void);
static void App_Ultrasonic_DelayUs(uint32_t delay_us);
static uint8_t App_Ultrasonic_WaitForEcho(GPIO_PinState state, uint32_t timeout_us);
static uint8_t App_Ultrasonic_ReturnNoEchoDistance(uint16_t *distance_cm);

void App_Ultrasonic_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    APP_ULTRASONIC_TRIG_GPIO_CLK_ENABLE();
    APP_ULTRASONIC_ECHO_GPIO_CLK_ENABLE();
    App_Ultrasonic_EnableCycleCounter();

    HAL_GPIO_WritePin(APP_ULTRASONIC_TRIG_GPIO_PORT, APP_ULTRASONIC_TRIG_GPIO_PIN, GPIO_PIN_RESET);

    gpio.Pin = APP_ULTRASONIC_TRIG_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(APP_ULTRASONIC_TRIG_GPIO_PORT, &gpio);

    gpio.Pin = APP_ULTRASONIC_ECHO_GPIO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(APP_ULTRASONIC_ECHO_GPIO_PORT, &gpio);

    APP_LOG("APP ULTRASONIC: ultrasonic init trig=%s echo=%s max_cm=%u no_echo_as_max=%u",
            APP_ULTRASONIC_TRIG_PIN_NAME,
            APP_ULTRASONIC_ECHO_PIN_NAME,
            (unsigned int)APP_ULTRASONIC_MAX_DISTANCE_CM,
            (unsigned int)APP_ULTRASONIC_NO_ECHO_AS_MAX_CM);
}

uint8_t App_Ultrasonic_ReadDistanceCm(uint16_t *distance_cm)
{
    uint32_t pulse_start_us = 0U;
    uint32_t pulse_width_us = 0U;
    uint32_t measured_cm = 0U;

    if (distance_cm == NULL)
    {
        return 0U;
    }

    *distance_cm = APP_ULTRASONIC_INVALID_CM;

    if (App_Ultrasonic_WaitForEcho(GPIO_PIN_RESET, APP_ULTRASONIC_ECHO_HIGH_TIMEOUT_US) == 0U)
    {
        return 0U;
    }

    HAL_GPIO_WritePin(APP_ULTRASONIC_TRIG_GPIO_PORT, APP_ULTRASONIC_TRIG_GPIO_PIN, GPIO_PIN_RESET);
    App_Ultrasonic_DelayUs(2U);
    HAL_GPIO_WritePin(APP_ULTRASONIC_TRIG_GPIO_PORT, APP_ULTRASONIC_TRIG_GPIO_PIN, GPIO_PIN_SET);
    App_Ultrasonic_DelayUs(APP_ULTRASONIC_TRIGGER_PULSE_US);
    HAL_GPIO_WritePin(APP_ULTRASONIC_TRIG_GPIO_PORT, APP_ULTRASONIC_TRIG_GPIO_PIN, GPIO_PIN_RESET);

    if (App_Ultrasonic_WaitForEcho(GPIO_PIN_SET, APP_ULTRASONIC_ECHO_START_TIMEOUT_US) == 0U)
    {
        return App_Ultrasonic_ReturnNoEchoDistance(distance_cm);
    }

    pulse_start_us = App_Ultrasonic_Micros();

    if (App_Ultrasonic_WaitForEcho(GPIO_PIN_RESET, APP_ULTRASONIC_ECHO_HIGH_TIMEOUT_US) == 0U)
    {
        return App_Ultrasonic_ReturnNoEchoDistance(distance_cm);
    }

    pulse_width_us = App_Ultrasonic_Micros() - pulse_start_us;
    measured_cm = pulse_width_us / APP_ULTRASONIC_CM_DIVISOR_US;

    if ((measured_cm == 0U) || (measured_cm > APP_ULTRASONIC_MAX_DISTANCE_CM))
    {
        return 0U;
    }

    *distance_cm = (uint16_t)measured_cm;
    return 1U;
}

static void App_Ultrasonic_EnableCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t App_Ultrasonic_Micros(void)
{
    uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();

    if (hclk_hz == 0U)
    {
        return 0U;
    }

    return DWT->CYCCNT / (hclk_hz / 1000000U);
}

static void App_Ultrasonic_DelayUs(uint32_t delay_us)
{
    uint32_t start_us = App_Ultrasonic_Micros();

    while ((App_Ultrasonic_Micros() - start_us) < delay_us)
    {
    }
}

static uint8_t App_Ultrasonic_WaitForEcho(GPIO_PinState state, uint32_t timeout_us)
{
    uint32_t start_us = App_Ultrasonic_Micros();

    while (HAL_GPIO_ReadPin(APP_ULTRASONIC_ECHO_GPIO_PORT, APP_ULTRASONIC_ECHO_GPIO_PIN) != state)
    {
        if ((App_Ultrasonic_Micros() - start_us) >= timeout_us)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t App_Ultrasonic_ReturnNoEchoDistance(uint16_t *distance_cm)
{
#if (APP_ULTRASONIC_NO_ECHO_AS_MAX_CM != 0)
    if (distance_cm != NULL)
    {
        *distance_cm = APP_ULTRASONIC_MAX_DISTANCE_CM;
    }

    return 1U;
#else
    (void)distance_cm;
    return 0U;
#endif
}
