#define MOTOR_DRIVER_INTERNAL_IMPLEMENTATION

#include "motor_driver.h"

#include "bringup_log.h"
#include "tim.h"

#ifndef MOTOR_DRIVER_ADC1_IN2_GPIO_INIT
#define MOTOR_DRIVER_ADC1_IN2_GPIO_INIT 0
#endif

#define MOTOR_DRIVER_PWM_SCALE (1000U)
#define MOTOR_DRIVER_ADC1_CHANNEL (2U)
#define MOTOR_DRIVER_STOP_LOG_INTERVAL_MS 500U
#define MOTOR_DRIVER_DUTY_LOG_INTERVAL_MS 500U
#define MOTOR_DRIVER_VERBOSE_LOGS (APP_DEBUG_VERBOSE || APP_DEBUG_MOTOR_VERBOSE)

static int16_t MotorDriver_ClampDuty(int16_t duty);
static uint32_t MotorDriver_DutyToCompare(int16_t duty);
static int16_t MotorDriver_SetDualPwm(uint32_t forward_channel,
                                      uint32_t reverse_channel,
                                      int16_t duty);
static void MotorDriver_InitAdc1In2Placeholder(void);
static void MotorDriver_LogStopAllExecuted(void);
static void MotorDriver_LogFinalDuty(char motor_name, int16_t duty);

void MotorDriver_Init(void)
{
    MotorDriver_InitAdc1In2Placeholder();

    MotorDriver_StopAll();

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }

    MotorDriver_ResetEncoders();

    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }
}

void MotorDriver_StopAll(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
    MotorDriver_LogStopAllExecuted();
}

void MotorDriver_SetMotorA(int16_t duty)
{
    int16_t final_duty = MotorDriver_SetDualPwm(TIM_CHANNEL_1, TIM_CHANNEL_2, duty);
    MotorDriver_LogFinalDuty('A', final_duty);
}

void MotorDriver_SetMotorB(int16_t duty)
{
    int16_t final_duty = MotorDriver_SetDualPwm(TIM_CHANNEL_3, TIM_CHANNEL_4, duty);
    MotorDriver_LogFinalDuty('B', final_duty);
}

int32_t MotorDriver_GetEncoderA(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

int32_t MotorDriver_GetEncoderB(void)
{
    return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

void MotorDriver_ResetEncoders(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
}

static int16_t MotorDriver_ClampDuty(int16_t duty)
{
    if (duty > MOTOR_DRIVER_DUTY_MAX)
    {
        duty = MOTOR_DRIVER_DUTY_MAX;
    }
    else if (duty < MOTOR_DRIVER_DUTY_MIN)
    {
        duty = MOTOR_DRIVER_DUTY_MIN;
    }

    if (duty > MOTOR_DRIVER_BRINGUP_DUTY_LIMIT)
    {
        duty = MOTOR_DRIVER_BRINGUP_DUTY_LIMIT;
    }
    else if (duty < -MOTOR_DRIVER_BRINGUP_DUTY_LIMIT)
    {
        duty = -MOTOR_DRIVER_BRINGUP_DUTY_LIMIT;
    }

    return duty;
}

static uint32_t MotorDriver_DutyToCompare(int16_t duty)
{
    uint32_t magnitude = (duty < 0) ? (uint32_t)(-duty) : (uint32_t)duty;
    uint32_t period_counts = htim3.Init.Period + 1U;
    uint32_t compare = (period_counts * magnitude) / MOTOR_DRIVER_PWM_SCALE;

    if ((magnitude > 0U) && (compare == 0U))
    {
        compare = 1U;
    }

    return compare;
}

static int16_t MotorDriver_SetDualPwm(uint32_t forward_channel,
                                      uint32_t reverse_channel,
                                      int16_t duty)
{
    duty = MotorDriver_ClampDuty(duty);

    if (duty > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, reverse_channel, 0U);
        __HAL_TIM_SET_COMPARE(&htim3, forward_channel, MotorDriver_DutyToCompare(duty));
    }
    else if (duty < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, forward_channel, 0U);
        __HAL_TIM_SET_COMPARE(&htim3, reverse_channel, MotorDriver_DutyToCompare(duty));
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, forward_channel, 0U);
        __HAL_TIM_SET_COMPARE(&htim3, reverse_channel, 0U);
    }

    return duty;
}

static void MotorDriver_InitAdc1In2Placeholder(void)
{
    /*
     * PA2 is also USART2_TX in this bring-up project. Leave the GPIO mode
     * untouched by default so motor test logs keep working.
     */
#if MOTOR_DRIVER_ADC1_IN2_GPIO_INIT
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

    __HAL_RCC_ADC1_CLK_ENABLE();

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE_Msk) | ADC_CCR_ADCPRE_0;
    ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP2_Msk) | ADC_SMPR2_SMP2_2;
    ADC1->SQR1 &= ~ADC_SQR1_L_Msk;
    ADC1->SQR3 = (ADC1->SQR3 & ~ADC_SQR3_SQ1_Msk) | MOTOR_DRIVER_ADC1_CHANNEL;
    ADC1->CR2 |= ADC_CR2_ADON;
}

static void MotorDriver_LogStopAllExecuted(void)
{
#if MOTOR_DRIVER_VERBOSE_LOGS
    static uint32_t last_log_ms = 0U;
    static uint8_t has_logged = 0U;
    uint32_t now_ms = HAL_GetTick();

    if ((has_logged == 0U) || ((now_ms - last_log_ms) >= MOTOR_DRIVER_STOP_LOG_INTERVAL_MS))
    {
        last_log_ms = now_ms;
        has_logged = 1U;
        APP_LOG("MOTOR: stop_all executed=1 ccr1=%lu ccr2=%lu ccr3=%lu ccr4=%lu",
                (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1),
                (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2),
                (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3),
                (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4));
    }
#endif
}

static void MotorDriver_LogFinalDuty(char motor_name, int16_t duty)
{
#if MOTOR_DRIVER_VERBOSE_LOGS
    static uint32_t last_log_a_ms = 0U;
    static uint32_t last_log_b_ms = 0U;
    static uint8_t has_logged_a = 0U;
    static uint8_t has_logged_b = 0U;
    uint32_t now_ms = HAL_GetTick();
    uint32_t *last_log_ms = (motor_name == 'A') ? &last_log_a_ms : &last_log_b_ms;
    uint8_t *has_logged = (motor_name == 'A') ? &has_logged_a : &has_logged_b;

    if ((*has_logged == 0U) || ((now_ms - *last_log_ms) >= MOTOR_DRIVER_DUTY_LOG_INTERVAL_MS))
    {
        *last_log_ms = now_ms;
        *has_logged = 1U;
        APP_LOG("MOTOR: set%c final_duty=%d", motor_name, (int)duty);
    }
#else
    (void)motor_name;
    (void)duty;
#endif
}
