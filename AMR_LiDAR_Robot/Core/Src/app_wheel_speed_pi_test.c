#include "app_wheel_speed_pi_test.h"

#include "bringup_log.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "motor_driver.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

#include <stdint.h>

#define APP_PI_SAMPLE_MS 200U
#define APP_PI_START_DELAY_MS 3000U
#define APP_PI_RUN_MS 5000U
#define APP_PI_WAIT_LOG_MS 1000U

#define APP_PI_TARGET_COUNTS_PER_SAMPLE 800
#define APP_PI_BASE_DUTY 420
#define APP_PI_DUTY_MIN 0
#define APP_PI_DUTY_MAX 500
#define APP_PI_KP_X100 5
#define APP_PI_KI_X100 0
#define APP_PI_INTEGRAL_LIMIT 2000

#define APP_PI_LEFT_TIM (&htim2)
#define APP_PI_RIGHT_TIM (&htim4)
#define APP_PI_LEFT_NAME "TIM2_CH1_CH2_PA0_PA1"
#define APP_PI_RIGHT_NAME "TIM4_CH1_CH2_PB6_PB7"

typedef enum
{
    APP_PI_STATE_WAIT_START = 0,
    APP_PI_STATE_RUNNING,
    APP_PI_STATE_DONE
} AppPiState;

typedef struct
{
    int32_t integral;
    int32_t filtered_counts;
    int16_t duty;
} AppPiWheelState;

static AppPiState app_pi_state = APP_PI_STATE_WAIT_START;
static AppPiWheelState app_pi_left = {0};
static AppPiWheelState app_pi_right = {0};
static uint8_t app_pi_ready = 0U;
static uint32_t app_pi_armed_ms = 0U;
static uint32_t app_pi_run_start_ms = 0U;
static uint32_t app_pi_last_sample_ms = 0U;
static uint32_t app_pi_last_wait_log_ms = 0U;
static uint32_t app_pi_prev_left = 0U;
static uint32_t app_pi_prev_right = 0U;

static uint8_t App_WheelSpeedPiTest_StartTimers(void);
static uint32_t App_WheelSpeedPiTest_ReadCounter(const TIM_HandleTypeDef *timer);
static int32_t App_WheelSpeedPiTest_DeltaWithPeriod(uint32_t current,
                                                    uint32_t previous,
                                                    uint32_t period);
static int32_t App_WheelSpeedPiTest_AbsI32(int32_t value);
static int32_t App_WheelSpeedPiTest_ClampI32(int32_t value, int32_t min_value, int32_t max_value);
static int32_t App_WheelSpeedPiTest_FilterMeasurement(AppPiWheelState *wheel, int32_t measured_counts);
static int16_t App_WheelSpeedPiTest_UpdateWheel(AppPiWheelState *wheel, int32_t measured_counts);
static void App_WheelSpeedPiTest_ResetControl(void);
static void App_WheelSpeedPiTest_StopComplete(void);

void App_WheelSpeedPiTest_Init(void)
{
    app_pi_ready = 0U;
    app_pi_state = APP_PI_STATE_WAIT_START;
    app_pi_armed_ms = HAL_GetTick();
    app_pi_run_start_ms = 0U;
    app_pi_last_sample_ms = 0U;
    app_pi_last_wait_log_ms = 0U;
    App_WheelSpeedPiTest_ResetControl();

    APP_LOG("APP PI: lifted-wheel test armed start_delay_ms=%u run_ms=%u sample_ms=%u target=%d base=%d duty_min=%d duty_max=%d kp_x100=%d ki_x100=%d",
            (unsigned int)APP_PI_START_DELAY_MS,
            (unsigned int)APP_PI_RUN_MS,
            (unsigned int)APP_PI_SAMPLE_MS,
            APP_PI_TARGET_COUNTS_PER_SAMPLE,
            APP_PI_BASE_DUTY,
            APP_PI_DUTY_MIN,
            APP_PI_DUTY_MAX,
            APP_PI_KP_X100,
            APP_PI_KI_X100);
    APP_LOG("APP PI: encoder left=%s periodL=%lu right=%s periodR=%lu",
            APP_PI_LEFT_NAME,
            (unsigned long)APP_PI_LEFT_TIM->Init.Period,
            APP_PI_RIGHT_NAME,
            (unsigned long)APP_PI_RIGHT_TIM->Init.Period);

    if (App_WheelSpeedPiTest_StartTimers() == 0U)
    {
        MotorDriver_StopAll();
        app_pi_state = APP_PI_STATE_DONE;
        APP_LOG("APP PI: timer start failed, motor output disabled");
        return;
    }

    __HAL_TIM_SET_COUNTER(APP_PI_LEFT_TIM, 0U);
    __HAL_TIM_SET_COUNTER(APP_PI_RIGHT_TIM, 0U);
    app_pi_prev_left = App_WheelSpeedPiTest_ReadCounter(APP_PI_LEFT_TIM);
    app_pi_prev_right = App_WheelSpeedPiTest_ReadCounter(APP_PI_RIGHT_TIM);
    MotorDriver_StopAll();
    app_pi_ready = 1U;

    APP_LOG("APP PI: timers started, initial safe stop done");
}

void App_WheelSpeedPiTest_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint32_t now_ms = HAL_GetTick();

        if (app_pi_ready == 0U)
        {
            MotorDriver_StopAll();
            osDelay(1000U);
            continue;
        }

        switch (app_pi_state)
        {
            case APP_PI_STATE_WAIT_START:
                if ((app_pi_last_wait_log_ms == 0U) ||
                    ((now_ms - app_pi_last_wait_log_ms) >= APP_PI_WAIT_LOG_MS))
                {
                    app_pi_last_wait_log_ms = now_ms;
                    APP_LOG("APP PI: waiting start_delay elapsed_ms=%lu",
                            (unsigned long)(now_ms - app_pi_armed_ms));
                }

                if ((now_ms - app_pi_armed_ms) >= APP_PI_START_DELAY_MS)
                {
                    __HAL_TIM_SET_COUNTER(APP_PI_LEFT_TIM, 0U);
                    __HAL_TIM_SET_COUNTER(APP_PI_RIGHT_TIM, 0U);
                    app_pi_prev_left = App_WheelSpeedPiTest_ReadCounter(APP_PI_LEFT_TIM);
                    app_pi_prev_right = App_WheelSpeedPiTest_ReadCounter(APP_PI_RIGHT_TIM);
                    App_WheelSpeedPiTest_ResetControl();
                    app_pi_run_start_ms = now_ms;
                    app_pi_last_sample_ms = now_ms;
                    app_pi_state = APP_PI_STATE_RUNNING;
                    Chassis_SetRaw(APP_PI_BASE_DUTY, APP_PI_BASE_DUTY);
                    APP_LOG("APP PI: start closed-loop target=%d base=%d",
                            APP_PI_TARGET_COUNTS_PER_SAMPLE,
                            APP_PI_BASE_DUTY);
                }
                break;

            case APP_PI_STATE_RUNNING:
                if ((now_ms - app_pi_run_start_ms) >= APP_PI_RUN_MS)
                {
                    App_WheelSpeedPiTest_StopComplete();
                    break;
                }

                if ((now_ms - app_pi_last_sample_ms) >= APP_PI_SAMPLE_MS)
                {
                    uint32_t left_count = App_WheelSpeedPiTest_ReadCounter(APP_PI_LEFT_TIM);
                    uint32_t right_count = App_WheelSpeedPiTest_ReadCounter(APP_PI_RIGHT_TIM);
                    int32_t left_delta = App_WheelSpeedPiTest_DeltaWithPeriod(left_count,
                                                                               app_pi_prev_left,
                                                                               APP_PI_LEFT_TIM->Init.Period);
                    int32_t right_delta = App_WheelSpeedPiTest_DeltaWithPeriod(right_count,
                                                                                app_pi_prev_right,
                                                                                APP_PI_RIGHT_TIM->Init.Period);
                    int32_t raw_left = App_WheelSpeedPiTest_AbsI32(left_delta);
                    int32_t raw_right = App_WheelSpeedPiTest_AbsI32(right_delta);
                    int32_t filtered_left = App_WheelSpeedPiTest_FilterMeasurement(&app_pi_left, raw_left);
                    int32_t filtered_right = App_WheelSpeedPiTest_FilterMeasurement(&app_pi_right, raw_right);
                    int32_t err_left = APP_PI_TARGET_COUNTS_PER_SAMPLE - filtered_left;
                    int32_t err_right = APP_PI_TARGET_COUNTS_PER_SAMPLE - filtered_right;
                    int16_t pwm_left = App_WheelSpeedPiTest_UpdateWheel(&app_pi_left, filtered_left);
                    int16_t pwm_right = App_WheelSpeedPiTest_UpdateWheel(&app_pi_right, filtered_right);

                    app_pi_prev_left = left_count;
                    app_pi_prev_right = right_count;
                    app_pi_last_sample_ms = now_ms;

                    Chassis_SetRaw(pwm_left, pwm_right);
                    APP_LOG("APP PI: target=%d rawL=%ld rawR=%ld filtL=%ld filtR=%ld pwmL=%d pwmR=%d errL=%ld errR=%ld",
                            APP_PI_TARGET_COUNTS_PER_SAMPLE,
                            (long)raw_left,
                            (long)raw_right,
                            (long)filtered_left,
                            (long)filtered_right,
                            pwm_left,
                            pwm_right,
                            (long)err_left,
                            (long)err_right);
                }
                break;

            case APP_PI_STATE_DONE:
            default:
                osDelay(1000U);
                continue;
        }

        osDelay(20U);
    }
}

static uint8_t App_WheelSpeedPiTest_StartTimers(void)
{
    HAL_StatusTypeDef pwm1 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_StatusTypeDef pwm2 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_StatusTypeDef pwm3 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_StatusTypeDef pwm4 = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_StatusTypeDef enc_left = HAL_TIM_Encoder_Start(APP_PI_LEFT_TIM, TIM_CHANNEL_ALL);
    HAL_StatusTypeDef enc_right = HAL_TIM_Encoder_Start(APP_PI_RIGHT_TIM, TIM_CHANNEL_ALL);

    APP_LOG("APP PI: pwm_start tim3 ch1=%u ch2=%u ch3=%u ch4=%u encL=%u encR=%u",
            (unsigned int)pwm1,
            (unsigned int)pwm2,
            (unsigned int)pwm3,
            (unsigned int)pwm4,
            (unsigned int)enc_left,
            (unsigned int)enc_right);

    return ((pwm1 == HAL_OK) &&
            (pwm2 == HAL_OK) &&
            (pwm3 == HAL_OK) &&
            (pwm4 == HAL_OK) &&
            (enc_left == HAL_OK) &&
            (enc_right == HAL_OK)) ? 1U : 0U;
}

static uint32_t App_WheelSpeedPiTest_ReadCounter(const TIM_HandleTypeDef *timer)
{
    return __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef *)timer);
}

static int32_t App_WheelSpeedPiTest_DeltaWithPeriod(uint32_t current,
                                                    uint32_t previous,
                                                    uint32_t period)
{
    uint64_t counter_range = (uint64_t)period + 1ULL;
    uint64_t current_count = (uint64_t)current;
    uint64_t previous_count = (uint64_t)previous;
    uint64_t forward;
    uint64_t backward;

    if (counter_range == 0ULL)
    {
        return (int32_t)(current - previous);
    }

    current_count %= counter_range;
    previous_count %= counter_range;

    if (current_count >= previous_count)
    {
        forward = current_count - previous_count;
    }
    else
    {
        forward = (counter_range - previous_count) + current_count;
    }

    if (previous_count >= current_count)
    {
        backward = previous_count - current_count;
    }
    else
    {
        backward = (counter_range - current_count) + previous_count;
    }

    if (forward <= backward)
    {
        return (int32_t)forward;
    }

    return (int32_t)(-((int64_t)backward));
}

static int32_t App_WheelSpeedPiTest_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t App_WheelSpeedPiTest_ClampI32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static int32_t App_WheelSpeedPiTest_FilterMeasurement(AppPiWheelState *wheel, int32_t measured_counts)
{
    wheel->filtered_counts = ((wheel->filtered_counts * 3) + measured_counts) / 4;

    return wheel->filtered_counts;
}

static int16_t App_WheelSpeedPiTest_UpdateWheel(AppPiWheelState *wheel, int32_t measured_counts)
{
    int32_t error = APP_PI_TARGET_COUNTS_PER_SAMPLE - measured_counts;
    int32_t control;

    wheel->integral = App_WheelSpeedPiTest_ClampI32(wheel->integral + error,
                                                    -APP_PI_INTEGRAL_LIMIT,
                                                    APP_PI_INTEGRAL_LIMIT);

    control = APP_PI_BASE_DUTY +
              ((APP_PI_KP_X100 * error) + (APP_PI_KI_X100 * wheel->integral)) / 100;
    control = App_WheelSpeedPiTest_ClampI32(control, APP_PI_DUTY_MIN, APP_PI_DUTY_MAX);
    wheel->duty = (int16_t)control;

    return wheel->duty;
}

static void App_WheelSpeedPiTest_ResetControl(void)
{
    app_pi_left.integral = 0;
    app_pi_left.filtered_counts = 0;
    app_pi_left.duty = APP_PI_BASE_DUTY;
    app_pi_right.integral = 0;
    app_pi_right.filtered_counts = 0;
    app_pi_right.duty = APP_PI_BASE_DUTY;
}

static void App_WheelSpeedPiTest_StopComplete(void)
{
    MotorDriver_StopAll();
    app_pi_state = APP_PI_STATE_DONE;
    APP_LOG("APP PI: complete stop");
}
