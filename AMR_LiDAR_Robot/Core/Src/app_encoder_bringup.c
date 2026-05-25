#include "app_encoder_bringup.h"

#include "bringup_log.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

#include <stdint.h>

#define APP_ENCODER_SAMPLE_MS 200U

#ifndef APP_ENCODER_RAW_LOG_INTERVAL_MS
#define APP_ENCODER_RAW_LOG_INTERVAL_MS 1000U
#endif

#ifndef APP_ENCODER_MOTOR_SPIN_ENABLE
#define APP_ENCODER_MOTOR_SPIN_ENABLE 1U
#endif

#ifndef APP_ENCODER_MOTOR_START_DELAY_MS
#define APP_ENCODER_MOTOR_START_DELAY_MS 3000U
#endif

#ifndef APP_ENCODER_MOTOR_SWEEP_ENABLE
#define APP_ENCODER_MOTOR_SWEEP_ENABLE 1U
#endif

#ifndef APP_ENCODER_MOTOR_SWEEP_STAGE_MS
#define APP_ENCODER_MOTOR_SWEEP_STAGE_MS 1200U
#endif

#ifndef APP_ENCODER_MOTOR_SWEEP_STOP_MS
#define APP_ENCODER_MOTOR_SWEEP_STOP_MS 800U
#endif

#ifndef APP_ENCODER_MOTOR_SWEEP_HEARTBEAT_MS
#define APP_ENCODER_MOTOR_SWEEP_HEARTBEAT_MS 500U
#endif

#ifndef APP_ENCODER_MOTOR_GLOBAL_TIMEOUT_MS
#define APP_ENCODER_MOTOR_GLOBAL_TIMEOUT_MS 9000U
#endif

#ifndef APP_ENCODER_MOTOR_STAGE_TIMEOUT_MARGIN_MS
#define APP_ENCODER_MOTOR_STAGE_TIMEOUT_MARGIN_MS 500U
#endif

#ifndef APP_ENCODER_MOTOR_RUN_MS
#define APP_ENCODER_MOTOR_RUN_MS 5000U
#endif

#ifndef APP_ENCODER_MOTOR_LEFT_DUTY
#define APP_ENCODER_MOTOR_LEFT_DUTY 360
#endif

#ifndef APP_ENCODER_MOTOR_RIGHT_DUTY
#define APP_ENCODER_MOTOR_RIGHT_DUTY 360
#endif

#if APP_ENCODER_MOTOR_SPIN_ENABLE
#include "chassis.h"
#include "motor_driver.h"
#endif

/*
 * TODO: These CPR and wheel diameter values are initial bring-up defaults.
 * Measure one full wheel revolution on the real chassis and update them before
 * using the velocity estimate for control decisions.
 */
#define APP_ENCODER_LEFT_COUNTS_PER_REV 390U
#define APP_ENCODER_RIGHT_COUNTS_PER_REV 390U
#define APP_ENCODER_WHEEL_DIAMETER_MM 65U
#define APP_ENCODER_PI_X10000 31416L

/*
 * Current CubeMX timer mapping:
 *   left encoder  -> TIM2 CH1/CH2, PA0/PA1
 *   right encoder -> TIM4 CH1/CH2, PB6/PB7
 *
 * TODO: If encoder wiring changes, update these timer handle macros and the
 * matching CubeMX TIM/GPIO encoder-mode configuration.
 */
#ifndef APP_ENCODER_BRINGUP_LEFT_TIM
#define APP_ENCODER_BRINGUP_LEFT_TIM (&htim2)
#endif

#ifndef APP_ENCODER_BRINGUP_RIGHT_TIM
#define APP_ENCODER_BRINGUP_RIGHT_TIM (&htim4)
#endif

#ifndef APP_ENCODER_BRINGUP_LEFT_NAME
#define APP_ENCODER_BRINGUP_LEFT_NAME "TIM2_CH1_CH2_PA0_PA1"
#endif

#ifndef APP_ENCODER_BRINGUP_RIGHT_NAME
#define APP_ENCODER_BRINGUP_RIGHT_NAME "TIM4_CH1_CH2_PB6_PB7"
#endif

#if APP_ENCODER_MOTOR_SPIN_ENABLE
typedef enum
{
    APP_ENCODER_MOTOR_STATE_ARMED = 0,
    APP_ENCODER_MOTOR_STATE_RUNNING,
    APP_ENCODER_MOTOR_STATE_BETWEEN_STOP,
    APP_ENCODER_MOTOR_STATE_STOPPED
} AppEncoderMotorState;
#endif

static uint8_t app_encoder_ready = 0U;
static uint32_t app_encoder_prev_left = 0U;
static uint32_t app_encoder_prev_right = 0U;
static uint32_t app_encoder_last_raw_log_ms = 0U;

#if APP_ENCODER_MOTOR_SPIN_ENABLE
static AppEncoderMotorState app_encoder_motor_state = APP_ENCODER_MOTOR_STATE_ARMED;
static uint32_t app_encoder_motor_armed_ms = 0U;
static uint32_t app_encoder_motor_run_start_ms = 0U;
static uint32_t app_encoder_motor_stop_start_ms = 0U;
static uint32_t app_encoder_motor_last_wait_log_ms = 0U;
#if APP_ENCODER_MOTOR_SWEEP_ENABLE
typedef struct
{
    int16_t target_duty;
    int16_t left_duty;
    int16_t right_duty;
} AppEncoderCompStage;

static const AppEncoderCompStage app_encoder_motor_sweep_stages[] = {
    {420, 405, 420},
    {420, 410, 420},
    {420, 415, 420},
};
static uint8_t app_encoder_motor_sweep_stage = 0U;
static uint32_t app_encoder_sweep_last_heartbeat_ms = 0U;
static int64_t app_encoder_sweep_sum_left_delta = 0;
static int64_t app_encoder_sweep_sum_right_delta = 0;
static int64_t app_encoder_sweep_sum_left_velocity_x10 = 0;
static int64_t app_encoder_sweep_sum_right_velocity_x10 = 0;
static uint32_t app_encoder_sweep_sample_count = 0U;
#endif
#endif

static uint8_t App_EncoderBringup_TimerConfigured(const TIM_HandleTypeDef *timer);
static uint32_t App_EncoderBringup_ReadCounter(const TIM_HandleTypeDef *timer);
static int32_t App_EncoderBringup_DeltaWithPeriod(uint32_t current,
                                                  uint32_t previous,
                                                  uint32_t period);
static int32_t App_EncoderBringup_CountsPerSecondX10(int32_t delta_counts,
                                                     uint32_t sample_ms);
static int32_t App_EncoderBringup_RevolutionsPerSecondX1000(int32_t delta_counts,
                                                            uint32_t sample_ms,
                                                            uint32_t counts_per_rev);
static int32_t App_EncoderBringup_WheelVelocityMmPerSecondX10(int32_t delta_counts,
                                                              uint32_t sample_ms,
                                                              uint32_t counts_per_rev,
                                                              uint32_t wheel_diameter_mm);
#if APP_ENCODER_MOTOR_SPIN_ENABLE
static void App_EncoderBringup_MotorSpinInit(void);
static void App_EncoderBringup_MotorSpinUpdate(uint32_t now_ms,
                                               int32_t left_delta,
                                               int32_t right_delta,
                                               int32_t left_velocity_x10,
                                               int32_t right_velocity_x10);
static void App_EncoderBringup_MotorSafetyCheck(uint32_t now_ms);
static void App_EncoderBringup_StartMotorPwmForSpinTest(void);
static void App_EncoderBringup_LogPwmCompare(void);
static void App_EncoderBringup_MotorApplyRaw(int16_t left_duty, int16_t right_duty);
#if APP_ENCODER_MOTOR_SWEEP_ENABLE
static void App_EncoderBringup_SweepStartStage(uint8_t stage, uint32_t now_ms);
static void App_EncoderBringup_SweepResetStats(void);
static void App_EncoderBringup_SweepAccumulate(int32_t left_delta,
                                               int32_t right_delta,
                                               int32_t left_velocity_x10,
                                               int32_t right_velocity_x10);
static void App_EncoderBringup_SweepLogStageResult(uint8_t stage);
static void App_EncoderBringup_SweepFinishStage(uint32_t now_ms);
#endif
#endif

void App_EncoderBringup_Init(void)
{
    TIM_HandleTypeDef *left_timer = APP_ENCODER_BRINGUP_LEFT_TIM;
    TIM_HandleTypeDef *right_timer = APP_ENCODER_BRINGUP_RIGHT_TIM;

    app_encoder_ready = 0U;
    app_encoder_last_raw_log_ms = 0U;

    APP_LOG("APP ENC: init sample_ms=%u left=%s periodL=%lu right=%s periodR=%lu cprL=%u cprR=%u wheel_diam_mm=%u",
            (unsigned int)APP_ENCODER_SAMPLE_MS,
            APP_ENCODER_BRINGUP_LEFT_NAME,
            (unsigned long)((left_timer != NULL) ? left_timer->Init.Period : 0U),
            APP_ENCODER_BRINGUP_RIGHT_NAME,
            (unsigned long)((right_timer != NULL) ? right_timer->Init.Period : 0U),
            (unsigned int)APP_ENCODER_LEFT_COUNTS_PER_REV,
            (unsigned int)APP_ENCODER_RIGHT_COUNTS_PER_REV,
            (unsigned int)APP_ENCODER_WHEEL_DIAMETER_MM);

    if ((App_EncoderBringup_TimerConfigured(left_timer) == 0U) ||
        (App_EncoderBringup_TimerConfigured(right_timer) == 0U))
    {
        APP_LOG("APP ENC: encoder timer handle not configured");
        return;
    }

    __HAL_TIM_SET_COUNTER(left_timer, 0U);
    __HAL_TIM_SET_COUNTER(right_timer, 0U);

    HAL_StatusTypeDef left_status = HAL_TIM_Encoder_Start(left_timer, TIM_CHANNEL_ALL);
    HAL_StatusTypeDef right_status = HAL_TIM_Encoder_Start(right_timer, TIM_CHANNEL_ALL);

    if ((left_status != HAL_OK) || (right_status != HAL_OK))
    {
        APP_LOG("APP ENC: encoder start failed left_status=%u right_status=%u",
                (unsigned int)left_status,
                (unsigned int)right_status);
        return;
    }

    app_encoder_prev_left = App_EncoderBringup_ReadCounter(left_timer);
    app_encoder_prev_right = App_EncoderBringup_ReadCounter(right_timer);
    app_encoder_ready = 1U;

    APP_LOG("APP ENC: encoder timers started left=%s right=%s",
            APP_ENCODER_BRINGUP_LEFT_NAME,
            APP_ENCODER_BRINGUP_RIGHT_NAME);

#if APP_ENCODER_MOTOR_SPIN_ENABLE
    App_EncoderBringup_MotorSpinInit();
#endif
}

void App_EncoderBringup_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        TIM_HandleTypeDef *left_timer = APP_ENCODER_BRINGUP_LEFT_TIM;
        TIM_HandleTypeDef *right_timer = APP_ENCODER_BRINGUP_RIGHT_TIM;

        if (app_encoder_ready == 0U)
        {
            APP_LOG("APP ENC: encoder timer handle not configured");
            osDelay(1000U);
            continue;
        }

        uint32_t left_count = App_EncoderBringup_ReadCounter(left_timer);
        uint32_t right_count = App_EncoderBringup_ReadCounter(right_timer);
        int32_t left_delta = App_EncoderBringup_DeltaWithPeriod(left_count,
                                                                app_encoder_prev_left,
                                                                left_timer->Init.Period);
        int32_t right_delta = App_EncoderBringup_DeltaWithPeriod(right_count,
                                                                 app_encoder_prev_right,
                                                                 right_timer->Init.Period);

        app_encoder_prev_left = left_count;
        app_encoder_prev_right = right_count;

        int32_t left_velocity_x10 = App_EncoderBringup_WheelVelocityMmPerSecondX10(left_delta,
                                                                                   APP_ENCODER_SAMPLE_MS,
                                                                                   APP_ENCODER_LEFT_COUNTS_PER_REV,
                                                                                   APP_ENCODER_WHEEL_DIAMETER_MM);
        int32_t right_velocity_x10 = App_EncoderBringup_WheelVelocityMmPerSecondX10(right_delta,
                                                                                    APP_ENCODER_SAMPLE_MS,
                                                                                    APP_ENCODER_RIGHT_COUNTS_PER_REV,
                                                                                    APP_ENCODER_WHEEL_DIAMETER_MM);

#if APP_ENCODER_MOTOR_SPIN_ENABLE && APP_ENCODER_MOTOR_SWEEP_ENABLE
        uint32_t raw_log_now_ms = HAL_GetTick();
        uint8_t log_raw = 0U;

        if ((app_encoder_last_raw_log_ms == 0U) ||
            ((raw_log_now_ms - app_encoder_last_raw_log_ms) >= APP_ENCODER_RAW_LOG_INTERVAL_MS))
        {
            app_encoder_last_raw_log_ms = raw_log_now_ms;
            log_raw = 1U;
        }

        if (log_raw != 0U)
#endif
        {
        int32_t left_cps_x10 = App_EncoderBringup_CountsPerSecondX10(left_delta,
                                                                      APP_ENCODER_SAMPLE_MS);
        int32_t right_cps_x10 = App_EncoderBringup_CountsPerSecondX10(right_delta,
                                                                       APP_ENCODER_SAMPLE_MS);
        int32_t left_rps_x1000 = App_EncoderBringup_RevolutionsPerSecondX1000(left_delta,
                                                                              APP_ENCODER_SAMPLE_MS,
                                                                              APP_ENCODER_LEFT_COUNTS_PER_REV);
        int32_t right_rps_x1000 = App_EncoderBringup_RevolutionsPerSecondX1000(right_delta,
                                                                               APP_ENCODER_SAMPLE_MS,
                                                                               APP_ENCODER_RIGHT_COUNTS_PER_REV);

        APP_LOG("APP ENC: rawL=%lu rawR=%lu dL=%ld dR=%ld cpsL_x10=%ld cpsR_x10=%ld rpsL_x1000=%ld rpsR_x1000=%ld vL_x10=%ldmm/s vR_x10=%ldmm/s",
                (unsigned long)left_count,
                (unsigned long)right_count,
                (long)left_delta,
                (long)right_delta,
                (long)left_cps_x10,
                (long)right_cps_x10,
                (long)left_rps_x1000,
                (long)right_rps_x1000,
                (long)left_velocity_x10,
                (long)right_velocity_x10);
        }

#if APP_ENCODER_MOTOR_SPIN_ENABLE
        App_EncoderBringup_MotorSpinUpdate(HAL_GetTick(),
                                           left_delta,
                                           right_delta,
                                           left_velocity_x10,
                                           right_velocity_x10);
        App_EncoderBringup_MotorSafetyCheck(HAL_GetTick());
#endif

        osDelay(APP_ENCODER_SAMPLE_MS);
    }
}

static uint8_t App_EncoderBringup_TimerConfigured(const TIM_HandleTypeDef *timer)
{
    return ((timer != NULL) && (timer->Instance != NULL)) ? 1U : 0U;
}

static uint32_t App_EncoderBringup_ReadCounter(const TIM_HandleTypeDef *timer)
{
    if (timer == NULL)
    {
        return 0U;
    }

    return __HAL_TIM_GET_COUNTER((TIM_HandleTypeDef *)timer);
}

static int32_t App_EncoderBringup_DeltaWithPeriod(uint32_t current,
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

static int32_t App_EncoderBringup_CountsPerSecondX10(int32_t delta_counts,
                                                     uint32_t sample_ms)
{
    if (sample_ms == 0U)
    {
        return 0;
    }

    return (int32_t)(((int64_t)delta_counts * 10000LL) / (int64_t)sample_ms);
}

static int32_t App_EncoderBringup_RevolutionsPerSecondX1000(int32_t delta_counts,
                                                            uint32_t sample_ms,
                                                            uint32_t counts_per_rev)
{
    if ((sample_ms == 0U) || (counts_per_rev == 0U))
    {
        return 0;
    }

    return (int32_t)(((int64_t)delta_counts * 1000000LL) /
                     ((int64_t)sample_ms * (int64_t)counts_per_rev));
}

static int32_t App_EncoderBringup_WheelVelocityMmPerSecondX10(int32_t delta_counts,
                                                              uint32_t sample_ms,
                                                              uint32_t counts_per_rev,
                                                              uint32_t wheel_diameter_mm)
{
    int64_t numerator;
    int64_t denominator;

    if ((sample_ms == 0U) || (counts_per_rev == 0U))
    {
        return 0;
    }

    numerator = (int64_t)delta_counts *
                1000LL *
                (int64_t)APP_ENCODER_PI_X10000 *
                (int64_t)wheel_diameter_mm *
                10LL;
    denominator = (int64_t)sample_ms *
                  (int64_t)counts_per_rev *
                  10000LL;

    return (int32_t)(numerator / denominator);
}

#if APP_ENCODER_MOTOR_SPIN_ENABLE
static void App_EncoderBringup_MotorSpinInit(void)
{
    app_encoder_motor_state = APP_ENCODER_MOTOR_STATE_ARMED;
    app_encoder_motor_armed_ms = HAL_GetTick();
    app_encoder_motor_run_start_ms = 0U;
    app_encoder_motor_stop_start_ms = 0U;
    app_encoder_motor_last_wait_log_ms = 0U;
#if APP_ENCODER_MOTOR_SWEEP_ENABLE
    app_encoder_motor_sweep_stage = 0U;
    app_encoder_sweep_last_heartbeat_ms = 0U;
    App_EncoderBringup_SweepResetStats();
#endif

    APP_LOG("APP ENC MOTOR: lifted-wheel test armed start_delay_ms=%u run_ms=%u dutyL=%d dutyR=%d sweep_enable=%u",
            (unsigned int)APP_ENCODER_MOTOR_START_DELAY_MS,
            (unsigned int)APP_ENCODER_MOTOR_RUN_MS,
            APP_ENCODER_MOTOR_LEFT_DUTY,
            APP_ENCODER_MOTOR_RIGHT_DUTY,
            (unsigned int)APP_ENCODER_MOTOR_SWEEP_ENABLE);
    APP_LOG("APP ENC MOTOR: skip Chassis_Init in encoder motor test");
    APP_LOG("APP ENC MOTOR: no non-blocking motor init found");
    App_EncoderBringup_StartMotorPwmForSpinTest();
    MotorDriver_StopAll();
    APP_LOG("APP ENC MOTOR: initial safe stop done");
    App_EncoderBringup_LogPwmCompare();
}

static void App_EncoderBringup_MotorSpinUpdate(uint32_t now_ms,
                                               int32_t left_delta,
                                               int32_t right_delta,
                                               int32_t left_velocity_x10,
                                               int32_t right_velocity_x10)
{
    switch (app_encoder_motor_state)
    {
        case APP_ENCODER_MOTOR_STATE_ARMED:
            if ((app_encoder_motor_last_wait_log_ms == 0U) ||
                ((now_ms - app_encoder_motor_last_wait_log_ms) >= 1000U))
            {
                app_encoder_motor_last_wait_log_ms = now_ms;
                APP_LOG("APP ENC MOTOR: waiting start_delay elapsed_ms=%lu",
                        (unsigned long)(now_ms - app_encoder_motor_armed_ms));
            }

            if ((now_ms - app_encoder_motor_armed_ms) >= APP_ENCODER_MOTOR_START_DELAY_MS)
            {
#if APP_ENCODER_MOTOR_SWEEP_ENABLE
                app_encoder_motor_sweep_stage = 0U;
                App_EncoderBringup_SweepStartStage(app_encoder_motor_sweep_stage,
                                                   now_ms);
#else
                app_encoder_motor_run_start_ms = now_ms;
                app_encoder_motor_state = APP_ENCODER_MOTOR_STATE_RUNNING;
                App_EncoderBringup_MotorApplyRaw(APP_ENCODER_MOTOR_LEFT_DUTY,
                                                 APP_ENCODER_MOTOR_RIGHT_DUTY);
                APP_LOG("APP ENC MOTOR: start low-speed forward dutyL=%d dutyR=%d",
                        APP_ENCODER_MOTOR_LEFT_DUTY,
                        APP_ENCODER_MOTOR_RIGHT_DUTY);
#endif
            }
            break;

        case APP_ENCODER_MOTOR_STATE_RUNNING:
#if APP_ENCODER_MOTOR_SWEEP_ENABLE
            App_EncoderBringup_SweepAccumulate(left_delta,
                                               right_delta,
                                               left_velocity_x10,
                                               right_velocity_x10);

            if ((now_ms - app_encoder_sweep_last_heartbeat_ms) >= APP_ENCODER_MOTOR_SWEEP_HEARTBEAT_MS)
            {
                app_encoder_sweep_last_heartbeat_ms = now_ms;
                APP_LOG("APP ENC COMP: stage=%u running elapsed_ms=%lu",
                        (unsigned int)(app_encoder_motor_sweep_stage + 1U),
                        (unsigned long)(now_ms - app_encoder_motor_run_start_ms));
            }

            if ((now_ms - app_encoder_motor_run_start_ms) >= APP_ENCODER_MOTOR_SWEEP_STAGE_MS)
            {
                App_EncoderBringup_SweepFinishStage(now_ms);
            }
#else
            if ((now_ms - app_encoder_motor_run_start_ms) >= APP_ENCODER_MOTOR_RUN_MS)
            {
                MotorDriver_StopAll();
                app_encoder_motor_state = APP_ENCODER_MOTOR_STATE_STOPPED;
                APP_LOG("APP ENC MOTOR: stop after test window");
                App_EncoderBringup_LogPwmCompare();
            }
#endif
            break;

        case APP_ENCODER_MOTOR_STATE_BETWEEN_STOP:
#if APP_ENCODER_MOTOR_SWEEP_ENABLE
            if ((now_ms - app_encoder_motor_stop_start_ms) >= APP_ENCODER_MOTOR_SWEEP_STOP_MS)
            {
                App_EncoderBringup_SweepStartStage(app_encoder_motor_sweep_stage,
                                                   now_ms);
            }
#endif
            break;

        case APP_ENCODER_MOTOR_STATE_STOPPED:
        default:
            break;
    }
}

static void App_EncoderBringup_MotorSafetyCheck(uint32_t now_ms)
{
    if (app_encoder_motor_state == APP_ENCODER_MOTOR_STATE_STOPPED)
    {
        return;
    }

    if ((now_ms - app_encoder_motor_armed_ms) >= APP_ENCODER_MOTOR_GLOBAL_TIMEOUT_MS)
    {
        MotorDriver_StopAll();
        app_encoder_motor_state = APP_ENCODER_MOTOR_STATE_STOPPED;
        APP_LOG("APP ENC SAFETY: global timeout stop");
        App_EncoderBringup_LogPwmCompare();
        return;
    }

#if APP_ENCODER_MOTOR_SWEEP_ENABLE
    if ((app_encoder_motor_state == APP_ENCODER_MOTOR_STATE_RUNNING) &&
        ((now_ms - app_encoder_motor_run_start_ms) >=
         (APP_ENCODER_MOTOR_SWEEP_STAGE_MS + APP_ENCODER_MOTOR_STAGE_TIMEOUT_MARGIN_MS)))
    {
        APP_LOG("APP ENC SAFETY: stage timeout stop stage=%u elapsed_ms=%lu",
                (unsigned int)(app_encoder_motor_sweep_stage + 1U),
                (unsigned long)(now_ms - app_encoder_motor_run_start_ms));
        App_EncoderBringup_SweepFinishStage(now_ms);
    }
#endif
}

static void App_EncoderBringup_MotorApplyRaw(int16_t left_duty, int16_t right_duty)
{
    APP_LOG("APP ENC MOTOR: before Chassis_SetRaw rawL=%d rawR=%d",
            left_duty,
            right_duty);
    Chassis_SetRaw(left_duty, right_duty);
    APP_LOG("APP ENC MOTOR: after Chassis_SetRaw rawL=%d rawR=%d",
            left_duty,
            right_duty);
    App_EncoderBringup_LogPwmCompare();
}

static void App_EncoderBringup_StartMotorPwmForSpinTest(void)
{
    HAL_StatusTypeDef ch1_status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_StatusTypeDef ch2_status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_StatusTypeDef ch3_status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_StatusTypeDef ch4_status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    APP_LOG("APP ENC MOTOR: pwm_start tim3 ch1=%u ch2=%u ch3=%u ch4=%u",
            (unsigned int)ch1_status,
            (unsigned int)ch2_status,
            (unsigned int)ch3_status,
            (unsigned int)ch4_status);
}

static void App_EncoderBringup_LogPwmCompare(void)
{
    APP_LOG("APP ENC MOTOR: pwm ccr1=%lu ccr2=%lu ccr3=%lu ccr4=%lu",
            (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1),
            (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2),
            (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3),
            (unsigned long)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4));
}

#if APP_ENCODER_MOTOR_SWEEP_ENABLE
static void App_EncoderBringup_SweepStartStage(uint8_t stage, uint32_t now_ms)
{
    const AppEncoderCompStage *comp_stage = &app_encoder_motor_sweep_stages[stage];

    app_encoder_motor_run_start_ms = now_ms;
    app_encoder_motor_state = APP_ENCODER_MOTOR_STATE_RUNNING;
    app_encoder_sweep_last_heartbeat_ms = now_ms;
    App_EncoderBringup_SweepResetStats();

    APP_LOG("APP ENC COMP: stage=%u target=%d left=%d right=%d start",
            (unsigned int)(stage + 1U),
            comp_stage->target_duty,
            comp_stage->left_duty,
            comp_stage->right_duty);
    App_EncoderBringup_MotorApplyRaw(comp_stage->left_duty,
                                     comp_stage->right_duty);
}

static void App_EncoderBringup_SweepResetStats(void)
{
    app_encoder_sweep_sum_left_delta = 0;
    app_encoder_sweep_sum_right_delta = 0;
    app_encoder_sweep_sum_left_velocity_x10 = 0;
    app_encoder_sweep_sum_right_velocity_x10 = 0;
    app_encoder_sweep_sample_count = 0U;
}

static void App_EncoderBringup_SweepAccumulate(int32_t left_delta,
                                               int32_t right_delta,
                                               int32_t left_velocity_x10,
                                               int32_t right_velocity_x10)
{
    app_encoder_sweep_sum_left_delta += (int64_t)left_delta;
    app_encoder_sweep_sum_right_delta += (int64_t)right_delta;
    app_encoder_sweep_sum_left_velocity_x10 += (int64_t)left_velocity_x10;
    app_encoder_sweep_sum_right_velocity_x10 += (int64_t)right_velocity_x10;
    app_encoder_sweep_sample_count++;
}

static void App_EncoderBringup_SweepLogStageResult(uint8_t stage)
{
    const AppEncoderCompStage *comp_stage = &app_encoder_motor_sweep_stages[stage];
    int32_t avg_left_delta = 0;
    int32_t avg_right_delta = 0;
    int32_t avg_left_velocity_x10 = 0;
    int32_t avg_right_velocity_x10 = 0;
    int32_t ratio_x1000 = 0;

    if (app_encoder_sweep_sample_count > 0U)
    {
        avg_left_delta = (int32_t)(app_encoder_sweep_sum_left_delta /
                                   (int64_t)app_encoder_sweep_sample_count);
        avg_right_delta = (int32_t)(app_encoder_sweep_sum_right_delta /
                                    (int64_t)app_encoder_sweep_sample_count);
        avg_left_velocity_x10 = (int32_t)(app_encoder_sweep_sum_left_velocity_x10 /
                                          (int64_t)app_encoder_sweep_sample_count);
        avg_right_velocity_x10 = (int32_t)(app_encoder_sweep_sum_right_velocity_x10 /
                                           (int64_t)app_encoder_sweep_sample_count);
    }

    if (avg_right_delta != 0)
    {
        ratio_x1000 = (int32_t)(((int64_t)avg_left_delta * 1000LL) /
                                (int64_t)avg_right_delta);
    }

    APP_LOG("APP ENC COMP: stage=%u target=%d left=%d right=%d avg_dL=%ld avg_dR=%ld avg_vL_x10=%ld avg_vR_x10=%ld ratio_x1000=%ld",
            (unsigned int)(stage + 1U),
            comp_stage->target_duty,
            comp_stage->left_duty,
            comp_stage->right_duty,
            (long)avg_left_delta,
            (long)avg_right_delta,
            (long)avg_left_velocity_x10,
            (long)avg_right_velocity_x10,
            (long)ratio_x1000);
}

static void App_EncoderBringup_SweepFinishStage(uint32_t now_ms)
{
    App_EncoderBringup_SweepLogStageResult(app_encoder_motor_sweep_stage);
    MotorDriver_StopAll();
    App_EncoderBringup_LogPwmCompare();

    if ((uint32_t)(app_encoder_motor_sweep_stage + 1U) >=
        (sizeof(app_encoder_motor_sweep_stages) / sizeof(app_encoder_motor_sweep_stages[0])))
    {
        app_encoder_motor_state = APP_ENCODER_MOTOR_STATE_STOPPED;
        APP_LOG("APP ENC COMP: complete stop");
        return;
    }

    app_encoder_motor_sweep_stage++;
    app_encoder_motor_stop_start_ms = now_ms;
    app_encoder_motor_state = APP_ENCODER_MOTOR_STATE_BETWEEN_STOP;
    APP_LOG("APP ENC COMP: inter-stage stop");
}
#endif
#endif
