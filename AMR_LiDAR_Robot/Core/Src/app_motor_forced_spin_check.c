#include "app_motor_forced_spin_check.h"

#include "bringup_log.h"
#include "cmsis_os.h"
#include "motor_driver.h"

#include <stdint.h>

#define APP_MOTOR_FORCED_SPIN_DUTY 350
#define APP_MOTOR_FORCED_SPIN_INITIAL_STOP_MS 2000U
#define APP_MOTOR_FORCED_SPIN_RUN_MS 2000U
#define APP_MOTOR_FORCED_SPIN_BETWEEN_STOP_MS 1000U
#define APP_MOTOR_FORCED_SPIN_LOG_STEP_MS 500U

static void App_MotorForcedSpinCheck_StopPhase(const char *phase, uint32_t duration_ms);
static void App_MotorForcedSpinCheck_RunPhase(const char *phase,
                                              int16_t motor_a_duty,
                                              int16_t motor_b_duty,
                                              uint32_t duration_ms);
static uint32_t App_MotorForcedSpinCheck_StepMs(uint32_t elapsed_ms, uint32_t duration_ms);

void App_MotorForcedSpinCheck_Run(void)
{
    APP_LOG("[MOTOR_FORCED_SPIN] start duty=%d sequence=stop2s,A2s,stop1s,B2s,stop1s,AB2s",
            APP_MOTOR_FORCED_SPIN_DUTY);

    MotorDriver_Init();

    App_MotorForcedSpinCheck_StopPhase("initial_stop", APP_MOTOR_FORCED_SPIN_INITIAL_STOP_MS);
    App_MotorForcedSpinCheck_RunPhase("motor_a_forward",
                                      APP_MOTOR_FORCED_SPIN_DUTY,
                                      0,
                                      APP_MOTOR_FORCED_SPIN_RUN_MS);
    App_MotorForcedSpinCheck_StopPhase("between_a_b", APP_MOTOR_FORCED_SPIN_BETWEEN_STOP_MS);
    App_MotorForcedSpinCheck_RunPhase("motor_b_forward",
                                      0,
                                      APP_MOTOR_FORCED_SPIN_DUTY,
                                      APP_MOTOR_FORCED_SPIN_RUN_MS);
    App_MotorForcedSpinCheck_StopPhase("between_b_both", APP_MOTOR_FORCED_SPIN_BETWEEN_STOP_MS);
    App_MotorForcedSpinCheck_RunPhase("both_forward",
                                      APP_MOTOR_FORCED_SPIN_DUTY,
                                      APP_MOTOR_FORCED_SPIN_DUTY,
                                      APP_MOTOR_FORCED_SPIN_RUN_MS);

    MotorDriver_StopAll();
    APP_LOG("[MOTOR_FORCED_SPIN] done final_stop=1");
}

static void App_MotorForcedSpinCheck_StopPhase(const char *phase, uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0U;

    APP_LOG("[MOTOR_FORCED_SPIN] phase=%s action=STOP duration_ms=%lu",
            phase,
            (unsigned long)duration_ms);

    while (elapsed_ms < duration_ms)
    {
        uint32_t step_ms = App_MotorForcedSpinCheck_StepMs(elapsed_ms, duration_ms);

        MotorDriver_StopAll();
        osDelay(step_ms);
        elapsed_ms += step_ms;
    }
}

static void App_MotorForcedSpinCheck_RunPhase(const char *phase,
                                              int16_t motor_a_duty,
                                              int16_t motor_b_duty,
                                              uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0U;

    APP_LOG("[MOTOR_FORCED_SPIN] phase=%s action=RUN motor_a=%d motor_b=%d duration_ms=%lu",
            phase,
            (int)motor_a_duty,
            (int)motor_b_duty,
            (unsigned long)duration_ms);

    while (elapsed_ms < duration_ms)
    {
        uint32_t step_ms = App_MotorForcedSpinCheck_StepMs(elapsed_ms, duration_ms);

        MotorDriver_SetMotorA(motor_a_duty);
        MotorDriver_SetMotorB(motor_b_duty);
        osDelay(step_ms);
        elapsed_ms += step_ms;
    }
}

static uint32_t App_MotorForcedSpinCheck_StepMs(uint32_t elapsed_ms, uint32_t duration_ms)
{
    uint32_t remaining_ms = duration_ms - elapsed_ms;

    if (remaining_ms > APP_MOTOR_FORCED_SPIN_LOG_STEP_MS)
    {
        return APP_MOTOR_FORCED_SPIN_LOG_STEP_MS;
    }

    return remaining_ms;
}
