#include "app_motor_forced_spin_check.h"

#include "app_test_config.h"

#if APP_MOTOR_FORCED_SPIN_CHECK_ENABLE

#include "bringup_log.h"
#include "cmsis_os.h"
#include "motor_driver.h"

#include <stdint.h>

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
    APP_LOG("[MOTOR_FORCED_SPIN] start target=high-duty left-right sign combo sequence=stop2s,AnegBpos500,stop1s,AnegBneg500,stop1s,AposBpos500,stop1s,AposBneg500,stop1s,trim,stop1s,stronger,stop1s");

    MotorDriver_Init();

    App_MotorForcedSpinCheck_StopPhase("initial_stop", APP_MOTOR_FORCED_SPIN_INITIAL_STOP_MS);
    App_MotorForcedSpinCheck_RunPhase("combo_Aneg_Bpos_500",
                                      -500,
                                      500,
                                      APP_MOTOR_FORCED_SPIN_RUN_MS);
    App_MotorForcedSpinCheck_StopPhase("after_combo_Aneg_Bpos_500", APP_MOTOR_FORCED_SPIN_BETWEEN_STOP_MS);
    App_MotorForcedSpinCheck_RunPhase("combo_Aneg_Bneg_500",
                                      -500,
                                      -500,
                                      APP_MOTOR_FORCED_SPIN_RUN_MS);
    App_MotorForcedSpinCheck_StopPhase("after_combo_Aneg_Bneg_500", APP_MOTOR_FORCED_SPIN_BETWEEN_STOP_MS);
    App_MotorForcedSpinCheck_RunPhase("combo_Apos_Bpos_500",
                                      500,
                                      500,
                                      APP_MOTOR_FORCED_SPIN_RUN_MS);
    App_MotorForcedSpinCheck_StopPhase("after_combo_Apos_Bpos_500", APP_MOTOR_FORCED_SPIN_BETWEEN_STOP_MS);
    App_MotorForcedSpinCheck_RunPhase("combo_Apos_Bneg_500",
                                      500,
                                      -500,
                                      APP_MOTOR_FORCED_SPIN_RUN_MS);
    App_MotorForcedSpinCheck_StopPhase("after_combo_Apos_Bneg_500", APP_MOTOR_FORCED_SPIN_BETWEEN_STOP_MS);
    App_MotorForcedSpinCheck_RunPhase("forward_trim_test",
                                      -520,
                                      600,
                                      APP_MOTOR_FORCED_SPIN_RUN_MS);
    App_MotorForcedSpinCheck_StopPhase("after_forward_trim_test", APP_MOTOR_FORCED_SPIN_BETWEEN_STOP_MS);
    App_MotorForcedSpinCheck_RunPhase("stronger_forward_test",
                                      -600,
                                      600,
                                      APP_MOTOR_FORCED_SPIN_RUN_MS);
    App_MotorForcedSpinCheck_StopPhase("after_stronger_forward_test", APP_MOTOR_FORCED_SPIN_BETWEEN_STOP_MS);

    MotorDriver_StopAll();
    APP_LOG("[MOTOR_FORCED_SPIN] done final_stop=1");
}

static void App_MotorForcedSpinCheck_StopPhase(const char *phase, uint32_t duration_ms)
{
    uint32_t elapsed_ms = 0U;

    APP_LOG("[MOTOR_FORCED_SPIN] phase=%s motor_a_cmd=0 motor_b_cmd=0 duration_ms=%lu",
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

    APP_LOG("[MOTOR_FORCED_SPIN] phase=%s motor_a_cmd=%d motor_b_cmd=%d duration_ms=%lu",
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

#else

void App_MotorForcedSpinCheck_Run(void)
{
}

#endif
