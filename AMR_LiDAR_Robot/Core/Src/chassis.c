#define CHASSIS_INTERNAL_IMPLEMENTATION

#include "chassis.h"

#include "bringup_log.h"
#include "motor_driver.h"
#include "stm32f4xx_hal.h"

#define CHASSIS_STOP_LOG_INTERVAL_MS 500U
#define CHASSIS_VERBOSE_LOGS (APP_DEBUG_VERBOSE || APP_DEBUG_MOTOR_VERBOSE)

static int16_t Chassis_ClampDuty(int16_t duty);
static int16_t Chassis_ApplySign(int16_t duty, int16_t sign);
static void Chassis_LogStopExecuted(void);
static void Chassis_RecordCommand(int16_t left_duty, int16_t right_duty);

static ChassisCommandStatus_t chassis_last_command = {0, 0, 0U};

void Chassis_Init(void)
{
    MotorDriver_Init();
    Chassis_Stop();
}

void Chassis_Stop(void)
{
    Chassis_RecordCommand(0, 0);
    MotorDriver_StopAll();
    Chassis_LogStopExecuted();
}

void Chassis_SetRaw(int16_t left_duty, int16_t right_duty)
{
    Chassis_RecordCommand(left_duty, right_duty);
    MotorDriver_SetMotorA(Chassis_ApplySign(left_duty, CHASSIS_LEFT_SIGN));
    MotorDriver_SetMotorB(Chassis_ApplySign(right_duty, CHASSIS_RIGHT_SIGN));
}

void Chassis_Forward(int16_t duty)
{
    Chassis_SetRaw(duty, duty);
}

void Chassis_Backward(int16_t duty)
{
    Chassis_SetRaw((int16_t)-duty, (int16_t)-duty);
}

void Chassis_TurnLeft(int16_t duty)
{
    Chassis_SetRaw((int16_t)-duty, duty);
}

void Chassis_TurnRight(int16_t duty)
{
    Chassis_SetRaw(duty, (int16_t)-duty);
}

void Chassis_GetLastCommand(ChassisCommandStatus_t *status)
{
    if (status == NULL)
    {
        return;
    }

    *status = chassis_last_command;
}

static int16_t Chassis_ClampDuty(int16_t duty)
{
    if (duty > CHASSIS_DUTY_MAX)
    {
        duty = CHASSIS_DUTY_MAX;
    }
    else if (duty < CHASSIS_DUTY_MIN)
    {
        duty = CHASSIS_DUTY_MIN;
    }

    if (duty > CHASSIS_MAX_OPENLOOP_DUTY)
    {
        duty = CHASSIS_MAX_OPENLOOP_DUTY;
    }
    else if (duty < -CHASSIS_MAX_OPENLOOP_DUTY)
    {
        duty = -CHASSIS_MAX_OPENLOOP_DUTY;
    }

    return duty;
}

static int16_t Chassis_ApplySign(int16_t duty, int16_t sign)
{
    duty = Chassis_ClampDuty(duty);

    if (sign < 0)
    {
        return (int16_t)-duty;
    }

    return duty;
}

static void Chassis_RecordCommand(int16_t left_duty, int16_t right_duty)
{
    chassis_last_command.left_duty = Chassis_ClampDuty(left_duty);
    chassis_last_command.right_duty = Chassis_ClampDuty(right_duty);
    chassis_last_command.last_update_ms = HAL_GetTick();
}

static void Chassis_LogStopExecuted(void)
{
#if CHASSIS_VERBOSE_LOGS
    static uint32_t last_log_ms = 0U;
    static uint8_t has_logged = 0U;
    uint32_t now_ms = HAL_GetTick();

    if ((has_logged == 0U) || ((now_ms - last_log_ms) >= CHASSIS_STOP_LOG_INTERVAL_MS))
    {
        last_log_ms = now_ms;
        has_logged = 1U;
        APP_LOG("CHASSIS: stop executed=1");
    }
#endif
}
