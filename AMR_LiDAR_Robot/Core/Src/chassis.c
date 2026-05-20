#include "chassis.h"

#include "motor_driver.h"

static int16_t Chassis_ClampDuty(int16_t duty);
static int16_t Chassis_ApplySign(int16_t duty, int16_t sign);

void Chassis_Init(void)
{
    MotorDriver_Init();
    Chassis_Stop();
}

void Chassis_Stop(void)
{
    MotorDriver_StopAll();
}

void Chassis_SetRaw(int16_t left_duty, int16_t right_duty)
{
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
