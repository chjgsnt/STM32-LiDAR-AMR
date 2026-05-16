#include "motor.h"

void Motor_Init(void)
{
    /* TODO: Initialize motor control GPIO and PWM channels. */
}

void Motor_SetPwm(int16_t left_pwm, int16_t right_pwm)
{
    (void)left_pwm;
    (void)right_pwm;
    /* TODO: Apply signed PWM commands to the motor driver. */
}

void Motor_Stop(void)
{
    /* TODO: Disable motor outputs safely. */
}
