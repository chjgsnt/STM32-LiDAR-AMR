#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/*
 * Motor control module.
 *
 * This module will convert motion commands into motor direction and PWM output
 * signals for the mobile robot chassis.
 */

void Motor_Init(void);
void Motor_SetPwm(int16_t left_pwm, int16_t right_pwm);
void Motor_Stop(void);

#endif /* MOTOR_H */
