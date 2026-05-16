#ifndef PID_H
#define PID_H

/*
 * PID control module.
 *
 * This module will contain reusable PID control helpers for motor speed,
 * heading, and other closed-loop control tasks.
 */

void PID_Init(void);
float PID_Update(float setpoint, float measurement);
void PID_Reset(void);

#endif /* PID_H */
