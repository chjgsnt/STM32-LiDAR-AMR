#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_DRIVER_DUTY_MIN (-1000)
#define MOTOR_DRIVER_DUTY_MAX (1000)
#define MOTOR_DRIVER_BRINGUP_DUTY_LIMIT (300)

void MotorDriver_Init(void);
void MotorDriver_StopAll(void);
void MotorDriver_SetMotorA(int16_t duty);
void MotorDriver_SetMotorB(int16_t duty);

int32_t MotorDriver_GetEncoderA(void);
int32_t MotorDriver_GetEncoderB(void);
void MotorDriver_ResetEncoders(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
