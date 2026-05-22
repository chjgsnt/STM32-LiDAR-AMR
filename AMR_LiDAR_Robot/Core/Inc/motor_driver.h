#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "app_test_config.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_DRIVER_DUTY_MIN (-1000)
#define MOTOR_DRIVER_DUTY_MAX (1000)
#define MOTOR_DRIVER_BRINGUP_DUTY_LIMIT (600)

void MotorDriver_Init(void);
void MotorDriver_StopAll(void);
void MotorDriver_SetMotorA(int16_t duty);
void MotorDriver_SetMotorB(int16_t duty);

int32_t MotorDriver_GetEncoderA(void);
int32_t MotorDriver_GetEncoderB(void);
void MotorDriver_ResetEncoders(void);

#if APP_LIDAR_OBSTACLE_STOP_CHECK_ENABLE && \
    !defined(APP_LIDAR_STOP_CHECK_MOTOR_OWNER) && \
    !defined(CHASSIS_INTERNAL_IMPLEMENTATION) && \
    !defined(MOTOR_DRIVER_INTERNAL_IMPLEMENTATION)
#pragma GCC poison MotorDriver_Init MotorDriver_StopAll MotorDriver_SetMotorA MotorDriver_SetMotorB
#endif

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
