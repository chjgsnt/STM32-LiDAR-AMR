#ifndef CHASSIS_H
#define CHASSIS_H

#include "app_test_config.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CHASSIS_LEFT_SIGN
#define CHASSIS_LEFT_SIGN  (-1)
#endif

#ifndef CHASSIS_RIGHT_SIGN
#define CHASSIS_RIGHT_SIGN (1)
#endif

#define CHASSIS_DUTY_MIN (-1000)
#define CHASSIS_DUTY_MAX (1000)
#define CHASSIS_AIR_TEST_DUTY (300)
#define CHASSIS_GROUND_TEST_DUTY (500)
#define CHASSIS_MAX_OPENLOOP_DUTY (600)

void Chassis_Init(void);
void Chassis_Stop(void);
void Chassis_SetRaw(int16_t left_duty, int16_t right_duty);
void Chassis_Forward(int16_t duty);
void Chassis_Backward(int16_t duty);
void Chassis_TurnLeft(int16_t duty);
void Chassis_TurnRight(int16_t duty);

#if APP_LIDAR_OBSTACLE_STOP_CHECK_ENABLE && \
    !defined(APP_LIDAR_STOP_CHECK_MOTOR_OWNER) && \
    !defined(CHASSIS_INTERNAL_IMPLEMENTATION)
#pragma GCC poison Chassis_Init Chassis_Stop Chassis_SetRaw Chassis_Forward Chassis_Backward Chassis_TurnLeft Chassis_TurnRight
#endif

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
