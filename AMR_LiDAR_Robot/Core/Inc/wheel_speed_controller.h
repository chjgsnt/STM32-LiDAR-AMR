#ifndef WHEEL_SPEED_CONTROLLER_H
#define WHEEL_SPEED_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t target_ticks_per_sample;
    int16_t feedforward_duty;
    int32_t kp_num;
    int32_t ki_num;
    int32_t gain_den;
    int32_t integral_limit;
    int16_t duty_min;
    int16_t duty_max;
} WheelSpeedController_Config_t;

typedef struct
{
    int32_t error;
    int32_t integral;
    int16_t duty;
} WheelSpeedController_State_t;

void WheelSpeedController_Reset(WheelSpeedController_State_t *state);
int16_t WheelSpeedController_Update(WheelSpeedController_State_t *state,
                                    const WheelSpeedController_Config_t *config,
                                    int32_t measured_ticks);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_SPEED_CONTROLLER_H */
