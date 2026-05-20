#include "wheel_speed_controller.h"

static int32_t WheelSpeedController_ClampI32(int32_t value, int32_t min_value, int32_t max_value);
static int16_t WheelSpeedController_ClampDuty(int32_t value, int16_t min_value, int16_t max_value);

void WheelSpeedController_Reset(WheelSpeedController_State_t *state)
{
    if (state == 0)
    {
        return;
    }

    state->error = 0;
    state->integral = 0;
    state->duty = 0;
}

int16_t WheelSpeedController_Update(WheelSpeedController_State_t *state,
                                    const WheelSpeedController_Config_t *config,
                                    int32_t measured_ticks)
{
    if ((state == 0) || (config == 0) || (config->gain_den == 0))
    {
        return 0;
    }

    state->error = config->target_ticks_per_sample - measured_ticks;
    state->integral = WheelSpeedController_ClampI32(state->integral + state->error,
                                                    -config->integral_limit,
                                                    config->integral_limit);

    int32_t control = (int32_t)config->feedforward_duty;
    control += ((config->kp_num * state->error) + (config->ki_num * state->integral)) /
               config->gain_den;

    state->duty = WheelSpeedController_ClampDuty(control, config->duty_min, config->duty_max);

    return state->duty;
}

static int32_t WheelSpeedController_ClampI32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static int16_t WheelSpeedController_ClampDuty(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return (int16_t)value;
}
