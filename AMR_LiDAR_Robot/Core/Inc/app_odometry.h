#ifndef APP_ODOMETRY_H
#define APP_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Differential-drive odometry calibration.
 *
 * ODO_ENCODER_TICKS_PER_REV is the encoder count per motor shaft revolution.
 * ODO_GEAR_RATIO converts motor-shaft revolutions to wheel revolutions. If
 * the encoder is already measured at the wheel, keep ODO_GEAR_RATIO at 1.0.
 */
#ifndef ODO_WHEEL_RADIUS_M
#define ODO_WHEEL_RADIUS_M 0.0325f
#endif

#ifndef ODO_WHEEL_BASE_M
#define ODO_WHEEL_BASE_M 0.150f
#endif

#ifndef ODO_ENCODER_TICKS_PER_REV
#define ODO_ENCODER_TICKS_PER_REV 390.0f
#endif

#ifndef ODO_GEAR_RATIO
#define ODO_GEAR_RATIO 1.0f
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x_m;
    float y_m;
    float theta_rad;
} OdomPose_t;

typedef struct
{
    int32_t raw_left_delta;
    int32_t raw_right_delta;
    int32_t signed_left_delta;
    int32_t signed_right_delta;
    float delta_left_m;
    float delta_right_m;
    uint32_t last_update_ms;
} OdomSample_t;

void Odom_Init(void);
void Odom_Update(void);
void Odom_Reset(void);
bool Odom_GetPose(OdomPose_t *pose);
bool Odom_GetLastSample(OdomSample_t *sample);

void AppOdo_Init(void);
void AppOdo_Reset(void);
void AppOdo_Update(float dt_s);
void AppOdo_GetPose(float *x, float *y, float *theta);
void AppOdo_GetVelocity(float *v, float *w);

#ifdef __cplusplus
}
#endif

#endif /* APP_ODOMETRY_H */
