#ifndef APP_ODOMETRY_H
#define APP_ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

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

#ifdef __cplusplus
}
#endif

#endif /* APP_ODOMETRY_H */
