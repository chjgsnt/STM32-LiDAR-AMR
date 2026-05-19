#ifndef MPU6500_H
#define MPU6500_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float accel_pitch_deg;
    float accel_roll_deg;
    float fused_pitch_deg;
    float fused_roll_deg;
    uint32_t last_update_ms;
    uint8_t is_ready;
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
} MPU6500_Data_t;

const MPU6500_Data_t *MPU6500_GetData(void);
bool MPU6500_GetLatest(MPU6500_Data_t *out);

#endif /* MPU6500_H */
