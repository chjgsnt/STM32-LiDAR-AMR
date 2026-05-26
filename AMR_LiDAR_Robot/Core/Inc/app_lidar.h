#ifndef APP_LIDAR_H
#define APP_LIDAR_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t ready;
    uint32_t rx_bytes;
    uint32_t valid_points;
    uint16_t front_min_mm;
    uint16_t front_wide_min_mm;
    uint16_t left_min_mm;
    uint16_t right_min_mm;
    uint8_t front_valid;
    uint8_t front_wide_valid;
    uint8_t left_valid;
    uint8_t right_valid;
    float nearest_angle_deg;
    uint16_t nearest_distance_mm;
    uint32_t last_rx_tick_ms;
    uint32_t last_valid_update_ms;
    uint32_t last_update_ms;
    uint32_t uart4_error_count;
    uint32_t uart4_error_code;
} AppLidarStatus;

/*
 * front/front_wide/left/right distance fields use 0xFFFF when the corresponding valid flag is 0.
 * nearest_distance_mm also uses 0xFFFF when no nearest reliable point was observed.
 * last_rx_tick_ms is the HAL tick when UART4 last received any byte.
 * last_valid_update_ms and last_update_ms are the HAL tick when the parser last accepted a valid scan distance node.
 * The returned pointer is owned by app_lidar and is updated by App_Lidar_Task().
 */
const AppLidarStatus *App_Lidar_GetStatus(void);
void App_Lidar_Init(void);
void App_Lidar_Task(void);
void App_Lidar_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* APP_LIDAR_H */
