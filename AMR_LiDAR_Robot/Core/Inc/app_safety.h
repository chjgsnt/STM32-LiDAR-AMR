#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_FAULT_NONE = 0,
    APP_FAULT_LIDAR_TIMEOUT,
    APP_FAULT_ENCODER_STALL
} AppFaultCode_t;

typedef struct
{
    AppFaultCode_t fault_code;
    uint32_t fault_time_ms;
    uint32_t last_lidar_update_ms;
    int16_t pwm_left;
    int16_t pwm_right;
    int32_t raw_left_delta;
    int32_t raw_right_delta;
} AppSafetyStatus_t;

void App_Safety_Init(void);
void App_Safety_Update(void);
void App_Safety_ClearFault(void);
bool App_Safety_GetStatus(AppSafetyStatus_t *status);
const char *App_Safety_FaultName(AppFaultCode_t code);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAFETY_H */
