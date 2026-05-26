#ifndef APP_FAULT_H
#define APP_FAULT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    FAULT_NONE = 0,
    FAULT_LIDAR_TIMEOUT,
    FAULT_ENCODER_STALL,
    FAULT_PLANNER_STUCK,
    FAULT_USER_ESTOP
} AppFaultCode;

void AppFault_Init(void);
void AppFault_Set(AppFaultCode code);
void AppFault_Clear(void);
AppFaultCode AppFault_Get(void);
bool AppFault_IsActive(void);
const char *AppFault_Name(AppFaultCode code);

#ifdef __cplusplus
}
#endif

#endif /* APP_FAULT_H */
