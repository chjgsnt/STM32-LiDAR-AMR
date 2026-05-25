#ifndef AMR_SYSTEM_H
#define AMR_SYSTEM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    AMR_STATE_IDLE = 0,
    AMR_STATE_EXPLORE,
    AMR_STATE_AVOID,
    AMR_STATE_RETURN,
    AMR_STATE_FAULT,
    AMR_STATE_ESTOP
} AMR_State_t;

void AMR_Init(void);
void AMR_StateMachine_Update(void);

AMR_State_t AMR_GetState(void);
uint32_t AMR_GetStateEnterMs(void);
const char *AMR_StateName(AMR_State_t state);
const char *AMR_StateShortName(AMR_State_t state);
uint8_t AMR_SetState(AMR_State_t next_state, const char *reason);

void AMR_RequestStart(const char *reason);
void AMR_RequestStop(const char *reason);
void AMR_RequestReturn(const char *reason);
void AMR_RequestEStop(const char *reason);
void AMR_RequestResetFault(const char *reason);

#ifdef __cplusplus
}
#endif

#endif /* AMR_SYSTEM_H */
