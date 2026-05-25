#ifndef APP_RETURN_PATH_H
#define APP_RETURN_PATH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PATH_ACT_NONE = 0,
    PATH_ACT_FORWARD,
    PATH_ACT_BACKUP,
    PATH_ACT_TURN_LEFT,
    PATH_ACT_TURN_RIGHT,
    PATH_ACT_TURN_AROUND,
    PATH_ACT_STOP
} PathAction_t;

#define PATH_STACK_MAX 128U

typedef enum
{
    RETURN_EXEC_IDLE = 0,
    RETURN_EXEC_RUNNING,
    RETURN_EXEC_DONE,
    RETURN_EXEC_EMPTY
} ReturnExecState_t;

void ReturnPath_Init(void);
void ReturnPath_Reset(void);
void ReturnPath_Record(PathAction_t action, uint32_t duration_ms, int16_t left_pwm, int16_t right_pwm);
bool ReturnPath_HasActions(void);
bool ReturnPath_PopInverse(PathAction_t *action,
                           uint32_t *duration_ms,
                           int16_t *left_pwm,
                           int16_t *right_pwm);
uint16_t ReturnPath_Count(void);
void ReturnPath_Dump(void);
const char *ReturnPath_ActionName(PathAction_t action);

void ReturnExecutor_Start(void);
void ReturnExecutor_Stop(const char *reason);
void ReturnExecutor_Update(void);
ReturnExecState_t ReturnExecutor_GetState(void);
const char *ReturnExecutor_StateName(ReturnExecState_t state);

#ifdef __cplusplus
}
#endif

#endif /* APP_RETURN_PATH_H */
