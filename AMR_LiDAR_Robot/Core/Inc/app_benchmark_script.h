#ifndef APP_BENCHMARK_SCRIPT_H
#define APP_BENCHMARK_SCRIPT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SCRIPT_IDLE = 0,
    SCRIPT_RUNNING,
    SCRIPT_WAIT_OBSTACLE_CLEAR,
    SCRIPT_AUTO_FORWARD,
    SCRIPT_AUTO_TURN_RIGHT,
    SCRIPT_AUTO_WAIT_CLEAR,
    SCRIPT_RETURN_AUTO_TURN_AROUND,
    SCRIPT_RETURN_AUTO_FORWARD,
    SCRIPT_RETURN_AUTO_TURN_LEFT,
    SCRIPT_RETURN_AUTO_WAIT_CLEAR,
    SCRIPT_DONE,
    SCRIPT_ABORTED,
    SCRIPT_FAULT
} BenchmarkScriptState_t;

typedef enum
{
    SCRIPT_ACT_STOP = 0,
    SCRIPT_ACT_WAIT,
    SCRIPT_ACT_FORWARD,
    SCRIPT_ACT_BACKUP,
    SCRIPT_ACT_TURN_LEFT,
    SCRIPT_ACT_TURN_RIGHT,
    SCRIPT_ACT_TURN_AROUND
} ScriptAction;

void AppBenchmarkScript_Init(void);
void AppBenchmarkScript_StartExit(void);
void AppBenchmarkScript_StartReturn(void);
void AppBenchmarkScript_StartAuto(void);
void AppBenchmarkScript_StartReturnAuto(void);
void AppBenchmarkScript_SetRouteExit(const char *tokens);
void AppBenchmarkScript_SetRouteReturn(const char *tokens);
void AppBenchmarkScript_StartRouteExit(void);
void AppBenchmarkScript_StartRouteReturn(void);
uint8_t AppBenchmarkRoute_HasExit(void);
uint8_t AppBenchmarkRoute_HasReturn(void);
void AppBenchmarkRoute_RunExit(void);
void AppBenchmarkRoute_RunReturn(void);
uint8_t AppBenchmarkRoute_IsActive(void);
uint8_t AppBenchmarkRoute_IsReturnActive(void);
void AppBenchmarkRoute_Stop(const char *reason);
void AppBenchmarkScript_PrintRouteStatus(void);
void AppBenchmarkScript_ClearRoutes(void);
void AppBenchmarkScript_StartBenchForward(uint32_t duration_ms);
void AppBenchmarkScript_PrintBenchStatus(void);
void AppBenchmarkScript_Stop(const char *reason);
void AppBenchmarkScript_Reset(void);
void AppBenchmarkScript_Update(void);
void AppBenchmarkScript_PrintStatus(void);
uint8_t AppBenchmarkScript_IsActive(void);
uint8_t AppBenchmarkScript_IsAutoActive(void);
uint8_t AppBenchmarkScript_IsReturnActive(void);
uint8_t AppBenchmarkScript_IsReturnAutoActive(void);
BenchmarkScriptState_t AppBenchmarkScript_GetState(void);
const char *AppBenchmarkScript_StateName(BenchmarkScriptState_t state);
const char *AppBenchmarkScript_ActionName(ScriptAction action);

#ifdef __cplusplus
}
#endif

#endif /* APP_BENCHMARK_SCRIPT_H */
