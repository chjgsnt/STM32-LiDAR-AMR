#ifndef APP_EXPLORER_H
#define APP_EXPLORER_H

#include "app_map.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef APP_EXPLORER_EXIT_X
#define APP_EXPLORER_EXIT_X (APP_MAP_W - 1)
#endif

#ifndef APP_EXPLORER_EXIT_Y
#define APP_EXPLORER_EXIT_Y (APP_MAP_H - 1)
#endif

#ifndef APP_EXPLORER_PATH_MAX
#define APP_EXPLORER_PATH_MAX 25U
#endif

typedef enum
{
    EXP_IDLE = 0,
    EXP_EXPLORE,
    EXP_TURNING,
    EXP_MOVING_TO_CELL,
    EXP_EXIT_FOUND,
    EXP_RETURNING,
    EXP_DONE,
    EXP_FAULT
} AppExplorerState_t;

typedef struct
{
    AppExplorerState_t state;
    int current_cx;
    int current_cy;
    int target_cx;
    int target_cy;
    AppMapDir heading;
    AppMapDir target_heading;
    uint8_t path_len;
    uint8_t return_index;
    uint8_t skeleton_only;
} AppExplorerStatus_t;

void AppExplorer_Init(void);
void AppExplorer_Reset(void);
void AppExplorer_StartExplore(void);
void AppExplorer_StartReturn(void);
void AppExplorer_Stop(void);
void AppExplorer_Update(void);
AppExplorerState_t AppExplorer_GetState(void);
const char *AppExplorer_StateName(AppExplorerState_t state);
void AppExplorer_PrintStatus(void);
bool AppExplorer_GetStatus(AppExplorerStatus_t *status);

#endif /* APP_EXPLORER_H */
