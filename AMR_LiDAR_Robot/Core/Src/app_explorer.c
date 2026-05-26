#include "app_explorer.h"

#include "app_fault.h"
#include "app_map.h"
#include "bringup_log.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#define APP_EXPLORER_UPDATE_PERIOD_MS 100U
#define APP_EXPLORER_WAIT_LOG_MS 1000U
#define APP_EXPLORER_TURN_SKELETON_MS 100U
#define APP_EXPLORER_DIR_COUNT 4U

typedef struct
{
    int cx;
    int cy;
} AppExplorerCell_t;

static AppExplorerState_t app_exp_state = EXP_IDLE;
static AppExplorerCell_t app_exp_path[APP_EXPLORER_PATH_MAX];
static uint8_t app_exp_path_len = 0U;
static uint8_t app_exp_return_index = 0U;
static int app_exp_current_cx = 0;
static int app_exp_current_cy = 0;
static int app_exp_target_cx = 0;
static int app_exp_target_cy = 0;
static AppMapDir app_exp_heading = APP_MAP_DIR_EAST;
static AppMapDir app_exp_target_heading = APP_MAP_DIR_EAST;
static uint8_t app_exp_backtracking = 0U;
static uint8_t app_exp_initialized = 0U;
static uint32_t app_exp_last_update_ms = 0U;
static uint32_t app_exp_state_enter_ms = 0U;
static uint32_t app_exp_last_wait_log_ms = 0U;

static void AppExplorer_SetState(AppExplorerState_t next_state, const char *reason);
static void AppExplorer_SyncCurrentCell(void);
static void AppExplorer_StartPathAtCurrentCell(void);
static uint8_t AppExplorer_AppendPathCell(int cx, int cy);
static uint8_t AppExplorer_CellEquals(const AppExplorerCell_t *cell, int cx, int cy);
static uint8_t AppExplorer_InBounds(int cx, int cy);
static uint8_t AppExplorer_IsExitCell(int cx, int cy);
static uint8_t AppExplorer_SelectNextCell(int cx, int cy, AppMapDir heading, int *nx, int *ny, AppMapDir *dir);
static void AppExplorer_SetMoveTarget(int nx, int ny, AppMapDir dir, uint8_t backtracking);
static void AppExplorer_UpdateExplore(void);
static void AppExplorer_UpdateTurning(void);
static void AppExplorer_UpdateMovingToCell(void);
static void AppExplorer_UpdateReturning(void);
static void AppExplorer_StartReturnStep(void);
static uint8_t AppExplorer_DirToDelta(AppMapDir dir, int *dx, int *dy);
static AppMapDir AppExplorer_LeftOf(AppMapDir dir);
static AppMapDir AppExplorer_RightOf(AppMapDir dir);
static AppMapDir AppExplorer_BackOf(AppMapDir dir);
static uint32_t AppExplorer_ElapsedMs(uint32_t now_ms, uint32_t then_ms);

void AppExplorer_Init(void)
{
    app_exp_initialized = 1U;
    AppExplorer_Reset();
    APP_LOG("EXP: init mode=skeleton exit=(%d,%d) path_max=%u",
            APP_EXPLORER_EXIT_X,
            APP_EXPLORER_EXIT_Y,
            (unsigned int)APP_EXPLORER_PATH_MAX);
}

void AppExplorer_Reset(void)
{
    app_exp_state = EXP_IDLE;
    app_exp_path_len = 0U;
    app_exp_return_index = 0U;
    app_exp_target_cx = 0;
    app_exp_target_cy = 0;
    app_exp_target_heading = APP_MAP_DIR_EAST;
    app_exp_backtracking = 0U;
    app_exp_last_update_ms = 0U;
    app_exp_state_enter_ms = HAL_GetTick();
    app_exp_last_wait_log_ms = 0U;
    AppExplorer_SyncCurrentCell();
}

void AppExplorer_StartExplore(void)
{
    if (app_exp_initialized == 0U)
    {
        AppExplorer_Init();
    }

    AppExplorer_SyncCurrentCell();
    AppExplorer_StartPathAtCurrentCell();
    AppExplorer_SetState(EXP_EXPLORE, "start_explore");
    APP_LOG("EXP: explore start cell=(%d,%d) exit=(%d,%d)",
            app_exp_current_cx,
            app_exp_current_cy,
            APP_EXPLORER_EXIT_X,
            APP_EXPLORER_EXIT_Y);
}

void AppExplorer_StartReturn(void)
{
    if (app_exp_initialized == 0U)
    {
        AppExplorer_Init();
    }

    AppExplorer_SyncCurrentCell();
    if (app_exp_path_len == 0U)
    {
        AppExplorer_StartPathAtCurrentCell();
    }

    if ((app_exp_path_len <= 1U) && (AppExplorer_IsExitCell(app_exp_current_cx, app_exp_current_cy) == 0U))
    {
        APP_LOG("EXP: no return path");
        AppExplorer_SetState(EXP_DONE, "return_empty");
        return;
    }

    app_exp_return_index = (app_exp_path_len > 0U) ? (uint8_t)(app_exp_path_len - 1U) : 0U;
    AppExplorer_SetState(EXP_RETURNING, "start_return");
    APP_LOG("EXP: return start path_len=%u current=(%d,%d)",
            (unsigned int)app_exp_path_len,
            app_exp_current_cx,
            app_exp_current_cy);
    AppExplorer_StartReturnStep();
}

void AppExplorer_Stop(void)
{
    if (app_exp_state != EXP_IDLE)
    {
        AppExplorer_SetState(EXP_IDLE, "stop");
    }
}

void AppExplorer_Update(void)
{
    uint32_t now_ms;

    if (app_exp_initialized == 0U)
    {
        AppExplorer_Init();
    }

    now_ms = HAL_GetTick();
    if ((app_exp_last_update_ms != 0U) &&
        (AppExplorer_ElapsedMs(now_ms, app_exp_last_update_ms) < APP_EXPLORER_UPDATE_PERIOD_MS))
    {
        return;
    }
    app_exp_last_update_ms = now_ms;

    if (AppFault_IsActive())
    {
        if (app_exp_state != EXP_FAULT)
        {
            AppExplorer_SetState(EXP_FAULT, "fault_active");
            APP_LOG("EXP: fault active, planner stopped");
        }
        return;
    }

    switch (app_exp_state)
    {
        case EXP_EXPLORE:
            AppExplorer_UpdateExplore();
            break;

        case EXP_TURNING:
            AppExplorer_UpdateTurning();
            break;

        case EXP_MOVING_TO_CELL:
            AppExplorer_UpdateMovingToCell();
            break;

        case EXP_RETURNING:
            AppExplorer_UpdateReturning();
            break;

        default:
            break;
    }
}

AppExplorerState_t AppExplorer_GetState(void)
{
    return app_exp_state;
}

const char *AppExplorer_StateName(AppExplorerState_t state)
{
    switch (state)
    {
        case EXP_IDLE:
            return "EXP_IDLE";

        case EXP_EXPLORE:
            return "EXP_EXPLORE";

        case EXP_TURNING:
            return "EXP_TURNING";

        case EXP_MOVING_TO_CELL:
            return "EXP_MOVING_TO_CELL";

        case EXP_EXIT_FOUND:
            return "EXP_EXIT_FOUND";

        case EXP_RETURNING:
            return "EXP_RETURNING";

        case EXP_DONE:
            return "EXP_DONE";

        case EXP_FAULT:
            return "EXP_FAULT";

        default:
            return "EXP_UNKNOWN";
    }
}

void AppExplorer_PrintStatus(void)
{
    AppExplorer_SyncCurrentCell();
    APP_LOG("EXP: state=%s cell=(%d,%d) target=(%d,%d) heading=%c target_heading=%c path_len=%u return_index=%u mode=skeleton",
            AppExplorer_StateName(app_exp_state),
            app_exp_current_cx,
            app_exp_current_cy,
            app_exp_target_cx,
            app_exp_target_cy,
            AppMap_DirChar(app_exp_heading),
            AppMap_DirChar(app_exp_target_heading),
            (unsigned int)app_exp_path_len,
            (unsigned int)app_exp_return_index);
}

bool AppExplorer_GetStatus(AppExplorerStatus_t *status)
{
    if (status == NULL)
    {
        return false;
    }

    if (app_exp_initialized == 0U)
    {
        AppExplorer_Init();
    }

    AppExplorer_SyncCurrentCell();
    status->state = app_exp_state;
    status->current_cx = app_exp_current_cx;
    status->current_cy = app_exp_current_cy;
    status->target_cx = app_exp_target_cx;
    status->target_cy = app_exp_target_cy;
    status->heading = app_exp_heading;
    status->target_heading = app_exp_target_heading;
    status->path_len = app_exp_path_len;
    status->return_index = app_exp_return_index;
    status->skeleton_only = 1U;

    return true;
}

static void AppExplorer_SetState(AppExplorerState_t next_state, const char *reason)
{
    if (app_exp_state == next_state)
    {
        return;
    }

    APP_LOG("EXP: state %s -> %s reason=%s",
            AppExplorer_StateName(app_exp_state),
            AppExplorer_StateName(next_state),
            (reason != NULL) ? reason : "none");
    app_exp_state = next_state;
    app_exp_state_enter_ms = HAL_GetTick();
    app_exp_last_wait_log_ms = 0U;
}

static void AppExplorer_SyncCurrentCell(void)
{
    AppMapSummary_t map_summary;

    if (AppMap_GetSummary(&map_summary) == false)
    {
        AppMap_GetRobotCell(&app_exp_current_cx, &app_exp_current_cy);
        return;
    }

    app_exp_current_cx = map_summary.robot_cx;
    app_exp_current_cy = map_summary.robot_cy;
    app_exp_heading = map_summary.heading;
}

static void AppExplorer_StartPathAtCurrentCell(void)
{
    app_exp_path_len = 0U;
    (void)AppExplorer_AppendPathCell(app_exp_current_cx, app_exp_current_cy);
}

static uint8_t AppExplorer_AppendPathCell(int cx, int cy)
{
    if (AppExplorer_InBounds(cx, cy) == 0U)
    {
        return 0U;
    }

    if ((app_exp_path_len > 0U) &&
        (AppExplorer_CellEquals(&app_exp_path[app_exp_path_len - 1U], cx, cy) != 0U))
    {
        return 1U;
    }

    if (app_exp_path_len >= APP_EXPLORER_PATH_MAX)
    {
        APP_LOG("EXP: path full drop cell=(%d,%d)", cx, cy);
        return 0U;
    }

    app_exp_path[app_exp_path_len].cx = cx;
    app_exp_path[app_exp_path_len].cy = cy;
    app_exp_path_len++;

    return 1U;
}

static uint8_t AppExplorer_CellEquals(const AppExplorerCell_t *cell, int cx, int cy)
{
    if (cell == NULL)
    {
        return 0U;
    }

    return ((cell->cx == cx) && (cell->cy == cy)) ? 1U : 0U;
}

static uint8_t AppExplorer_InBounds(int cx, int cy)
{
    return ((cx >= 0) && (cx < APP_MAP_W) && (cy >= 0) && (cy < APP_MAP_H)) ? 1U : 0U;
}

static uint8_t AppExplorer_IsExitCell(int cx, int cy)
{
    return ((cx == APP_EXPLORER_EXIT_X) && (cy == APP_EXPLORER_EXIT_Y)) ? 1U : 0U;
}

static uint8_t AppExplorer_SelectNextCell(int cx, int cy, AppMapDir heading, int *nx, int *ny, AppMapDir *dir)
{
    AppMapDir order[APP_EXPLORER_DIR_COUNT];
    uint32_t i;

    order[0] = heading;
    order[1] = AppExplorer_LeftOf(heading);
    order[2] = AppExplorer_RightOf(heading);
    order[3] = AppExplorer_BackOf(heading);

    for (i = 0U; i < APP_EXPLORER_DIR_COUNT; i++)
    {
        int dx = 0;
        int dy = 0;
        int tx;
        int ty;
        uint8_t known = 0U;
        uint8_t has_wall = 0U;

        if (AppExplorer_DirToDelta(order[i], &dx, &dy) == 0U)
        {
            continue;
        }

        tx = cx + dx;
        ty = cy + dy;
        if (AppExplorer_InBounds(tx, ty) == 0U)
        {
            continue;
        }

        AppMap_GetWall(cx, cy, order[i], &known, &has_wall);
        if ((known != 0U) && (has_wall != 0U))
        {
            continue;
        }

        if (AppMap_IsVisited(tx, ty) == false)
        {
            if (nx != NULL)
            {
                *nx = tx;
            }

            if (ny != NULL)
            {
                *ny = ty;
            }

            if (dir != NULL)
            {
                *dir = order[i];
            }

            return 1U;
        }
    }

    return 0U;
}

static void AppExplorer_SetMoveTarget(int nx, int ny, AppMapDir dir, uint8_t backtracking)
{
    app_exp_target_cx = nx;
    app_exp_target_cy = ny;
    app_exp_target_heading = dir;
    app_exp_backtracking = backtracking;
    APP_LOG("EXP: plan turn %c then move to cell=(%d,%d)%s",
            AppMap_DirChar(dir),
            nx,
            ny,
            (backtracking != 0U) ? " backtrack" : "");
    AppExplorer_SetState(EXP_TURNING, (backtracking != 0U) ? "backtrack_plan" : "frontier_plan");
}

static void AppExplorer_UpdateExplore(void)
{
    int nx = 0;
    int ny = 0;
    AppMapDir dir = APP_MAP_DIR_EAST;

    AppExplorer_SyncCurrentCell();
    AppMap_MarkVisited(app_exp_current_cx, app_exp_current_cy);

    if (app_exp_path_len == 0U)
    {
        AppExplorer_StartPathAtCurrentCell();
    }
    else if (AppExplorer_CellEquals(&app_exp_path[app_exp_path_len - 1U],
                                    app_exp_current_cx,
                                    app_exp_current_cy) == 0U)
    {
        (void)AppExplorer_AppendPathCell(app_exp_current_cx, app_exp_current_cy);
    }

    if (AppExplorer_IsExitCell(app_exp_current_cx, app_exp_current_cy) != 0U)
    {
        APP_LOG("EXP: exit found cell=(%d,%d)", app_exp_current_cx, app_exp_current_cy);
        AppExplorer_SetState(EXP_EXIT_FOUND, "exit_cell");
        return;
    }

    if (AppExplorer_SelectNextCell(app_exp_current_cx, app_exp_current_cy, app_exp_heading, &nx, &ny, &dir) != 0U)
    {
        AppExplorer_SetMoveTarget(nx, ny, dir, 0U);
        return;
    }

    if (app_exp_path_len > 1U)
    {
        nx = app_exp_path[app_exp_path_len - 2U].cx;
        ny = app_exp_path[app_exp_path_len - 2U].cy;
        if (nx > app_exp_current_cx)
        {
            dir = APP_MAP_DIR_EAST;
        }
        else if (nx < app_exp_current_cx)
        {
            dir = APP_MAP_DIR_WEST;
        }
        else if (ny > app_exp_current_cy)
        {
            dir = APP_MAP_DIR_NORTH;
        }
        else
        {
            dir = APP_MAP_DIR_SOUTH;
        }
        AppExplorer_SetMoveTarget(nx, ny, dir, 1U);
        return;
    }

    APP_LOG("EXP: frontier exhausted at cell=(%d,%d)", app_exp_current_cx, app_exp_current_cy);
    AppExplorer_SetState(EXP_DONE, "frontier_exhausted");
}

static void AppExplorer_UpdateTurning(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (AppExplorer_ElapsedMs(now_ms, app_exp_state_enter_ms) < APP_EXPLORER_TURN_SKELETON_MS)
    {
        return;
    }

    APP_LOG("EXP: action skeleton move to cell=(%d,%d) heading=%c",
            app_exp_target_cx,
            app_exp_target_cy,
            AppMap_DirChar(app_exp_target_heading));
    AppExplorer_SetState(EXP_MOVING_TO_CELL, "turn_done_skeleton");
}

static void AppExplorer_UpdateMovingToCell(void)
{
    uint32_t now_ms = HAL_GetTick();

    AppExplorer_SyncCurrentCell();
    if ((app_exp_current_cx == app_exp_target_cx) &&
        (app_exp_current_cy == app_exp_target_cy))
    {
        AppMap_MarkVisited(app_exp_current_cx, app_exp_current_cy);
        if (app_exp_backtracking != 0U)
        {
            if (app_exp_path_len > 1U)
            {
                app_exp_path_len--;
            }
        }
        else
        {
            (void)AppExplorer_AppendPathCell(app_exp_current_cx, app_exp_current_cy);
        }

        APP_LOG("EXP: reached cell=(%d,%d) path_len=%u",
                app_exp_current_cx,
                app_exp_current_cy,
                (unsigned int)app_exp_path_len);

        if (AppExplorer_IsExitCell(app_exp_current_cx, app_exp_current_cy) != 0U)
        {
            APP_LOG("EXP: exit found cell=(%d,%d)", app_exp_current_cx, app_exp_current_cy);
            AppExplorer_SetState(EXP_EXIT_FOUND, "exit_cell");
        }
        else
        {
            AppExplorer_SetState(EXP_EXPLORE, "cell_reached");
        }
        return;
    }

    if ((app_exp_last_wait_log_ms == 0U) ||
        (AppExplorer_ElapsedMs(now_ms, app_exp_last_wait_log_ms) >= APP_EXPLORER_WAIT_LOG_MS))
    {
        APP_LOG("EXP: waiting current=(%d,%d) target=(%d,%d) mode=skeleton",
                app_exp_current_cx,
                app_exp_current_cy,
                app_exp_target_cx,
                app_exp_target_cy);
        app_exp_last_wait_log_ms = now_ms;
    }
}

static void AppExplorer_UpdateReturning(void)
{
    uint32_t now_ms = HAL_GetTick();

    AppExplorer_SyncCurrentCell();
    if ((app_exp_current_cx == 0) && (app_exp_current_cy == 0))
    {
        APP_LOG("EXP: return complete");
        AppExplorer_SetState(EXP_DONE, "return_complete");
        return;
    }

    if ((app_exp_current_cx == app_exp_target_cx) &&
        (app_exp_current_cy == app_exp_target_cy))
    {
        if (app_exp_return_index > 0U)
        {
            app_exp_return_index--;
        }

        AppExplorer_StartReturnStep();
        return;
    }

    if ((app_exp_last_wait_log_ms == 0U) ||
        (AppExplorer_ElapsedMs(now_ms, app_exp_last_wait_log_ms) >= APP_EXPLORER_WAIT_LOG_MS))
    {
        APP_LOG("EXP: return step to cell=(%d,%d) current=(%d,%d) mode=skeleton",
                app_exp_target_cx,
                app_exp_target_cy,
                app_exp_current_cx,
                app_exp_current_cy);
        app_exp_last_wait_log_ms = now_ms;
    }
}

static void AppExplorer_StartReturnStep(void)
{
    if (app_exp_return_index == 0U)
    {
        APP_LOG("EXP: return complete");
        AppExplorer_SetState(EXP_DONE, "return_complete");
        return;
    }

    app_exp_target_cx = app_exp_path[app_exp_return_index - 1U].cx;
    app_exp_target_cy = app_exp_path[app_exp_return_index - 1U].cy;
    APP_LOG("EXP: return step to cell=(%d,%d) remaining=%u",
            app_exp_target_cx,
            app_exp_target_cy,
            (unsigned int)app_exp_return_index);
}

static uint8_t AppExplorer_DirToDelta(AppMapDir dir, int *dx, int *dy)
{
    if ((dx == NULL) || (dy == NULL))
    {
        return 0U;
    }

    *dx = 0;
    *dy = 0;

    switch (dir)
    {
        case APP_MAP_DIR_NORTH:
            *dy = 1;
            return 1U;

        case APP_MAP_DIR_EAST:
            *dx = 1;
            return 1U;

        case APP_MAP_DIR_SOUTH:
            *dy = -1;
            return 1U;

        case APP_MAP_DIR_WEST:
            *dx = -1;
            return 1U;

        default:
            return 0U;
    }
}

static AppMapDir AppExplorer_LeftOf(AppMapDir dir)
{
    switch (dir)
    {
        case APP_MAP_DIR_NORTH:
            return APP_MAP_DIR_WEST;

        case APP_MAP_DIR_EAST:
            return APP_MAP_DIR_NORTH;

        case APP_MAP_DIR_SOUTH:
            return APP_MAP_DIR_EAST;

        case APP_MAP_DIR_WEST:
        default:
            return APP_MAP_DIR_SOUTH;
    }
}

static AppMapDir AppExplorer_RightOf(AppMapDir dir)
{
    switch (dir)
    {
        case APP_MAP_DIR_NORTH:
            return APP_MAP_DIR_EAST;

        case APP_MAP_DIR_EAST:
            return APP_MAP_DIR_SOUTH;

        case APP_MAP_DIR_SOUTH:
            return APP_MAP_DIR_WEST;

        case APP_MAP_DIR_WEST:
        default:
            return APP_MAP_DIR_NORTH;
    }
}

static AppMapDir AppExplorer_BackOf(AppMapDir dir)
{
    switch (dir)
    {
        case APP_MAP_DIR_NORTH:
            return APP_MAP_DIR_SOUTH;

        case APP_MAP_DIR_EAST:
            return APP_MAP_DIR_WEST;

        case APP_MAP_DIR_SOUTH:
            return APP_MAP_DIR_NORTH;

        case APP_MAP_DIR_WEST:
        default:
            return APP_MAP_DIR_EAST;
    }
}

static uint32_t AppExplorer_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    if (then_ms == 0U)
    {
        return UINT32_MAX;
    }

    if (now_ms >= then_ms)
    {
        return now_ms - then_ms;
    }

    return 0U;
}
