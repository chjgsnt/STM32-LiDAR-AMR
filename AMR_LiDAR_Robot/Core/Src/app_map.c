#include "app_map.h"

#include "amr_system.h"
#include "app_fault.h"
#include "app_lidar.h"
#include "app_odometry.h"
#include "bringup_log.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <string.h>

#define APP_MAP_UPDATE_PERIOD_MS 200U
#define APP_MAP_SUMMARY_LOG_INTERVAL_MS 5000U
#define APP_MAP_BOUNDS_WARN_INTERVAL_MS 5000U
#define APP_MAP_LIDAR_MAX_AGE_MS 1000U
#define APP_MAP_DIR_COUNT 4U
#define APP_MAP_PI 3.14159265358979323846f

static AppMapCell_t app_map_cells[APP_MAP_H][APP_MAP_W];
static int app_map_robot_cx = 0;
static int app_map_robot_cy = 0;
static AppMapDir app_map_heading = APP_MAP_DIR_EAST;
static uint32_t app_map_last_update_ms = 0U;
static uint32_t app_map_last_summary_ms = 0U;
static uint32_t app_map_last_bounds_warn_ms = 0U;
static uint8_t app_map_initialized = 0U;

static uint8_t AppMap_InBounds(int cx, int cy);
static uint8_t AppMap_StateAllowsUpdate(AMR_State_t state);
static void AppMap_SetWallLocal(int cx, int cy, AppMapDir dir, uint8_t has_wall);
static uint8_t *AppMap_WallPtr(AppMapCell_t *cell, AppMapDir dir);
static uint8_t *AppMap_KnownPtr(AppMapCell_t *cell, AppMapDir dir);
static AppMapDir AppMap_OppositeDir(AppMapDir dir);
static AppMapDir AppMap_RotateLeft(AppMapDir dir);
static AppMapDir AppMap_RotateRight(AppMapDir dir);
static AppMapDir AppMap_HeadingFromTheta(float theta_rad);
static uint8_t AppMap_PoseToCell(float x_m, float y_m, int *cx, int *cy);
static void AppMap_UpdateLidarDirection(int cx,
                                        int cy,
                                        AppMapDir dir,
                                        uint8_t valid,
                                        uint16_t distance_mm);
static uint8_t AppMap_IsUniqueEdge(int cx, int cy, AppMapDir dir);
static uint16_t AppMap_WallThresholdMm(void);
static uint32_t AppMap_ElapsedMs(uint32_t now_ms, uint32_t then_ms);
static int32_t AppMap_ScaleFloatRounded(float value, float multiplier);
static const char *AppMap_FixedSign(int32_t value);
static uint32_t AppMap_FixedWhole(int32_t value, int32_t decimal_scale);
static uint32_t AppMap_FixedFraction(int32_t value, int32_t decimal_scale);

void AppMap_Init(void)
{
    AppMap_Reset();
    APP_LOG("MAP: init cells=%ux%u cell_mm=%lu wall_threshold_mm=%u",
            (unsigned int)APP_MAP_W,
            (unsigned int)APP_MAP_H,
            (unsigned long)AppMap_FixedWhole(AppMap_ScaleFloatRounded(APP_MAP_CELL_SIZE_M, 1000.0f), 1),
            (unsigned int)AppMap_WallThresholdMm());
}

void AppMap_Reset(void)
{
    (void)memset(app_map_cells, 0, sizeof(app_map_cells));
    app_map_robot_cx = 0;
    app_map_robot_cy = 0;
    app_map_heading = APP_MAP_DIR_EAST;
    app_map_last_update_ms = 0U;
    app_map_last_summary_ms = HAL_GetTick();
    app_map_last_bounds_warn_ms = 0U;
    app_map_initialized = 1U;
    AppMap_MarkVisited(0, 0);
    APP_LOG("MAP: reset cell=(0,0)");
}

void AppMap_UpdateFromPoseAndLidar(void)
{
    OdomPose_t pose;
    const AppLidarStatus *lidar;
    uint32_t now_ms;
    uint32_t lidar_age_ms;
    AMR_State_t state;
    uint8_t clamped;
    int cx;
    int cy;
    AppMapDir front_dir;
    AppMapDir left_dir;
    AppMapDir right_dir;

    if (app_map_initialized == 0U)
    {
        AppMap_Init();
    }

    now_ms = HAL_GetTick();
    if ((app_map_last_update_ms != 0U) &&
        (AppMap_ElapsedMs(now_ms, app_map_last_update_ms) < APP_MAP_UPDATE_PERIOD_MS))
    {
        return;
    }
    app_map_last_update_ms = now_ms;

    state = AMR_GetState();
    if ((AppMap_StateAllowsUpdate(state) == 0U) || AppFault_IsActive())
    {
        return;
    }

    if (Odom_GetPose(&pose) == false)
    {
        return;
    }

    clamped = AppMap_PoseToCell(pose.x_m, pose.y_m, &cx, &cy);
    app_map_robot_cx = cx;
    app_map_robot_cy = cy;
    app_map_heading = AppMap_HeadingFromTheta(pose.theta_rad);
    AppMap_MarkVisited(cx, cy);

    if ((clamped != 0U) &&
        ((app_map_last_bounds_warn_ms == 0U) ||
         (AppMap_ElapsedMs(now_ms, app_map_last_bounds_warn_ms) >= APP_MAP_BOUNDS_WARN_INTERVAL_MS)))
    {
        int32_t x_mm = AppMap_ScaleFloatRounded(pose.x_m, 1000.0f);
        int32_t y_mm = AppMap_ScaleFloatRounded(pose.y_m, 1000.0f);

        APP_LOG("MAP: warning pose outside map x=%s%lu.%03lu y=%s%lu.%03lu clamped cell=(%d,%d)",
                AppMap_FixedSign(x_mm),
                (unsigned long)AppMap_FixedWhole(x_mm, 1000),
                (unsigned long)AppMap_FixedFraction(x_mm, 1000),
                AppMap_FixedSign(y_mm),
                (unsigned long)AppMap_FixedWhole(y_mm, 1000),
                (unsigned long)AppMap_FixedFraction(y_mm, 1000),
                cx,
                cy);
        app_map_last_bounds_warn_ms = now_ms;
    }

    lidar = App_Lidar_GetStatus();
    if ((lidar == NULL) || (lidar->ready == 0U) || (lidar->last_valid_update_ms == 0U))
    {
        return;
    }

    lidar_age_ms = AppMap_ElapsedMs(now_ms, lidar->last_valid_update_ms);
    if (lidar_age_ms > APP_MAP_LIDAR_MAX_AGE_MS)
    {
        return;
    }

    front_dir = app_map_heading;
    left_dir = AppMap_RotateLeft(app_map_heading);
    right_dir = AppMap_RotateRight(app_map_heading);

    AppMap_UpdateLidarDirection(cx, cy, front_dir, lidar->front_valid, lidar->front_min_mm);
    AppMap_UpdateLidarDirection(cx, cy, left_dir, lidar->left_valid, lidar->left_min_mm);
    AppMap_UpdateLidarDirection(cx, cy, right_dir, lidar->right_valid, lidar->right_min_mm);

    if ((app_map_last_summary_ms == 0U) ||
        (AppMap_ElapsedMs(now_ms, app_map_last_summary_ms) >= APP_MAP_SUMMARY_LOG_INTERVAL_MS))
    {
        AppMap_PrintSummary();
        app_map_last_summary_ms = now_ms;
    }
}

void AppMap_GetRobotCell(int *cx, int *cy)
{
    if (app_map_initialized == 0U)
    {
        AppMap_Init();
    }

    if (cx != NULL)
    {
        *cx = app_map_robot_cx;
    }

    if (cy != NULL)
    {
        *cy = app_map_robot_cy;
    }
}

void AppMap_MarkVisited(int cx, int cy)
{
    if (AppMap_InBounds(cx, cy) == 0U)
    {
        return;
    }

    app_map_cells[cy][cx].visited = 1U;
}

bool AppMap_IsVisited(int cx, int cy)
{
    if (app_map_initialized == 0U)
    {
        AppMap_Init();
    }

    if (AppMap_InBounds(cx, cy) == 0U)
    {
        return false;
    }

    return (app_map_cells[cy][cx].visited != 0U);
}

void AppMap_SetWall(int cx, int cy, AppMapDir dir, uint8_t has_wall)
{
    static const int8_t dx[APP_MAP_DIR_COUNT] = {0, 1, 0, -1};
    static const int8_t dy[APP_MAP_DIR_COUNT] = {1, 0, -1, 0};
    int nx;
    int ny;

    if ((AppMap_InBounds(cx, cy) == 0U) || ((uint32_t)dir >= APP_MAP_DIR_COUNT))
    {
        return;
    }

    AppMap_SetWallLocal(cx, cy, dir, has_wall);

    nx = cx + dx[(uint32_t)dir];
    ny = cy + dy[(uint32_t)dir];
    if (AppMap_InBounds(nx, ny) != 0U)
    {
        AppMap_SetWallLocal(nx, ny, AppMap_OppositeDir(dir), has_wall);
    }
}

void AppMap_GetWall(int cx, int cy, AppMapDir dir, uint8_t *known, uint8_t *has_wall)
{
    AppMapCell_t *cell;
    uint8_t *known_ptr;
    uint8_t *wall_ptr;

    if ((known != NULL))
    {
        *known = 0U;
    }

    if ((has_wall != NULL))
    {
        *has_wall = 0U;
    }

    if ((AppMap_InBounds(cx, cy) == 0U) || ((uint32_t)dir >= APP_MAP_DIR_COUNT))
    {
        return;
    }

    cell = &app_map_cells[cy][cx];
    known_ptr = AppMap_KnownPtr(cell, dir);
    wall_ptr = AppMap_WallPtr(cell, dir);

    if ((known != NULL) && (known_ptr != NULL))
    {
        *known = *known_ptr;
    }

    if ((has_wall != NULL) && (wall_ptr != NULL))
    {
        *has_wall = *wall_ptr;
    }
}

void AppMap_PrintSummary(void)
{
    AppMapSummary_t summary;

    if (AppMap_GetSummary(&summary) == false)
    {
        return;
    }

    APP_LOG("MAP: cell=(%d,%d) heading=%c visited=%u known_edges=%u walls=%u",
            summary.robot_cx,
            summary.robot_cy,
            AppMap_DirChar(summary.heading),
            (unsigned int)summary.visited_count,
            (unsigned int)summary.known_edges,
            (unsigned int)summary.walls);
}

void AppMap_PrintGrid(void)
{
    int y;

    if (app_map_initialized == 0U)
    {
        AppMap_Init();
    }

    APP_LOG("MAP GRID:");
    for (y = APP_MAP_H - 1; y >= 0; y--)
    {
        char row[(APP_MAP_W * 2) + 1];
        uint32_t pos = 0U;
        int x;

        for (x = 0; x < APP_MAP_W; x++)
        {
            char marker = '?';

            if ((x == app_map_robot_cx) && (y == app_map_robot_cy))
            {
                marker = 'R';
            }
            else if (app_map_cells[y][x].visited != 0U)
            {
                marker = '.';
            }

            row[pos] = marker;
            pos++;

            if (x < (APP_MAP_W - 1))
            {
                row[pos] = ' ';
                pos++;
            }
        }

        row[pos] = '\0';
        APP_LOG("%s", row);
    }
}

bool AppMap_GetSummary(AppMapSummary_t *summary)
{
    uint16_t visited = 0U;
    uint16_t known_edges = 0U;
    uint16_t walls = 0U;
    int y;

    if (summary == NULL)
    {
        return false;
    }

    if (app_map_initialized == 0U)
    {
        AppMap_Init();
    }

    for (y = 0; y < APP_MAP_H; y++)
    {
        int x;

        for (x = 0; x < APP_MAP_W; x++)
        {
            AppMapDir dir;

            if (app_map_cells[y][x].visited != 0U)
            {
                visited++;
            }

            for (dir = APP_MAP_DIR_NORTH; dir <= APP_MAP_DIR_WEST; dir++)
            {
                uint8_t known = 0U;
                uint8_t has_wall = 0U;

                if (AppMap_IsUniqueEdge(x, y, dir) == 0U)
                {
                    continue;
                }

                AppMap_GetWall(x, y, dir, &known, &has_wall);
                if (known != 0U)
                {
                    known_edges++;
                    if (has_wall != 0U)
                    {
                        walls++;
                    }
                }
            }
        }
    }

    summary->robot_cx = app_map_robot_cx;
    summary->robot_cy = app_map_robot_cy;
    summary->heading = app_map_heading;
    summary->visited_count = visited;
    summary->known_edges = known_edges;
    summary->walls = walls;

    return true;
}

const char *AppMap_DirName(AppMapDir dir)
{
    switch (dir)
    {
        case APP_MAP_DIR_NORTH:
            return "NORTH";

        case APP_MAP_DIR_EAST:
            return "EAST";

        case APP_MAP_DIR_SOUTH:
            return "SOUTH";

        case APP_MAP_DIR_WEST:
            return "WEST";

        default:
            return "?";
    }
}

char AppMap_DirChar(AppMapDir dir)
{
    switch (dir)
    {
        case APP_MAP_DIR_NORTH:
            return 'N';

        case APP_MAP_DIR_EAST:
            return 'E';

        case APP_MAP_DIR_SOUTH:
            return 'S';

        case APP_MAP_DIR_WEST:
            return 'W';

        default:
            return '?';
    }
}

static uint8_t AppMap_InBounds(int cx, int cy)
{
    return ((cx >= 0) && (cx < APP_MAP_W) && (cy >= 0) && (cy < APP_MAP_H)) ? 1U : 0U;
}

static uint8_t AppMap_StateAllowsUpdate(AMR_State_t state)
{
    return ((state == AMR_STATE_EXPLORE) ||
            (state == AMR_STATE_AVOID) ||
            (state == AMR_STATE_RETURN)) ? 1U : 0U;
}

static void AppMap_SetWallLocal(int cx, int cy, AppMapDir dir, uint8_t has_wall)
{
    AppMapCell_t *cell;
    uint8_t *known_ptr;
    uint8_t *wall_ptr;

    if ((AppMap_InBounds(cx, cy) == 0U) || ((uint32_t)dir >= APP_MAP_DIR_COUNT))
    {
        return;
    }

    cell = &app_map_cells[cy][cx];
    known_ptr = AppMap_KnownPtr(cell, dir);
    wall_ptr = AppMap_WallPtr(cell, dir);

    if ((known_ptr == NULL) || (wall_ptr == NULL))
    {
        return;
    }

    *known_ptr = 1U;
    *wall_ptr = (has_wall != 0U) ? 1U : 0U;
}

static uint8_t *AppMap_WallPtr(AppMapCell_t *cell, AppMapDir dir)
{
    if (cell == NULL)
    {
        return NULL;
    }

    switch (dir)
    {
        case APP_MAP_DIR_NORTH:
            return &cell->wall_north;

        case APP_MAP_DIR_EAST:
            return &cell->wall_east;

        case APP_MAP_DIR_SOUTH:
            return &cell->wall_south;

        case APP_MAP_DIR_WEST:
            return &cell->wall_west;

        default:
            return NULL;
    }
}

static uint8_t *AppMap_KnownPtr(AppMapCell_t *cell, AppMapDir dir)
{
    if (cell == NULL)
    {
        return NULL;
    }

    switch (dir)
    {
        case APP_MAP_DIR_NORTH:
            return &cell->known_north;

        case APP_MAP_DIR_EAST:
            return &cell->known_east;

        case APP_MAP_DIR_SOUTH:
            return &cell->known_south;

        case APP_MAP_DIR_WEST:
            return &cell->known_west;

        default:
            return NULL;
    }
}

static AppMapDir AppMap_OppositeDir(AppMapDir dir)
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

static AppMapDir AppMap_RotateLeft(AppMapDir dir)
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

static AppMapDir AppMap_RotateRight(AppMapDir dir)
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

static AppMapDir AppMap_HeadingFromTheta(float theta_rad)
{
    while (theta_rad < 0.0f)
    {
        theta_rad += (2.0f * APP_MAP_PI);
    }

    while (theta_rad >= (2.0f * APP_MAP_PI))
    {
        theta_rad -= (2.0f * APP_MAP_PI);
    }

    if ((theta_rad < (APP_MAP_PI * 0.25f)) || (theta_rad >= (APP_MAP_PI * 1.75f)))
    {
        return APP_MAP_DIR_EAST;
    }

    if (theta_rad < (APP_MAP_PI * 0.75f))
    {
        return APP_MAP_DIR_NORTH;
    }

    if (theta_rad < (APP_MAP_PI * 1.25f))
    {
        return APP_MAP_DIR_WEST;
    }

    return APP_MAP_DIR_SOUTH;
}

static uint8_t AppMap_PoseToCell(float x_m, float y_m, int *cx, int *cy)
{
    int local_cx;
    int local_cy;
    uint8_t clamped = 0U;

    if (x_m < 0.0f)
    {
        local_cx = 0;
        clamped = 1U;
    }
    else
    {
        local_cx = (int)(x_m / APP_MAP_CELL_SIZE_M);
        if (local_cx >= APP_MAP_W)
        {
            local_cx = APP_MAP_W - 1;
            clamped = 1U;
        }
    }

    if (y_m < 0.0f)
    {
        local_cy = 0;
        clamped = 1U;
    }
    else
    {
        local_cy = (int)(y_m / APP_MAP_CELL_SIZE_M);
        if (local_cy >= APP_MAP_H)
        {
            local_cy = APP_MAP_H - 1;
            clamped = 1U;
        }
    }

    if (cx != NULL)
    {
        *cx = local_cx;
    }

    if (cy != NULL)
    {
        *cy = local_cy;
    }

    return clamped;
}

static void AppMap_UpdateLidarDirection(int cx,
                                        int cy,
                                        AppMapDir dir,
                                        uint8_t valid,
                                        uint16_t distance_mm)
{
    if ((valid == 0U) || (distance_mm == 0U) || (distance_mm == 0xFFFFU))
    {
        return;
    }

    AppMap_SetWall(cx, cy, dir, (distance_mm <= AppMap_WallThresholdMm()) ? 1U : 0U);
}

static uint8_t AppMap_IsUniqueEdge(int cx, int cy, AppMapDir dir)
{
    static const int8_t dx[APP_MAP_DIR_COUNT] = {0, 1, 0, -1};
    static const int8_t dy[APP_MAP_DIR_COUNT] = {1, 0, -1, 0};
    int nx;
    int ny;

    if ((uint32_t)dir >= APP_MAP_DIR_COUNT)
    {
        return 0U;
    }

    nx = cx + dx[(uint32_t)dir];
    ny = cy + dy[(uint32_t)dir];
    if (AppMap_InBounds(nx, ny) == 0U)
    {
        return 1U;
    }

    return ((dir == APP_MAP_DIR_NORTH) || (dir == APP_MAP_DIR_EAST)) ? 1U : 0U;
}

static uint16_t AppMap_WallThresholdMm(void)
{
    int32_t threshold_mm = AppMap_ScaleFloatRounded(APP_MAP_WALL_THRESHOLD_M, 1000.0f);

    if (threshold_mm < 0)
    {
        return 0U;
    }

    if (threshold_mm > 0xFFFF)
    {
        return 0xFFFFU;
    }

    return (uint16_t)threshold_mm;
}

static uint32_t AppMap_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
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

static int32_t AppMap_ScaleFloatRounded(float value, float multiplier)
{
    float scaled = value * multiplier;

    if (scaled < 0.0f)
    {
        return (int32_t)(scaled - 0.5f);
    }

    return (int32_t)(scaled + 0.5f);
}

static const char *AppMap_FixedSign(int32_t value)
{
    return (value < 0) ? "-" : "";
}

static uint32_t AppMap_FixedWhole(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;

    return (uint32_t)(abs_value / decimal_scale);
}

static uint32_t AppMap_FixedFraction(int32_t value, int32_t decimal_scale)
{
    int32_t abs_value = (value < 0) ? -value : value;

    return (uint32_t)(abs_value % decimal_scale);
}
