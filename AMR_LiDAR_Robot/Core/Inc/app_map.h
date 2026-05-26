#ifndef APP_MAP_MODULE_H
#define APP_MAP_MODULE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Lightweight cell-level occupancy map for the 5x5 maze benchmark.
 * The map is intentionally fixed-size and allocation-free for embedded use.
 */
#ifndef APP_MAP_W
#define APP_MAP_W 5
#endif

#ifndef APP_MAP_H
#define APP_MAP_H 5
#endif

#ifndef APP_MAP_CELL_SIZE_M
#define APP_MAP_CELL_SIZE_M 0.70f
#endif

#ifndef APP_MAP_WALL_THRESHOLD_M
#define APP_MAP_WALL_THRESHOLD_M 0.45f
#endif

typedef enum
{
    APP_MAP_DIR_NORTH = 0,
    APP_MAP_DIR_EAST,
    APP_MAP_DIR_SOUTH,
    APP_MAP_DIR_WEST
} AppMapDir;

typedef struct
{
    uint8_t visited;
    uint8_t wall_north;
    uint8_t wall_east;
    uint8_t wall_south;
    uint8_t wall_west;
    uint8_t known_north;
    uint8_t known_east;
    uint8_t known_south;
    uint8_t known_west;
} AppMapCell_t;

typedef struct
{
    int robot_cx;
    int robot_cy;
    AppMapDir heading;
    uint16_t visited_count;
    uint16_t known_edges;
    uint16_t walls;
} AppMapSummary_t;

void AppMap_Init(void);
void AppMap_Reset(void);
void AppMap_UpdateFromPoseAndLidar(void);
void AppMap_GetRobotCell(int *cx, int *cy);
void AppMap_MarkVisited(int cx, int cy);
bool AppMap_IsVisited(int cx, int cy);
void AppMap_SetWall(int cx, int cy, AppMapDir dir, uint8_t has_wall);
void AppMap_GetWall(int cx, int cy, AppMapDir dir, uint8_t *known, uint8_t *has_wall);
void AppMap_PrintSummary(void);
void AppMap_PrintGrid(void);
bool AppMap_GetSummary(AppMapSummary_t *summary);
const char *AppMap_DirName(AppMapDir dir);
char AppMap_DirChar(AppMapDir dir);

#endif /* APP_MAP_MODULE_H */
