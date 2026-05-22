#ifndef NAVIGATION_TASK_H
#define NAVIGATION_TASK_H

#include <stdint.h>

#include "slam_types.h"

/* ── 遥测路径最大点数 ── */
#define NAVIGATION_PATH_TELEMETRY_MAX_POINTS 64U

/* ── 迷宫图参数 ── */
#define MAZE_COLS                5U
#define MAZE_ROWS                5U
#define MAZE_NODE_COUNT          (MAZE_COLS * MAZE_ROWS)
#define MAZE_DIR_COUNT           4U
#define CELL_SIZE_M              0.70f

/* 墙壁检测阈值：小于此距离判定为墙 */
#define WALL_DETECT_THRESHOLD_M  0.45f
/* LiDAR 扇区半宽（度） */
#define WALL_SECTOR_HALF_DEG     15.0f

/* ── 方向枚举 ── */
typedef enum {
    MAZE_DIR_EAST  = 0,   /* +X */
    MAZE_DIR_NORTH = 1,   /* +Y */
    MAZE_DIR_WEST  = 2,   /* -X */
    MAZE_DIR_SOUTH = 3    /* -Y */
} MazeEdgeDir_t;

/* ── 边状态 ── */
typedef enum {
    EDGE_UNKNOWN = 0,
    EDGE_OPEN    = 1,
    EDGE_WALL    = 2
} MazeEdgeState_t;

/* ── 探索状态机 ── */
typedef enum {
    MAZE_EXPLORE_IDLE      = 0,
    MAZE_EXPLORE_SCAN      = 1,   /* 扫描当前格四向墙壁 */
    MAZE_EXPLORE_CHOOSE    = 2,   /* 选择下一步方向 */
    MAZE_EXPLORE_MOVING    = 3,   /* 正在移动到邻格 */
    MAZE_EXPLORE_BACKTRACK = 4,   /* BFS 回溯中 */
    MAZE_EXPLORE_RETURN    = 5,   /* 探索完成，返回起点 */
    MAZE_EXPLORE_DONE      = 6    /* 全部完成 */
} MazeExploreState_t;

/* ── 迷宫图结构体 ── */
typedef struct {
    MazeEdgeState_t edges[MAZE_NODE_COUNT][MAZE_DIR_COUNT];
    uint8_t         visited[MAZE_NODE_COUNT];
} MazeGraph_t;

/* ── 导航状态 ── */
typedef enum {
    NAVIGATION_STATUS_IDLE = 0,
    NAVIGATION_STATUS_OK = 1,
    NAVIGATION_STATUS_REACHED = 2,
    NAVIGATION_STATUS_FAILED = 3,
    NAVIGATION_STATUS_BUSY = 4
} NavigationStatus_t;

typedef struct {
    float x_m;
    float y_m;
} NavigationPathPoint_t;

typedef struct {
    uint8_t goal_valid;
    uint8_t target_valid;
    NavigationStatus_t last_status;
    uint32_t update_count;
    uint32_t fail_count;
    uint16_t raw_path_len;
    uint16_t smooth_path_len;
    float distance_to_goal_m;
    SlamPose2D_t current_pose;
    SlamPose2D_t goal_pose;
    SlamPose2D_t target_pose;
} NavigationTaskStats_t;

/* ── FreeRTOS 线程入口 ── */
void StartNavigationTask(void *argument);

/* ── 迷宫探索 API ── */
void NavigationTask_SetStartCell(uint8_t col, uint8_t row);
void NavigationTask_GetMazeGraph(MazeGraph_t *graph_out);   /* 遥测用 */
MazeExploreState_t NavigationTask_GetExploreState(void);
int8_t NavigationTask_GetMazeCurrentNode(void);
int8_t NavigationTask_GetMazeStartNode(void);

/* ── 原有 API（保留兼容） ── */
void NavigationTask_SetGoal(float goal_x_m, float goal_y_m);
void NavigationTask_ClearGoal(void);
void NavigationTask_StartCommandRx(void);
void NavigationTask_HandleCommandRxCompleteFromIsr(void);
NavigationStatus_t NavigationTask_Update(void);
void NavigationTask_GetStatsSnapshot(NavigationTaskStats_t *stats);
uint16_t NavigationTask_CopySmoothPathPoints(NavigationPathPoint_t *points, uint16_t max_points);

#endif /* NAVIGATION_TASK_H */
