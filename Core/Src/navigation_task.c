/*
 * navigation_task.c — 5×5 迷宫图探索实现
 *
 * 将迷宫建模为 25 节点无向图，使用 DFS 遍历 + BFS 最短路回溯。
 * 墙壁检测利用 360° LiDAR 同时读取四个方向扇区的最小距离。
 * 移动控制调用 pid.c 已有的 Start_Relative_Move() 接口。
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"

#include "../Inc/freertos_app.h"
#include "../Inc/localization_task.h"
#include "../Inc/mapping_task.h"
#include "../Inc/navigation_task.h"
#include "../Inc/scan_preprocess.h"
#include "../Inc/usart.h"
#include "pid.h"
#include "system.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  常量
 * ═══════════════════════════════════════════════════════════════════════════ */

#define NAVIGATION_TASK_PERIOD_MS          100U
#define NAVIGATION_COMMAND_MAX_LEN         64U
#define NAVIGATION_MAX_PATH_POINTS         128U
#define NAVIGATION_REACH_DISTANCE_M        0.1f
#define NAVIGATION_MIN_SEGMENT_M           0.1f

/* 扫墙用的 LiDAR 相关参数 */
#define WALL_LIDAR_MIN_QUALITY             8U
#define WALL_LIDAR_MIN_DISTANCE_MM         8.0f
#define WALL_LIDAR_MAX_DISTANCE_MM         4500.0f

/* DFS 栈最大深度 */
#define DFS_STACK_MAX                      MAZE_NODE_COUNT

/* BFS 路径最大步数 */
#define BFS_PATH_MAX                       MAZE_NODE_COUNT

/* 运动完成等待超时（毫秒） */
#define MOVE_TIMEOUT_MS                    15000U

/* 扫墙前的稳定等待（毫秒），让 LiDAR 完成至少一帧完整扫描 */
#define SCAN_SETTLE_MS                     400U

/* ═══════════════════════════════════════════════════════════════════════════
 *  方向 ↔ 位移/角度 映射表
 * ═══════════════════════════════════════════════════════════════════════════ */

/* 方向到位移增量 (col_delta, row_delta) */
static const int8_t s_dir_dcol[4] = { +1,  0, -1,  0 };  /* E, N, W, S */
static const int8_t s_dir_drow[4] = {  0, +1,  0, -1 };

/*
 * 方向到世界角度偏移（相对于机器人朝向 0° = +X = EAST）
 * EAST=0°, NORTH=90°, WEST=180°, SOUTH=270°
 */
static const float s_dir_world_angle_offset[4] = { 0.0f, 90.0f, 180.0f, 270.0f };

/* ═══════════════════════════════════════════════════════════════════════════
 *  模块静态变量
 * ═══════════════════════════════════════════════════════════════════════════ */

static NavigationTaskStats_t g_navigationStats;
static SlamPose2D_t g_navigationGoal;
static uint8_t g_navigationGoalValid = 0U;
static uint8_t g_navigationCommandRxByte = 0U;
static char g_navigationCommandLine[NAVIGATION_COMMAND_MAX_LEN];
static char g_navigationPendingCommand[NAVIGATION_COMMAND_MAX_LEN];
static volatile uint8_t g_navigationCommandIndex = 0U;
static volatile uint8_t g_navigationCommandPending = 0U;

static NavigationPathPoint_t g_navigationSmoothPath[NAVIGATION_MAX_PATH_POINTS];
static uint16_t g_navigationSmoothPathLen = 0U;

/* 迷宫图 */
static MazeGraph_t s_maze_graph;
static MazeExploreState_t s_explore_state = MAZE_EXPLORE_IDLE;
static int8_t s_current_node = -1;
static int8_t s_start_node = -1;
static uint8_t s_start_col = 0U;
static uint8_t s_start_row = 0U;
static uint8_t s_start_cell_set = 0U;

/* DFS 栈 */
static int8_t s_dfs_stack[DFS_STACK_MAX];
static uint8_t s_dfs_stack_top = 0U;

/* BFS 回溯路径 */
static MazeEdgeDir_t s_backtrack_path[BFS_PATH_MAX];
static uint8_t s_backtrack_len = 0U;
static uint8_t s_backtrack_idx = 0U;

/* 扫描稳定计时 */
static uint32_t s_scan_settle_start_ms = 0U;

/* ═══════════════════════════════════════════════════════════════════════════
 *  图操作工具函数
 * ═══════════════════════════════════════════════════════════════════════════ */

static int8_t nav_node_id(uint8_t col, uint8_t row)
{
    if ((col >= MAZE_COLS) || (row >= MAZE_ROWS)) {
        return -1;
    }

    return (int8_t)(row * MAZE_COLS + col);
}

static void nav_node_to_colrow(int8_t node_id, uint8_t *col, uint8_t *row)
{
    *col = (uint8_t)(node_id % (int8_t)MAZE_COLS);
    *row = (uint8_t)(node_id / (int8_t)MAZE_COLS);
}

static int8_t nav_neighbor_id(int8_t node_id, MazeEdgeDir_t dir)
{
    uint8_t col, row;
    int8_t nc, nr;

    if ((node_id < 0) || (node_id >= (int8_t)MAZE_NODE_COUNT) || (dir >= MAZE_DIR_COUNT)) {
        return -1;
    }

    nav_node_to_colrow(node_id, &col, &row);
    nc = (int8_t)col + s_dir_dcol[dir];
    nr = (int8_t)row + s_dir_drow[dir];

    if ((nc < 0) || (nc >= (int8_t)MAZE_COLS) || (nr < 0) || (nr >= (int8_t)MAZE_ROWS)) {
        return -1;
    }

    return nav_node_id((uint8_t)nc, (uint8_t)nr);
}

static MazeEdgeDir_t nav_opposite_dir(MazeEdgeDir_t dir)
{
    return (MazeEdgeDir_t)((dir + 2U) % MAZE_DIR_COUNT);
}

/*
 * 给定节点 ID，返回其格子中心的世界坐标（以起点为原点）。
 */
static void nav_cell_center(int8_t node_id, float *x_m, float *y_m)
{
    uint8_t col, row;

    nav_node_to_colrow(node_id, &col, &row);
    *x_m = ((float)col - (float)s_start_col) * CELL_SIZE_M;
    *y_m = ((float)row - (float)s_start_row) * CELL_SIZE_M;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  LiDAR 扇区距离读取（360° 同时扫四向）
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * 计算两个角度之间的最短差值（度），结果在 [-180, 180]。
 */
static float nav_angle_diff_deg(float a, float b)
{
    float d = a - b;

    while (d > 180.0f) {
        d -= 360.0f;
    }

    while (d < -180.0f) {
        d += 360.0f;
    }

    return d;
}

/*
 * 从最新一帧 LiDAR 扫描中，读取世界坐标系下指定方向扇区的最小距离。
 *
 * target_world_angle_deg: 世界坐标系下的目标方向角度（度）。
 *
 * 遍历 g_lidarScanBuf 中最近的一个已完成帧，筛选出世界角度落在
 * [target - WALL_SECTOR_HALF_DEG, target + WALL_SECTOR_HALF_DEG] 范围内的
 * 可用点，返回其中最小的距离（米）。
 *
 * 如果扇区内没有可用点，返回一个大值（99.0f）表示无障碍物。
 */
static float nav_lidar_sector_min_dist_m(float target_world_angle_deg)
{
    float min_dist_m = 99.0f;
    uint8_t buf_idx;
    SlamPose2D_t pose;
    float heading_deg;

    /* 获取当前里程计朝向 */
    LocalizationTask_GetPoseSnapshot(&pose);
    heading_deg = pose.theta_deg;

    /*
     * 遍历所有 LiDAR 扫描缓冲区，找到有数据的那些帧。
     * 因为导航线程不在消息队列上，所以直接读全局缓冲区。
     * g_lidarScanBuf 是 4 个缓冲区的数组，由 LiDAR 解析线程写入。
     */
    for (buf_idx = 0U; buf_idx < LIDAR_SCAN_BUFFER_COUNT; buf_idx++) {
        const LidarScanBuffer_t *scan = &g_lidarScanBuf[buf_idx];
        uint16_t point_idx;

        if (scan->point_count == 0U) {
            continue;
        }

        for (point_idx = 0U; point_idx < scan->point_count; point_idx++) {
            const LidarPoint_t *pt = &scan->points[point_idx];
            float world_angle_deg;
            float angle_diff;
            float dist_m;

            /* 跳过低质量或无效距离的点 */
            if (pt->quality < WALL_LIDAR_MIN_QUALITY) {
                continue;
            }

            if ((pt->distance_mm < WALL_LIDAR_MIN_DISTANCE_MM) ||
                (pt->distance_mm > WALL_LIDAR_MAX_DISTANCE_MM)) {
                continue;
            }

            /* LiDAR 本体角 → 世界角度 */
            world_angle_deg = ScanPreprocess_BeamWorldAngleDeg(heading_deg, pt->angle_deg);

            /* 判断是否落在目标扇区内 */
            angle_diff = nav_angle_diff_deg(world_angle_deg, target_world_angle_deg);
            if (fabsf(angle_diff) <= WALL_SECTOR_HALF_DEG) {
                dist_m = pt->distance_mm * 0.001f;
                if (dist_m < min_dist_m) {
                    min_dist_m = dist_m;
                }
            }
        }
    }

    return min_dist_m;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  墙壁扫描
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * 在当前节点中心，同时读取四个方向的墙壁状况。
 * 利用 360° LiDAR 无需旋转，直接按世界角度分四个扇区。
 */
static void nav_scan_walls_at_current_node(void)
{
    uint8_t dir;
    SlamPose2D_t pose;
    float heading_deg;
    int8_t nb;

    if (s_current_node < 0) {
        return;
    }

    LocalizationTask_GetPoseSnapshot(&pose);
    heading_deg = pose.theta_deg;
    (void)heading_deg;  /* heading 已被 nav_lidar_sector_min_dist_m 内部使用 */

    for (dir = 0U; dir < MAZE_DIR_COUNT; dir++) {
        float target_angle_deg;
        float dist_m;

        /* 迷宫边界：如果该方向没有邻格，直接标记为墙 */
        nb = nav_neighbor_id(s_current_node, (MazeEdgeDir_t)dir);
        if (nb < 0) {
            s_maze_graph.edges[s_current_node][dir] = EDGE_WALL;
            continue;
        }

        /* 已知边状态则跳过 */
        if (s_maze_graph.edges[s_current_node][dir] != EDGE_UNKNOWN) {
            continue;
        }

        /* 世界坐标系下该方向的角度 */
        target_angle_deg = s_dir_world_angle_offset[dir];

        dist_m = nav_lidar_sector_min_dist_m(target_angle_deg);

        if (dist_m < WALL_DETECT_THRESHOLD_M) {
            s_maze_graph.edges[s_current_node][dir] = EDGE_WALL;
        } else {
            s_maze_graph.edges[s_current_node][dir] = EDGE_OPEN;
        }

        /* 对称更新邻格的对面边 */
        s_maze_graph.edges[nb][nav_opposite_dir((MazeEdgeDir_t)dir)] =
            s_maze_graph.edges[s_current_node][dir];
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  BFS 最短路
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * 在已知 OPEN 边的图上做 BFS，输出从 src 到 dst 的方向序列。
 * 返回 1 表示找到路径，0 表示不可达。
 */
static uint8_t nav_bfs_path(int8_t src, int8_t dst,
                             MazeEdgeDir_t *path_out, uint8_t *path_len)
{
    int8_t bfs_queue[MAZE_NODE_COUNT];
    int8_t bfs_parent[MAZE_NODE_COUNT];
    MazeEdgeDir_t bfs_parent_dir[MAZE_NODE_COUNT];
    uint8_t bfs_visited[MAZE_NODE_COUNT];
    uint8_t head = 0U;
    uint8_t tail = 0U;
    int8_t current;
    uint8_t dir;

    *path_len = 0U;

    if (src == dst) {
        return 1U;
    }

    (void)memset(bfs_visited, 0, sizeof(bfs_visited));
    (void)memset(bfs_parent, -1, sizeof(bfs_parent));

    bfs_queue[tail++] = src;
    bfs_visited[src] = 1U;

    while (head < tail) {
        current = bfs_queue[head++];

        for (dir = 0U; dir < MAZE_DIR_COUNT; dir++) {
            int8_t nb = nav_neighbor_id(current, (MazeEdgeDir_t)dir);

            if (nb < 0) {
                continue;
            }

            if (bfs_visited[nb] != 0U) {
                continue;
            }

            /* 只走已知为 OPEN 的边 */
            if (s_maze_graph.edges[current][dir] != EDGE_OPEN) {
                continue;
            }

            bfs_parent[nb] = current;
            bfs_parent_dir[nb] = (MazeEdgeDir_t)dir;
            bfs_visited[nb] = 1U;

            if (nb == dst) {
                /* 回溯路径 */
                int8_t trace = dst;
                uint8_t len = 0U;
                MazeEdgeDir_t reverse_path[BFS_PATH_MAX];

                while (trace != src) {
                    reverse_path[len++] = bfs_parent_dir[trace];
                    trace = bfs_parent[trace];
                }

                /* 反转得到正序路径 */
                for (uint8_t i = 0U; i < len; i++) {
                    path_out[i] = reverse_path[len - 1U - i];
                }

                *path_len = len;
                return 1U;
            }

            bfs_queue[tail++] = nb;
        }
    }

    return 0U; /* 不可达 */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  DFS 探索 —— 选择下一个方向
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * 从当前节点的四个方向中，选择第一个 OPEN 且未访问的邻格。
 * 优先级：EAST → NORTH → WEST → SOUTH。
 * 返回方向索引，如果没有可走的方向则返回 -1。
 */
static int8_t nav_choose_unvisited_neighbor(void)
{
    uint8_t dir;

    for (dir = 0U; dir < MAZE_DIR_COUNT; dir++) {
        int8_t nb;

        if (s_maze_graph.edges[s_current_node][dir] != EDGE_OPEN) {
            continue;
        }

        nb = nav_neighbor_id(s_current_node, (MazeEdgeDir_t)dir);
        if ((nb >= 0) && (s_maze_graph.visited[nb] == 0U)) {
            return (int8_t)dir;
        }
    }

    return -1;
}

/*
 * 发起一次从当前格到指定方向邻格的移动。
 */
static void nav_move_one_cell(MazeEdgeDir_t dir)
{
    float dx = (float)s_dir_dcol[dir] * CELL_SIZE_M;
    float dy = (float)s_dir_drow[dir] * CELL_SIZE_M;

    Start_Relative_Move(dx, dy);
}

/*
 * 检查当前移动是否完成（或超时）。
 */
static uint8_t nav_is_move_complete(void)
{
    return (g_relative_move_state == RELATIVE_MOVE_IDLE) ? 1U : 0U;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  探索状态机主逻辑
 * ═══════════════════════════════════════════════════════════════════════════ */

static void nav_explore_tick(void)
{
    switch (s_explore_state) {
        case MAZE_EXPLORE_IDLE:
            /* 等待用户通过 UART 设置起点格子 */
            break;

        case MAZE_EXPLORE_SCAN:
        {
            uint32_t now_ms = HAL_GetTick();

            /*
             * 到达新格子后等待一小段时间，确保 LiDAR 完成至少一帧完整 360° 扫描，
             * 避免用移动中不完整的帧做墙壁检测。
             */
            if ((now_ms - s_scan_settle_start_ms) < SCAN_SETTLE_MS) {
                break;
            }

            /* 扫描当前格四向墙壁 */
            nav_scan_walls_at_current_node();

            /* 标记当前格已访问 */
            s_maze_graph.visited[s_current_node] = 1U;

            s_explore_state = MAZE_EXPLORE_CHOOSE;
            break;
        }

        case MAZE_EXPLORE_CHOOSE:
        {
            int8_t chosen_dir = nav_choose_unvisited_neighbor();

            if (chosen_dir >= 0) {
                int8_t target_node = nav_neighbor_id(s_current_node, (MazeEdgeDir_t)chosen_dir);

                /* 压入 DFS 栈 */
                if (s_dfs_stack_top < DFS_STACK_MAX) {
                    s_dfs_stack[s_dfs_stack_top++] = s_current_node;
                }

                /* 发起移动 */
                nav_move_one_cell((MazeEdgeDir_t)chosen_dir);
                s_current_node = target_node;
                s_explore_state = MAZE_EXPLORE_MOVING;
            } else {
                /* 没有未访问邻格，需要回溯 */
                if (s_dfs_stack_top > 0U) {
                    /* BFS 找从 current 到栈顶（上一个分叉点）的最短路 */
                    int8_t backtrack_target = s_dfs_stack[s_dfs_stack_top - 1U];

                    /* 向上弹栈直到找到一个有未访问邻居的祖先 */
                    while (s_dfs_stack_top > 0U) {
                        int8_t ancestor = s_dfs_stack[s_dfs_stack_top - 1U];
                        uint8_t has_unvisited = 0U;
                        uint8_t d;

                        for (d = 0U; d < MAZE_DIR_COUNT; d++) {
                            if (s_maze_graph.edges[ancestor][d] == EDGE_OPEN) {
                                int8_t nb = nav_neighbor_id(ancestor, (MazeEdgeDir_t)d);
                                if ((nb >= 0) && (s_maze_graph.visited[nb] == 0U)) {
                                    has_unvisited = 1U;
                                    break;
                                }
                            }
                        }

                        if (has_unvisited != 0U) {
                            backtrack_target = ancestor;
                            break;
                        }

                        s_dfs_stack_top--;
                    }

                    if (s_dfs_stack_top == 0U) {
                        /* 栈清空了，所有节点都已访问，开始返回起点 */
                        s_explore_state = MAZE_EXPLORE_RETURN;
                        if (nav_bfs_path(s_current_node, s_start_node,
                                          s_backtrack_path, &s_backtrack_len) != 0U) {
                            s_backtrack_idx = 0U;
                            if (s_backtrack_len > 0U) {
                                nav_move_one_cell(s_backtrack_path[0U]);
                                s_current_node = nav_neighbor_id(s_current_node,
                                                                  s_backtrack_path[0U]);
                                s_backtrack_idx = 1U;
                            } else {
                                /* 已经在起点 */
                                s_explore_state = MAZE_EXPLORE_DONE;
                            }
                        } else {
                            /* 不可达？异常，强制完成 */
                            s_explore_state = MAZE_EXPLORE_DONE;
                        }
                        break;
                    }

                    /* BFS 找回溯路径 */
                    if (nav_bfs_path(s_current_node, backtrack_target,
                                      s_backtrack_path, &s_backtrack_len) != 0U) {
                        s_backtrack_idx = 0U;
                        if (s_backtrack_len > 0U) {
                            nav_move_one_cell(s_backtrack_path[0U]);
                            s_current_node = nav_neighbor_id(s_current_node,
                                                              s_backtrack_path[0U]);
                            s_backtrack_idx = 1U;
                        }
                        s_explore_state = MAZE_EXPLORE_BACKTRACK;
                    } else {
                        /* 不可达，尝试返回起点 */
                        s_explore_state = MAZE_EXPLORE_RETURN;
                    }
                } else {
                    /* 栈为空，探索完成 */
                    s_explore_state = MAZE_EXPLORE_RETURN;

                    if (s_current_node == s_start_node) {
                        s_explore_state = MAZE_EXPLORE_DONE;
                    } else if (nav_bfs_path(s_current_node, s_start_node,
                                             s_backtrack_path, &s_backtrack_len) != 0U) {
                        s_backtrack_idx = 0U;
                        if (s_backtrack_len > 0U) {
                            nav_move_one_cell(s_backtrack_path[0U]);
                            s_current_node = nav_neighbor_id(s_current_node,
                                                              s_backtrack_path[0U]);
                            s_backtrack_idx = 1U;
                        } else {
                            s_explore_state = MAZE_EXPLORE_DONE;
                        }
                    } else {
                        s_explore_state = MAZE_EXPLORE_DONE;
                    }
                }
            }
            break;
        }

        case MAZE_EXPLORE_MOVING:
            /* 等待 relative move 完成 */
            if (nav_is_move_complete() != 0U) {
                /* 到达新格子，开始扫墙 */
                s_scan_settle_start_ms = HAL_GetTick();
                s_explore_state = MAZE_EXPLORE_SCAN;
            }
            break;

        case MAZE_EXPLORE_BACKTRACK:
            /* 等待当前段移动完成 */
            if (nav_is_move_complete() != 0U) {
                if (s_backtrack_idx < s_backtrack_len) {
                    /* 继续走下一段 */
                    nav_move_one_cell(s_backtrack_path[s_backtrack_idx]);
                    s_current_node = nav_neighbor_id(s_current_node,
                                                      s_backtrack_path[s_backtrack_idx]);
                    s_backtrack_idx++;
                } else {
                    /* 回溯完成，到达有未访问邻居的分叉点，重新扫墙和选择 */
                    s_scan_settle_start_ms = HAL_GetTick();
                    s_explore_state = MAZE_EXPLORE_SCAN;
                }
            }
            break;

        case MAZE_EXPLORE_RETURN:
            /* 沿 BFS 路径返回起点 */
            if (nav_is_move_complete() != 0U) {
                if (s_backtrack_idx < s_backtrack_len) {
                    nav_move_one_cell(s_backtrack_path[s_backtrack_idx]);
                    s_current_node = nav_neighbor_id(s_current_node,
                                                      s_backtrack_path[s_backtrack_idx]);
                    s_backtrack_idx++;
                } else {
                    /* 已返回起点 */
                    s_explore_state = MAZE_EXPLORE_DONE;
                }
            }
            break;

        case MAZE_EXPLORE_DONE:
            /* 保持静止 */
            Control_SetManualCommand(0.0f, 0.0f);
            break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  UART 命令解析
 * ═══════════════════════════════════════════════════════════════════════════ */

static void nav_parse_command(const char *cmd)
{
    int col_val, row_val;

    if (cmd == NULL) {
        return;
    }

    /* "CELL col row" — 设置起点并开始探索 */
    if (sscanf(cmd, "CELL %d %d", &col_val, &row_val) == 2) {
        NavigationTask_SetStartCell((uint8_t)col_val, (uint8_t)row_val);
        return;
    }

    /* "NAVC" — 停止探索 */
    if (strncmp(cmd, "NAVC", 4U) == 0) {
        NavigationTask_ClearGoal();
        return;
    }

    /* "NAV x y" — 设置导航目标（保留兼容） */
    {
        float gx, gy;
        if (sscanf(cmd, "NAV %f %f", &gx, &gy) == 2) {
            NavigationTask_SetGoal(gx, gy);
            return;
        }
    }

    /* "Pdx,dy" — 相对位移推送 */
    {
        float pdx, pdy;
        if (sscanf(cmd, "P%f,%f", &pdx, &pdy) == 2) {
            Start_Relative_Move(pdx, pdy);
            return;
        }
    }
}

static void nav_process_pending_command(void)
{
    if (g_navigationCommandPending != 0U) {
        g_navigationCommandPending = 0U;
        nav_parse_command(g_navigationPendingCommand);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  公开 API
 * ═══════════════════════════════════════════════════════════════════════════ */

void NavigationTask_SetStartCell(uint8_t col, uint8_t row)
{
    int8_t node;

    if ((col >= MAZE_COLS) || (row >= MAZE_ROWS)) {
        return;
    }

    node = nav_node_id(col, row);

    if (g_navigationMutex != NULL) {
        (void)osMutexAcquire(g_navigationMutex, osWaitForever);
    }

    /* 初始化迷宫图 */
    (void)memset(&s_maze_graph, 0, sizeof(s_maze_graph));
    s_dfs_stack_top = 0U;
    s_backtrack_len = 0U;
    s_backtrack_idx = 0U;
    s_start_col = col;
    s_start_row = row;
    s_start_node = node;
    s_current_node = node;
    s_start_cell_set = 1U;

    /* 迷宫外边界预初始化：边缘格子向外的方向标记为 WALL */
    for (uint8_t r = 0U; r < MAZE_ROWS; r++) {
        for (uint8_t c = 0U; c < MAZE_COLS; c++) {
            int8_t nid = nav_node_id(c, r);
            if (nid < 0) continue;

            if (c == 0U)                  s_maze_graph.edges[nid][MAZE_DIR_WEST]  = EDGE_WALL;
            if (c == (MAZE_COLS - 1U))    s_maze_graph.edges[nid][MAZE_DIR_EAST]  = EDGE_WALL;
            if (r == 0U)                  s_maze_graph.edges[nid][MAZE_DIR_SOUTH] = EDGE_WALL;
            if (r == (MAZE_ROWS - 1U))    s_maze_graph.edges[nid][MAZE_DIR_NORTH] = EDGE_WALL;
        }
    }

    /* 让机器人先停止，然后开始扫描当前格 */
    Control_SetManualCommand(0.0f, 0.0f);
    s_scan_settle_start_ms = HAL_GetTick();
    s_explore_state = MAZE_EXPLORE_SCAN;

    if (g_navigationMutex != NULL) {
        (void)osMutexRelease(g_navigationMutex);
    }
}

void NavigationTask_GetMazeGraph(MazeGraph_t *graph_out)
{
    if (graph_out == NULL) {
        return;
    }

    if (g_navigationMutex != NULL) {
        (void)osMutexAcquire(g_navigationMutex, osWaitForever);
    }

    *graph_out = s_maze_graph;

    if (g_navigationMutex != NULL) {
        (void)osMutexRelease(g_navigationMutex);
    }
}

MazeExploreState_t NavigationTask_GetExploreState(void)
{
    return s_explore_state;
}

void NavigationTask_SetGoal(float goal_x_m, float goal_y_m)
{
    if (g_navigationMutex != NULL) {
        (void)osMutexAcquire(g_navigationMutex, osWaitForever);
    }

    g_navigationGoal.x_m = goal_x_m;
    g_navigationGoal.y_m = goal_y_m;
    g_navigationGoalValid = 1U;

    if (g_navigationMutex != NULL) {
        (void)osMutexRelease(g_navigationMutex);
    }
}

void NavigationTask_ClearGoal(void)
{
    if (g_navigationMutex != NULL) {
        (void)osMutexAcquire(g_navigationMutex, osWaitForever);
    }

    g_navigationGoalValid = 0U;
    s_explore_state = MAZE_EXPLORE_IDLE;
    s_start_cell_set = 0U;
    Control_SetManualCommand(0.0f, 0.0f);

    if (g_navigationMutex != NULL) {
        (void)osMutexRelease(g_navigationMutex);
    }
}

void NavigationTask_StartCommandRx(void)
{
    g_navigationCommandIndex = 0U;
    g_navigationCommandPending = 0U;
    (void)memset(g_navigationCommandLine, 0, sizeof(g_navigationCommandLine));
    (void)memset(g_navigationPendingCommand, 0, sizeof(g_navigationPendingCommand));
    (void)HAL_UART_Receive_IT(&huart5, &g_navigationCommandRxByte, 1U);
}

void NavigationTask_HandleCommandRxCompleteFromIsr(void)
{
    char c = (char)g_navigationCommandRxByte;

    if ((c == '\n') || (c == '\r')) {
        if (g_navigationCommandIndex > 0U) {
            g_navigationCommandLine[g_navigationCommandIndex] = '\0';
            (void)memcpy(g_navigationPendingCommand,
                         g_navigationCommandLine,
                         g_navigationCommandIndex + 1U);
            g_navigationCommandPending = 1U;
            g_navigationCommandIndex = 0U;
        }
    } else if (g_navigationCommandIndex < (NAVIGATION_COMMAND_MAX_LEN - 1U)) {
        g_navigationCommandLine[g_navigationCommandIndex++] = c;
    }

    (void)HAL_UART_Receive_IT(&huart5, &g_navigationCommandRxByte, 1U);
}

NavigationStatus_t NavigationTask_Update(void)
{
    nav_explore_tick();

    if (s_explore_state == MAZE_EXPLORE_DONE) {
        return NAVIGATION_STATUS_REACHED;
    }

    if (s_explore_state == MAZE_EXPLORE_IDLE) {
        return NAVIGATION_STATUS_IDLE;
    }

    return NAVIGATION_STATUS_BUSY;
}

void NavigationTask_GetStatsSnapshot(NavigationTaskStats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    if (g_navigationMutex != NULL) {
        (void)osMutexAcquire(g_navigationMutex, osWaitForever);
    }

    *stats = g_navigationStats;

    if (g_navigationMutex != NULL) {
        (void)osMutexRelease(g_navigationMutex);
    }
}

uint16_t NavigationTask_CopySmoothPathPoints(NavigationPathPoint_t *points, uint16_t max_points)
{
    uint16_t count;

    if ((points == NULL) || (max_points == 0U)) {
        return 0U;
    }

    if (g_navigationMutex != NULL) {
        (void)osMutexAcquire(g_navigationMutex, osWaitForever);
    }

    count = (g_navigationSmoothPathLen < max_points) ?
             g_navigationSmoothPathLen : max_points;
    if (count > 0U) {
        (void)memcpy(points, g_navigationSmoothPath, count * sizeof(NavigationPathPoint_t));
    }

    if (g_navigationMutex != NULL) {
        (void)osMutexRelease(g_navigationMutex);
    }

    return count;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  FreeRTOS 线程入口
 * ═══════════════════════════════════════════════════════════════════════════ */

void StartNavigationTask(void *argument)
{
    (void)argument;
    (void)memset(&g_navigationStats, 0, sizeof(g_navigationStats));
    (void)memset(&s_maze_graph, 0, sizeof(s_maze_graph));

    for (;;) {
        /* 处理 UART 命令 */
        nav_process_pending_command();

        /* 推进探索状态机 */
        if (s_start_cell_set != 0U) {
            nav_explore_tick();
        }

        /* 更新统计信息 */
        {
            SlamPose2D_t pose;
            LocalizationTask_GetPoseSnapshot(&pose);

            if (g_navigationMutex != NULL) {
                (void)osMutexAcquire(g_navigationMutex, osWaitForever);
            }

            g_navigationStats.update_count++;
            g_navigationStats.current_pose = pose;
            g_navigationStats.last_status =
                (s_explore_state == MAZE_EXPLORE_DONE)  ? NAVIGATION_STATUS_REACHED :
                (s_explore_state == MAZE_EXPLORE_IDLE)  ? NAVIGATION_STATUS_IDLE :
                                                          NAVIGATION_STATUS_BUSY;

            if (g_navigationMutex != NULL) {
                (void)osMutexRelease(g_navigationMutex);
            }
        }

        osDelay(NAVIGATION_TASK_PERIOD_MS);
    }
}