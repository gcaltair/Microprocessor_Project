#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"

#include "../Inc/freertos_app.h"
#include "../Inc/localization_task.h"
#include "../Inc/mapping_task.h"
#include "../Inc/navigation_task.h"
#include "../Inc/occupancy_grid.h"
#include "pid.h"
#include "system.h"

#define NAVIGATION_TASK_PERIOD_MS          600U
#define NAVIGATION_GRID_DOWNSAMPLE         2U
#define NAVIGATION_MAX_WIDTH_CELLS         ((OGM_MAX_WIDTH_CELLS + NAVIGATION_GRID_DOWNSAMPLE - 1U) / NAVIGATION_GRID_DOWNSAMPLE)
#define NAVIGATION_MAX_HEIGHT_CELLS        ((OGM_MAX_HEIGHT_CELLS + NAVIGATION_GRID_DOWNSAMPLE - 1U) / NAVIGATION_GRID_DOWNSAMPLE)
#define NAVIGATION_MAX_CELL_COUNT          (NAVIGATION_MAX_WIDTH_CELLS * NAVIGATION_MAX_HEIGHT_CELLS)
#define NAVIGATION_MAX_PATH_POINTS         128U
#define NAVIGATION_COST_STRAIGHT           10U
#define NAVIGATION_COST_INF                65535U
#define NAVIGATION_PARENT_NONE             0xFFU
#define NAVIGATION_PARENT_START            0xFEU
#define NAVIGATION_FLAG_OPEN               0x01U
#define NAVIGATION_FLAG_CLOSED             0x02U
#define NAVIGATION_INFLATE_RADIUS_CELLS    1
#define NAVIGATION_REACH_DISTANCE_M        0.03f
#define NAVIGATION_MIN_SEGMENT_M           0.025f
#define NAVIGATION_FREE_CELL_THRESHOLD     (-3)

typedef struct {
    int16_t x;
    int16_t y;
} NavigationGridPoint_t;

typedef struct {
    float x_m;
    float y_m;
} NavigationWorldPoint_t;

static const int8_t g_navigationDx4[4] = {1, -1, 0, 0};
static const int8_t g_navigationDy4[4] = {0, 0, 1, -1};
static const uint8_t g_navigationReverseDir4[4] = {1U, 0U, 3U, 2U};

static NavigationTaskStats_t g_navigationStats;
static SlamPose2D_t g_navigationGoal;
static uint8_t g_navigationGoalValid = 0U;
static MappingGridMeta_t g_navigationMapMeta;
static uint16_t g_navigationWidthCells = 0U;
static uint16_t g_navigationHeightCells = 0U;
static int8_t g_navigationMapCells[OGM_MAX_CELL_COUNT];
static uint16_t g_navigationGScore[NAVIGATION_MAX_CELL_COUNT];
static uint8_t g_navigationFlags[NAVIGATION_MAX_CELL_COUNT];
static uint8_t g_navigationParentDir[NAVIGATION_MAX_CELL_COUNT];
static uint16_t g_navigationOpen[NAVIGATION_MAX_CELL_COUNT];
static uint16_t g_navigationOpenCount = 0U;
static NavigationGridPoint_t g_navigationRawPath[NAVIGATION_MAX_PATH_POINTS];
static NavigationGridPoint_t g_navigationSmoothGridPath[NAVIGATION_MAX_PATH_POINTS];
static NavigationWorldPoint_t g_navigationSmoothPath[NAVIGATION_MAX_PATH_POINTS];
static uint16_t g_navigationRawPathLen = 0U;
static uint16_t g_navigationSmoothPathLen = 0U;

static void navigation_lock(void)
{
    /* 保护导航目标和统计结构，避免上位机/其他线程设置目标时与导航线程冲突。 */
    if (g_navigationMutex != NULL) {
        (void)osMutexAcquire(g_navigationMutex, osWaitForever);
    }
}

static void navigation_unlock(void)
{
    /* 与 navigation_lock 成对使用。 */
    if (g_navigationMutex != NULL) {
        (void)osMutexRelease(g_navigationMutex);
    }
}

static uint16_t navigation_cell_index(uint16_t x, uint16_t y)
{
    /* 导航内部使用一维数组保存 A* 状态，减少二维数组带来的额外开销。 */
    return (uint16_t)(y * g_navigationWidthCells + x);
}

static uint8_t navigation_is_inside(int16_t x, int16_t y)
{
    /* 判断导航栅格坐标是否落在当前地图对应的导航范围内。 */
    if ((x < 0) || (y < 0)) {
        return 0U;
    }

    if (((uint16_t)x >= g_navigationWidthCells) || ((uint16_t)y >= g_navigationHeightCells)) {
        return 0U;
    }

    return 1U;
}

static uint8_t navigation_load_map_snapshot(void)
{
    uint32_t cell_count;

    /* 规划前复制一份地图快照，避免 A* 搜索期间长时间占用建图互斥锁。 */
    if (MappingTask_GetGridMeta(&g_navigationMapMeta) == 0U) {
        return 0U;
    }

    cell_count = (uint32_t)g_navigationMapMeta.width_cells * g_navigationMapMeta.height_cells;
    if ((cell_count == 0U) || (cell_count > OGM_MAX_CELL_COUNT)) {
        return 0U;
    }

    if (MappingTask_CopyGridCells(g_navigationMapCells, OGM_MAX_CELL_COUNT) == 0U) {
        return 0U;
    }

    g_navigationWidthCells = (uint16_t)((g_navigationMapMeta.width_cells + NAVIGATION_GRID_DOWNSAMPLE - 1U) /
                                        NAVIGATION_GRID_DOWNSAMPLE);
    g_navigationHeightCells = (uint16_t)((g_navigationMapMeta.height_cells + NAVIGATION_GRID_DOWNSAMPLE - 1U) /
                                         NAVIGATION_GRID_DOWNSAMPLE);
    if ((g_navigationWidthCells == 0U) ||
        (g_navigationHeightCells == 0U) ||
        (g_navigationWidthCells > NAVIGATION_MAX_WIDTH_CELLS) ||
        (g_navigationHeightCells > NAVIGATION_MAX_HEIGHT_CELLS)) {
        return 0U;
    }

    return 1U;
}

static uint8_t navigation_world_to_nav_cell(float x_m, float y_m, NavigationGridPoint_t *cell)
{
    SlamGridCoord_t map_cell;

    /* 先用建图模块的坐标转换，再把 5cm 地图栅格折算成 10cm 导航栅格。 */
    if (cell == NULL) {
        return 0U;
    }

    if (MappingTask_WorldToCell(x_m, y_m, &map_cell) == 0U) {
        return 0U;
    }

    cell->x = (int16_t)(map_cell.x / (int16_t)NAVIGATION_GRID_DOWNSAMPLE);
    cell->y = (int16_t)(map_cell.y / (int16_t)NAVIGATION_GRID_DOWNSAMPLE);
    return navigation_is_inside(cell->x, cell->y);
}

static NavigationWorldPoint_t navigation_nav_cell_to_world(const NavigationGridPoint_t *cell)
{
    NavigationWorldPoint_t point;
    float map_cell_center_x;
    float map_cell_center_y;

    /* 导航栅格点转换回世界坐标时，使用其覆盖的建图栅格块中心。 */
    map_cell_center_x = ((float)cell->x * (float)NAVIGATION_GRID_DOWNSAMPLE) +
                        ((float)NAVIGATION_GRID_DOWNSAMPLE * 0.5f);
    map_cell_center_y = ((float)cell->y * (float)NAVIGATION_GRID_DOWNSAMPLE) +
                        ((float)NAVIGATION_GRID_DOWNSAMPLE * 0.5f);
    point.x_m = g_navigationMapMeta.origin_x_m +
                (map_cell_center_x * g_navigationMapMeta.resolution_m_per_cell);
    point.y_m = g_navigationMapMeta.origin_y_m +
                (map_cell_center_y * g_navigationMapMeta.resolution_m_per_cell);
    return point;
}

static uint8_t navigation_map_cell_known_free(int16_t map_x, int16_t map_y)
{
    uint32_t index;
    int8_t value;

    /* 未知区域和地图外部都不作为可通行区域，避免导航穿过未确认空间。 */
    if ((map_x < 0) ||
        (map_y < 0) ||
        ((uint16_t)map_x >= g_navigationMapMeta.width_cells) ||
        ((uint16_t)map_y >= g_navigationMapMeta.height_cells)) {
        return 0U;
    }

    index = (uint32_t)(uint16_t)map_y * g_navigationMapMeta.width_cells + (uint32_t)(uint16_t)map_x;
    value = g_navigationMapCells[index];
    return (value <= NAVIGATION_FREE_CELL_THRESHOLD) ? 1U : 0U;
}

static uint8_t navigation_nav_cell_known_free(int16_t nav_x, int16_t nav_y)
{
    int16_t map_x_start;
    int16_t map_y_start;
    int16_t map_x;
    int16_t map_y;

    /* 一个导航栅格覆盖 2x2 个建图栅格，必须全部已知空闲才允许通行。 */
    if (navigation_is_inside(nav_x, nav_y) == 0U) {
        return 0U;
    }

    map_x_start = (int16_t)(nav_x * (int16_t)NAVIGATION_GRID_DOWNSAMPLE);
    map_y_start = (int16_t)(nav_y * (int16_t)NAVIGATION_GRID_DOWNSAMPLE);

    for (map_y = map_y_start; map_y < map_y_start + (int16_t)NAVIGATION_GRID_DOWNSAMPLE; ++map_y) {
        for (map_x = map_x_start; map_x < map_x_start + (int16_t)NAVIGATION_GRID_DOWNSAMPLE; ++map_x) {
            if (navigation_map_cell_known_free(map_x, map_y) == 0U) {
                return 0U;
            }
        }
    }

    return 1U;
}

static uint8_t navigation_is_blocked_inflated(int16_t nav_x,
                                              int16_t nav_y,
                                              const NavigationGridPoint_t *start_cell)
{
    int16_t check_y;
    int16_t check_x;

    /* 起点使用当前车体位置，允许它从未知/边界处脱困；其他格子需要经过膨胀检测。 */
    if ((start_cell != NULL) && (nav_x == start_cell->x) && (nav_y == start_cell->y)) {
        return 0U;
    }

    for (check_y = (int16_t)(nav_y - NAVIGATION_INFLATE_RADIUS_CELLS);
         check_y <= (int16_t)(nav_y + NAVIGATION_INFLATE_RADIUS_CELLS);
         ++check_y) {
        for (check_x = (int16_t)(nav_x - NAVIGATION_INFLATE_RADIUS_CELLS);
             check_x <= (int16_t)(nav_x + NAVIGATION_INFLATE_RADIUS_CELLS);
             ++check_x) {
            if (navigation_nav_cell_known_free(check_x, check_y) == 0U) {
                return 1U;
            }
        }
    }

    return 0U;
}

static uint16_t navigation_heuristic(const NavigationGridPoint_t *cell, const NavigationGridPoint_t *goal)
{
    uint16_t dx;
    uint16_t dy;

    /* 4 邻域 A* 使用曼哈顿距离作为启发函数。 */
    dx = (cell->x > goal->x) ? (uint16_t)(cell->x - goal->x) : (uint16_t)(goal->x - cell->x);
    dy = (cell->y > goal->y) ? (uint16_t)(cell->y - goal->y) : (uint16_t)(goal->y - cell->y);
    return (uint16_t)((dx + dy) * NAVIGATION_COST_STRAIGHT);
}

static void navigation_astar_reset(uint16_t cell_count)
{
    uint16_t idx;

    /* 每次规划前清空 A* 的代价、状态和父节点记录。 */
    for (idx = 0U; idx < cell_count; ++idx) {
        g_navigationGScore[idx] = NAVIGATION_COST_INF;
        g_navigationFlags[idx] = 0U;
        g_navigationParentDir[idx] = NAVIGATION_PARENT_NONE;
    }

    g_navigationOpenCount = 0U;
    g_navigationRawPathLen = 0U;
    g_navigationSmoothPathLen = 0U;
}

static void navigation_open_push(uint16_t index)
{
    /* 线性 open 表足够覆盖 48x48 导航栅格，避免引入堆结构的复杂度。 */
    if (g_navigationOpenCount < NAVIGATION_MAX_CELL_COUNT) {
        g_navigationOpen[g_navigationOpenCount++] = index;
        g_navigationFlags[index] |= NAVIGATION_FLAG_OPEN;
    }
}

static uint16_t navigation_open_pop_best(const NavigationGridPoint_t *goal)
{
    uint16_t best_open_pos = 0U;
    uint16_t best_index = g_navigationOpen[0];
    uint16_t best_f = NAVIGATION_COST_INF;
    uint16_t open_pos;

    /* 从 open 表中取 f=g+h 最小的节点。 */
    for (open_pos = 0U; open_pos < g_navigationOpenCount; ++open_pos) {
        uint16_t index = g_navigationOpen[open_pos];
        NavigationGridPoint_t cell;
        uint16_t f_score;

        cell.x = (int16_t)(index % g_navigationWidthCells);
        cell.y = (int16_t)(index / g_navigationWidthCells);
        f_score = (uint16_t)(g_navigationGScore[index] + navigation_heuristic(&cell, goal));
        if (f_score < best_f) {
            best_f = f_score;
            best_index = index;
            best_open_pos = open_pos;
        }
    }

    g_navigationOpen[best_open_pos] = g_navigationOpen[--g_navigationOpenCount];
    g_navigationFlags[best_index] &= (uint8_t)(~NAVIGATION_FLAG_OPEN);
    g_navigationFlags[best_index] |= NAVIGATION_FLAG_CLOSED;
    return best_index;
}

static uint8_t navigation_build_raw_path(uint16_t goal_index, uint16_t start_index)
{
    NavigationGridPoint_t reversed_path[NAVIGATION_MAX_PATH_POINTS];
    uint16_t reversed_len = 0U;
    uint16_t current_index = goal_index;
    uint8_t last_forward_dir = NAVIGATION_PARENT_NONE;
    uint8_t have_last_dir = 0U;
    uint16_t reverse_pos;

    /* 从终点沿父节点回溯到起点，得到反向路径。 */
    while (reversed_len < NAVIGATION_MAX_PATH_POINTS) {
        uint8_t parent_dir;

        reversed_path[reversed_len].x = (int16_t)(current_index % g_navigationWidthCells);
        reversed_path[reversed_len].y = (int16_t)(current_index / g_navigationWidthCells);
        reversed_len++;

        if (current_index == start_index) {
            break;
        }

        parent_dir = g_navigationParentDir[current_index];
        if ((parent_dir == NAVIGATION_PARENT_NONE) || (parent_dir == NAVIGATION_PARENT_START)) {
            return 0U;
        }

        current_index = navigation_cell_index((uint16_t)(reversed_path[reversed_len - 1U].x + g_navigationDx4[parent_dir]),
                                              (uint16_t)(reversed_path[reversed_len - 1U].y + g_navigationDy4[parent_dir]));
    }

    if ((reversed_len == 0U) ||
        (reversed_path[reversed_len - 1U].x != (int16_t)(start_index % g_navigationWidthCells)) ||
        (reversed_path[reversed_len - 1U].y != (int16_t)(start_index / g_navigationWidthCells))) {
        return 0U;
    }

    /* 只保留起点、拐点和终点，把 A* 栅格路径压缩成折线路径。 */
    g_navigationRawPathLen = 0U;
    for (reverse_pos = reversed_len; reverse_pos > 0U; --reverse_pos) {
        NavigationGridPoint_t current = reversed_path[reverse_pos - 1U];
        uint8_t store_point = 0U;

        if ((reverse_pos == reversed_len) || (reverse_pos == 1U)) {
            store_point = 1U;
        } else {
            NavigationGridPoint_t previous = reversed_path[reverse_pos];
            uint8_t forward_dir;

            if (current.x > previous.x) {
                forward_dir = 0U;
            } else if (current.x < previous.x) {
                forward_dir = 1U;
            } else if (current.y > previous.y) {
                forward_dir = 2U;
            } else {
                forward_dir = 3U;
            }

            if ((have_last_dir != 0U) && (forward_dir != last_forward_dir)) {
                store_point = 1U;
            }
            last_forward_dir = forward_dir;
            have_last_dir = 1U;
        }

        if (store_point != 0U) {
            if (g_navigationRawPathLen >= NAVIGATION_MAX_PATH_POINTS) {
                return 0U;
            }
            g_navigationRawPath[g_navigationRawPathLen++] = current;
        }
    }

    return (g_navigationRawPathLen > 0U) ? 1U : 0U;
}

static uint8_t navigation_find_path_map(const NavigationGridPoint_t *start_cell,
                                        const NavigationGridPoint_t *goal_cell)
{
    uint16_t cell_count;
    uint16_t start_index;
    uint16_t goal_index;

    /* 真正的 A* 主循环：在 4 邻域中扩展代价最低的节点，直到到达终点或 open 表耗尽。 */
    if ((start_cell == NULL) || (goal_cell == NULL)) {
        return 0U;
    }

    if ((navigation_is_inside(start_cell->x, start_cell->y) == 0U) ||
        (navigation_is_inside(goal_cell->x, goal_cell->y) == 0U)) {
        return 0U;
    }

    if (navigation_is_blocked_inflated(goal_cell->x, goal_cell->y, start_cell) != 0U) {
        return 0U;
    }

    cell_count = (uint16_t)(g_navigationWidthCells * g_navigationHeightCells);
    start_index = navigation_cell_index((uint16_t)start_cell->x, (uint16_t)start_cell->y);
    goal_index = navigation_cell_index((uint16_t)goal_cell->x, (uint16_t)goal_cell->y);
    navigation_astar_reset(cell_count);

    if (start_index == goal_index) {
        g_navigationRawPath[0] = *start_cell;
        g_navigationRawPathLen = 1U;
        return 1U;
    }

    g_navigationGScore[start_index] = 0U;
    g_navigationParentDir[start_index] = NAVIGATION_PARENT_START;
    navigation_open_push(start_index);

    while (g_navigationOpenCount > 0U) {
        uint16_t current_index = navigation_open_pop_best(goal_cell);
        NavigationGridPoint_t current_cell;
        uint8_t dir;

        if (current_index == goal_index) {
            return navigation_build_raw_path(goal_index, start_index);
        }

        current_cell.x = (int16_t)(current_index % g_navigationWidthCells);
        current_cell.y = (int16_t)(current_index / g_navigationWidthCells);

        for (dir = 0U; dir < 4U; ++dir) {
            int16_t next_x = (int16_t)(current_cell.x + g_navigationDx4[dir]);
            int16_t next_y = (int16_t)(current_cell.y + g_navigationDy4[dir]);
            uint16_t next_index;
            uint16_t tentative_g;

            if ((navigation_is_inside(next_x, next_y) == 0U) ||
                (navigation_is_blocked_inflated(next_x, next_y, start_cell) != 0U)) {
                continue;
            }

            next_index = navigation_cell_index((uint16_t)next_x, (uint16_t)next_y);
            if ((g_navigationFlags[next_index] & NAVIGATION_FLAG_CLOSED) != 0U) {
                continue;
            }

            tentative_g = (uint16_t)(g_navigationGScore[current_index] + NAVIGATION_COST_STRAIGHT);
            if (((g_navigationFlags[next_index] & NAVIGATION_FLAG_OPEN) == 0U) ||
                (tentative_g < g_navigationGScore[next_index])) {
                g_navigationGScore[next_index] = tentative_g;
                g_navigationParentDir[next_index] = g_navigationReverseDir4[dir];

                if ((g_navigationFlags[next_index] & NAVIGATION_FLAG_OPEN) == 0U) {
                    navigation_open_push(next_index);
                }
            }
        }
    }

    return 0U;
}

static uint8_t navigation_line_free(const NavigationGridPoint_t *start_cell,
                                    const NavigationGridPoint_t *end_cell)
{
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
    int16_t dx;
    int16_t dy;
    int16_t sx;
    int16_t sy;
    int16_t err;

    /* Bresenham 视线检测：如果直线经过膨胀障碍，则不能把中间路径点删掉。 */
    if ((start_cell == NULL) || (end_cell == NULL)) {
        return 0U;
    }

    x0 = start_cell->x;
    y0 = start_cell->y;
    x1 = end_cell->x;
    y1 = end_cell->y;
    dx = (int16_t)abs(x1 - x0);
    dy = (int16_t)(-abs(y1 - y0));
    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;
    err = (int16_t)(dx + dy);

    for (;;) {
        int16_t e2;

        if (navigation_is_blocked_inflated(x0, y0, start_cell) != 0U) {
            return 0U;
        }

        if ((x0 == x1) && (y0 == y1)) {
            return 1U;
        }

        e2 = (int16_t)(2 * err);
        if (e2 >= dy) {
            err = (int16_t)(err + dy);
            x0 = (int16_t)(x0 + sx);
        }
        if (e2 <= dx) {
            err = (int16_t)(err + dx);
            y0 = (int16_t)(y0 + sy);
        }
    }
}

static uint8_t navigation_build_smooth_path_from_raw_path(void)
{
    uint16_t raw_index = 0U;

    /* 用视线法从 A* 拐点中尽量跳到更远的点，减少直角折线。 */
    if (g_navigationRawPathLen == 0U) {
        return 0U;
    }

    g_navigationSmoothPathLen = 0U;
    g_navigationSmoothGridPath[g_navigationSmoothPathLen] = g_navigationRawPath[0];
    g_navigationSmoothPath[g_navigationSmoothPathLen] =
        navigation_nav_cell_to_world(&g_navigationSmoothGridPath[g_navigationSmoothPathLen]);
    g_navigationSmoothPathLen++;

    while (raw_index < (uint16_t)(g_navigationRawPathLen - 1U)) {
        uint16_t candidate;
        uint16_t best_next = (uint16_t)(raw_index + 1U);

        for (candidate = (uint16_t)(g_navigationRawPathLen - 1U); candidate > raw_index; --candidate) {
            if (navigation_line_free(&g_navigationRawPath[raw_index], &g_navigationRawPath[candidate]) != 0U) {
                best_next = candidate;
                break;
            }
        }

        if (g_navigationSmoothPathLen >= NAVIGATION_MAX_PATH_POINTS) {
            return 0U;
        }

        g_navigationSmoothGridPath[g_navigationSmoothPathLen] = g_navigationRawPath[best_next];
        g_navigationSmoothPath[g_navigationSmoothPathLen] =
            navigation_nav_cell_to_world(&g_navigationSmoothGridPath[g_navigationSmoothPathLen]);
        g_navigationSmoothPathLen++;
        raw_index = best_next;
    }

    return 1U;
}

static void navigation_update_stats_locked(NavigationStatus_t status,
                                           const SlamPose2D_t *current_pose,
                                           const SlamPose2D_t *goal_pose,
                                           const SlamPose2D_t *target_pose,
                                           float distance_to_goal_m,
                                           uint8_t target_valid)
{
    /* 统一更新导航统计，便于后续接入遥测或调试打印。 */
    g_navigationStats.update_count++;
    g_navigationStats.goal_valid = g_navigationGoalValid;
    g_navigationStats.target_valid = target_valid;
    g_navigationStats.last_status = status;
    g_navigationStats.raw_path_len = g_navigationRawPathLen;
    g_navigationStats.smooth_path_len = g_navigationSmoothPathLen;
    g_navigationStats.distance_to_goal_m = distance_to_goal_m;

    if (status == NAVIGATION_STATUS_FAILED) {
        g_navigationStats.fail_count++;
    }

    if (current_pose != NULL) {
        g_navigationStats.current_pose = *current_pose;
    }

    if (goal_pose != NULL) {
        g_navigationStats.goal_pose = *goal_pose;
    }

    if (target_pose != NULL) {
        g_navigationStats.target_pose = *target_pose;
    } else {
        (void)memset(&g_navigationStats.target_pose, 0, sizeof(g_navigationStats.target_pose));
    }
}

void NavigationTask_SetGoal(float goal_x_m, float goal_y_m)
{
    /* 设置终点后，导航线程会在下一次周期更新中自动规划并下发局部目标。 */
    navigation_lock();
    (void)memset(&g_navigationGoal, 0, sizeof(g_navigationGoal));
    g_navigationGoal.x_m = goal_x_m;
    g_navigationGoal.y_m = goal_y_m;
    g_navigationGoalValid = 1U;
    g_navigationStats.goal_valid = 1U;
    g_navigationStats.goal_pose = g_navigationGoal;
    g_navigationStats.last_status = NAVIGATION_STATUS_IDLE;
    navigation_unlock();
}

void NavigationTask_ClearGoal(void)
{
    /* 清除目标只影响导航线程，不强制停止当前正在执行的相对位移。 */
    navigation_lock();
    g_navigationGoalValid = 0U;
    (void)memset(&g_navigationGoal, 0, sizeof(g_navigationGoal));
    g_navigationStats.goal_valid = 0U;
    g_navigationStats.target_valid = 0U;
    g_navigationStats.last_status = NAVIGATION_STATUS_IDLE;
    g_navigationStats.raw_path_len = 0U;
    g_navigationStats.smooth_path_len = 0U;
    navigation_unlock();
}

NavigationStatus_t NavigationTask_Update(void)
{
    SlamPose2D_t current_pose;
    SlamPose2D_t goal_pose;
    SlamPose2D_t target_pose;
    NavigationGridPoint_t start_cell;
    NavigationGridPoint_t goal_cell;
    float dx_goal;
    float dy_goal;
    float distance_to_goal_m;
    NavigationStatus_t status = NAVIGATION_STATUS_FAILED;
    uint8_t target_valid = 0U;

    /* 读取目标快照；没有目标时保持空闲，不干预控制。 */
    navigation_lock();
    if (g_navigationGoalValid == 0U) {
        navigation_update_stats_locked(NAVIGATION_STATUS_IDLE, NULL, NULL, NULL, 0.0f, 0U);
        navigation_unlock();
        return NAVIGATION_STATUS_IDLE;
    }
    goal_pose = g_navigationGoal;
    navigation_unlock();

    LocalizationTask_GetEstimatedPoseSnapshot(&current_pose);
    dx_goal = goal_pose.x_m - current_pose.x_m;
    dy_goal = goal_pose.y_m - current_pose.y_m;
    distance_to_goal_m = sqrtf((dx_goal * dx_goal) + (dy_goal * dy_goal));

    if (distance_to_goal_m <= NAVIGATION_REACH_DISTANCE_M) {
        /* 到达终点后清除目标，避免后续周期重复下发微小移动。 */
        navigation_lock();
        g_navigationGoalValid = 0U;
        navigation_update_stats_locked(NAVIGATION_STATUS_REACHED,
                                       &current_pose,
                                       &goal_pose,
                                       NULL,
                                       distance_to_goal_m,
                                       0U);
        navigation_unlock();
        return NAVIGATION_STATUS_REACHED;
    }

    if ((navigation_load_map_snapshot() == 0U) ||
        (navigation_world_to_nav_cell(current_pose.x_m, current_pose.y_m, &start_cell) == 0U) ||
        (navigation_world_to_nav_cell(goal_pose.x_m, goal_pose.y_m, &goal_cell) == 0U) ||
        (navigation_find_path_map(&start_cell, &goal_cell) == 0U) ||
        (navigation_build_smooth_path_from_raw_path() == 0U)) {
        navigation_lock();
        navigation_update_stats_locked(NAVIGATION_STATUS_FAILED,
                                       &current_pose,
                                       &goal_pose,
                                       NULL,
                                       distance_to_goal_m,
                                       0U);
        navigation_unlock();
        return NAVIGATION_STATUS_FAILED;
    }

    if (g_navigationSmoothPathLen >= 2U) {
        target_pose.x_m = g_navigationSmoothPath[1].x_m;
        target_pose.y_m = g_navigationSmoothPath[1].y_m;
    } else {
        target_pose.x_m = goal_pose.x_m;
        target_pose.y_m = goal_pose.y_m;
    }
    target_pose.theta_deg = current_pose.theta_deg;
    target_pose.timestamp_ms = HAL_GetTick();
    target_valid = 1U;

    if ((g_relative_move_state == RELATIVE_MOVE_IDLE) && (g_control_mode != CONTROL_MODE_SPEED_TEST)) {
        float dx_target = target_pose.x_m - current_pose.x_m;
        float dy_target = target_pose.y_m - current_pose.y_m;
        float target_distance_m = sqrtf((dx_target * dx_target) + (dy_target * dy_target));

        /* 只有控制空闲时才下发下一段相对位移，避免打断正在执行的运动。 */
        if (target_distance_m >= NAVIGATION_MIN_SEGMENT_M) {
            Start_Relative_Move(dx_target, dy_target);
        }
        status = NAVIGATION_STATUS_OK;
    } else {
        status = NAVIGATION_STATUS_BUSY;
    }

    navigation_lock();
    navigation_update_stats_locked(status,
                                   &current_pose,
                                   &goal_pose,
                                   &target_pose,
                                   distance_to_goal_m,
                                   target_valid);
    navigation_unlock();
    return status;
}

void NavigationTask_GetStatsSnapshot(NavigationTaskStats_t *stats)
{
    /* 复制导航统计快照，调用方不需要持有导航互斥锁。 */
    if (stats == NULL) {
        return;
    }

    navigation_lock();
    *stats = g_navigationStats;
    navigation_unlock();
}

void StartNavigationTask(void *argument)
{
    (void)argument;

    /* 固定周期运行导航状态机，对齐参考代码中的 600ms 导航调度节奏。 */
    for (;;) {
        (void)NavigationTask_Update();
        osDelay(NAVIGATION_TASK_PERIOD_MS);
    }
}
