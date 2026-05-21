#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"

#include "../Inc/freertos_app.h"
#include "../Inc/localization_task.h"
#include "../Inc/mapping_task.h"
#include "../Inc/navigation_task.h"
#include "../Inc/occupancy_grid.h"
#include "../Inc/usart.h"
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
#define NAVIGATION_REACH_DISTANCE_M        0.1f
#define NAVIGATION_MIN_SEGMENT_M           0.1f
#define NAVIGATION_FREE_CELL_THRESHOLD     (-3)
#define NAVIGATION_OCCUPIED_CELL_THRESHOLD (5)
#define NAVIGATION_COMMAND_MAX_LEN         64U

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
static uint8_t g_navigationCommandRxByte = 0U;
static char g_navigationCommandLine[NAVIGATION_COMMAND_MAX_LEN];
static char g_navigationPendingCommand[NAVIGATION_COMMAND_MAX_LEN];
static volatile uint8_t g_navigationCommandIndex = 0U;
static volatile uint8_t g_navigationCommandPending = 0U;
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
static NavigationPathPoint_t g_navigationPublishedPath[NAVIGATION_PATH_TELEMETRY_MAX_POINTS];
static uint16_t g_navigationPublishedPathLen = 0U;
static uint8_t g_navigationPlanOnlyMode = 0U;

static void navigation_lock(void);
static void navigation_unlock(void);

/*
 * 解析导航串口命令中的目标坐标参数。
 *
 * 输入格式要求为两个用空白字符或逗号分隔的浮点数，例如 "1.20 -0.35" 或 "1.20,-0.35"。
 * 解析成功时把结果写入 x_m 和 y_m，单位保持为导航模块统一使用的米；
 * 任一指针为空、缺少第一个数或缺少第二个数时都返回失败。
 */
static uint8_t navigation_parse_float_pair(const char *text, float *x_m, float *y_m)
{
    char *end_ptr;
    char *second_start;
    float parsed_x;
    float parsed_y;

    /* 解析 "x y" 两个浮点数，单位约定为米。 */
    if ((text == NULL) || (x_m == NULL) || (y_m == NULL)) {
        return 0U;
    }

    parsed_x = strtof(text, &end_ptr);
    if (end_ptr == text) {
        return 0U;
    }

    second_start = end_ptr;
    while ((*second_start == ' ') || (*second_start == '\t')) {
        second_start++;
    }
    if (*second_start == ',') {
        second_start++;
    }
    while ((*second_start == ' ') || (*second_start == '\t')) {
        second_start++;
    }

    parsed_y = strtof(second_start, &end_ptr);
    if (end_ptr == second_start) {
        return 0U;
    }

    *x_m = parsed_x;
    *y_m = parsed_y;
    return 1U;
}

/*
 * 在线程上下文中处理串口中断已经拼好的导航命令。
 *
 * 中断服务函数只负责接收字节并保存完整命令行，本函数在导航任务中取出
 * 待处理命令，避免在中断里调用较重的解析和目标设置逻辑。当前支持：
 *   NAV x y : 设置世界坐标目标点，单位为米；
 *   NAVC    : 清除当前导航目标。
 *   Pdx,dy  : 绕过 A*，直接向基础位置环下发相对位移，单位为米。
 */
static void navigation_process_pending_command(void)
{
    char command[NAVIGATION_COMMAND_MAX_LEN];
    uint8_t has_command = 0U;

    /* 从中断缓冲区取出完整命令行，实际解析放在线程上下文执行。 */
    navigation_lock();
    if (g_navigationCommandPending != 0U) {
        (void)memcpy(command, g_navigationPendingCommand, sizeof(command));
        g_navigationCommandPending = 0U;
        has_command = 1U;
    }
    navigation_unlock();

    if (has_command == 0U) {
        return;
    }

    if ((command[0] == 'P') && (command[1] == 'L') && (command[2] == 'A') && (command[3] == 'N') &&
        (command[4] == ' ')) {
        float goal_x_m;
        float goal_y_m;

        if (navigation_parse_float_pair(&command[5], &goal_x_m, &goal_y_m) != 0U) {
            NavigationTask_SetPlanGoal(goal_x_m, goal_y_m);
        }
    } else if ((command[0] == 'N') && (command[1] == 'A') && (command[2] == 'V') && (command[3] == ' ')) {
        float goal_x_m;
        float goal_y_m;

        if (navigation_parse_float_pair(&command[4], &goal_x_m, &goal_y_m) != 0U) {
            NavigationTask_SetGoal(goal_x_m, goal_y_m);
        }
    } else if ((command[0] == 'N') && (command[1] == 'A') && (command[2] == 'V') && (command[3] == 'C')) {
        NavigationTask_ClearGoal();
    } else if ((command[0] == 'P') || (command[0] == 'p')) {
        float dx_m;
        float dy_m;

        if (navigation_parse_float_pair(&command[1], &dx_m, &dy_m) != 0U) {
            /*
             * 调试命令直接使用基础位置环。先清导航目标，避免导航任务下一周期
             * 继续规划并覆盖这次手动下发的相对位移。
             */
            NavigationTask_ClearGoal();
            Start_Relative_Move(dx_m, dy_m);
        }
    }
}

/*
 * 获取导航模块互斥锁。
 *
 * 互斥锁保护导航目标、统计信息以及命令缓冲状态，防止串口命令、其他任务
 * 与导航任务同时读写这些共享数据。若系统初始化阶段互斥锁尚未创建，则
 * 直接跳过，便于早期调用保持可用。
 */
static void navigation_lock(void)
{
    /* 保护导航目标和统计结构，避免上位机/其他线程设置目标时与导航线程冲突。 */
    if (g_navigationMutex != NULL) {
        (void)osMutexAcquire(g_navigationMutex, osWaitForever);
    }
}

/*
 * 释放导航模块互斥锁。
 *
 * 必须与 navigation_lock 成对使用；同样兼容互斥锁尚未创建的早期阶段。
 */
static void navigation_unlock(void)
{
    /* 与 navigation_lock 成对使用。 */
    if (g_navigationMutex != NULL) {
        (void)osMutexRelease(g_navigationMutex);
    }
}

/*
 * 将二维导航栅格坐标转换为一维数组下标。
 *
 * A* 的代价、标志位、父节点和 open 表都使用一维数组保存；调用者需保证
 * x/y 已经在当前导航地图范围内，否则会得到无效下标。
 */
static uint16_t navigation_cell_index(uint16_t x, uint16_t y)
{
    /* 导航内部使用一维数组保存 A* 状态，减少二维数组带来的额外开销。 */
    return (uint16_t)(y * g_navigationWidthCells + x);
}

/*
 * 判断导航栅格坐标是否位于当前导航地图内部。
 *
 * 该函数只检查下采样后的导航地图边界，不判断对应栅格是否可通行。
 * 返回 1 表示在范围内，返回 0 表示越界。
 */
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

/*
 * 从建图模块复制一份供路径规划使用的地图快照。
 *
 * 规划期间不长期持有建图模块锁，而是先复制元数据和栅格数组，再按
 * NAVIGATION_GRID_DOWNSAMPLE 把原始地图尺寸折算为导航栅格尺寸。
 * 若地图无效、尺寸超过静态缓冲区或复制失败，则返回 0。
 */
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

/*
 * 将世界坐标转换为导航栅格坐标。
 *
 * 先复用 MappingTask_WorldToCell 完成世界坐标到原始建图栅格的转换，
 * 再按导航下采样比例折算到 A* 使用的低分辨率栅格。转换后的栅格必须
 * 位于导航地图内部，否则返回失败。
 */
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

/*
 * 将导航栅格坐标转换回世界坐标。
 *
 * 结果取该导航栅格覆盖的原始建图栅格块中心点，用于生成局部目标点。
 * 调用者需传入有效 cell 指针，并保证 cell 位于当前地图范围内。
 */
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

/*
 * 判断原始建图栅格是否可作为可通行空间。
 *
 * 地图外部一律视为不可通行，防止路径规划穿出当前地图边界。地图内部
 * 当前采用“不是明确占据就可通行”的策略：小于占据阈值 5 的值都允许
 * 通过，因此未知区域也可以被探索路径使用。
 */
static uint8_t navigation_map_cell_known_free(int16_t map_x, int16_t map_y)
{
    uint32_t index;
    int8_t value;

    /* 越界区域不作为可通行区域 */
    if ((map_x < 0) ||
        (map_y < 0) ||
        ((uint16_t)map_x >= g_navigationMapMeta.width_cells) ||
        ((uint16_t)map_y >= g_navigationMapMeta.height_cells)) {
        return 0U;
    }

    index = (uint32_t)(uint16_t)map_y * g_navigationMapMeta.width_cells + (uint32_t)(uint16_t)map_x;
    value = g_navigationMapCells[index];

    /*
     * 原来的判断是：(value <= NAVIGATION_FREE_CELL_THRESHOLD) ? 1U : 0U;
     * 也就是只有明确探明为空地（<=-3）才让走，导致未知区域（0）把路堵死。
     * 现在改为：只要不是明确的墙（占据阈值通常为 5），哪怕是未知的 0，也都允许通行！
     */
    return (value < NAVIGATION_OCCUPIED_CELL_THRESHOLD) ? 1U : 0U;
}

/*
 * 判断一个下采样后的导航栅格是否可通行。
 *
 * 一个导航栅格覆盖 NAVIGATION_GRID_DOWNSAMPLE x NAVIGATION_GRID_DOWNSAMPLE
 * 个原始建图栅格；只有覆盖范围内所有原始栅格都允许通行时，该导航栅格
 * 才被视为可通行。这样可以避免下采样后窄障碍被漏掉。
 */
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

/*
 * 判断导航栅格在障碍物膨胀后是否被阻挡。
 *
 * 为了给车体尺寸和定位误差留出余量，会检查目标栅格周围
 * NAVIGATION_INFLATE_RADIUS_CELLS 范围内是否存在不可通行格。起点特殊
 * 放行，避免车辆当前所在栅格因为贴近障碍或地图边界而无法启动规划。
 */
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

/*
 * 计算 A* 搜索使用的启发式代价。
 *
 * 当前路径只允许上下左右四邻域移动，因此使用曼哈顿距离乘以直行代价。
 * 该启发式不会高估四邻域路径代价，适合作为 A* 的 h 值。
 */
static uint16_t navigation_heuristic(const NavigationGridPoint_t *cell, const NavigationGridPoint_t *goal)
{
    uint16_t dx;
    uint16_t dy;

    /* 4 邻域 A* 使用曼哈顿距离作为启发函数。 */
    dx = (cell->x > goal->x) ? (uint16_t)(cell->x - goal->x) : (uint16_t)(goal->x - cell->x);
    dy = (cell->y > goal->y) ? (uint16_t)(cell->y - goal->y) : (uint16_t)(goal->y - cell->y);
    return (uint16_t)((dx + dy) * NAVIGATION_COST_STRAIGHT);
}

/*
 * 重置一次 A* 搜索需要的临时状态。
 *
 * 将所有节点代价设为无穷大，清空 open/closed 标志和父节点方向，同时
 * 清空上一轮生成的原始路径与平滑路径长度。
 */
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

/*
 * 把一个节点加入 A* open 表。
 *
 * open 表使用线性数组实现，地图尺寸较小时可避免维护堆结构的额外复杂度。
 * 加入节点时同步设置 OPEN 标志，防止同一节点被重复加入。
 */
static void navigation_open_push(uint16_t index)
{
    /* 线性 open 表足够覆盖 48x48 导航栅格，避免引入堆结构的复杂度。 */
    if (g_navigationOpenCount < NAVIGATION_MAX_CELL_COUNT) {
        g_navigationOpen[g_navigationOpenCount++] = index;
        g_navigationFlags[index] |= NAVIGATION_FLAG_OPEN;
    }
}

/*
 * 从 A* open 表中取出 f=g+h 最小的节点。
 *
 * 由于 open 表是线性数组，本函数每次遍历所有候选节点计算 f 值，取出后
 * 用末尾元素填补空位，并把该节点标记为 CLOSED。
 */
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

/*
 * 根据 A* 父节点记录回溯并生成原始折线路径。
 *
 * A* 完成后从 goal_index 沿父节点方向回到 start_index，先得到反向路径，
 * 再正向输出。输出时只保留起点、转折点和终点，去掉同一直线上的中间格，
 * 降低后续平滑与控制目标的点数。
 */
static uint8_t navigation_direction_between(const NavigationGridPoint_t *start_cell,
                                            const NavigationGridPoint_t *end_cell,
                                            uint8_t *direction)
{
    if ((start_cell == NULL) || (end_cell == NULL) || (direction == NULL)) {
        return 0U;
    }

    if ((end_cell->x == (int16_t)(start_cell->x + 1)) && (end_cell->y == start_cell->y)) {
        *direction = 0U;
        return 1U;
    }
    if ((end_cell->x == (int16_t)(start_cell->x - 1)) && (end_cell->y == start_cell->y)) {
        *direction = 1U;
        return 1U;
    }
    if ((end_cell->x == start_cell->x) && (end_cell->y == (int16_t)(start_cell->y + 1))) {
        *direction = 2U;
        return 1U;
    }
    if ((end_cell->x == start_cell->x) && (end_cell->y == (int16_t)(start_cell->y - 1))) {
        *direction = 3U;
        return 1U;
    }

    return 0U;
}

static uint8_t navigation_build_raw_path(uint16_t goal_index, uint16_t start_index)
{
    NavigationGridPoint_t reversed_path[NAVIGATION_MAX_PATH_POINTS];
    uint16_t reversed_len = 0U;
    uint16_t current_index = goal_index;
    uint8_t last_forward_dir = NAVIGATION_PARENT_NONE;
    uint8_t have_last_dir = 0U;
    NavigationGridPoint_t previous;
    uint16_t forward_pos;

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
    g_navigationRawPath[g_navigationRawPathLen++] = reversed_path[reversed_len - 1U];
    previous = reversed_path[reversed_len - 1U];

    for (forward_pos = (uint16_t)(reversed_len - 1U); forward_pos > 0U; --forward_pos) {
        NavigationGridPoint_t current = reversed_path[forward_pos - 1U];
        uint8_t forward_dir;

        if (navigation_direction_between(&previous, &current, &forward_dir) == 0U) {
            return 0U;
        }

        if ((have_last_dir != 0U) && (forward_dir != last_forward_dir)) {
            if ((g_navigationRawPath[g_navigationRawPathLen - 1U].x != previous.x) ||
                (g_navigationRawPath[g_navigationRawPathLen - 1U].y != previous.y)) {
                if (g_navigationRawPathLen >= NAVIGATION_MAX_PATH_POINTS) {
                    return 0U;
                }
                g_navigationRawPath[g_navigationRawPathLen++] = previous;
            }
        }

        last_forward_dir = forward_dir;
        have_last_dir = 1U;
        previous = current;
    }

    if ((g_navigationRawPath[g_navigationRawPathLen - 1U].x != reversed_path[0].x) ||
        (g_navigationRawPath[g_navigationRawPathLen - 1U].y != reversed_path[0].y)) {
        if (g_navigationRawPathLen >= NAVIGATION_MAX_PATH_POINTS) {
            return 0U;
        }
        g_navigationRawPath[g_navigationRawPathLen++] = reversed_path[0];
    }

    return (g_navigationRawPathLen > 0U) ? 1U : 0U;
}

/*
 * 在导航栅格地图上执行 A* 路径搜索。
 *
 * 输入为起点和终点的导航栅格坐标。函数会先检查边界和终点膨胀碰撞，再
 * 使用四邻域扩展搜索；搜索成功时填充 g_navigationRawPath，失败时返回 0。
 */
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

/*
 * 检查两个导航栅格点之间的直线路径是否可通行。
 *
 * 使用 Bresenham 栅格遍历算法枚举直线经过的所有导航格，并对每个格执行
 * 障碍物膨胀检查。该函数用于路径平滑，确保删除中间拐点后不会穿过障碍。
 */
static uint8_t navigation_line_free(const NavigationGridPoint_t *start_cell,
                                    const NavigationGridPoint_t *end_cell)
{
    /* Bresenham 视线检测：如果直线经过膨胀障碍，则不能把中间路径点删掉。 */
    if ((start_cell == NULL) || (end_cell == NULL)) {
        return 0U;
    }

    if ((start_cell->x != end_cell->x) && (start_cell->y != end_cell->y)) {
        return 0U;
    }

    if (start_cell->x == end_cell->x) {
        int16_t y = start_cell->y;
        int16_t step = (end_cell->y >= start_cell->y) ? 1 : -1;

        for (;;) {
            if (navigation_is_blocked_inflated(start_cell->x, y, start_cell) != 0U) {
                return 0U;
            }
            if (y == end_cell->y) {
                return 1U;
            }
            y = (int16_t)(y + step);
        }
    } else {
        int16_t x = start_cell->x;
        int16_t step = (end_cell->x >= start_cell->x) ? 1 : -1;

        for (;;) {
            if (navigation_is_blocked_inflated(x, start_cell->y, start_cell) != 0U) {
                return 0U;
            }
            if (x == end_cell->x) {
                return 1U;
            }
            x = (int16_t)(x + step);
        }
    }
}

/*
 * 基于原始折线路径生成更少拐点的平滑路径。
 *
 * 从当前原始路径点开始，尽量向后寻找最远且直线可达的点作为下一个点。
 * 每个保留下来的导航栅格点同时转换为世界坐标，供控制层作为局部目标使用。
 */
static uint8_t navigation_orthogonal_link_free(const NavigationGridPoint_t *start_cell,
                                               const NavigationGridPoint_t *end_cell,
                                               const NavigationGridPoint_t *preferred_next,
                                               NavigationGridPoint_t *link_points,
                                               uint8_t *link_len)
{
    NavigationGridPoint_t corners[2];
    uint8_t first_corner = 0U;
    uint8_t attempt;

    if ((start_cell == NULL) || (end_cell == NULL) || (link_points == NULL) || (link_len == NULL)) {
        return 0U;
    }

    if (navigation_line_free(start_cell, end_cell) != 0U) {
        link_points[0] = *end_cell;
        *link_len = 1U;
        return 1U;
    }

    corners[0].x = start_cell->x;
    corners[0].y = end_cell->y;
    corners[1].x = end_cell->x;
    corners[1].y = start_cell->y;

    if (preferred_next != NULL) {
        if (preferred_next->y == start_cell->y) {
            first_corner = 1U;
        } else if (preferred_next->x == start_cell->x) {
            first_corner = 0U;
        }
    }

    for (attempt = 0U; attempt < 2U; ++attempt) {
        uint8_t corner_index = (attempt == 0U) ? first_corner : (uint8_t)(1U - first_corner);
        NavigationGridPoint_t corner = corners[corner_index];

        if (((corner.x == start_cell->x) && (corner.y == start_cell->y)) ||
            ((corner.x == end_cell->x) && (corner.y == end_cell->y)) ||
            (navigation_is_inside(corner.x, corner.y) == 0U)) {
            continue;
        }

        if ((navigation_line_free(start_cell, &corner) != 0U) &&
            (navigation_line_free(&corner, end_cell) != 0U)) {
            link_points[0] = corner;
            link_points[1] = *end_cell;
            *link_len = 2U;
            return 1U;
        }
    }

    return 0U;
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
        NavigationGridPoint_t best_link[2];
        uint8_t best_link_len = 1U;
        uint8_t link_pos;

        best_link[0] = g_navigationRawPath[best_next];

        for (candidate = (uint16_t)(g_navigationRawPathLen - 1U); candidate > raw_index; --candidate) {
            NavigationGridPoint_t link_points[2];
            uint8_t link_len = 0U;

            if (navigation_orthogonal_link_free(&g_navigationRawPath[raw_index],
                                                &g_navigationRawPath[candidate],
                                                &g_navigationRawPath[raw_index + 1U],
                                                link_points,
                                                &link_len) != 0U) {
                best_next = candidate;
                best_link[0] = link_points[0];
                best_link[1] = link_points[1];
                best_link_len = link_len;
                break;
            }
        }

        for (link_pos = 0U; link_pos < best_link_len; ++link_pos) {
            if ((g_navigationSmoothGridPath[g_navigationSmoothPathLen - 1U].x == best_link[link_pos].x) &&
                (g_navigationSmoothGridPath[g_navigationSmoothPathLen - 1U].y == best_link[link_pos].y)) {
                continue;
            }

            if (g_navigationSmoothPathLen >= NAVIGATION_MAX_PATH_POINTS) {
                return 0U;
            }

            g_navigationSmoothGridPath[g_navigationSmoothPathLen] = best_link[link_pos];
            g_navigationSmoothPath[g_navigationSmoothPathLen] =
                navigation_nav_cell_to_world(&g_navigationSmoothGridPath[g_navigationSmoothPathLen]);
            g_navigationSmoothPathLen++;
        }
        raw_index = best_next;
    }

    return 1U;
}

/*
 * 在已持有导航互斥锁的前提下更新统计快照。
 *
 * 该函数集中维护导航状态、路径长度、当前位姿、目标位姿、局部目标和失败
 * 计数，方便 UI、遥测或调试接口读取一致的 NavigationTaskStats_t 快照。
 */
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

static void navigation_clear_published_path_locked(void)
{
    g_navigationPublishedPathLen = 0U;
}

static void navigation_publish_smooth_path_locked(void)
{
    uint16_t idx;
    uint16_t copy_len = g_navigationSmoothPathLen;

    if (copy_len > NAVIGATION_PATH_TELEMETRY_MAX_POINTS) {
        copy_len = NAVIGATION_PATH_TELEMETRY_MAX_POINTS;
    }

    for (idx = 0U; idx < copy_len; ++idx) {
        g_navigationPublishedPath[idx].x_m = g_navigationSmoothPath[idx].x_m;
        g_navigationPublishedPath[idx].y_m = g_navigationSmoothPath[idx].y_m;
    }

    g_navigationPublishedPathLen = copy_len;
}

static void navigation_set_goal_mode(float goal_x_m, float goal_y_m, uint8_t plan_only)
{
    navigation_lock();
    (void)memset(&g_navigationGoal, 0, sizeof(g_navigationGoal));
    g_navigationGoal.x_m = goal_x_m;
    g_navigationGoal.y_m = goal_y_m;
    g_navigationGoalValid = 1U;
    g_navigationPlanOnlyMode = plan_only;
    g_navigationStats.goal_valid = 1U;
    g_navigationStats.goal_pose = g_navigationGoal;
    g_navigationStats.last_status = NAVIGATION_STATUS_IDLE;
    g_navigationStats.raw_path_len = 0U;
    g_navigationStats.smooth_path_len = 0U;
    navigation_clear_published_path_locked();
    navigation_unlock();
}

/*
 * 设置新的世界坐标导航目标。
 *
 * 上位机或其他任务调用该接口后，导航任务会在下一次周期更新中读取目标、
 * 重新规划路径并下发局部相对位移。该接口只记录目标，不立即执行规划。
 */
void NavigationTask_SetGoal(float goal_x_m, float goal_y_m)
{
    /* 设置终点后，导航线程会在下一次周期更新中自动规划并下发局部目标。 */
    navigation_set_goal_mode(goal_x_m, goal_y_m, 0U);
}

void NavigationTask_SetPlanGoal(float goal_x_m, float goal_y_m)
{
    navigation_set_goal_mode(goal_x_m, goal_y_m, 1U);
}

/*
 * 清除当前导航目标和路径统计。
 *
 * 清除后 NavigationTask_Update 会回到 IDLE 状态。该函数不直接终止已经下发
 * 给运动控制层的相对位移，只阻止导航任务继续规划和下发新的局部目标。
 */
void NavigationTask_ClearGoal(void)
{
    /* 清除目标只影响导航线程，不强制停止当前正在执行的相对位移。 */
    navigation_lock();
    g_navigationGoalValid = 0U;
    g_navigationPlanOnlyMode = 0U;
    (void)memset(&g_navigationGoal, 0, sizeof(g_navigationGoal));
    g_navigationStats.goal_valid = 0U;
    g_navigationStats.target_valid = 0U;
    g_navigationStats.last_status = NAVIGATION_STATUS_IDLE;
    g_navigationStats.raw_path_len = 0U;
    g_navigationStats.smooth_path_len = 0U;
    navigation_clear_published_path_locked();
    navigation_unlock();
}

/*
 * 启动导航命令串口的单字节中断接收。
 *
 * UART5 每收到一个字节都会进入接收完成回调，由
 * NavigationTask_HandleCommandRxCompleteFromIsr 继续拼接命令行并重新挂接
 * 下一次接收。
 */
void NavigationTask_StartCommandRx(void)
{
    /* UART5 同时用于遥测发送和命令接收；这里挂 1 字节中断接收即可。 */
    g_navigationCommandIndex = 0U;
    g_navigationCommandPending = 0U;
    (void)HAL_UART_Receive_IT(&huart5, &g_navigationCommandRxByte, 1U);
}

/*
 * 串口接收完成中断中的导航命令字节处理。
 *
 * 本函数只做低开销的行缓冲：遇到换行时把完整命令复制到 pending 缓冲区，
 * 交给导航任务线程解析；普通字符追加到当前行；超长命令直接丢弃并重新
 * 开始接收，防止缓冲区溢出。
 */
void NavigationTask_HandleCommandRxCompleteFromIsr(void)
{
    uint8_t byte = g_navigationCommandRxByte;

    /* 中断里只拼命令行：NAV x y 或 NAVC。满行/换行后交给导航线程解析。 */
    if ((byte == '\n') || (byte == '\r')) {
        if (g_navigationCommandIndex > 0U) {
            g_navigationCommandLine[g_navigationCommandIndex] = '\0';
            if (g_navigationCommandPending == 0U) {
                (void)memcpy(g_navigationPendingCommand,
                             g_navigationCommandLine,
                             sizeof(g_navigationPendingCommand));
                g_navigationCommandPending = 1U;
            }
            g_navigationCommandIndex = 0U;
        }
    } else if (g_navigationCommandIndex < (NAVIGATION_COMMAND_MAX_LEN - 1U)) {
        g_navigationCommandLine[g_navigationCommandIndex++] = (char)byte;
    } else {
        g_navigationCommandIndex = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart5, &g_navigationCommandRxByte, 1U);
}

/*
 * 执行一次导航状态机更新。
 *
 * 更新流程包括：读取目标快照、读取当前定位、判断是否到达、复制地图快照、
 * 将起终点转换到导航栅格、执行 A* 搜索、平滑路径、选择下一个局部目标，
 * 并在运动控制空闲时下发相对位移。函数返回本周期的导航状态。
 */
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
    uint8_t plan_only = 0U;

    /* 读取目标快照；没有目标时保持空闲，不干预控制。 */
    navigation_lock();
    if (g_navigationGoalValid == 0U) {
        navigation_clear_published_path_locked();
        navigation_update_stats_locked(NAVIGATION_STATUS_IDLE, NULL, NULL, NULL, 0.0f, 0U);
        navigation_unlock();
        return NAVIGATION_STATUS_IDLE;
    }
    goal_pose = g_navigationGoal;
    plan_only = g_navigationPlanOnlyMode;
    navigation_unlock();

    LocalizationTask_GetEstimatedPoseSnapshot(&current_pose);
    dx_goal = goal_pose.x_m - current_pose.x_m;
    dy_goal = goal_pose.y_m - current_pose.y_m;
    distance_to_goal_m = sqrtf((dx_goal * dx_goal) + (dy_goal * dy_goal));

    if (distance_to_goal_m <= NAVIGATION_REACH_DISTANCE_M) {
        /* 到达终点后清除目标，避免后续周期重复下发微小移动。 */
        navigation_lock();
        g_navigationGoalValid = 0U;
        g_navigationPlanOnlyMode = 0U;
        navigation_clear_published_path_locked();
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
        navigation_clear_published_path_locked();
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
        if ((g_navigationSmoothGridPath[1].x != start_cell.x) &&
            (g_navigationSmoothGridPath[1].y == start_cell.y)) {
            target_pose.x_m = g_navigationSmoothPath[1].x_m;
            target_pose.y_m = current_pose.y_m;
        } else if ((g_navigationSmoothGridPath[1].y != start_cell.y) &&
                   (g_navigationSmoothGridPath[1].x == start_cell.x)) {
            target_pose.x_m = current_pose.x_m;
            target_pose.y_m = g_navigationSmoothPath[1].y_m;
        } else {
            float dx_target_abs = fabsf(g_navigationSmoothPath[1].x_m - current_pose.x_m);
            float dy_target_abs = fabsf(g_navigationSmoothPath[1].y_m - current_pose.y_m);

            if (dx_target_abs >= dy_target_abs) {
                target_pose.x_m = g_navigationSmoothPath[1].x_m;
                target_pose.y_m = current_pose.y_m;
            } else {
                target_pose.x_m = current_pose.x_m;
                target_pose.y_m = g_navigationSmoothPath[1].y_m;
            }
        }
    } else {
        float dx_goal_abs = fabsf(goal_pose.x_m - current_pose.x_m);
        float dy_goal_abs = fabsf(goal_pose.y_m - current_pose.y_m);

        if (dx_goal_abs >= dy_goal_abs) {
            target_pose.x_m = goal_pose.x_m;
            target_pose.y_m = current_pose.y_m;
        } else {
            target_pose.x_m = current_pose.x_m;
            target_pose.y_m = goal_pose.y_m;
        }
    }
    target_pose.theta_deg = current_pose.theta_deg;
    target_pose.timestamp_ms = HAL_GetTick();
    target_valid = 1U;

    if (plan_only != 0U) {
        status = NAVIGATION_STATUS_OK;
    } else if ((g_relative_move_state == RELATIVE_MOVE_IDLE) && (g_control_mode != CONTROL_MODE_SPEED_TEST)) {
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
    navigation_publish_smooth_path_locked();
    navigation_update_stats_locked(status,
                                   &current_pose,
                                   &goal_pose,
                                   &target_pose,
                                   distance_to_goal_m,
                                   target_valid);
    navigation_unlock();
    return status;
}

/*
 * 获取导航统计信息快照。
 *
 * 调用者传入非空 stats 指针后，本函数在互斥锁保护下复制当前统计结构。
 * 返回的是调用瞬间的一致快照，调用者后续读取不需要继续持有导航互斥锁。
 */
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

uint16_t NavigationTask_CopySmoothPathPoints(NavigationPathPoint_t *points, uint16_t max_points)
{
    uint16_t idx;
    uint16_t copy_len;

    if ((points == NULL) || (max_points == 0U)) {
        return 0U;
    }

    navigation_lock();
    copy_len = g_navigationPublishedPathLen;
    if (copy_len > max_points) {
        copy_len = max_points;
    }

    for (idx = 0U; idx < copy_len; ++idx) {
        points[idx] = g_navigationPublishedPath[idx];
    }
    navigation_unlock();

    return copy_len;
}

/*
 * FreeRTOS 导航任务入口。
 *
 * 任务固定周期运行：先处理串口命令，再执行一次导航更新，最后延时到下个
 * 周期。argument 当前未使用，保留以匹配 FreeRTOS 任务函数签名。
 */
void StartNavigationTask(void *argument)
{
    (void)argument;

    /* 固定周期运行导航状态机，对齐参考代码中的 600ms 导航调度节奏。 */
    for (;;) {
        navigation_process_pending_command();
        (void)NavigationTask_Update();
        osDelay(NAVIGATION_TASK_PERIOD_MS);
    }
}
