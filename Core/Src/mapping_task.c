#include <math.h>
#include <string.h>

#include "../Inc/mapping_task.h"
#include "../Inc/freertos_app.h"
#include "../Inc/scan_preprocess.h"

#define MAPPING_GRID_WIDTH_CELLS        96U
#define MAPPING_GRID_HEIGHT_CELLS       96U
#define MAPPING_GRID_RESOLUTION_M       0.05f
#define MAPPING_GRID_ORIGIN_X_M         (-0.5f)
#define MAPPING_GRID_ORIGIN_Y_M         (-0.5f)
#define MAPPING_POINT_STRIDE            4U
#define MAPPING_FREE_CELL_THRESHOLD     (-3)
#define MAPPING_OCCUPIED_CELL_THRESHOLD 5
#define DEG_TO_RAD                      0.01745329251994329577f

/* 固件端实际维护的占据栅格地图，后续通过遥测发送给上位机。 */
static OccupancyGrid_t g_mappingGrid;
static MappingTaskStats_t g_mappingStats;

typedef enum {
    MAPPING_CELL_OCCUPIED = 0,
    MAPPING_CELL_FREE = 1,
    MAPPING_CELL_UNKNOWN = 2
} MappingCellState_t;

static uint8_t mapping_task_normalize_downsample(uint8_t downsample)
{
    /* 下采样倍率不能为 0；传入 0 时按不下采样处理。 */
    return (downsample == 0U) ? 1U : downsample;
}

static MappingCellState_t mapping_task_get_cell_state(int16_t cell_x, int16_t cell_y)
{
    int8_t center_value = 0;

    /* 地图外部按占用处理，防止规划或探索逻辑把车开出已知地图范围。 */
    if (OccupancyGrid_IsInside(&g_mappingGrid, cell_x, cell_y) == 0U) {
        return MAPPING_CELL_OCCUPIED;
    }

    /* 读取失败也按占用处理，保证安全优先。 */
    if (OccupancyGrid_GetCell(&g_mappingGrid, cell_x, cell_y, &center_value) == 0U) {
        return MAPPING_CELL_OCCUPIED;
    }

    /* 栅格值越大表示越可能被占用。 */
    if (center_value >= MAPPING_OCCUPIED_CELL_THRESHOLD) {
        return MAPPING_CELL_OCCUPIED;
    }

    /* 栅格值越小表示越确定为空闲。 */
    if (center_value <= MAPPING_FREE_CELL_THRESHOLD) {
        return MAPPING_CELL_FREE;
    }

    return MAPPING_CELL_UNKNOWN;
}

static void mapping_task_ensure_grid_initialized(void)
{
    /* 若地图已经初始化，直接返回，避免重复清空已有建图结果。 */
    if (g_mappingGrid.initialized != 0U) {
        return;
    }

    /* 96x96 栅格以起点附近为中心，分辨率 5 cm，总覆盖约 4.8 m x 4.8 m。 */
    if (OccupancyGrid_Init(&g_mappingGrid,
                           MAPPING_GRID_WIDTH_CELLS,
                           MAPPING_GRID_HEIGHT_CELLS,
                           MAPPING_GRID_RESOLUTION_M,
                           MAPPING_GRID_ORIGIN_X_M,
                           MAPPING_GRID_ORIGIN_Y_M) != 0U) {
        OccupancyGrid_Reset(&g_mappingGrid);
        (void)memset(&g_mappingStats, 0, sizeof(g_mappingStats));
        g_mappingStats.grid_initialized = 1U;
    }
}

static void mapping_task_lock_grid(void)
{
    /* 建图、遥测和路径查询共享同一张地图，因此所有访问都通过互斥锁保护。 */
    if (g_gridMutex != NULL) {
        (void)osMutexAcquire(g_gridMutex, osWaitForever);
    }
}

static void mapping_task_unlock_grid(void)
{
    /* 与 mapping_task_lock_grid 成对使用，释放地图互斥锁。 */
    if (g_gridMutex != NULL) {
        (void)osMutexRelease(g_gridMutex);
    }
}

static void mapping_task_update_stats(const LidarScanMsg_t *scan_msg,
                                      uint8_t robot_inside_grid,
                                      const SlamGridCoord_t *robot_cell,
                                      uint16_t endpoints_written,
                                      MappingSkipReason_t skip_reason)
{
    /* 统计信息用于上位机判断地图是否在更新，以及为什么被跳过。 */
    if (scan_msg == NULL) {
        return;
    }

    /* 把本帧建图输入和处理结果同步到遥测统计结构。 */
    g_mappingStats.grid_initialized = g_mappingGrid.initialized;
    g_mappingStats.robot_inside_grid = robot_inside_grid;
    g_mappingStats.map_update_active = (skip_reason == MAPPING_SKIP_REASON_NONE) ? 1U : 0U;
    g_mappingStats.last_skip_reason = (uint8_t)skip_reason;
    g_mappingStats.last_scan_match_reject_reason = scan_msg->scan_match_reject_reason;
    g_mappingStats.update_count++;
    g_mappingStats.last_scan_sequence = scan_msg->scan_sequence;
    g_mappingStats.last_usable_points = scan_msg->quality.usable_point_count;
    g_mappingStats.last_endpoints_written = endpoints_written;
    g_mappingStats.last_localization_mode = (LocalizationMode_t)scan_msg->localization_mode;
    g_mappingStats.last_localization_inliers = scan_msg->localization_inliers;
    g_mappingStats.last_localization_fitness_m = scan_msg->localization_fitness_m;
    g_mappingStats.last_odom_delta_theta_deg = scan_msg->odom_delta_theta_deg;
    g_mappingStats.last_odom_delta_translation_m = scan_msg->odom_delta_translation_m;
    g_mappingStats.last_scan_match_tested_candidates = scan_msg->scan_match_tested_candidates;
    g_mappingStats.last_scan_match_used_points = scan_msg->scan_match_used_points;
    g_mappingStats.last_scan_match_best_score = scan_msg->scan_match_best_score;
    g_mappingStats.last_scan_match_second_score = scan_msg->scan_match_second_score;
    g_mappingStats.last_scan_match_score_margin = scan_msg->scan_match_score_margin;
    g_mappingStats.last_scan_match_dx_m = scan_msg->scan_match_dx_m;
    g_mappingStats.last_scan_match_dy_m = scan_msg->scan_match_dy_m;
    g_mappingStats.last_scan_match_dtheta_deg = scan_msg->scan_match_dtheta_deg;
    g_mappingStats.last_pose = scan_msg->corrected_pose;

    if (skip_reason == MAPPING_SKIP_REASON_TURNING) {
        /* 记录因为转弯暂停建图的次数。 */
        g_mappingStats.skipped_turning_count++;
    } else if (skip_reason == MAPPING_SKIP_REASON_SETTLE) {
        /* 记录因为转弯后稳定等待而跳过的次数。 */
        g_mappingStats.skipped_settle_count++;
    } else if (skip_reason == MAPPING_SKIP_REASON_QUALITY) {
        /* 记录因为雷达质量或里程计跳变过大而跳过的次数。 */
        g_mappingStats.skipped_quality_count++;
    }

    /* 记录机器人所在栅格；若机器人不在地图内，则写入无效坐标。 */
    if (robot_cell != NULL) {
        g_mappingStats.last_robot_cell = *robot_cell;
    } else {
        g_mappingStats.last_robot_cell.x = -1;
        g_mappingStats.last_robot_cell.y = -1;
    }
}

static void mapping_task_update_grid_from_scan(const LidarScanMsg_t *scan_msg)
{
    SlamGridCoord_t robot_cell;
    const LidarScanBuffer_t *scan_buffer;
    uint16_t idx;
    uint16_t endpoints_written = 0U;
    uint8_t robot_inside_grid;

    /* 检查消息、扫描缓冲区索引和地图初始化状态。 */
    if ((scan_msg == NULL) ||
        (scan_msg->scan_index >= LIDAR_SCAN_BUFFER_COUNT) ||
        (g_mappingGrid.initialized == 0U)) {
        return;
    }

    /*
     * 一整帧扫描更新期间保持地图锁，避免遥测或路径检查读到只更新了一半的地图。
     */
    mapping_task_lock_grid();
    robot_inside_grid = OccupancyGrid_WorldToCell(&g_mappingGrid,
                                                  scan_msg->corrected_pose.x_m,
                                                  scan_msg->corrected_pose.y_m,
                                                  &robot_cell);
    if (robot_inside_grid == 0U) {
        /* 如果里程计位姿已经离开地图范围，则不写射线，只更新调试状态。 */
        mapping_task_update_stats(scan_msg,
                                  0U,
                                  NULL,
                                  0U,
                                  (MappingSkipReason_t)scan_msg->map_skip_reason);
        mapping_task_unlock_grid();
        return;
    }

    if (scan_msg->map_update_allowed == 0U) {
        /* localization 已判定本帧不适合建图，但仍要让上位机看到最新跳过原因。 */
        mapping_task_update_stats(scan_msg,
                                  1U,
                                  &robot_cell,
                                  0U,
                                  (MappingSkipReason_t)scan_msg->map_skip_reason);
        mapping_task_unlock_grid();
        return;
    }

    scan_buffer = &g_lidarScanBuf[scan_msg->scan_index];

    /*
     * 每条可用雷达束会先清空机器人到端点之间的栅格，再把端点标记为占用。
     * MAPPING_POINT_STRIDE 用于抽样处理，降低 STM32 的建图计算负载。
     */
    for (idx = 0U; idx < scan_buffer->point_count; idx = (uint16_t)(idx + MAPPING_POINT_STRIDE)) {
        SlamGridCoord_t endpoint_cell;
        float beam_angle_rad;
        float world_x_m;
        float world_y_m;
        float distance_m;

        if (ScanPreprocess_IsPointUsable(&scan_buffer->points[idx]) == 0U) {
            /* 跳过距离、质量或角度不可靠的雷达点。 */
            continue;
        }

        /* 使用纯里程计位姿，把雷达极坐标点转换到世界坐标系。 */
        distance_m = scan_buffer->points[idx].distance_mm * 0.001f;
        beam_angle_rad = ScanPreprocess_BeamWorldAngleDeg(scan_msg->corrected_pose.theta_deg,
                                                          scan_buffer->points[idx].angle_deg) *
                         DEG_TO_RAD;
        world_x_m = scan_msg->corrected_pose.x_m + distance_m * cosf(beam_angle_rad);
        world_y_m = scan_msg->corrected_pose.y_m + distance_m * sinf(beam_angle_rad);

        if (OccupancyGrid_WorldToCell(&g_mappingGrid, world_x_m, world_y_m, &endpoint_cell) == 0U) {
            /* 地图外的回波直接忽略，避免把边界裁剪成错误的墙。 */
            continue;
        }

        /* 对当前雷达束做射线更新，并统计成功写入的端点数量。 */
        OccupancyGrid_TraceRay(&g_mappingGrid, &robot_cell, &endpoint_cell, 1U);
        endpoints_written++;
    }

    mapping_task_update_stats(scan_msg,
                              1U,
                              &robot_cell,
                              endpoints_written,
                              MAPPING_SKIP_REASON_NONE);
    mapping_task_unlock_grid();
}

void StartMappingTask(void *argument)
{
    LidarScanMsg_t scan_msg;

    (void)argument;
    (void)memset(&scan_msg, 0, sizeof(scan_msg));
    /* 建图线程启动时确保占据栅格已经可用。 */
    mapping_task_ensure_grid_initialized();

    for (;;) {
        uint8_t free_idx;

        /* 等待 localization 线程转发的、已经完成建图门控判断的雷达帧。 */
        if (osMessageQueueGet(g_localizedScanQueue, &scan_msg, NULL, osWaitForever) != osOK) {
            continue;
        }

        /* 地图可能被复位或尚未初始化，因此每帧处理前都做一次轻量检查。 */
        mapping_task_ensure_grid_initialized();
        mapping_task_update_grid_from_scan(&scan_msg);

        free_idx = scan_msg.scan_index;
        g_lidarScanBuf[free_idx].point_count = 0U;
        /* 建图写入完成后再把扫描缓冲区归还给雷达解析线程。 */
        (void)osMessageQueuePut(g_lidarFreeQueue, &free_idx, 0U, osWaitForever);

        osDelay(1U);
    }
}

void MappingTask_ResetGrid(void)
{
    /* 外部请求清空地图时，先保证地图对象存在。 */
    mapping_task_ensure_grid_initialized();
    if (g_mappingGrid.initialized == 0U) {
        return;
    }

    /* 清空栅格和统计信息，并保留“地图已初始化”的状态。 */
    mapping_task_lock_grid();
    OccupancyGrid_Reset(&g_mappingGrid);
    (void)memset(&g_mappingStats, 0, sizeof(g_mappingStats));
    g_mappingStats.grid_initialized = 1U;
    mapping_task_unlock_grid();
}

void MappingTask_GetStatsSnapshot(MappingTaskStats_t *stats)
{
    /* 复制建图统计快照，供遥测线程无阻塞地格式化输出。 */
    if (stats == NULL) {
        return;
    }

    mapping_task_lock_grid();
    *stats = g_mappingStats;
    mapping_task_unlock_grid();
}

void MappingTask_GetRenderDimensions(uint8_t downsample, uint16_t *width, uint16_t *height)
{
    /* 根据下采样倍率计算 ASCII 渲染后的宽高。 */
    downsample = mapping_task_normalize_downsample(downsample);

    /* 调用方可以只查询宽或只查询高，因此两个输出指针都允许为空。 */
    if (width != NULL) {
        *width = (uint16_t)((MAPPING_GRID_WIDTH_CELLS + downsample - 1U) / downsample);
    }

    if (height != NULL) {
        *height = (uint16_t)((MAPPING_GRID_HEIGHT_CELLS + downsample - 1U) / downsample);
    }
}

uint8_t MappingTask_GetGridMeta(MappingGridMeta_t *meta)
{
    /* 读取地图尺寸、分辨率和世界坐标原点，供上位机还原栅格坐标。 */
    if (meta == NULL) {
        return 0U;
    }

    mapping_task_lock_grid();
    /* 地图未初始化时无法提供有效元数据。 */
    if (g_mappingGrid.initialized == 0U) {
        mapping_task_unlock_grid();
        return 0U;
    }

    meta->width_cells = g_mappingGrid.spec.width_cells;
    meta->height_cells = g_mappingGrid.spec.height_cells;
    meta->resolution_m_per_cell = g_mappingGrid.spec.resolution_m_per_cell;
    meta->origin_x_m = g_mappingGrid.spec.origin_x_m;
    meta->origin_y_m = g_mappingGrid.spec.origin_y_m;
    mapping_task_unlock_grid();
    return 1U;
}

uint8_t MappingTask_CopyGridCells(int8_t *cells_buffer, uint16_t buffer_len)
{
    /* 拷贝整张地图给遥测或调试接口，调用方必须提供足够大的缓冲区。 */
    if ((cells_buffer == NULL) || (buffer_len < OGM_MAX_CELL_COUNT)) {
        return 0U;
    }

    mapping_task_lock_grid();
    /* 地图未初始化时不返回栅格数据。 */
    if (g_mappingGrid.initialized == 0U) {
        mapping_task_unlock_grid();
        return 0U;
    }

    (void)memcpy(cells_buffer, g_mappingGrid.cells, OGM_MAX_CELL_COUNT);
    mapping_task_unlock_grid();
    return 1U;
}

uint8_t MappingTask_CopyGridRows(uint16_t row_offset,
                                 uint16_t row_count,
                                 int8_t *cells_buffer,
                                 uint16_t buffer_len)
{
    uint16_t width_cells;
    uint16_t cell_count;

    /* 按行拷贝地图片段，用于分包发送大地图数据。 */
    if ((cells_buffer == NULL) || (row_count == 0U)) {
        return 0U;
    }

    mapping_task_lock_grid();
    /* 地图未初始化时不返回任何行数据。 */
    if (g_mappingGrid.initialized == 0U) {
        mapping_task_unlock_grid();
        return 0U;
    }

    width_cells = g_mappingGrid.spec.width_cells;
    /* 检查请求的行范围是否完全落在地图内部。 */
    if ((row_offset >= g_mappingGrid.spec.height_cells) ||
        ((uint32_t)row_offset + row_count > g_mappingGrid.spec.height_cells)) {
        mapping_task_unlock_grid();
        return 0U;
    }

    cell_count = (uint16_t)(row_count * width_cells);
    /* 调用方缓冲区不足时直接失败，避免越界写入。 */
    if (buffer_len < cell_count) {
        mapping_task_unlock_grid();
        return 0U;
    }

    (void)memcpy(cells_buffer,
                 &g_mappingGrid.cells[row_offset * width_cells],
                 cell_count);
    mapping_task_unlock_grid();
    return 1U;
}

uint8_t MappingTask_BeginGridRead(void)
{
    mapping_task_lock_grid();
    if (g_mappingGrid.initialized == 0U) {
        mapping_task_unlock_grid();
        return 0U;
    }

    return 1U;
}

void MappingTask_EndGridRead(void)
{
    mapping_task_unlock_grid();
}

uint8_t MappingTask_ReadCellDuringGridRead(int16_t cell_x, int16_t cell_y, int8_t *value)
{
    return OccupancyGrid_GetCell(&g_mappingGrid, cell_x, cell_y, value);
}

uint8_t MappingTask_RenderAsciiRow(uint16_t render_row, uint8_t downsample, char *buffer, uint16_t buffer_size)
{
    uint16_t render_width;
    uint16_t render_height;
    uint16_t render_col;
    uint16_t block_y_start;
    uint16_t block_y_end;

    /* 将占据栅格的一行渲染成 ASCII，主要用于串口调试或轻量遥测。 */
    if ((buffer == NULL) || (buffer_size == 0U)) {
        return 0U;
    }

    MappingTask_GetRenderDimensions(downsample, &render_width, &render_height);
    downsample = mapping_task_normalize_downsample(downsample);

    /* 输出缓冲区需要容纳一整行字符和结尾的 '\0'。 */
    if ((render_row >= render_height) || (buffer_size <= render_width)) {
        return 0U;
    }

    /* 渲染时第 0 行显示地图顶部，而栅格内部 y=0 表示底部。 */
    block_y_start = (uint16_t)(MAPPING_GRID_HEIGHT_CELLS - 1U - (render_row * downsample));
    block_y_end = (block_y_start >= (downsample - 1U)) ? (uint16_t)(block_y_start - (downsample - 1U)) : 0U;

    mapping_task_lock_grid();
    for (render_col = 0U; render_col < render_width; ++render_col) {
        uint16_t block_x_start = (uint16_t)(render_col * downsample);
        uint16_t block_x_end = block_x_start + downsample;
        uint16_t cell_y;
        uint16_t cell_x;
        uint8_t has_free = 0U;
        char symbol = ' ';

        /* 一个 ASCII 字符代表 downsample x downsample 的栅格块。 */
        if (block_x_end > MAPPING_GRID_WIDTH_CELLS) {
            block_x_end = MAPPING_GRID_WIDTH_CELLS;
        }

        for (cell_y = block_y_start + 1U; cell_y > block_y_end; --cell_y) {
            for (cell_x = block_x_start; cell_x < block_x_end; ++cell_x) {
                int8_t cell_value;

                /* 机器人所在块优先显示为 R。 */
                if ((g_mappingStats.robot_inside_grid != 0U) &&
                    (g_mappingStats.last_robot_cell.x == (int16_t)cell_x) &&
                    (g_mappingStats.last_robot_cell.y == (int16_t)(cell_y - 1U))) {
                    symbol = 'R';
                    break;
                }

                if (OccupancyGrid_GetCell(&g_mappingGrid, cell_x, (int32_t)(cell_y - 1U), &cell_value) == 0U) {
                    continue;
                }

                /* 占用块优先显示为 #。 */
                if (cell_value >= 10) {
                    symbol = '#';
                    break;
                }

                /* 只要块中存在明确空闲栅格，后续可显示为 '.'。 */
                if (cell_value <= -10) {
                    has_free = 1U;
                }
            }

            if ((symbol == 'R') || (symbol == '#')) {
                break;
            }
        }

        if ((symbol == ' ') && (has_free != 0U)) {
            /* 没有机器人和障碍时，用 '.' 表示该块包含已知空闲区域。 */
            symbol = '.';
        }

        buffer[render_col] = symbol;
    }
    mapping_task_unlock_grid();

    buffer[render_width] = '\0';
    return 1U;
}

uint8_t MappingTask_WorldToCell(float x_m, float y_m, SlamGridCoord_t *cell)
{
    uint8_t result;

    /* 把世界坐标转换为地图栅格坐标，供规划和调试接口使用。 */
    if (cell == NULL) {
        return 0U;
    }

    mapping_task_lock_grid();
    result = OccupancyGrid_WorldToCell(&g_mappingGrid, x_m, y_m, cell);
    mapping_task_unlock_grid();
    return result;
}

uint8_t MappingTask_IsCellInside(int16_t cell_x, int16_t cell_y)
{
    uint8_t result;

    /* 查询某个栅格坐标是否位于当前地图范围内。 */
    mapping_task_lock_grid();
    result = OccupancyGrid_IsInside(&g_mappingGrid, cell_x, cell_y);
    mapping_task_unlock_grid();
    return result;
}

uint8_t MappingTask_IsCellKnownFree(int16_t cell_x, int16_t cell_y)
{
    uint8_t result;

    /* 查询某个栅格是否已经被建图确认为可通行。 */
    mapping_task_lock_grid();
    result = (mapping_task_get_cell_state(cell_x, cell_y) == MAPPING_CELL_FREE) ? 1U : 0U;
    mapping_task_unlock_grid();
    return result;
}

uint8_t MappingTask_IsCellUnknown(int16_t cell_x, int16_t cell_y)
{
    uint8_t result;

    /* 查询某个栅格是否仍处于未知状态。 */
    mapping_task_lock_grid();
    result = (mapping_task_get_cell_state(cell_x, cell_y) == MAPPING_CELL_UNKNOWN) ? 1U : 0U;
    mapping_task_unlock_grid();
    return result;
}
