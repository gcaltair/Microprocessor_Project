#include <math.h>
#include <string.h>

#include "../Inc/mapping_task.h"
#include "../Inc/freertos_app.h"
#include "../Inc/scan_preprocess.h"

#define MAPPING_GRID_WIDTH_CELLS        96U
#define MAPPING_GRID_HEIGHT_CELLS       96U
#define MAPPING_GRID_RESOLUTION_M       0.05f
#define MAPPING_GRID_ORIGIN_X_M         (-2.4f)
#define MAPPING_GRID_ORIGIN_Y_M         (-2.4f)
#define MAPPING_POINT_STRIDE            4U
#define MAPPING_FREE_CELL_THRESHOLD     (-3)
#define MAPPING_OCCUPIED_CELL_THRESHOLD 10
#define MAPPING_PATH_INFLATION_RADIUS   1
#define MAPPING_PATH_STEP_REPEAT_LIMIT  3
#define DEG_TO_RAD                      0.01745329251994329577f

static OccupancyGrid_t g_mappingGrid;
static MappingTaskStats_t g_mappingStats;
static uint16_t g_mappingPlanQueue[OGM_MAX_CELL_COUNT];
static int8_t g_mappingParentDir[OGM_MAX_CELL_COUNT];

static uint8_t mapping_task_normalize_downsample(uint8_t downsample)
{
    return (downsample == 0U) ? 1U : downsample;
}

static uint16_t mapping_task_linear_index(int16_t cell_x, int16_t cell_y)
{
    return (uint16_t)((uint16_t)cell_y * g_mappingGrid.spec.width_cells + (uint16_t)cell_x);
}

static void mapping_task_index_to_cell(uint16_t linear_index, SlamGridCoord_t *cell)
{
    if (cell == NULL) {
        return;
    }

    cell->x = (int16_t)(linear_index % g_mappingGrid.spec.width_cells);
    cell->y = (int16_t)(linear_index / g_mappingGrid.spec.width_cells);
}

static uint8_t mapping_task_cell_is_navigable(int16_t cell_x, int16_t cell_y)
{
    int8_t center_value = 0;
    int16_t offset_y;

    if (OccupancyGrid_IsInside(&g_mappingGrid, cell_x, cell_y) == 0U) {
        return 0U;
    }

    if (OccupancyGrid_GetCell(&g_mappingGrid, cell_x, cell_y, &center_value) == 0U) {
        return 0U;
    }

    if (center_value > MAPPING_FREE_CELL_THRESHOLD) {
        return 0U;
    }

    for (offset_y = -MAPPING_PATH_INFLATION_RADIUS; offset_y <= MAPPING_PATH_INFLATION_RADIUS; ++offset_y) {
        int16_t offset_x;

        for (offset_x = -MAPPING_PATH_INFLATION_RADIUS; offset_x <= MAPPING_PATH_INFLATION_RADIUS; ++offset_x) {
            int8_t neighbor_value = 0;

            if (OccupancyGrid_IsInside(&g_mappingGrid, cell_x + offset_x, cell_y + offset_y) == 0U) {
                return 0U;
            }

            if (OccupancyGrid_GetCell(&g_mappingGrid,
                                      cell_x + offset_x,
                                      cell_y + offset_y,
                                      &neighbor_value) == 0U) {
                return 0U;
            }

            if (neighbor_value >= MAPPING_OCCUPIED_CELL_THRESHOLD) {
                return 0U;
            }
        }
    }

    return 1U;
}

static uint8_t mapping_task_reconstruct_path(uint16_t start_index,
                                             uint16_t goal_index,
                                             SlamGridCoord_t *path,
                                             uint16_t max_path_length,
                                             uint16_t *path_length)
{
    static const int8_t s_parent_dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    static const int8_t s_parent_dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    SlamGridCoord_t current_cell;
    SlamGridCoord_t previous_cell;
    uint16_t current_index = goal_index;
    uint16_t count = 0U;
    int8_t previous_dir = -128;
    uint8_t repeated_steps = 0U;

    if ((path == NULL) || (path_length == NULL) || (max_path_length == 0U)) {
        return 0U;
    }

    while (current_index != start_index) {
        int8_t parent_dir = g_mappingParentDir[current_index];

        if (parent_dir < 0) {
            break;
        }

        mapping_task_index_to_cell(current_index, &current_cell);
        if ((count == 0U) ||
            (parent_dir != previous_dir) ||
            (repeated_steps >= MAPPING_PATH_STEP_REPEAT_LIMIT)) {
            if (count >= max_path_length) {
                return 0U;
            }

            path[count++] = current_cell;
            repeated_steps = 0U;
        }

        previous_dir = parent_dir;
        repeated_steps++;
        previous_cell = current_cell;
        previous_cell.x = (int16_t)(previous_cell.x + s_parent_dx[parent_dir]);
        previous_cell.y = (int16_t)(previous_cell.y + s_parent_dy[parent_dir]);
        current_index = mapping_task_linear_index(previous_cell.x, previous_cell.y);
    }

    if ((count == 0U) && (start_index == goal_index)) {
        *path_length = 0U;
        return 1U;
    }

    if (count == 0U) {
        return 0U;
    }

    for (uint16_t left = 0U, right = (uint16_t)(count - 1U); left < right; ++left, --right) {
        SlamGridCoord_t temp = path[left];
        path[left] = path[right];
        path[right] = temp;
    }

    *path_length = count;
    return 1U;
}

static void mapping_task_ensure_grid_initialized(void)
{
    if (g_mappingGrid.initialized != 0U) {
        return;
    }

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
    if (g_gridMutex != NULL) {
        (void)osMutexAcquire(g_gridMutex, osWaitForever);
    }
}

static void mapping_task_unlock_grid(void)
{
    if (g_gridMutex != NULL) {
        (void)osMutexRelease(g_gridMutex);
    }
}

static void mapping_task_update_stats(const LidarScanMsg_t *scan_msg,
                                      uint8_t robot_inside_grid,
                                      const SlamGridCoord_t *robot_cell,
                                      uint16_t endpoints_written)
{
    if (scan_msg == NULL) {
        return;
    }

    g_mappingStats.grid_initialized = g_mappingGrid.initialized;
    g_mappingStats.robot_inside_grid = robot_inside_grid;
    g_mappingStats.update_count++;
    g_mappingStats.last_scan_sequence = scan_msg->scan_sequence;
    g_mappingStats.last_usable_points = scan_msg->quality.usable_point_count;
    g_mappingStats.last_endpoints_written = endpoints_written;
    g_mappingStats.last_localization_mode = (LocalizationMode_t)scan_msg->localization_mode;
    g_mappingStats.last_localization_inliers = scan_msg->localization_inliers;
    g_mappingStats.last_localization_fitness_m = scan_msg->localization_fitness_m;
    g_mappingStats.last_pose = scan_msg->corrected_pose;

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

    if ((scan_msg == NULL) ||
        (scan_msg->scan_index >= LIDAR_SCAN_BUFFER_COUNT) ||
        (g_mappingGrid.initialized == 0U)) {
        return;
    }

    mapping_task_lock_grid();
    robot_inside_grid = OccupancyGrid_WorldToCell(&g_mappingGrid,
                                                  scan_msg->corrected_pose.x_m,
                                                  scan_msg->corrected_pose.y_m,
                                                  &robot_cell);
    if (robot_inside_grid == 0U) {
        mapping_task_update_stats(scan_msg, 0U, NULL, 0U);
        mapping_task_unlock_grid();
        return;
    }

    scan_buffer = &g_lidarScanBuf[scan_msg->scan_index];

    for (idx = 0U; idx < scan_buffer->point_count; idx = (uint16_t)(idx + MAPPING_POINT_STRIDE)) {
        SlamGridCoord_t endpoint_cell;
        float beam_angle_rad;
        float world_x_m;
        float world_y_m;
        float distance_m;

        if (ScanPreprocess_IsPointUsable(&scan_buffer->points[idx]) == 0U) {
            continue;
        }

        distance_m = scan_buffer->points[idx].distance_mm * 0.001f;
        beam_angle_rad = (scan_msg->corrected_pose.theta_deg + scan_buffer->points[idx].angle_deg) * DEG_TO_RAD;
        world_x_m = scan_msg->corrected_pose.x_m + distance_m * cosf(beam_angle_rad);
        world_y_m = scan_msg->corrected_pose.y_m + distance_m * sinf(beam_angle_rad);

        if (OccupancyGrid_WorldToCell(&g_mappingGrid, world_x_m, world_y_m, &endpoint_cell) == 0U) {
            continue;
        }

        OccupancyGrid_TraceRay(&g_mappingGrid, &robot_cell, &endpoint_cell, 1U);
        endpoints_written++;
    }

    mapping_task_update_stats(scan_msg, 1U, &robot_cell, endpoints_written);
    mapping_task_unlock_grid();
}

void StartMappingTask(void *argument)
{
    LidarScanMsg_t scan_msg;

    (void)argument;
    (void)memset(&scan_msg, 0, sizeof(scan_msg));
    mapping_task_ensure_grid_initialized();

    for (;;) {
        if (osMessageQueueGet(g_localizedScanQueue, &scan_msg, NULL, osWaitForever) != osOK) {
            continue;
        }

        mapping_task_ensure_grid_initialized();
        mapping_task_update_grid_from_scan(&scan_msg);

        if ((Freertos_GetLidarBinaryTxEnabled() == 0U) &&
            (Freertos_GetTelemetryScanStreamingEnabled() == 0U)) {
            uint8_t free_idx = scan_msg.scan_index;
            g_lidarScanBuf[free_idx].point_count = 0U;
            (void)osMessageQueuePut(g_lidarFreeQueue, &free_idx, 0U, osWaitForever);
        } else if (g_lidarTxQueue != NULL) {
            (void)osMessageQueuePut(g_lidarTxQueue, &scan_msg, 0U, osWaitForever);
        } else {
            uint8_t free_idx = scan_msg.scan_index;
            g_lidarScanBuf[free_idx].point_count = 0U;
            (void)osMessageQueuePut(g_lidarFreeQueue, &free_idx, 0U, osWaitForever);
        }

        osDelay(1U);
    }
}

void MappingTask_ResetGrid(void)
{
    mapping_task_ensure_grid_initialized();
    if (g_mappingGrid.initialized == 0U) {
        return;
    }

    mapping_task_lock_grid();
    OccupancyGrid_Reset(&g_mappingGrid);
    (void)memset(&g_mappingStats, 0, sizeof(g_mappingStats));
    g_mappingStats.grid_initialized = 1U;
    mapping_task_unlock_grid();
}

void MappingTask_GetStatsSnapshot(MappingTaskStats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    mapping_task_lock_grid();
    *stats = g_mappingStats;
    mapping_task_unlock_grid();
}

void MappingTask_GetRenderDimensions(uint8_t downsample, uint16_t *width, uint16_t *height)
{
    downsample = mapping_task_normalize_downsample(downsample);

    if (width != NULL) {
        *width = (uint16_t)((MAPPING_GRID_WIDTH_CELLS + downsample - 1U) / downsample);
    }

    if (height != NULL) {
        *height = (uint16_t)((MAPPING_GRID_HEIGHT_CELLS + downsample - 1U) / downsample);
    }
}

uint8_t MappingTask_GetGridMeta(MappingGridMeta_t *meta)
{
    if (meta == NULL) {
        return 0U;
    }

    mapping_task_lock_grid();
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
    if ((cells_buffer == NULL) || (buffer_len < OGM_MAX_CELL_COUNT)) {
        return 0U;
    }

    mapping_task_lock_grid();
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

    if ((cells_buffer == NULL) || (row_count == 0U)) {
        return 0U;
    }

    mapping_task_lock_grid();
    if (g_mappingGrid.initialized == 0U) {
        mapping_task_unlock_grid();
        return 0U;
    }

    width_cells = g_mappingGrid.spec.width_cells;
    if ((row_offset >= g_mappingGrid.spec.height_cells) ||
        ((uint32_t)row_offset + row_count > g_mappingGrid.spec.height_cells)) {
        mapping_task_unlock_grid();
        return 0U;
    }

    cell_count = (uint16_t)(row_count * width_cells);
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

uint8_t MappingTask_RenderAsciiRow(uint16_t render_row, uint8_t downsample, char *buffer, uint16_t buffer_size)
{
    uint16_t render_width;
    uint16_t render_height;
    uint16_t render_col;
    uint16_t block_y_start;
    uint16_t block_y_end;

    if ((buffer == NULL) || (buffer_size == 0U)) {
        return 0U;
    }

    MappingTask_GetRenderDimensions(downsample, &render_width, &render_height);
    downsample = mapping_task_normalize_downsample(downsample);

    if ((render_row >= render_height) || (buffer_size <= render_width)) {
        return 0U;
    }

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

        if (block_x_end > MAPPING_GRID_WIDTH_CELLS) {
            block_x_end = MAPPING_GRID_WIDTH_CELLS;
        }

        for (cell_y = block_y_start + 1U; cell_y > block_y_end; --cell_y) {
            for (cell_x = block_x_start; cell_x < block_x_end; ++cell_x) {
                int8_t cell_value;

                if ((g_mappingStats.robot_inside_grid != 0U) &&
                    (g_mappingStats.last_robot_cell.x == (int16_t)cell_x) &&
                    (g_mappingStats.last_robot_cell.y == (int16_t)(cell_y - 1U))) {
                    symbol = 'R';
                    break;
                }

                if (OccupancyGrid_GetCell(&g_mappingGrid, cell_x, (int32_t)(cell_y - 1U), &cell_value) == 0U) {
                    continue;
                }

                if (cell_value >= 10) {
                    symbol = '#';
                    break;
                }

                if (cell_value <= -10) {
                    has_free = 1U;
                }
            }

            if ((symbol == 'R') || (symbol == '#')) {
                break;
            }
        }

        if ((symbol == ' ') && (has_free != 0U)) {
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

    if (cell == NULL) {
        return 0U;
    }

    mapping_task_lock_grid();
    result = OccupancyGrid_WorldToCell(&g_mappingGrid, x_m, y_m, cell);
    mapping_task_unlock_grid();
    return result;
}

uint8_t MappingTask_CellToWorld(const SlamGridCoord_t *cell, SlamWaypoint2D_t *waypoint)
{
    if ((cell == NULL) || (waypoint == NULL)) {
        return 0U;
    }

    mapping_task_lock_grid();
    if (OccupancyGrid_IsInside(&g_mappingGrid, cell->x, cell->y) == 0U) {
        mapping_task_unlock_grid();
        return 0U;
    }

    waypoint->x_m = g_mappingGrid.spec.origin_x_m +
                    ((float)cell->x + 0.5f) * g_mappingGrid.spec.resolution_m_per_cell;
    waypoint->y_m = g_mappingGrid.spec.origin_y_m +
                    ((float)cell->y + 0.5f) * g_mappingGrid.spec.resolution_m_per_cell;
    mapping_task_unlock_grid();
    return 1U;
}

uint8_t MappingTask_PlanPath(const SlamGridCoord_t *start,
                             const SlamGridCoord_t *goal,
                             SlamGridCoord_t *path,
                             uint16_t max_path_length,
                             uint16_t *path_length)
{
    static const int8_t s_neighbor_dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    static const int8_t s_neighbor_dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};
    static const int8_t s_reverse_dir[8] = {1, 0, 3, 2, 7, 6, 5, 4};
    uint16_t queue_head = 0U;
    uint16_t queue_tail = 0U;
    uint16_t start_index;
    uint16_t goal_index;
    uint8_t found = 0U;

    if ((start == NULL) || (goal == NULL) || (path == NULL) || (path_length == NULL) || (max_path_length == 0U)) {
        return 0U;
    }

    mapping_task_lock_grid();

    if ((OccupancyGrid_IsInside(&g_mappingGrid, start->x, start->y) == 0U) ||
        (OccupancyGrid_IsInside(&g_mappingGrid, goal->x, goal->y) == 0U) ||
        (mapping_task_cell_is_navigable(start->x, start->y) == 0U) ||
        (mapping_task_cell_is_navigable(goal->x, goal->y) == 0U)) {
        mapping_task_unlock_grid();
        return 0U;
    }

    (void)memset(g_mappingParentDir, -128, sizeof(g_mappingParentDir));

    start_index = mapping_task_linear_index(start->x, start->y);
    goal_index = mapping_task_linear_index(goal->x, goal->y);
    g_mappingParentDir[start_index] = -1;
    g_mappingPlanQueue[queue_tail++] = start_index;

    while (queue_head < queue_tail) {
        uint16_t current_index = g_mappingPlanQueue[queue_head++];
        SlamGridCoord_t current_cell;

        if (current_index == goal_index) {
            found = 1U;
            break;
        }

        mapping_task_index_to_cell(current_index, &current_cell);
        for (uint8_t neighbor_idx = 0U; neighbor_idx < 8U; ++neighbor_idx) {
            int16_t next_x = (int16_t)(current_cell.x + s_neighbor_dx[neighbor_idx]);
            int16_t next_y = (int16_t)(current_cell.y + s_neighbor_dy[neighbor_idx]);
            uint16_t next_index;

            if (mapping_task_cell_is_navigable(next_x, next_y) == 0U) {
                continue;
            }

            next_index = mapping_task_linear_index(next_x, next_y);
            if (g_mappingParentDir[next_index] != -128) {
                continue;
            }

            g_mappingParentDir[next_index] = s_reverse_dir[neighbor_idx];
            if (queue_tail >= OGM_MAX_CELL_COUNT) {
                break;
            }
            g_mappingPlanQueue[queue_tail++] = next_index;
        }
    }

    if (found == 0U) {
        mapping_task_unlock_grid();
        return 0U;
    }

    found = mapping_task_reconstruct_path(start_index, goal_index, path, max_path_length, path_length);
    mapping_task_unlock_grid();
    return found;
}
