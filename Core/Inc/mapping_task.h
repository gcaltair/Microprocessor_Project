#ifndef MAPPING_TASK_H
#define MAPPING_TASK_H

#include <stdint.h>

#include "occupancy_grid.h"
#include "slam_types.h"

typedef enum {
    MAPPING_SKIP_REASON_NONE = 0,
    MAPPING_SKIP_REASON_TURNING = 1,
    MAPPING_SKIP_REASON_SETTLE = 2,
    MAPPING_SKIP_REASON_QUALITY = 3,
    MAPPING_SKIP_REASON_RECOVERY = 4
} MappingSkipReason_t;

typedef struct {
    uint8_t grid_initialized;
    uint8_t robot_inside_grid;
    uint8_t map_update_active;
    uint8_t last_skip_reason;
    uint32_t update_count;
    uint32_t last_scan_sequence;
    uint16_t last_usable_points;
    uint16_t last_endpoints_written;
    uint8_t last_localization_mode;
    uint16_t last_localization_inliers;
    uint32_t skipped_turning_count;
    uint32_t skipped_settle_count;
    uint32_t skipped_quality_count;
    uint32_t skipped_recovery_count;
    float last_localization_fitness_m;
    float last_odom_delta_theta_deg;
    float last_odom_delta_translation_m;
    SlamGridCoord_t last_robot_cell;
    SlamPose2D_t last_pose;
} MappingTaskStats_t;

typedef struct {
    uint16_t width_cells;
    uint16_t height_cells;
    float resolution_m_per_cell;
    float origin_x_m;
    float origin_y_m;
} MappingGridMeta_t;

void StartMappingTask(void *argument);
void MappingTask_ResetGrid(void);
void MappingTask_GetStatsSnapshot(MappingTaskStats_t *stats);
void MappingTask_GetRenderDimensions(uint8_t downsample, uint16_t *width, uint16_t *height);
uint8_t MappingTask_RenderAsciiRow(uint16_t render_row, uint8_t downsample, char *buffer, uint16_t buffer_size);
uint8_t MappingTask_GetGridMeta(MappingGridMeta_t *meta);
uint8_t MappingTask_CopyGridCells(int8_t *cells_buffer, uint16_t buffer_len);
uint8_t MappingTask_CopyGridRows(uint16_t row_offset,
                                 uint16_t row_count,
                                 int8_t *cells_buffer,
                                 uint16_t buffer_len);
uint8_t MappingTask_WorldToCell(float x_m, float y_m, SlamGridCoord_t *cell);
uint8_t MappingTask_IsCellInside(int16_t cell_x, int16_t cell_y);
uint8_t MappingTask_IsCellKnownFree(int16_t cell_x, int16_t cell_y);
uint8_t MappingTask_IsCellUnknown(int16_t cell_x, int16_t cell_y);
/* ---- Navigation interface ---- */
OccupancyGrid_t *MappingTask_GetGrid(void);

#endif /* MAPPING_TASK_H */
