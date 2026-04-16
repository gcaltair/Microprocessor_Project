#ifndef MAPPING_TASK_H
#define MAPPING_TASK_H

#include <stdint.h>

#include "occupancy_grid.h"
#include "slam_types.h"

typedef struct {
    uint8_t grid_initialized;
    uint8_t robot_inside_grid;
    uint32_t update_count;
    uint32_t last_scan_sequence;
    uint16_t last_usable_points;
    uint16_t last_endpoints_written;
    uint8_t last_localization_mode;
    uint16_t last_localization_inliers;
    float last_localization_fitness_m;
    SlamGridCoord_t last_robot_cell;
    SlamPose2D_t last_pose;
} MappingTaskStats_t;

void StartMappingTask(void *argument);
void MappingTask_ResetGrid(void);
void MappingTask_GetStatsSnapshot(MappingTaskStats_t *stats);
void MappingTask_GetRenderDimensions(uint8_t downsample, uint16_t *width, uint16_t *height);
uint8_t MappingTask_RenderAsciiRow(uint16_t render_row, uint8_t downsample, char *buffer, uint16_t buffer_size);

#endif /* MAPPING_TASK_H */
