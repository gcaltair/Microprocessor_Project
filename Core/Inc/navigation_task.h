#ifndef NAVIGATION_TASK_H
#define NAVIGATION_TASK_H

#include <stdint.h>

#include "slam_types.h"

typedef enum {
    NAVIGATION_STATE_IDLE = 0,
    NAVIGATION_STATE_ACTIVE = 1,
    NAVIGATION_STATE_GOAL_REACHED = 2,
    NAVIGATION_STATE_FAILED = 3,
    NAVIGATION_STATE_CANCELLED = 4
} NavigationState_t;

typedef struct {
    NavigationState_t state;
    uint32_t plan_count;
    uint32_t completion_count;
    uint32_t failure_count;
    uint16_t path_length;
    uint16_t current_waypoint_index;
    SlamGridCoord_t goal_cell;
    SlamGridCoord_t current_waypoint_cell;
    float last_waypoint_distance_m;
} NavigationTaskStats_t;

void NavigationTask_Reset(void);
uint8_t NavigationTask_StartGoalCell(int16_t goal_x, int16_t goal_y);
void NavigationTask_Cancel(void);
void NavigationTask_Service(void);
void NavigationTask_GetStatsSnapshot(NavigationTaskStats_t *stats);
uint8_t NavigationTask_CopyPath(SlamGridCoord_t *path_buffer,
                                uint16_t max_path_length,
                                uint16_t *path_length);

#endif /* NAVIGATION_TASK_H */
