#include <math.h>
#include <string.h>

#include "../Inc/navigation_task.h"
#include "localization_task.h"
#include "mapping_task.h"
#include "pid.h"
#include "system.h"

#define NAVIGATION_MAX_PATH_WAYPOINTS      64U
#define NAVIGATION_WAYPOINT_TOLERANCE_M    0.10f
#define NAVIGATION_MAX_SEGMENT_DISTANCE_M  0.18f
#define NAVIGATION_MIN_COMMAND_DISTANCE_M  0.03f
#define NAVIGATION_DEG_TO_RAD              0.01745329251994329577f

static NavigationTaskStats_t g_navigationStats;
static SlamGridCoord_t g_navigationPath[NAVIGATION_MAX_PATH_WAYPOINTS];

static void navigation_set_state(NavigationState_t state)
{
    g_navigationStats.state = state;
}

static void navigation_clear_path(void)
{
    (void)memset(g_navigationPath, 0, sizeof(g_navigationPath));
    g_navigationStats.path_length = 0U;
    g_navigationStats.current_waypoint_index = 0U;
    g_navigationStats.current_waypoint_cell.x = -1;
    g_navigationStats.current_waypoint_cell.y = -1;
    g_navigationStats.last_waypoint_distance_m = 0.0f;
}

static void navigation_fail(void)
{
    navigation_clear_path();
    g_navigationStats.failure_count++;
    navigation_set_state(NAVIGATION_STATE_FAILED);
}

void NavigationTask_Reset(void)
{
    (void)memset(&g_navigationStats, 0, sizeof(g_navigationStats));
    g_navigationStats.goal_cell.x = -1;
    g_navigationStats.goal_cell.y = -1;
    g_navigationStats.current_waypoint_cell.x = -1;
    g_navigationStats.current_waypoint_cell.y = -1;
}

uint8_t NavigationTask_StartGoalCell(int16_t goal_x, int16_t goal_y)
{
    SlamPose2D_t current_pose;
    SlamGridCoord_t start_cell;
    SlamGridCoord_t goal_cell;
    uint16_t path_length = 0U;

    goal_cell.x = goal_x;
    goal_cell.y = goal_y;

    LocalizationTask_GetEstimatedPoseSnapshot(&current_pose);
    if (current_pose.timestamp_ms == 0U) {
        navigation_fail();
        return 0U;
    }

    if (MappingTask_WorldToCell(current_pose.x_m, current_pose.y_m, &start_cell) == 0U) {
        navigation_fail();
        return 0U;
    }

    if (MappingTask_PlanPath(&start_cell,
                             &goal_cell,
                             g_navigationPath,
                             NAVIGATION_MAX_PATH_WAYPOINTS,
                             &path_length) == 0U) {
        navigation_fail();
        return 0U;
    }

    g_navigationStats.plan_count++;
    g_navigationStats.goal_cell = goal_cell;
    g_navigationStats.path_length = path_length;
    g_navigationStats.current_waypoint_index = 0U;
    g_navigationStats.last_waypoint_distance_m = 0.0f;

    if (path_length == 0U) {
        g_navigationStats.current_waypoint_cell = goal_cell;
        g_navigationStats.completion_count++;
        navigation_set_state(NAVIGATION_STATE_GOAL_REACHED);
        return 1U;
    }

    g_navigationStats.current_waypoint_cell = g_navigationPath[0];
    navigation_set_state(NAVIGATION_STATE_ACTIVE);
    return 1U;
}

void NavigationTask_Cancel(void)
{
    if (g_navigationStats.state == NAVIGATION_STATE_ACTIVE) {
        Control_StopCommand();
    }

    navigation_clear_path();
    navigation_set_state(NAVIGATION_STATE_CANCELLED);
}

void NavigationTask_Service(void)
{
    SlamPose2D_t current_pose;

    if (g_navigationStats.state != NAVIGATION_STATE_ACTIVE) {
        return;
    }

    if ((g_control_mode == CONTROL_MODE_POSITION) || (g_relative_move_state != RELATIVE_MOVE_IDLE)) {
        return;
    }

    LocalizationTask_GetEstimatedPoseSnapshot(&current_pose);
    if (current_pose.timestamp_ms == 0U) {
        navigation_fail();
        return;
    }

    while (g_navigationStats.current_waypoint_index < g_navigationStats.path_length) {
        SlamWaypoint2D_t waypoint;
        float delta_x_world_m;
        float delta_y_world_m;
        float distance_m;
        float theta_rad;
        float cos_theta;
        float sin_theta;
        float scale = 1.0f;
        float body_x_m;
        float body_y_m;

        g_navigationStats.current_waypoint_cell = g_navigationPath[g_navigationStats.current_waypoint_index];
        if (MappingTask_CellToWorld(&g_navigationStats.current_waypoint_cell, &waypoint) == 0U) {
            navigation_fail();
            return;
        }

        delta_x_world_m = waypoint.x_m - current_pose.x_m;
        delta_y_world_m = waypoint.y_m - current_pose.y_m;
        distance_m = sqrtf(delta_x_world_m * delta_x_world_m + delta_y_world_m * delta_y_world_m);
        g_navigationStats.last_waypoint_distance_m = distance_m;

        if (distance_m <= NAVIGATION_WAYPOINT_TOLERANCE_M) {
            g_navigationStats.current_waypoint_index++;
            continue;
        }

        if (distance_m > NAVIGATION_MAX_SEGMENT_DISTANCE_M) {
            scale = NAVIGATION_MAX_SEGMENT_DISTANCE_M / distance_m;
            delta_x_world_m *= scale;
            delta_y_world_m *= scale;
            distance_m = NAVIGATION_MAX_SEGMENT_DISTANCE_M;
        }

        theta_rad = current_pose.theta_deg * NAVIGATION_DEG_TO_RAD;
        cos_theta = cosf(theta_rad);
        sin_theta = sinf(theta_rad);
        body_x_m = cos_theta * delta_x_world_m + sin_theta * delta_y_world_m;
        body_y_m = -sin_theta * delta_x_world_m + cos_theta * delta_y_world_m;

        if (distance_m < NAVIGATION_MIN_COMMAND_DISTANCE_M) {
            g_navigationStats.current_waypoint_index++;
            continue;
        }

        Start_Relative_Move(body_x_m, body_y_m);
        return;
    }

    g_navigationStats.completion_count++;
    navigation_clear_path();
    g_navigationStats.goal_cell = (SlamGridCoord_t){ .x = -1, .y = -1 };
    navigation_set_state(NAVIGATION_STATE_GOAL_REACHED);
}

void NavigationTask_GetStatsSnapshot(NavigationTaskStats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    *stats = g_navigationStats;
}

uint8_t NavigationTask_CopyPath(SlamGridCoord_t *path_buffer,
                                uint16_t max_path_length,
                                uint16_t *path_length)
{
    uint16_t copy_length;

    if ((path_buffer == NULL) || (path_length == NULL) || (max_path_length == 0U)) {
        return 0U;
    }

    copy_length = g_navigationStats.path_length;
    if (copy_length > max_path_length) {
        return 0U;
    }

    if (copy_length > 0U) {
        (void)memcpy(path_buffer, g_navigationPath, copy_length * sizeof(g_navigationPath[0]));
    }

    *path_length = copy_length;
    return 1U;
}
