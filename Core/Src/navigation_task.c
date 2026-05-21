#include "navigation_task.h"
#include "astar.h"
#include "frontier.h"
#include "localization_task.h"
#include "pid.h"
#include "system.h"
#include <math.h>

typedef enum {
    NAV_IDLE,
    NAV_EXPLORING,
    NAV_GOING_TO_GOAL,
    NAV_RETURNING
} NavState_t;

static NavState_t g_nav_state = NAV_IDLE;
static Waypoint_t g_global_path[MAX_PATH_LENGTH];
static int16_t g_path_size = 0;
static int16_t g_current_waypoint_idx = 0;

void Navigation_UpdateControl(SlamPose2D_t* current_pose)
{
    Waypoint_t target;
    float dx, dy, dist;
    float target_angle;

    if (g_path_size == 0 || g_current_waypoint_idx >= g_path_size) {
        Control_StopCommand();
        return;
    }

    target = g_global_path[g_current_waypoint_idx];

    dx = target.x - current_pose->x_m;
    dy = target.y - current_pose->y_m;
    dist = sqrtf(dx * dx + dy * dy);

    if (dist < 0.10f) {
        g_current_waypoint_idx++;
        return;
    }

    target_angle = atan2f(dy, dx) * 180.0f / PI;
    Control_SetManualCommand(0.15f, target_angle);
}

void StartNavigationTask(void *argument)
{
    SlamPose2D_t curr_pose;
    int16_t goal_grid_x, goal_grid_y;

    (void)argument;

    for (;;) {
        LocalizationTask_GetEstimatedPoseSnapshot(&curr_pose);

        switch (g_nav_state) {
            case NAV_EXPLORING:
                if (g_current_waypoint_idx >= g_path_size) {
                    if (Frontier_FindNearest(curr_pose.x_m, curr_pose.y_m,
                                            &goal_grid_x, &goal_grid_y)) {
                        g_path_size = AStar_Plan(curr_pose.x_m, curr_pose.y_m,
                                                goal_grid_x, goal_grid_y,
                                                g_global_path);
                        g_current_waypoint_idx = 0;
                    }
                }
                break;

            case NAV_GOING_TO_GOAL:
                /* 直接 A* 到 Exit 坐标 */
                break;

            default:
                break;
        }

        Navigation_UpdateControl(&curr_pose);
        osDelay(100);
    }
}