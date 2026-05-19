#ifndef NAVIGATION_TASK_H
#define NAVIGATION_TASK_H

#include <stdint.h>

#include "slam_types.h"

typedef enum {
    NAVIGATION_STATUS_IDLE = 0,
    NAVIGATION_STATUS_OK = 1,
    NAVIGATION_STATUS_REACHED = 2,
    NAVIGATION_STATUS_FAILED = 3,
    NAVIGATION_STATUS_BUSY = 4
} NavigationStatus_t;

typedef struct {
    uint8_t goal_valid;
    uint8_t target_valid;
    NavigationStatus_t last_status;
    uint32_t update_count;
    uint32_t fail_count;
    uint16_t raw_path_len;
    uint16_t smooth_path_len;
    float distance_to_goal_m;
    SlamPose2D_t current_pose;
    SlamPose2D_t goal_pose;
    SlamPose2D_t target_pose;
} NavigationTaskStats_t;

/* FreeRTOS 导航线程入口：周期性规划路径，并在控制空闲时下发下一段局部目标。 */
void StartNavigationTask(void *argument);
/* 设置导航终点，坐标单位为米，坐标系与当前占据栅格地图一致。 */
void NavigationTask_SetGoal(float goal_x_m, float goal_y_m);
/* 清除当前导航目标，同时让导航线程回到空闲状态。 */
void NavigationTask_ClearGoal(void);
/* 手动触发一次导航更新，返回本次规划和下发结果。 */
NavigationStatus_t NavigationTask_Update(void);
/* 获取导航状态快照，主要供遥测或调试代码读取。 */
void NavigationTask_GetStatsSnapshot(NavigationTaskStats_t *stats);

#endif /* NAVIGATION_TASK_H */
