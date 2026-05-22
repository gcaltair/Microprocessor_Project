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
#define NAVIGATION_COMMAND_MAX_LEN         64U
#define NAVIGATION_MAX_PATH_POINTS         128U
#define NAVIGATION_REACH_DISTANCE_M        0.1f
#define NAVIGATION_MIN_SEGMENT_M           0.1f

static NavigationTaskStats_t g_navigationStats;
static SlamPose2D_t g_navigationGoal;
static uint8_t g_navigationGoalValid = 0U;
static uint8_t g_navigationCommandRxByte = 0U;
static char g_navigationCommandLine[NAVIGATION_COMMAND_MAX_LEN];
static char g_navigationPendingCommand[NAVIGATION_COMMAND_MAX_LEN];
static volatile uint8_t g_navigationCommandIndex = 0U;
static volatile uint8_t g_navigationCommandPending = 0U;

static NavigationPathPoint_t g_navigationSmoothPath[NAVIGATION_MAX_PATH_POINTS];
static uint16_t g_navigationSmoothPathLen = 0U;


void StartNavigationTask(void *argument) {
    for (;;) {
        // 您的黑盒任务主循环
        osDelay(600);
    }
}

void NavigationTask_SetGoal(float goal_x_m, float goal_y_m) {
    // 设置目标
}

void NavigationTask_ClearGoal(void) {
    // 清除目标
}

void NavigationTask_StartCommandRx(void) {
    // 开启指令串口接收中断
}

void NavigationTask_HandleCommandRxCompleteFromIsr(void) {
    // 中断处理
}

NavigationStatus_t NavigationTask_Update(void) {
    // 导航算法更新
    return NAVIGATION_STATUS_IDLE;
}

void NavigationTask_GetStatsSnapshot(NavigationTaskStats_t *stats) {
    // 状态统计读取
}

uint16_t NavigationTask_CopySmoothPathPoints(NavigationPathPoint_t *points, uint16_t max_points) {
    // 拷贝路径点
    return 0;
}