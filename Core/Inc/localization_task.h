#ifndef LOCALIZATION_TASK_H
#define LOCALIZATION_TASK_H

#include <stdint.h>

#include "freertos_app.h"

typedef struct {
    uint8_t initialized;
    uint32_t update_count;
    float last_odom_delta_theta_deg;
    float last_odom_delta_translation_m;
    uint8_t last_map_update_allowed;
    uint8_t last_map_skip_reason;
    uint8_t last_turning_detected;
    /* 上一帧雷达对应的里程计位姿（纯里程计模式下无预测/修正之分）。 */
    SlamPose2D_t last_pose;
    /* 当前最可信的位姿估计，供建图、控制和状态上报使用。 */
    SlamPose2D_t current_pose;
} LocalizationTaskStats_t;

/* FreeRTOS 定位线程入口：接收雷达帧，发布位姿，并执行建图门控。 */
void StartLocalizationTask(void *argument);
/* 清空定位统计、位姿快照和建图门控历史。 */
void LocalizationTask_Reset(void);
/* 获取定位统计快照，主要供遥测线程读取。 */
void LocalizationTask_GetStatsSnapshot(LocalizationTaskStats_t *stats);
/* 由里程计/控制侧写入最新位姿。 */
void LocalizationTask_UpdatePose(const SlamPose2D_t *odom_pose);
/* 读取当前位姿快照。 */
void LocalizationTask_GetPoseSnapshot(SlamPose2D_t *pose);


#endif /* LOCALIZATION_TASK_H */
