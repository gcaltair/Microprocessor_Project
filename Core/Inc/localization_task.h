#ifndef LOCALIZATION_TASK_H
#define LOCALIZATION_TASK_H

#include <stdint.h>

#include "freertos_app.h"

typedef struct {
    uint8_t initialized;
    uint32_t update_count;
    uint32_t odom_only_count;
    /* 为兼容遥测协议保留；纯里程计定位不会产生扫描匹配内点。 */
    uint16_t last_inliers;
    /* 为兼容遥测协议保留；纯里程计定位下该值固定为 0。 */
    float last_fitness_m;
    float last_odom_delta_theta_deg;
    float last_odom_delta_translation_m;
    uint8_t last_map_update_allowed;
    uint8_t last_map_skip_reason;
    uint8_t last_turning_detected;
    uint8_t last_scan_match_reject_reason;
    LocalizationMode_t last_mode;
    SlamPose2D_t last_predicted_pose;
    SlamPose2D_t last_corrected_pose;
    uint16_t last_scan_match_tested_candidates;
    uint16_t last_scan_match_used_points;
    float last_scan_match_best_score;
    float last_scan_match_second_score;
    float last_scan_match_score_margin;
    float last_scan_match_dx_m;
    float last_scan_match_dy_m;
    float last_scan_match_dtheta_deg;
    /* 当前最可信的位姿估计，供建图和状态上报使用。 */
    SlamPose2D_t current_estimated_pose;
    /* 去掉扫描匹配修正后，控制位姿与估计位姿保持一致。 */
    SlamPose2D_t current_control_pose;
} LocalizationTaskStats_t;

/* FreeRTOS 定位线程入口：接收雷达帧，发布纯里程计位姿，并执行建图门控。 */
void StartLocalizationTask(void *argument);
/* 清空定位统计、位姿快照和建图门控历史。 */
void LocalizationTask_Reset(void);
/* 获取定位统计快照，主要供遥测线程读取。 */
void LocalizationTask_GetStatsSnapshot(LocalizationTaskStats_t *stats);
/* 由里程计/控制侧写入最新预测位姿。 */
void LocalizationTask_UpdatePredictedPose(const SlamPose2D_t *odom_pose);
/* 读取当前估计位姿快照。 */
void LocalizationTask_GetEstimatedPoseSnapshot(SlamPose2D_t *pose);
/* 读取当前控制用位姿快照。 */
void LocalizationTask_GetControlPoseSnapshot(SlamPose2D_t *pose);

#endif /* LOCALIZATION_TASK_H */
