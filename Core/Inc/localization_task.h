#ifndef LOCALIZATION_TASK_H
#define LOCALIZATION_TASK_H

#include <stdint.h>

#include "freertos_app.h"

typedef struct {
    uint8_t initialized;
    uint32_t update_count;
    uint32_t icp_accept_count;
    uint32_t icp_reject_count;
    uint32_t odom_only_count;
    uint16_t last_reference_points;
    uint16_t last_current_points;
    uint16_t last_inliers;
    float last_fitness_m;
    float last_odom_delta_theta_deg;
    float last_odom_delta_translation_m;
    uint8_t last_map_update_allowed;
    uint8_t last_map_skip_reason;
    uint8_t last_turning_detected;
    uint8_t turn_recovery_active;
    uint32_t skipped_recovery_count;
    LocalizationMode_t last_mode;
    SlamPose2D_t last_predicted_pose;
    SlamPose2D_t last_corrected_pose;
    SlamPose2D_t current_estimated_pose;
    SlamPose2D_t current_control_pose;
    SlamPose2D_t last_correction_delta;
    SlamPose2D_t last_control_correction_delta;
} LocalizationTaskStats_t;

void StartLocalizationTask(void *argument);
void LocalizationTask_Reset(void);
void LocalizationTask_GetStatsSnapshot(LocalizationTaskStats_t *stats);
void LocalizationTask_UpdatePredictedPose(const SlamPose2D_t *odom_pose);
void LocalizationTask_GetEstimatedPoseSnapshot(SlamPose2D_t *pose);
void LocalizationTask_GetControlPoseSnapshot(SlamPose2D_t *pose);
// 在 localization_task.h 中添加
static inline void LocalizationTask_GetPoseSnapshot(SlamPose2D_t *pose) {
    LocalizationTask_GetEstimatedPoseSnapshot(pose);
}

#endif /* LOCALIZATION_TASK_H */
