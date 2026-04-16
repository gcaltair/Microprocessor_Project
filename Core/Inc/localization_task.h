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

#endif /* LOCALIZATION_TASK_H */
