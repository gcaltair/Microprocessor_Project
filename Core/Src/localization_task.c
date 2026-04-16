#include <math.h>
#include <string.h>

#include "../Inc/localization_task.h"
#include "../Inc/scan_preprocess.h"
#include "system.h"

#define LOCALIZATION_MAX_SAMPLE_POINTS          64U
#define LOCALIZATION_MIN_REFERENCE_POINTS       12U
#define LOCALIZATION_MIN_INLIERS                10U
#define LOCALIZATION_MAX_ITERATIONS             4U
#define LOCALIZATION_MAX_MATCH_DISTANCE_M       0.18f
#define LOCALIZATION_MAX_ACCEPT_FITNESS_M       0.08f
#define LOCALIZATION_MAX_CORRECTION_XY_M        0.12f
#define LOCALIZATION_MAX_CORRECTION_THETA_DEG   12.0f
#define LOCALIZATION_CONVERGENCE_XY_M           0.002f
#define LOCALIZATION_CONVERGENCE_THETA_DEG      0.2f
#define LOCALIZATION_CONTROL_CORRECTION_ALPHA   0.15f
#define LOCALIZATION_CONTROL_MAX_STEP_XY_M      0.004f
#define LOCALIZATION_CONTROL_MAX_STEP_THETA_DEG 0.35f
#define LOCALIZATION_CONTROL_MIN_WHEEL_SPEED_MPS 0.02f
#define LOCALIZATION_DEG_TO_RAD                 0.01745329251994329577f
#define LOCALIZATION_RAD_TO_DEG                 57.2957795130823208768f

typedef struct {
    float x_m;
    float y_m;
} LocalizationPoint_t;

typedef struct {
    uint8_t valid;
    uint16_t point_count;
    LocalizationPoint_t points[LOCALIZATION_MAX_SAMPLE_POINTS];
} LocalizationReferenceScan_t;

typedef struct {
    uint8_t accepted;
    uint16_t inliers;
    float fitness_m;
    float delta_x_m;
    float delta_y_m;
    float delta_theta_deg;
} LocalizationIcpResult_t;

static LocalizationReferenceScan_t g_referenceScan;
static LocalizationTaskStats_t g_localizationStats;
static SlamPose2D_t g_estimatedPose;
static SlamPose2D_t g_controlPose;
static SlamPose2D_t g_lastOdomPose;
static uint8_t g_estimatedPoseInitialized = 0U;
static LocalizationPoint_t g_sourcePoints[LOCALIZATION_MAX_SAMPLE_POINTS];
static LocalizationPoint_t g_workingPoints[LOCALIZATION_MAX_SAMPLE_POINTS];
static LocalizationPoint_t g_corrSource[LOCALIZATION_MAX_SAMPLE_POINTS];
static LocalizationPoint_t g_corrTarget[LOCALIZATION_MAX_SAMPLE_POINTS];

static float localization_wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static float localization_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static void localization_lock(void)
{
    if (g_localizationMutex != NULL) {
        (void)osMutexAcquire(g_localizationMutex, osWaitForever);
    }
}

static void localization_unlock(void)
{
    if (g_localizationMutex != NULL) {
        (void)osMutexRelease(g_localizationMutex);
    }
}

static void localization_transform_point(const LocalizationPoint_t *in_point,
                                         float cos_theta,
                                         float sin_theta,
                                         float tx_m,
                                         float ty_m,
                                         LocalizationPoint_t *out_point)
{
    out_point->x_m = cos_theta * in_point->x_m - sin_theta * in_point->y_m + tx_m;
    out_point->y_m = sin_theta * in_point->x_m + cos_theta * in_point->y_m + ty_m;
}

static void localization_apply_transform(LocalizationPoint_t *points,
                                         uint16_t point_count,
                                         float theta_deg,
                                         float tx_m,
                                         float ty_m)
{
    float theta_rad = theta_deg * LOCALIZATION_DEG_TO_RAD;
    float cos_theta = cosf(theta_rad);
    float sin_theta = sinf(theta_rad);
    uint16_t idx;

    for (idx = 0U; idx < point_count; ++idx) {
        LocalizationPoint_t transformed;
        localization_transform_point(&points[idx], cos_theta, sin_theta, tx_m, ty_m, &transformed);
        points[idx] = transformed;
    }
}

static void localization_apply_pose_delta(const SlamPose2D_t *input_pose,
                                          float delta_theta_deg,
                                          float delta_x_m,
                                          float delta_y_m,
                                          SlamPose2D_t *output_pose)
{
    float theta_rad = delta_theta_deg * LOCALIZATION_DEG_TO_RAD;
    float cos_theta = cosf(theta_rad);
    float sin_theta = sinf(theta_rad);

    *output_pose = *input_pose;
    output_pose->x_m = cos_theta * input_pose->x_m - sin_theta * input_pose->y_m + delta_x_m;
    output_pose->y_m = sin_theta * input_pose->x_m + cos_theta * input_pose->y_m + delta_y_m;
    output_pose->theta_deg = localization_wrap_angle_deg(input_pose->theta_deg + delta_theta_deg);
}

static void localization_rotate_vector(float x_m,
                                       float y_m,
                                       float theta_deg,
                                       float *out_x_m,
                                       float *out_y_m)
{
    float theta_rad = theta_deg * LOCALIZATION_DEG_TO_RAD;
    float cos_theta = cosf(theta_rad);
    float sin_theta = sinf(theta_rad);

    if (out_x_m != NULL) {
        *out_x_m = cos_theta * x_m - sin_theta * y_m;
    }

    if (out_y_m != NULL) {
        *out_y_m = sin_theta * x_m + cos_theta * y_m;
    }
}

static void localization_apply_control_correction(const SlamPose2D_t *target_pose,
                                                  SlamPose2D_t *applied_delta)
{
    float error_x_m;
    float error_y_m;
    float error_theta_deg;
    float step_x_m;
    float step_y_m;
    float step_theta_deg;

    if (target_pose == NULL) {
        return;
    }

    error_x_m = target_pose->x_m - g_controlPose.x_m;
    error_y_m = target_pose->y_m - g_controlPose.y_m;
    error_theta_deg = localization_wrap_angle_deg(target_pose->theta_deg - g_controlPose.theta_deg);

    step_x_m = localization_clamp_float(error_x_m * LOCALIZATION_CONTROL_CORRECTION_ALPHA,
                                        -LOCALIZATION_CONTROL_MAX_STEP_XY_M,
                                        LOCALIZATION_CONTROL_MAX_STEP_XY_M);
    step_y_m = localization_clamp_float(error_y_m * LOCALIZATION_CONTROL_CORRECTION_ALPHA,
                                        -LOCALIZATION_CONTROL_MAX_STEP_XY_M,
                                        LOCALIZATION_CONTROL_MAX_STEP_XY_M);
    step_theta_deg = localization_clamp_float(error_theta_deg * LOCALIZATION_CONTROL_CORRECTION_ALPHA,
                                              -LOCALIZATION_CONTROL_MAX_STEP_THETA_DEG,
                                              LOCALIZATION_CONTROL_MAX_STEP_THETA_DEG);

    g_controlPose.x_m += step_x_m;
    g_controlPose.y_m += step_y_m;
    g_controlPose.theta_deg = localization_wrap_angle_deg(g_controlPose.theta_deg + step_theta_deg);
    g_controlPose.timestamp_ms = target_pose->timestamp_ms;

    if (applied_delta != NULL) {
        applied_delta->x_m = step_x_m;
        applied_delta->y_m = step_y_m;
        applied_delta->theta_deg = step_theta_deg;
        applied_delta->timestamp_ms = target_pose->timestamp_ms;
    }
}

static uint8_t localization_control_fusion_enabled(void)
{
    if (g_control_mode != CONTROL_MODE_POSITION) {
        return 0U;
    }

    if (g_relative_move_state == RELATIVE_MOVE_IDLE) {
        return 0U;
    }

    if (fabsf(base_car_speed) > 0.001f) {
        return 1U;
    }

    if ((fabsf(g_left_speed) > LOCALIZATION_CONTROL_MIN_WHEEL_SPEED_MPS) ||
        (fabsf(g_right_speed) > LOCALIZATION_CONTROL_MIN_WHEEL_SPEED_MPS)) {
        return 1U;
    }

    return (g_relative_move_state == RELATIVE_MOVE_TURNING) ? 1U : 0U;
}

static uint16_t localization_build_world_points(const LidarScanMsg_t *scan_msg,
                                                const SlamPose2D_t *pose,
                                                LocalizationPoint_t *out_points)
{
    const LidarScanBuffer_t *scan_buffer;
    uint16_t usable_points = 0U;
    uint16_t stride;
    uint16_t selected_index = 0U;
    uint16_t idx;

    if ((scan_msg == NULL) ||
        (pose == NULL) ||
        (out_points == NULL) ||
        (scan_msg->scan_index >= LIDAR_SCAN_BUFFER_COUNT)) {
        return 0U;
    }

    scan_buffer = &g_lidarScanBuf[scan_msg->scan_index];
    for (idx = 0U; idx < scan_buffer->point_count; ++idx) {
        if (ScanPreprocess_IsPointUsable(&scan_buffer->points[idx]) != 0U) {
            usable_points++;
        }
    }

    if (usable_points == 0U) {
        return 0U;
    }

    stride = (usable_points > LOCALIZATION_MAX_SAMPLE_POINTS)
             ? (uint16_t)((usable_points + LOCALIZATION_MAX_SAMPLE_POINTS - 1U) / LOCALIZATION_MAX_SAMPLE_POINTS)
             : 1U;
    usable_points = 0U;

    for (idx = 0U; idx < scan_buffer->point_count; ++idx) {
        float angle_rad;
        float distance_m;

        if (ScanPreprocess_IsPointUsable(&scan_buffer->points[idx]) == 0U) {
            continue;
        }

        if ((usable_points % stride) != 0U) {
            usable_points++;
            continue;
        }

        distance_m = scan_buffer->points[idx].distance_mm * 0.001f;
        angle_rad = (pose->theta_deg + scan_buffer->points[idx].angle_deg) * LOCALIZATION_DEG_TO_RAD;
        out_points[selected_index].x_m = pose->x_m + distance_m * cosf(angle_rad);
        out_points[selected_index].y_m = pose->y_m + distance_m * sinf(angle_rad);

        selected_index++;
        usable_points++;
        if (selected_index >= LOCALIZATION_MAX_SAMPLE_POINTS) {
            break;
        }
    }

    return selected_index;
}

static uint16_t localization_find_correspondences(const LocalizationPoint_t *source_points,
                                                  uint16_t source_count,
                                                  const LocalizationPoint_t *reference_points,
                                                  uint16_t reference_count,
                                                  float *mean_sq_error)
{
    const float max_match_distance_sq = LOCALIZATION_MAX_MATCH_DISTANCE_M * LOCALIZATION_MAX_MATCH_DISTANCE_M;
    float total_sq_error = 0.0f;
    uint16_t inlier_count = 0U;
    uint16_t source_idx;

    for (source_idx = 0U; source_idx < source_count; ++source_idx) {
        float best_sq_distance = max_match_distance_sq;
        uint16_t best_target_idx = UINT16_MAX;
        uint16_t target_idx;

        for (target_idx = 0U; target_idx < reference_count; ++target_idx) {
            float dx = reference_points[target_idx].x_m - source_points[source_idx].x_m;
            float dy = reference_points[target_idx].y_m - source_points[source_idx].y_m;
            float sq_distance = dx * dx + dy * dy;

            if (sq_distance < best_sq_distance) {
                best_sq_distance = sq_distance;
                best_target_idx = target_idx;
            }
        }

        if (best_target_idx == UINT16_MAX) {
            continue;
        }

        g_corrSource[inlier_count] = source_points[source_idx];
        g_corrTarget[inlier_count] = reference_points[best_target_idx];
        total_sq_error += best_sq_distance;
        inlier_count++;
    }

    if (mean_sq_error != NULL) {
        *mean_sq_error = (inlier_count > 0U) ? (total_sq_error / (float)inlier_count) : 0.0f;
    }

    return inlier_count;
}

static uint8_t localization_compute_delta(uint16_t inlier_count,
                                          float *delta_theta_deg,
                                          float *delta_x_m,
                                          float *delta_y_m)
{
    float src_centroid_x = 0.0f;
    float src_centroid_y = 0.0f;
    float dst_centroid_x = 0.0f;
    float dst_centroid_y = 0.0f;
    float numerator = 0.0f;
    float denominator = 0.0f;
    float theta_rad;
    float cos_theta;
    float sin_theta;
    uint16_t idx;

    if ((delta_theta_deg == NULL) || (delta_x_m == NULL) || (delta_y_m == NULL) || (inlier_count == 0U)) {
        return 0U;
    }

    for (idx = 0U; idx < inlier_count; ++idx) {
        src_centroid_x += g_corrSource[idx].x_m;
        src_centroid_y += g_corrSource[idx].y_m;
        dst_centroid_x += g_corrTarget[idx].x_m;
        dst_centroid_y += g_corrTarget[idx].y_m;
    }

    src_centroid_x /= (float)inlier_count;
    src_centroid_y /= (float)inlier_count;
    dst_centroid_x /= (float)inlier_count;
    dst_centroid_y /= (float)inlier_count;

    for (idx = 0U; idx < inlier_count; ++idx) {
        float src_x = g_corrSource[idx].x_m - src_centroid_x;
        float src_y = g_corrSource[idx].y_m - src_centroid_y;
        float dst_x = g_corrTarget[idx].x_m - dst_centroid_x;
        float dst_y = g_corrTarget[idx].y_m - dst_centroid_y;

        numerator += src_x * dst_y - src_y * dst_x;
        denominator += src_x * dst_x + src_y * dst_y;
    }

    theta_rad = atan2f(numerator, denominator);
    cos_theta = cosf(theta_rad);
    sin_theta = sinf(theta_rad);
    *delta_theta_deg = theta_rad * LOCALIZATION_RAD_TO_DEG;
    *delta_x_m = dst_centroid_x - (cos_theta * src_centroid_x - sin_theta * src_centroid_y);
    *delta_y_m = dst_centroid_y - (sin_theta * src_centroid_x + cos_theta * src_centroid_y);
    return 1U;
}

static void localization_run_icp(const LidarScanMsg_t *scan_msg, LocalizationIcpResult_t *result)
{
    float accum_theta_deg = 0.0f;
    float accum_x_m = 0.0f;
    float accum_y_m = 0.0f;
    float mean_sq_error = 0.0f;
    LocalizationPoint_t reference_points[LOCALIZATION_MAX_SAMPLE_POINTS];
    uint8_t reference_valid;
    uint16_t reference_count;
    uint16_t source_count;
    uint16_t iteration;

    (void)memset(result, 0, sizeof(*result));
    source_count = localization_build_world_points(scan_msg, &scan_msg->pose_snapshot, g_sourcePoints);
    result->fitness_m = 0.0f;

    localization_lock();
    reference_valid = g_referenceScan.valid;
    reference_count = g_referenceScan.point_count;
    if (reference_count > LOCALIZATION_MAX_SAMPLE_POINTS) {
        reference_count = LOCALIZATION_MAX_SAMPLE_POINTS;
    }
    (void)memcpy(reference_points, g_referenceScan.points, reference_count * sizeof(reference_points[0]));
    g_localizationStats.last_current_points = source_count;
    g_localizationStats.last_reference_points = reference_count;
    localization_unlock();

    if ((source_count < LOCALIZATION_MIN_REFERENCE_POINTS) ||
        (reference_count < LOCALIZATION_MIN_REFERENCE_POINTS) ||
        (reference_valid == 0U)) {
        return;
    }

    (void)memcpy(g_workingPoints, g_sourcePoints, source_count * sizeof(g_workingPoints[0]));

    for (iteration = 0U; iteration < LOCALIZATION_MAX_ITERATIONS; ++iteration) {
        float delta_theta_deg;
        float delta_x_m;
        float delta_y_m;
        float step_cos;
        float step_sin;
        float next_accum_x_m;
        float next_accum_y_m;
        uint16_t inlier_count = localization_find_correspondences(g_workingPoints,
                                                                  source_count,
                                                                  reference_points,
                                                                  reference_count,
                                                                  &mean_sq_error);

        if (inlier_count < LOCALIZATION_MIN_INLIERS) {
            result->inliers = inlier_count;
            result->fitness_m = sqrtf(mean_sq_error);
            return;
        }

        if (localization_compute_delta(inlier_count, &delta_theta_deg, &delta_x_m, &delta_y_m) == 0U) {
            return;
        }

        localization_apply_transform(g_workingPoints, source_count, delta_theta_deg, delta_x_m, delta_y_m);
        step_cos = cosf(delta_theta_deg * LOCALIZATION_DEG_TO_RAD);
        step_sin = sinf(delta_theta_deg * LOCALIZATION_DEG_TO_RAD);
        next_accum_x_m = step_cos * accum_x_m - step_sin * accum_y_m + delta_x_m;
        next_accum_y_m = step_sin * accum_x_m + step_cos * accum_y_m + delta_y_m;
        accum_x_m = next_accum_x_m;
        accum_y_m = next_accum_y_m;
        accum_theta_deg += delta_theta_deg;
        result->inliers = inlier_count;
        result->fitness_m = sqrtf(mean_sq_error);

        if ((fabsf(delta_x_m) <= LOCALIZATION_CONVERGENCE_XY_M) &&
            (fabsf(delta_y_m) <= LOCALIZATION_CONVERGENCE_XY_M) &&
            (fabsf(delta_theta_deg) <= LOCALIZATION_CONVERGENCE_THETA_DEG)) {
            break;
        }
    }

    result->delta_x_m = accum_x_m;
    result->delta_y_m = accum_y_m;
    result->delta_theta_deg = accum_theta_deg;

    if ((result->inliers >= LOCALIZATION_MIN_INLIERS) &&
        (result->fitness_m <= LOCALIZATION_MAX_ACCEPT_FITNESS_M) &&
        (fabsf(result->delta_x_m) <= LOCALIZATION_MAX_CORRECTION_XY_M) &&
        (fabsf(result->delta_y_m) <= LOCALIZATION_MAX_CORRECTION_XY_M) &&
        (fabsf(result->delta_theta_deg) <= LOCALIZATION_MAX_CORRECTION_THETA_DEG)) {
        result->accepted = 1U;
    }
}

static void localization_store_reference(const LidarScanMsg_t *scan_msg, const SlamPose2D_t *reference_pose)
{
    uint16_t point_count = localization_build_world_points(scan_msg, reference_pose, g_referenceScan.points);

    if (point_count >= LOCALIZATION_MIN_REFERENCE_POINTS) {
        g_referenceScan.valid = 1U;
        g_referenceScan.point_count = point_count;
    } else {
        g_referenceScan.valid = 0U;
        g_referenceScan.point_count = 0U;
    }

    localization_lock();
    g_localizationStats.initialized = g_referenceScan.valid;
    g_localizationStats.last_reference_points = g_referenceScan.point_count;
    localization_unlock();
}

void LocalizationTask_UpdatePredictedPose(const SlamPose2D_t *odom_pose)
{
    float odom_delta_x_m;
    float odom_delta_y_m;
    float odom_delta_theta_deg;
    float predicted_delta_x_m;
    float predicted_delta_y_m;
    uint8_t control_fusion_enabled;

    if (odom_pose == NULL) {
        return;
    }

    control_fusion_enabled = localization_control_fusion_enabled();
    localization_lock();

    if (g_estimatedPoseInitialized == 0U) {
        g_estimatedPose = *odom_pose;
        g_controlPose = *odom_pose;
        g_lastOdomPose = *odom_pose;
        g_estimatedPoseInitialized = 1U;
        g_localizationStats.current_estimated_pose = g_estimatedPose;
        g_localizationStats.current_control_pose = g_controlPose;
        localization_unlock();
        return;
    }

    odom_delta_x_m = odom_pose->x_m - g_lastOdomPose.x_m;
    odom_delta_y_m = odom_pose->y_m - g_lastOdomPose.y_m;
    odom_delta_theta_deg = localization_wrap_angle_deg(odom_pose->theta_deg - g_lastOdomPose.theta_deg);

    localization_rotate_vector(odom_delta_x_m,
                               odom_delta_y_m,
                               g_estimatedPose.theta_deg - g_lastOdomPose.theta_deg,
                               &predicted_delta_x_m,
                               &predicted_delta_y_m);

    g_estimatedPose.x_m += predicted_delta_x_m;
    g_estimatedPose.y_m += predicted_delta_y_m;
    g_estimatedPose.theta_deg = localization_wrap_angle_deg(g_estimatedPose.theta_deg + odom_delta_theta_deg);
    g_estimatedPose.timestamp_ms = odom_pose->timestamp_ms;

    if (control_fusion_enabled != 0U) {
        g_controlPose.x_m += predicted_delta_x_m;
        g_controlPose.y_m += predicted_delta_y_m;
        g_controlPose.theta_deg = localization_wrap_angle_deg(g_controlPose.theta_deg + odom_delta_theta_deg);
        g_controlPose.timestamp_ms = odom_pose->timestamp_ms;
    } else {
        g_controlPose = *odom_pose;
    }

    g_lastOdomPose = *odom_pose;
    g_localizationStats.current_estimated_pose = g_estimatedPose;
    g_localizationStats.current_control_pose = g_controlPose;

    localization_unlock();
}

void LocalizationTask_GetEstimatedPoseSnapshot(SlamPose2D_t *pose)
{
    if (pose == NULL) {
        return;
    }

    localization_lock();
    if (g_estimatedPoseInitialized != 0U) {
        *pose = g_estimatedPose;
    } else {
        (void)memset(pose, 0, sizeof(*pose));
    }
    localization_unlock();
}

void LocalizationTask_GetControlPoseSnapshot(SlamPose2D_t *pose)
{
    if (pose == NULL) {
        return;
    }

    localization_lock();
    if (g_estimatedPoseInitialized != 0U) {
        *pose = g_controlPose;
    } else {
        (void)memset(pose, 0, sizeof(*pose));
    }
    localization_unlock();
}

static void localization_update_stats(const LidarScanMsg_t *scan_msg,
                                      const LocalizationIcpResult_t *result)
{
    localization_lock();
    g_localizationStats.update_count++;
    g_localizationStats.last_predicted_pose = scan_msg->pose_snapshot;
    g_localizationStats.last_corrected_pose = scan_msg->corrected_pose;
    g_localizationStats.current_estimated_pose = g_estimatedPose;
    g_localizationStats.current_control_pose = g_controlPose;
    g_localizationStats.last_inliers = scan_msg->localization_inliers;
    g_localizationStats.last_fitness_m = scan_msg->localization_fitness_m;
    g_localizationStats.last_mode = (LocalizationMode_t)scan_msg->localization_mode;
    g_localizationStats.last_correction_delta.x_m = result->delta_x_m;
    g_localizationStats.last_correction_delta.y_m = result->delta_y_m;
    g_localizationStats.last_correction_delta.theta_deg = result->delta_theta_deg;
    g_localizationStats.last_correction_delta.timestamp_ms = scan_msg->corrected_pose.timestamp_ms;

    if (scan_msg->localization_mode == LOCALIZATION_MODE_ICP_ACCEPTED) {
        g_localizationStats.icp_accept_count++;
    } else if (scan_msg->localization_mode == LOCALIZATION_MODE_ICP_REJECTED) {
        g_localizationStats.icp_reject_count++;
    } else {
        g_localizationStats.odom_only_count++;
    }
    localization_unlock();
}

void StartLocalizationTask(void *argument)
{
    LidarScanMsg_t scan_msg;

    (void)argument;
    (void)memset(&scan_msg, 0, sizeof(scan_msg));
    LocalizationTask_Reset();

    for (;;) {
        LocalizationIcpResult_t icp_result;

        if (osMessageQueueGet(g_lidarResultQueue, &scan_msg, NULL, osWaitForever) != osOK) {
            continue;
        }

        scan_msg.corrected_pose = scan_msg.pose_snapshot;
        scan_msg.localization_mode = LOCALIZATION_MODE_ODOMETRY_ONLY;
        scan_msg.localization_inliers = 0U;
        scan_msg.localization_fitness_m = 0.0f;
        (void)memset(&icp_result, 0, sizeof(icp_result));

        localization_run_icp(&scan_msg, &icp_result);
        scan_msg.localization_inliers = icp_result.inliers;
        scan_msg.localization_fitness_m = icp_result.fitness_m;

        if ((g_referenceScan.valid != 0U) && (g_referenceScan.point_count >= LOCALIZATION_MIN_REFERENCE_POINTS)) {
            if (icp_result.accepted != 0U) {
                localization_apply_pose_delta(&scan_msg.pose_snapshot,
                                              icp_result.delta_theta_deg,
                                              icp_result.delta_x_m,
                                              icp_result.delta_y_m,
                                              &scan_msg.corrected_pose);
                scan_msg.localization_mode = LOCALIZATION_MODE_ICP_ACCEPTED;
            } else {
                scan_msg.localization_mode = LOCALIZATION_MODE_ICP_REJECTED;
            }
        }

        localization_lock();
        g_estimatedPose = scan_msg.corrected_pose;
        g_estimatedPoseInitialized = 1U;
        g_localizationStats.current_estimated_pose = g_estimatedPose;
        if ((scan_msg.localization_mode == LOCALIZATION_MODE_ICP_ACCEPTED) &&
            (localization_control_fusion_enabled() != 0U)) {
            localization_apply_control_correction(&scan_msg.corrected_pose,
                                                  &g_localizationStats.last_control_correction_delta);
        } else {
            (void)memset(&g_localizationStats.last_control_correction_delta,
                         0,
                         sizeof(g_localizationStats.last_control_correction_delta));
        }
        g_localizationStats.current_control_pose = g_controlPose;
        g_lastOdomPose = scan_msg.pose_snapshot;
        localization_unlock();

        localization_store_reference(&scan_msg, &scan_msg.corrected_pose);
        localization_update_stats(&scan_msg, &icp_result);

        if (g_localizedScanQueue != NULL) {
            (void)osMessageQueuePut(g_localizedScanQueue, &scan_msg, 0U, osWaitForever);
        } else {
            uint8_t free_idx = scan_msg.scan_index;
            g_lidarScanBuf[free_idx].point_count = 0U;
            (void)osMessageQueuePut(g_lidarFreeQueue, &free_idx, 0U, osWaitForever);
        }
    }
}

void LocalizationTask_Reset(void)
{
    localization_lock();
    (void)memset(&g_referenceScan, 0, sizeof(g_referenceScan));
    (void)memset(&g_localizationStats, 0, sizeof(g_localizationStats));
    (void)memset(&g_estimatedPose, 0, sizeof(g_estimatedPose));
    (void)memset(&g_controlPose, 0, sizeof(g_controlPose));
    (void)memset(&g_lastOdomPose, 0, sizeof(g_lastOdomPose));
    g_estimatedPoseInitialized = 0U;
    localization_unlock();
}

void LocalizationTask_GetStatsSnapshot(LocalizationTaskStats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    localization_lock();
    *stats = g_localizationStats;
    localization_unlock();
}
