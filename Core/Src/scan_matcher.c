#include <math.h>
#include <string.h>

#include "scan_matcher.h"
#include "../Inc/mapping_task.h"
#include "../Inc/scan_preprocess.h"

#define SCAN_MATCH_POINT_STRIDE             6U
#define SCAN_MATCH_MIN_SCAN_POINTS          30U
#define SCAN_MATCH_MIN_MAP_UPDATES          10U
#define SCAN_MATCH_MIN_USED_POINTS          30U
#define SCAN_MATCH_MIN_OCCUPIED_HITS        12U
#define SCAN_MATCH_MIN_SCORE                25.0f
#define SCAN_MATCH_MIN_SCORE_MARGIN         6.0f

#define SCAN_MATCH_DX_STEP_M                0.02f
#define SCAN_MATCH_DY_STEP_M                0.02f
#define SCAN_MATCH_DTHETA_STEP_DEG          1.0f
#define SCAN_MATCH_MAX_ACCEPT_DX_M          0.04f
#define SCAN_MATCH_MAX_ACCEPT_DY_M          0.04f
#define SCAN_MATCH_MAX_ACCEPT_DTHETA_DEG    2.0f

#define SCAN_MATCH_OCCUPIED_THRESHOLD       5
#define SCAN_MATCH_FREE_THRESHOLD           (-3)
#define SCAN_MATCH_SCORE_OCCUPIED_HIT       3.0f
#define SCAN_MATCH_SCORE_FREE_MISS          (-1.0f)
#define SCAN_MATCH_SCORE_OUTSIDE            (-2.0f)
#define SCAN_MATCH_DEG_TO_RAD               0.01745329251994329577f

typedef struct {
    float x_m;
    float y_m;
} ScanMatchPoint_t;

typedef struct {
    float score;
    uint16_t used_points;
    uint16_t occupied_hits;
} ScanMatchScore_t;

static ScanMatchPoint_t s_scan_match_points[SCAN_MATCH_MIN_SCAN_POINTS * 4U];

static void scan_match_init_result(ScanMatcherResult_t *result)
{
    if (result != NULL) {
        (void)memset(result, 0, sizeof(*result));
        result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_NONE;
    }
}

static int16_t scan_match_world_to_cell_x(const MappingGridMeta_t *meta, float x_m)
{
    return (int16_t)floorf((x_m - meta->origin_x_m) / meta->resolution_m_per_cell);
}

static int16_t scan_match_world_to_cell_y(const MappingGridMeta_t *meta, float y_m)
{
    return (int16_t)floorf((y_m - meta->origin_y_m) / meta->resolution_m_per_cell);
}

static uint8_t scan_match_cell_inside(const MappingGridMeta_t *meta, int16_t cell_x, int16_t cell_y)
{
    if ((cell_x < 0) || (cell_y < 0)) {
        return 0U;
    }

    if (((uint16_t)cell_x >= meta->width_cells) ||
        ((uint16_t)cell_y >= meta->height_cells)) {
        return 0U;
    }

    return 1U;
}

static int8_t scan_match_get_cell(int16_t cell_x, int16_t cell_y)
{
    int8_t value = 0;

    (void)MappingTask_ReadCellDuringGridRead(cell_x, cell_y, &value);
    return value;
}

static uint8_t scan_match_has_occupied_neighbor(const MappingGridMeta_t *meta,
                                                int16_t center_x,
                                                int16_t center_y)
{
    int16_t dy;

    for (dy = -1; dy <= 1; ++dy) {
        int16_t dx;

        for (dx = -1; dx <= 1; ++dx) {
            int16_t cell_x = (int16_t)(center_x + dx);
            int16_t cell_y = (int16_t)(center_y + dy);

            if ((scan_match_cell_inside(meta, cell_x, cell_y) != 0U) &&
                (scan_match_get_cell(cell_x, cell_y) >= SCAN_MATCH_OCCUPIED_THRESHOLD)) {
                return 1U;
            }
        }
    }

    return 0U;
}

static uint16_t scan_match_extract_points(const LidarScanMsg_t *scan_msg)
{
    const LidarScanBuffer_t *scan_buffer;
    uint16_t idx;
    uint16_t count = 0U;

    if ((scan_msg == NULL) ||
        (scan_msg->scan_index >= LIDAR_SCAN_BUFFER_COUNT)) {
        return 0U;
    }

    scan_buffer = &g_lidarScanBuf[scan_msg->scan_index];
    for (idx = 0U;
         (idx < scan_buffer->point_count) && (count < (uint16_t)(sizeof(s_scan_match_points) / sizeof(s_scan_match_points[0])));
         idx = (uint16_t)(idx + SCAN_MATCH_POINT_STRIDE)) {
        const LidarPoint_t *point = &scan_buffer->points[idx];
        float angle_rad;
        float distance_m;

        if (ScanPreprocess_IsPointUsable(point) == 0U) {
            continue;
        }

        distance_m = point->distance_mm * 0.001f;
        angle_rad = ScanPreprocess_BeamWorldAngleDeg(0.0f, point->angle_deg) * SCAN_MATCH_DEG_TO_RAD;
        s_scan_match_points[count].x_m = distance_m * cosf(angle_rad);
        s_scan_match_points[count].y_m = distance_m * sinf(angle_rad);
        count++;
    }

    return count;
}

static ScanMatchScore_t scan_match_score_candidate(const MappingGridMeta_t *meta,
                                                   const SlamPose2D_t *pose,
                                                   uint16_t point_count)
{
    ScanMatchScore_t score;
    float theta_rad = pose->theta_deg * SCAN_MATCH_DEG_TO_RAD;
    float cos_theta = cosf(theta_rad);
    float sin_theta = sinf(theta_rad);
    uint16_t idx;

    (void)memset(&score, 0, sizeof(score));

    for (idx = 0U; idx < point_count; ++idx) {
        float world_x = pose->x_m +
                        (s_scan_match_points[idx].x_m * cos_theta) -
                        (s_scan_match_points[idx].y_m * sin_theta);
        float world_y = pose->y_m +
                        (s_scan_match_points[idx].x_m * sin_theta) +
                        (s_scan_match_points[idx].y_m * cos_theta);
        int16_t cell_x = scan_match_world_to_cell_x(meta, world_x);
        int16_t cell_y = scan_match_world_to_cell_y(meta, world_y);
        int8_t cell_value;

        score.used_points++;
        if (scan_match_cell_inside(meta, cell_x, cell_y) == 0U) {
            score.score += SCAN_MATCH_SCORE_OUTSIDE;
            continue;
        }

        if (scan_match_has_occupied_neighbor(meta, cell_x, cell_y) != 0U) {
            score.score += SCAN_MATCH_SCORE_OCCUPIED_HIT;
            score.occupied_hits++;
            continue;
        }

        cell_value = scan_match_get_cell(cell_x, cell_y);
        if (cell_value <= SCAN_MATCH_FREE_THRESHOLD) {
            score.score += SCAN_MATCH_SCORE_FREE_MISS;
        }
    }

    return score;
}

uint8_t ScanMatcher_CorrectPose(const LidarScanMsg_t *scan_msg,
                                SlamPose2D_t *corrected_pose,
                                ScanMatcherResult_t *result)
{
    MappingGridMeta_t meta;
    MappingTaskStats_t mapping_stats;
    SlamPose2D_t best_pose;
    float best_dx = 0.0f;
    float best_dy = 0.0f;
    float best_dtheta = 0.0f;
    float best_score = -1000000.0f;
    float second_score = -1000000.0f;
    uint16_t point_count;
    uint16_t tested_candidates = 0U;
    uint16_t best_hits = 0U;
    uint8_t best_on_edge = 0U;
    int8_t ix;

    scan_match_init_result(result);
    if ((scan_msg == NULL) || (corrected_pose == NULL)) {
        if (result != NULL) {
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_SCAN_QUALITY;
        }
        return 0U;
    }

    *corrected_pose = scan_msg->pose_snapshot;
    MappingTask_GetStatsSnapshot(&mapping_stats);
    if ((mapping_stats.grid_initialized == 0U) ||
        (mapping_stats.update_count < SCAN_MATCH_MIN_MAP_UPDATES) ||
        (MappingTask_GetGridMeta(&meta) == 0U)) {
        if (result != NULL) {
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_MAP_NOT_READY;
        }
        return 0U;
    }

    if (scan_msg->quality.usable_point_count < SCAN_MATCH_MIN_SCAN_POINTS) {
        if (result != NULL) {
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_SCAN_QUALITY;
        }
        return 0U;
    }

    point_count = scan_match_extract_points(scan_msg);
    if (point_count < SCAN_MATCH_MIN_USED_POINTS) {
        if (result != NULL) {
            result->used_points = point_count;
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_SCAN_QUALITY;
        }
        return 0U;
    }

    best_pose = scan_msg->pose_snapshot;
    if (MappingTask_BeginGridRead() == 0U) {
        if (result != NULL) {
            result->used_points = point_count;
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_MAP_NOT_READY;
        }
        return 0U;
    }

    for (ix = -2; ix <= 2; ++ix) {
        int8_t iy;

        for (iy = -2; iy <= 2; ++iy) {
            int8_t itheta;

            for (itheta = -2; itheta <= 2; ++itheta) {
                SlamPose2D_t candidate = scan_msg->pose_snapshot;
                ScanMatchScore_t candidate_score;
                float dx = (float)ix * SCAN_MATCH_DX_STEP_M;
                float dy = (float)iy * SCAN_MATCH_DY_STEP_M;
                float dtheta = (float)itheta * SCAN_MATCH_DTHETA_STEP_DEG;

                candidate.x_m += dx;
                candidate.y_m += dy;
                candidate.theta_deg += dtheta;
                candidate_score = scan_match_score_candidate(&meta, &candidate, point_count);
                tested_candidates++;

                if (candidate_score.score > best_score) {
                    second_score = best_score;
                    best_score = candidate_score.score;
                    best_pose = candidate;
                    best_dx = dx;
                    best_dy = dy;
                    best_dtheta = dtheta;
                    best_hits = candidate_score.occupied_hits;
                    best_on_edge = ((ix == -2) || (ix == 2) ||
                                    (iy == -2) || (iy == 2) ||
                                    (itheta == -2) || (itheta == 2)) ? 1U : 0U;
                } else if (candidate_score.score > second_score) {
                    second_score = candidate_score.score;
                }
            }
        }
    }
    MappingTask_EndGridRead();

    if (result != NULL) {
        result->tested_candidates = tested_candidates;
        result->used_points = point_count;
        result->occupied_hits = best_hits;
        result->best_score = best_score;
        result->second_score = second_score;
        result->score_margin = best_score - second_score;
        result->correction_dx_m = best_dx;
        result->correction_dy_m = best_dy;
        result->correction_dtheta_deg = best_dtheta;
    }

    if (best_score < SCAN_MATCH_MIN_SCORE) {
        if (result != NULL) {
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_LOW_SCORE;
        }
        return 0U;
    }

    if (best_hits < SCAN_MATCH_MIN_OCCUPIED_HITS) {
        if (result != NULL) {
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_TOO_FEW_HITS;
        }
        return 0U;
    }

    if ((best_score - second_score) < SCAN_MATCH_MIN_SCORE_MARGIN) {
        if (result != NULL) {
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_LOW_MARGIN;
        }
        return 0U;
    }

    if ((fabsf(best_dx) > SCAN_MATCH_MAX_ACCEPT_DX_M) ||
        (fabsf(best_dy) > SCAN_MATCH_MAX_ACCEPT_DY_M) ||
        (fabsf(best_dtheta) > SCAN_MATCH_MAX_ACCEPT_DTHETA_DEG)) {
        if (result != NULL) {
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_LARGE_CORRECTION;
        }
        return 0U;
    }

    if (best_on_edge != 0U) {
        if (result != NULL) {
            result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_EDGE_BEST;
        }
        return 0U;
    }

    *corrected_pose = best_pose;
    if (result != NULL) {
        result->success = 1U;
        result->reject_reason = (uint8_t)SCAN_MATCH_REJECT_NONE;
    }
    return 1U;
}
