#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include <stdint.h>

#include "../Inc/freertos_app.h"

typedef enum {
    SCAN_MATCH_REJECT_NONE = 0,
    SCAN_MATCH_REJECT_MAP_NOT_READY = 1,
    SCAN_MATCH_REJECT_SCAN_QUALITY = 2,
    SCAN_MATCH_REJECT_LOW_SCORE = 3,
    SCAN_MATCH_REJECT_LOW_MARGIN = 4,
    SCAN_MATCH_REJECT_TOO_FEW_HITS = 5,
    SCAN_MATCH_REJECT_LARGE_CORRECTION = 6,
    SCAN_MATCH_REJECT_EDGE_BEST = 7
} ScanMatcherRejectReason_t;

typedef struct {
    uint8_t success;
    uint8_t reject_reason;
    uint16_t tested_candidates;
    uint16_t used_points;
    uint16_t occupied_hits;
    float best_score;
    float second_score;
    float score_margin;
    float correction_dx_m;
    float correction_dy_m;
    float correction_dtheta_deg;
} ScanMatcherResult_t;

uint8_t ScanMatcher_CorrectPose(const LidarScanMsg_t *scan_msg,
                                SlamPose2D_t *corrected_pose,
                                ScanMatcherResult_t *result);

#endif /* SCAN_MATCHER_H */
