#include <string.h>

#include "../Inc/scan_preprocess.h"

const ScanPreprocessConfig_t g_scanPreprocessConfig = {
    .min_distance_mm = 80.0f,
    .max_distance_mm = 3500.0f,
    .min_quality = 8U,
};

uint8_t ScanPreprocess_IsPointUsable(const LidarPoint_t *point)
{
    if (point == NULL) {
        return 0U;
    }

    if (point->quality < g_scanPreprocessConfig.min_quality) {
        return 0U;
    }

    if ((point->distance_mm < g_scanPreprocessConfig.min_distance_mm) ||
        (point->distance_mm > g_scanPreprocessConfig.max_distance_mm)) {
        return 0U;
    }

    return 1U;
}

void ScanPreprocess_Analyze(const LidarPoint_t *points, uint16_t point_count, LidarScanQuality_t *quality)
{
    uint16_t idx;

    if (quality == NULL) {
        return;
    }

    (void)memset(quality, 0, sizeof(*quality));
    quality->raw_point_count = point_count;

    if ((points == NULL) || (point_count == 0U)) {
        return;
    }

    quality->first_angle_deg = points[0].angle_deg;
    quality->last_angle_deg = points[point_count - 1U].angle_deg;

    for (idx = 0U; idx < point_count; ++idx) {
        uint16_t distance_mm = (uint16_t)points[idx].distance_mm;

        if (points[idx].quality < g_scanPreprocessConfig.min_quality) {
            quality->rejected_quality_count++;
            continue;
        }

        if ((points[idx].distance_mm < g_scanPreprocessConfig.min_distance_mm) ||
            (points[idx].distance_mm > g_scanPreprocessConfig.max_distance_mm)) {
            quality->rejected_range_count++;
            continue;
        }

        if (quality->usable_point_count == 0U) {
            quality->min_distance_mm = distance_mm;
            quality->max_distance_mm = distance_mm;
        } else {
            if (distance_mm < quality->min_distance_mm) {
                quality->min_distance_mm = distance_mm;
            }
            if (distance_mm > quality->max_distance_mm) {
                quality->max_distance_mm = distance_mm;
            }
        }

        quality->usable_point_count++;
    }
}
