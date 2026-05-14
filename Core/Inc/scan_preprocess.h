#ifndef SCAN_PREPROCESS_H
#define SCAN_PREPROCESS_H

#include <stdint.h>

#include "../Src/lidar.h"

typedef struct {
    float min_distance_mm;
    float max_distance_mm;
    uint8_t min_quality;
} ScanPreprocessConfig_t;

typedef struct {
    uint16_t raw_point_count;
    uint16_t usable_point_count;
    uint16_t rejected_range_count;
    uint16_t rejected_quality_count;
    uint16_t min_distance_mm;
    uint16_t max_distance_mm;
    float first_angle_deg;
    float last_angle_deg;
} LidarScanQuality_t;

extern const ScanPreprocessConfig_t g_scanPreprocessConfig;

uint8_t ScanPreprocess_IsPointUsable(const LidarPoint_t *point);
void ScanPreprocess_Analyze(const LidarPoint_t *points, uint16_t point_count, LidarScanQuality_t *quality);
int8_t ScanPreprocess_GetAngleSign(void);
void ScanPreprocess_SetAngleSign(int8_t angle_sign);
float ScanPreprocess_BeamWorldAngleDeg(float pose_theta_deg, float scan_angle_deg);

#endif /* SCAN_PREPROCESS_H */
