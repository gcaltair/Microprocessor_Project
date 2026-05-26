#ifndef SLAM_TYPES_H
#define SLAM_TYPES_H

#include <stdint.h>

/*
 * Shared SLAM-facing data types.
 * Keep these types independent from task code so mapping, localization,
 * and planning can evolve without growing more coupling inside freertos.c.
 */

typedef struct {
    float x_m;
    float y_m;
    float theta_deg;
    uint32_t timestamp_ms;
} SlamPose2D_t;

typedef struct {
    int16_t x;
    int16_t y;
} SlamGridCoord_t;

typedef struct {
    uint16_t width_cells;
    uint16_t height_cells;
    float resolution_m_per_cell;
    float origin_x_m;
    float origin_y_m;
} SlamOccupancyGridSpec_t;

typedef struct {
    float x_m;
    float y_m;
} SlamWaypoint2D_t;

#endif /* SLAM_TYPES_H */
