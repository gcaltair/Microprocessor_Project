#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <stdint.h>

#include "slam_types.h"

#define OGM_MAX_WIDTH_CELLS             96U
#define OGM_MAX_HEIGHT_CELLS            96U
#define OGM_MAX_CELL_COUNT              (OGM_MAX_WIDTH_CELLS * OGM_MAX_HEIGHT_CELLS)

#define OGM_UNKNOWN_LOG_ODDS            0
#define OGM_FREE_LOG_ODDS_DELTA         (-3)
#define OGM_OCCUPIED_LOG_ODDS_DELTA     12
#define OGM_MIN_LOG_ODDS                (-100)
#define OGM_MAX_LOG_ODDS                100

typedef struct {
    SlamOccupancyGridSpec_t spec;
    uint8_t initialized;
    int8_t cells[OGM_MAX_CELL_COUNT];
} OccupancyGrid_t;

uint8_t OccupancyGrid_Init(OccupancyGrid_t *grid,
                           uint16_t width_cells,
                           uint16_t height_cells,
                           float resolution_m_per_cell,
                           float origin_x_m,
                           float origin_y_m);
void OccupancyGrid_Reset(OccupancyGrid_t *grid);
uint8_t OccupancyGrid_IsInside(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y);
uint8_t OccupancyGrid_WorldToCell(const OccupancyGrid_t *grid, float x_m, float y_m, SlamGridCoord_t *cell);
uint8_t OccupancyGrid_UpdateCell(OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y, int8_t delta_log_odds);
uint8_t OccupancyGrid_GetCell(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y, int8_t *value);
void OccupancyGrid_MarkFree(OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y);
void OccupancyGrid_MarkOccupied(OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y);
void OccupancyGrid_TraceRay(OccupancyGrid_t *grid,
                            const SlamGridCoord_t *start_cell,
                            const SlamGridCoord_t *end_cell,
                            uint8_t mark_endpoint_occupied);

/* ---- Navigation helpers ---- */
#define OGM_OCC_THRESHOLD   3   /* log-odds >= 3 视为占据 */
#define OGM_FREE_THRESHOLD  (-3) /* log-odds <= -3 视为可通行 */

uint8_t OccupancyGrid_IsOccupied(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y);
uint8_t OccupancyGrid_IsFree(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y);
uint8_t OccupancyGrid_IsUnknown(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y);
uint8_t OccupancyGrid_IsFrontier(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y);
void OccupancyGrid_CellToWorld(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y,
                                float *wx, float *wy);

#endif /* OCCUPANCY_GRID_H */
