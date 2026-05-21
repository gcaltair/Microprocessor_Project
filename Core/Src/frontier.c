#include "frontier.h"
#include "occupancy_grid.h"
#include "mapping_task.h"
#include "freertos_app.h"
#include <math.h>

uint8_t Frontier_FindNearest(float robot_x, float robot_y,
                              int16_t *goal_gx, int16_t *goal_gy)
{
    OccupancyGrid_t *grid = MappingTask_GetGrid();
    SlamGridCoord_t robot_cell;
    float best_dist = 1e9f;
    int16_t best_gx = -1, best_gy = -1;
    uint16_t gx, gy;
    uint8_t found = 0U;

    if ((grid == NULL) || (grid->initialized == 0U)) return 0U;

    if (g_gridMutex != NULL) {
        (void)osMutexAcquire(g_gridMutex, osWaitForever);
    }

    if (OccupancyGrid_WorldToCell(grid, robot_x, robot_y, &robot_cell) == 0U) {
        if (g_gridMutex != NULL) {
            (void)osMutexRelease(g_gridMutex);
        }
        return 0U;
    }

    for (gy = 0; gy < grid->spec.height_cells; gy++) {
        for (gx = 0; gx < grid->spec.width_cells; gx++) {
            float dx, dy, d;

            if (!OccupancyGrid_IsFrontier(grid, gx, gy)) continue;

            dx = (float)((int32_t)gx - robot_cell.x);
            dy = (float)((int32_t)gy - robot_cell.y);
            d = sqrtf(dx * dx + dy * dy);

            if (d < best_dist) {
                best_dist = d;
                best_gx = (int16_t)gx;
                best_gy = (int16_t)gy;
                found = 1U;
            }
        }
    }

    if (g_gridMutex != NULL) {
        (void)osMutexRelease(g_gridMutex);
    }

    if (!found) return 0U;

    *goal_gx = best_gx;
    *goal_gy = best_gy;
    return 1U;
}