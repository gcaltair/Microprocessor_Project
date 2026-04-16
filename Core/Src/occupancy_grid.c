#include <stdlib.h>
#include <string.h>

#include "../Inc/occupancy_grid.h"

static uint32_t occupancy_grid_linear_index(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y)
{
    return (uint32_t)cell_y * grid->spec.width_cells + (uint32_t)cell_x;
}

static int8_t clamp_log_odds(int16_t value)
{
    if (value > OGM_MAX_LOG_ODDS) {
        return OGM_MAX_LOG_ODDS;
    }

    if (value < OGM_MIN_LOG_ODDS) {
        return OGM_MIN_LOG_ODDS;
    }

    return (int8_t)value;
}

uint8_t OccupancyGrid_Init(OccupancyGrid_t *grid,
                           uint16_t width_cells,
                           uint16_t height_cells,
                           float resolution_m_per_cell,
                           float origin_x_m,
                           float origin_y_m)
{
    if ((grid == NULL) ||
        (width_cells == 0U) ||
        (height_cells == 0U) ||
        (width_cells > OGM_MAX_WIDTH_CELLS) ||
        (height_cells > OGM_MAX_HEIGHT_CELLS) ||
        (resolution_m_per_cell <= 0.0f)) {
        return 0U;
    }

    (void)memset(grid, 0, sizeof(*grid));
    grid->spec.width_cells = width_cells;
    grid->spec.height_cells = height_cells;
    grid->spec.resolution_m_per_cell = resolution_m_per_cell;
    grid->spec.origin_x_m = origin_x_m;
    grid->spec.origin_y_m = origin_y_m;
    grid->initialized = 1U;

    return 1U;
}

void OccupancyGrid_Reset(OccupancyGrid_t *grid)
{
    uint32_t cell_count;

    if ((grid == NULL) || (grid->initialized == 0U)) {
        return;
    }

    cell_count = (uint32_t)grid->spec.width_cells * grid->spec.height_cells;
    (void)memset(grid->cells, OGM_UNKNOWN_LOG_ODDS, cell_count);
}

uint8_t OccupancyGrid_IsInside(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y)
{
    if ((grid == NULL) || (grid->initialized == 0U)) {
        return 0U;
    }

    if ((cell_x < 0) || (cell_y < 0)) {
        return 0U;
    }

    if ((cell_x >= grid->spec.width_cells) || (cell_y >= grid->spec.height_cells)) {
        return 0U;
    }

    return 1U;
}

uint8_t OccupancyGrid_WorldToCell(const OccupancyGrid_t *grid, float x_m, float y_m, SlamGridCoord_t *cell)
{
    int32_t cell_x;
    int32_t cell_y;

    if ((grid == NULL) || (cell == NULL) || (grid->initialized == 0U)) {
        return 0U;
    }

    cell_x = (int32_t)((x_m - grid->spec.origin_x_m) / grid->spec.resolution_m_per_cell);
    cell_y = (int32_t)((y_m - grid->spec.origin_y_m) / grid->spec.resolution_m_per_cell);

    if (OccupancyGrid_IsInside(grid, cell_x, cell_y) == 0U) {
        return 0U;
    }

    cell->x = (int16_t)cell_x;
    cell->y = (int16_t)cell_y;
    return 1U;
}

uint8_t OccupancyGrid_UpdateCell(OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y, int8_t delta_log_odds)
{
    uint32_t linear_index;
    int16_t updated_value;

    if (OccupancyGrid_IsInside(grid, cell_x, cell_y) == 0U) {
        return 0U;
    }

    linear_index = occupancy_grid_linear_index(grid, cell_x, cell_y);
    updated_value = (int16_t)grid->cells[linear_index] + delta_log_odds;
    grid->cells[linear_index] = clamp_log_odds(updated_value);
    return 1U;
}

uint8_t OccupancyGrid_GetCell(const OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y, int8_t *value)
{
    if ((value == NULL) || (OccupancyGrid_IsInside(grid, cell_x, cell_y) == 0U)) {
        return 0U;
    }

    *value = grid->cells[occupancy_grid_linear_index(grid, cell_x, cell_y)];
    return 1U;
}

void OccupancyGrid_MarkFree(OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y)
{
    (void)OccupancyGrid_UpdateCell(grid, cell_x, cell_y, OGM_FREE_LOG_ODDS_DELTA);
}

void OccupancyGrid_MarkOccupied(OccupancyGrid_t *grid, int32_t cell_x, int32_t cell_y)
{
    (void)OccupancyGrid_UpdateCell(grid, cell_x, cell_y, OGM_OCCUPIED_LOG_ODDS_DELTA);
}

void OccupancyGrid_TraceRay(OccupancyGrid_t *grid,
                            const SlamGridCoord_t *start_cell,
                            const SlamGridCoord_t *end_cell,
                            uint8_t mark_endpoint_occupied)
{
    int32_t x0;
    int32_t y0;
    int32_t x1;
    int32_t y1;
    int32_t dx;
    int32_t dy;
    int32_t sx;
    int32_t sy;
    int32_t err;
    uint32_t remaining_steps;

    if ((grid == NULL) || (start_cell == NULL) || (end_cell == NULL) || (grid->initialized == 0U)) {
        return;
    }

    x0 = start_cell->x;
    y0 = start_cell->y;
    x1 = end_cell->x;
    y1 = end_cell->y;
    dx = abs(x1 - x0);
    sx = (x0 < x1) ? 1 : -1;
    dy = -abs(y1 - y0);
    sy = (y0 < y1) ? 1 : -1;
    err = dx + dy;
    remaining_steps = (uint32_t)dx + (uint32_t)(-dy) + 1U;

    while (remaining_steps-- > 0U) {
        int32_t e2;

        if ((x0 == x1) && (y0 == y1)) {
            if (mark_endpoint_occupied != 0U) {
                OccupancyGrid_MarkOccupied(grid, x0, y0);
            }
            return;
        }

        OccupancyGrid_MarkFree(grid, x0, y0);

        e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}
