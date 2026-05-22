#include "astar.h"
#include "navigation_task.h"
#include "occupancy_grid.h"
#include "mapping_task.h"
#include "freertos_app.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

#define ASTAR_MAX_NODES  512



static AStarNode_t g_nodes[ASTAR_MAX_NODES];
static uint16_t g_node_count = 0;

static int16_t astar_find_or_create(int16_t gx, int16_t gy)
{
    uint16_t i;
    AStarNode_t *n;

    for (i = 0; i < g_node_count; i++) {
        if (g_nodes[i].gx == gx && g_nodes[i].gy == gy)
            return (int16_t)i;
    }
    if (g_node_count >= ASTAR_MAX_NODES) return -1;

    n = &g_nodes[g_node_count];
    n->gx = gx;
    n->gy = gy;
    n->parent_idx = -1;
    n->g = 1e9f;
    n->f = 1e9f;
    n->open = 0;
    n->closed = 0;
    return (int16_t)(g_node_count++);
}

static int16_t astar_pop_best_open(void)
{
    float best_f = 1e9f;
    int16_t best_idx = -1;
    uint16_t i;

    for (i = 0; i < g_node_count; i++) {
        if (g_nodes[i].open && g_nodes[i].f < best_f) {
            best_f = g_nodes[i].f;
            best_idx = (int16_t)i;
        }
    }
    return best_idx;
}

/* 8方向邻居偏移 */
static const int8_t dx8[8] = {1, -1, 0, 0, 1, -1, 1, -1};
static const int8_t dy8[8] = {0, 0, 1, -1, 1, -1, -1, 1};

int16_t AStar_Plan(float start_x, float start_y,
                   int16_t goal_gx, int16_t goal_gy,
                   Waypoint_t *path_out)
{
    OccupancyGrid_t *grid = MappingTask_GetGrid();
    SlamGridCoord_t start_cell;
    int16_t start_gx, start_gy;
    int16_t start_idx;
    int16_t goal_idx = -1;
    Waypoint_t tmp[MAX_PATH_LENGTH];
    int16_t path_len = 0;
    int16_t node;
    int16_t i;

    if ((grid == NULL) || (grid->initialized == 0U) || (path_out == NULL)) {
        return 0;
    }

    if (g_gridMutex != NULL) {
        (void)osMutexAcquire(g_gridMutex, osWaitForever);
    }

    if (OccupancyGrid_WorldToCell(grid, start_x, start_y, &start_cell) == 0U) {
        if (g_gridMutex != NULL) {
            (void)osMutexRelease(g_gridMutex);
        }
        return 0;
    }

    start_gx = start_cell.x;
    start_gy = start_cell.y;

    /* 初始化节点池 */
    g_node_count = 0;
    memset(g_nodes, 0, sizeof(g_nodes));

    start_idx = astar_find_or_create(start_gx, start_gy);
    if (start_idx < 0) {
        if (g_gridMutex != NULL) {
            (void)osMutexRelease(g_gridMutex);
        }
        return 0;
    }

    g_nodes[start_idx].g = 0.0f;
    g_nodes[start_idx].f = 0.0f;
    g_nodes[start_idx].open = 1;

    /* A* 主循环 */
    while (1) {
        int16_t curr = astar_pop_best_open();
        uint8_t d;

        if (curr < 0) break; /* 无路可走 */

        g_nodes[curr].open = 0;
        g_nodes[curr].closed = 1;

        if (g_nodes[curr].gx == goal_gx && g_nodes[curr].gy == goal_gy) {
            goal_idx = curr;
            break;
        }

        for (d = 0; d < 8; d++) {
            int16_t nx = g_nodes[curr].gx + dx8[d];
            int16_t ny = g_nodes[curr].gy + dy8[d];
            int16_t nb_idx;
            float move_cost;
            float new_g;
            float hdx, hdy, h;

            if (OccupancyGrid_IsOccupied(grid, nx, ny)) continue;

            nb_idx = astar_find_or_create(nx, ny);
            if (nb_idx < 0) continue;
            if (g_nodes[nb_idx].closed) continue;

            move_cost = (d < 4) ? 1.0f : 1.414f;
            new_g = g_nodes[curr].g + move_cost;
            hdx = (float)(nx - goal_gx);
            hdy = (float)(ny - goal_gy);
            h = sqrtf(hdx * hdx + hdy * hdy);

            if (new_g < g_nodes[nb_idx].g) {
                g_nodes[nb_idx].g = new_g;
                g_nodes[nb_idx].f = new_g + h;
                g_nodes[nb_idx].parent_idx = curr;
                g_nodes[nb_idx].open = 1;
            }
        }
    }

    if (goal_idx < 0) {
        if (g_gridMutex != NULL) {
            (void)osMutexRelease(g_gridMutex);
        }
        return 0;
    }

    /* 回溯路径，反向存入临时数组 */
    path_len = 0;
    node = goal_idx;
    while (node >= 0 && path_len < MAX_PATH_LENGTH) {
        float wx, wy;
        OccupancyGrid_CellToWorld(grid, g_nodes[node].gx, g_nodes[node].gy, &wx, &wy);
        tmp[path_len].x = wx;
        tmp[path_len].y = wy;
        path_len++;
        node = g_nodes[node].parent_idx;
    }

    if (g_gridMutex != NULL) {
        (void)osMutexRelease(g_gridMutex);
    }

    /* 翻转为正向 */
    for (i = 0; i < path_len; i++) {
        path_out[i] = tmp[path_len - 1 - i];
    }

    return path_len;
}
