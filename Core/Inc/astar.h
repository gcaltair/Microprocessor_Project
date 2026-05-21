#ifndef ASTAR_H
#define ASTAR_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_ASTAR_NODES 1024  // 根据内存调整，5x5迷宫通常足够
#define MAX_PATH_LENGTH 128   // 路径最大点数

typedef struct {
    int16_t x, y;
    float cost;
    int16_t parent_idx;
} AStarNode_t;

typedef struct {
    float x, y;
} Waypoint_t;

// 输入当前坐标和目标坐标（网格索引），返回路径点的个数
int16_t AStar_Plan(int16_t start_x, int16_t start_y, int16_t goal_x, int16_t goal_y, Waypoint_t* path_out);

#endif