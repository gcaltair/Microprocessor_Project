#ifndef FRONTIER_H
#define FRONTIER_H

#include <stdint.h>
#include "occupancy_grid.h" // 假设你有这个头文件定义地图大小

// 寻找地图中“已知空闲”与“未知”的交界点
bool Frontier_FindNearest(int16_t curr_x, int16_t curr_y, int16_t* target_x, int16_t* target_y);

#endif