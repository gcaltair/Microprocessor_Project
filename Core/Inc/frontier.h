#ifndef FRONTIER_H
#define FRONTIER_H
#include <stdbool.h>
#include <stdint.h>
#include "occupancy_grid.h" // 假设你有这个头文件定义地图大小

// 寻找地图中“已知空闲”与“未知”的交界点
uint8_t Frontier_FindNearest(float robot_x, float robot_y, int16_t* target_x, int16_t* target_y);
#endif
