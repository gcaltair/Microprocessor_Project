#ifndef __ICP_MAP_H
#define __ICP_MAP_H

#include "system.h"
#include <stdint.h>

// --- 地图参数 ---
#define MAP_SIZE_M        10.0f       // 地图边长 (米)
#define MAP_RES_M         0.05f       // 地图分辨率 (米/格)，5cm
#define MAP_GRID_NUM      200         // 10m / 0.05m = 200格

// --- ICP 参数 ---
#define ICP_MAX_POINTS    30          // ICP算法处理的最大点数 (必须降采样，否则单片机跑不动)
#define ICP_MAX_ITER      10          // 最大迭代次数
#define ICP_DIST_THRESH   0.1f        // 匹配点对的距离阈值

// 地图数据: 0=未知, >0=障碍物
extern uint8_t g_occ_map[MAP_GRID_NUM][MAP_GRID_NUM];

// 点结构
typedef struct {
    float x;
    float y;
} Point2D_t;

// 函数声明
void ICP_Map_Init(void);
void ICP_Map_Update(void);
void ICP_Run(Point2D_t* curr, int curr_cnt, Point2D_t* last, int last_cnt, float* dx, float* dy, float* dth);

#endif//
// Created by Administrator on 2026/4/9.
//

#ifndef FINAL_FINA_ICP_MAP_H
#define FINAL_FINA_ICP_MAP_H

#endif //FINAL_FINA_ICP_MAP_H
