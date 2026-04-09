//
#include "icp_map.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// 全局地图变量
uint8_t g_occ_map[MAP_GRID_NUM][MAP_GRID_NUM];

// --- 静态变量 (仅本文件使用) ---
static Point2D_t g_last_scan[ICP_MAX_POINTS]; // 上一帧雷达点云 (经过降采样)
static uint8_t g_last_scan_cnt = 0;           // 上一帧点数
static uint8_t g_is_first_scan = 1;           // 第一帧标志

/**
 * @brief 初始化地图
 */
void ICP_Map_Init(void) {
    // 清空地图
    memset(g_occ_map, 0, sizeof(g_occ_map));
    // 初始化机器人在地图中心
    // 注意：g_x, g_y, g_th_continuous 已在 encoder.c 中定义
}

/**
 * @brief 建图与定位主函数
 * 调用时机：在 main.c 的主循环中，当 scan_data_ready_flag 置位时调用
 */
void ICP_Map_Update(void) {
    Point2D_t current_raw[ICP_MAX_POINTS]; // 原始点云 (降采样后)
    Point2D_t current_icp[ICP_MAX_POINTS];  // 用于ICP计算的点云 (全局坐标)
    Point2D_t global_pts[ICP_MAX_POINTS]; // 转换到全局坐标的点
    int raw_count = 0;
    float dx=0, dy=0, dth=0; // ICP 计算出的增量

    // 1. 数据预处理: 降采样
    // 雷达数据是极坐标，需要转为笛卡尔坐标，并每隔 N 个点取 1 个
    for (int i = 0; i < point_count; i += (point_count / ICP_MAX_POINTS + 1)) {
        if (raw_count >= ICP_MAX_POINTS) break;

        float angle_rad = lidar_points[i].angle_deg * M_PI / 180.0f;
        // 转换到雷达坐标系 (x向前，y向左)
        current_raw[raw_count].x = lidar_points[i].distance_mm * 0.001f * cosf(angle_rad);
        current_raw[raw_count].y = lidar_points[i].distance_mm * 0.001f * sinf(angle_rad);
        raw_count++;
    }

    // 2. 如果是第一帧，不计算 ICP，直接作为参考
    if (g_is_first_scan) {
        // 将当前帧数据复制给 g_last_scan
        for (int i = 0; i < raw_count; i++) {
            g_last_scan[i] = current_raw[i];
        }
        g_last_scan_cnt = raw_count;
        g_is_first_scan = 0;

        // 第一帧假设位置准确 (0,0)，直接更新地图
        for (int i = 0; i < raw_count; i++) {
            int mx = (int)((g_x + current_raw[i].x) / MAP_RES_M);
            int my = (int)((g_y + current_raw[i].y) / MAP_RES_M);
            if (mx >= 0 && mx < MAP_GRID_NUM && my >= 0 && my < MAP_GRID_NUM) {
                g_occ_map[my][mx] = 1; // 标记障碍物
            }
        }
        return;
    }

    // 3. ICP 配准计算 (核心定位步骤)
    // 将 current_raw 转换到当前里程计估计的位置 (作为 ICP 的初始猜测)
    for (int i = 0; i < raw_count; i++) {
        // 旋转 + 平移
        current_icp[i].x = g_x +
            current_raw[i].x * cosf(g_th_continuous) - current_raw[i].y * sinf(g_th_continuous);
        current_icp[i].y = g_y +
            current_raw[i].x * sinf(g_th_continuous) + current_raw[i].y * cosf(g_th_continuous);
    }

    // 运行 ICP 算法，求解 current_icp 相对于 g_last_scan 的位姿差
    // 这里的 dx, dy, dth 就是修正量
    ICP_Run(current_icp, raw_count, g_last_scan, g_last_scan_cnt, &dx, &dy, &dth);

    // 4. 【定位】更新全局位姿 (融合 ICP 结果)
    // 这一步是定位的核心，它修正了里程计的漂移
    g_x += dx;
    g_y += dy;
    g_th_continuous += dth;

    // 5. 【建图】更新地图与关键帧
    // 只有当移动距离较大时，才更新地图和参考帧 (防止地图模糊)
    static float last_map_x = -100, last_map_y = -100;
    if (fabsf(g_x - last_map_x) > 0.2f || fabsf(g_y - last_map_y) > 0.2f) {
        // 将当前扫描转换到全局坐标并存入地图
        for (int i = 0; i < raw_count; i++) {
            global_pts[i].x = g_x + current_raw[i].x * cosf(g_th_continuous) - current_raw[i].y * sinf(g_th_continuous);
            global_pts[i].y = g_y + current_raw[i].x * sinf(g_th_continuous) + current_raw[i].y * cosf(g_th_continuous);

            int mx = (int)(global_pts[i].x / MAP_RES_M);
            int my = (int)(global_pts[i].y / MAP_RES_M);
            if (mx >= 0 && mx < MAP_GRID_NUM && my >= 0 && my < MAP_GRID_NUM) {
                g_occ_map[my][mx] = 1;
            }
        }
        last_map_x = g_x;
        last_map_y = g_y;

        // 更新 g_last_scan 为当前帧 (作为下一次 ICP 的参考)
        for (int i = 0; i < raw_count && i < ICP_MAX_POINTS; i++) {
            g_last_scan[i] = current_icp[i];
        }
        g_last_scan_cnt = raw_count;
    }
}

/**
 * @brief 简易 ICP 算法实现 (点到点配准)
 * @note 仅适用于小位移，单片机上做了极大简化
 */
void ICP_Run(Point2D_t* curr, int curr_cnt, Point2D_t* last, int last_cnt, float* dx, float* dy, float* dth) {
    *dx = 0; *dy = 0; *dth = 0;

    // --- 1. 寻找对应点 (暴力搜索最近点) ---
    // 为了速度，这里简化：假设 curr 和 last 点数相同且顺序对应
    // (在运动很慢且降采样均匀的情况下近似成立)
    // 实际工程中需要计算距离矩阵找最近点，这里为了单片机性能省略
    if (curr_cnt == 0 || last_cnt == 0) return;

    // --- 2. 计算质心 (均值) ---
    Point2D_t c_curr = {0, 0};
    Point2D_t c_last = {0, 0};

    for (int i = 0; i < curr_cnt; i++) {
        c_curr.x += curr[i].x;
        c_curr.y += curr[i].y;
    }
    c_curr.x /= curr_cnt;
    c_curr.y /= curr_cnt;

    for (int i = 0; i < last_cnt; i++) {
        c_last.x += last[i].x;
        c_last.y += last[i].y;
    }
    c_last.x /= last_cnt;
    c_last.y /= last_cnt;

    // --- 3. 计算协方差矩阵 H (简化的 2x2 矩阵) ---
    // H = sum( (p_i - c_p) * (q_i - c_q)^T )
    float h11 = 0, h12 = 0, h21 = 0, h22 = 0;

    for (int i = 0; i < curr_cnt; i++) {
        int j = i % last_cnt; // 简单对应
        float px = curr[i].x - c_curr.x;
        float py = curr[i].y - c_curr.y;
        float qx = last[j].x - c_last.x;
        float qy = last[j].y - c_last.y;

        h11 += px * qx;
        h12 += px * qy;
        h21 += py * qx;
        h22 += py * qy;
    }

    // --- 4. 求解旋转矩阵 (计算角度) ---
    // 使用简化的公式 (基于迹最大化)
    // 旋转矩阵 R = [cos, -sin; sin, cos]
    // 这里通过求解特征值问题得到最优旋转
    float cos_theta, sin_theta;

    // 简单的 SVD 近似
    // 计算旋转角
    float angle_a = atan2f(h21 - h12, h11 + h22);
    float angle_b = atan2f(h21 + h12, h11 - h22);

    // 选择误差较小的角度
    float cos_a = cosf(angle_a);
    float sin_a = sinf(angle_a);
    float cos_b = cosf(angle_b);
    float sin_b = sinf(angle_b);

    // 这里简单取 angle_a (实际需要计算误差函数)
    *dth = angle_a;

    // --- 5. 计算平移 ---
    // t = c_last - R * c_curr
    *dx = c_last.x - (c_curr.x * cos_a - c_curr.y * sin_a);
    *dy = c_last.y - (c_curr.x * sin_a + c_curr.y * cos_a);
}//
// Created by Administrator on 2026/4/9.
//
