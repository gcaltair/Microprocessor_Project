#ifndef MAPPING_TASK_H
#define MAPPING_TASK_H

#include <stdint.h>

#include "occupancy_grid.h"
#include "slam_types.h"

typedef enum {
    MAPPING_SKIP_REASON_NONE = 0,
    /* 小车正在转弯，雷达帧会产生几何拖影，暂不写地图。 */
    MAPPING_SKIP_REASON_TURNING = 1,
    /* 转弯结束后的短暂稳定等待期，暂不接受扫描写地图。 */
    MAPPING_SKIP_REASON_SETTLE = 2,
    /* 雷达质量不足或里程计跳变过大，本帧不适合建图。 */
    MAPPING_SKIP_REASON_QUALITY = 3
} MappingSkipReason_t;

typedef struct {
    uint8_t grid_initialized;
    uint8_t robot_inside_grid;
    uint8_t map_update_active;
    uint8_t last_skip_reason;
    uint32_t update_count;
    uint32_t last_scan_sequence;
    uint16_t last_usable_points;
    uint16_t last_endpoints_written;
    uint8_t last_localization_mode;
    uint32_t skipped_turning_count;
    uint32_t skipped_settle_count;
    uint32_t skipped_quality_count;
    /* 面向遥测显示的定位字段；纯里程计模式下匹配相关值保持为 0。 */
    float last_odom_delta_theta_deg;
    float last_odom_delta_translation_m;
    SlamGridCoord_t last_robot_cell;
    SlamPose2D_t last_pose;
} MappingTaskStats_t;

typedef struct {
    uint16_t width_cells;
    uint16_t height_cells;
    float resolution_m_per_cell;
    float origin_x_m;
    float origin_y_m;
} MappingGridMeta_t;

/* FreeRTOS 建图线程入口：消费定位后的雷达帧并更新占据栅格。 */
void StartMappingTask(void *argument);
/* 清空当前占据栅格地图和建图统计。 */
void MappingTask_ResetGrid(void);
/* 获取建图统计快照，主要供遥测线程读取。 */
void MappingTask_GetStatsSnapshot(MappingTaskStats_t *stats);
/* 按下采样倍率计算 ASCII/遥测渲染尺寸。 */
void MappingTask_GetRenderDimensions(uint8_t downsample, uint16_t *width, uint16_t *height);
/* 将指定渲染行转换为 ASCII 字符串。 */
uint8_t MappingTask_RenderAsciiRow(uint16_t render_row, uint8_t downsample, char *buffer, uint16_t buffer_size);
/* 读取地图元数据，包括尺寸、分辨率和世界坐标原点。 */
uint8_t MappingTask_GetGridMeta(MappingGridMeta_t *meta);
/* 拷贝整张占据栅格地图。 */
uint8_t MappingTask_CopyGridCells(int8_t *cells_buffer, uint16_t buffer_len);
/* 按行拷贝占据栅格地图，用于分包传输。 */
uint8_t MappingTask_CopyGridRows(uint16_t row_offset,
                                 uint16_t row_count,
                                 int8_t *cells_buffer,
                                 uint16_t buffer_len);
/* 批量只读访问地图时使用，调用方必须 Begin/End 成对出现。 */
uint8_t MappingTask_BeginGridRead(void);
void MappingTask_EndGridRead(void);
uint8_t MappingTask_ReadCellDuringGridRead(int16_t cell_x, int16_t cell_y, int8_t *value);
/* 将世界坐标转换为地图栅格坐标。 */
uint8_t MappingTask_WorldToCell(float x_m, float y_m, SlamGridCoord_t *cell);
/* 查询栅格坐标是否在地图范围内。 */
uint8_t MappingTask_IsCellInside(int16_t cell_x, int16_t cell_y);
/* 查询栅格是否已知为空闲。 */
uint8_t MappingTask_IsCellKnownFree(int16_t cell_x, int16_t cell_y);
/* 查询栅格是否仍未知。 */
uint8_t MappingTask_IsCellUnknown(int16_t cell_x, int16_t cell_y);

#endif /* MAPPING_TASK_H */
