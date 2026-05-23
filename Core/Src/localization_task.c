#include <math.h>
#include <string.h>

#include "../Inc/localization_task.h"
#include "../Inc/control_logic.h"
#include "../Inc/mapping_task.h"
#include "system.h"

/* 当两帧雷达之间的里程计跳变过大时，暂停写入地图。 */
#define LOCALIZATION_MAP_MAX_ODOM_DELTA_XY_M       0.035f
#define LOCALIZATION_MAP_MAX_ODOM_DELTA_THETA_DEG  4.0f
#define LOCALIZATION_MAP_MIN_USABLE_POINTS         24U
#define LOCALIZATION_MAP_SETTLE_MS                 200U

/* 定位状态；这些快照会被控制、建图和遥测共同读取。 */
static LocalizationTaskStats_t g_localizationStats;
static uint8_t g_lastMappingTurnDetected = 0U;
static uint32_t g_mapSettleUntilMs = 0U;

/* 将角度差限制在 [-180, 180]，方便和建图门限比较。 */
static float localization_wrap_angle_deg(float angle_deg)
{
    /* 正方向超出 180 度时，折回到等价的负角度范围。 */
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    /* 负方向超出 -180 度时，折回到等价的正角度范围。 */
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static void localization_lock(void)
{
    /* 统一封装互斥锁，避免定位状态被多个线程同时读写。 */
    if (g_localizationMutex != NULL) {
        (void)osMutexAcquire(g_localizationMutex, osWaitForever);
    }
}

static void localization_unlock(void)
{
    /* 与 localization_lock 成对使用，释放定位状态互斥锁。 */
    if (g_localizationMutex != NULL) {
        (void)osMutexRelease(g_localizationMutex);
    }
}

static void localization_update_mapping_gate(LidarScanMsg_t *scan_msg)
{
    float odom_delta_x_m = 0.0f;
    float odom_delta_y_m = 0.0f;
    float odom_delta_translation_m = 0.0f;
    float odom_delta_theta_deg = 0.0f;
    uint8_t turning_detected;
    uint8_t settle_active = 0U;
    uint32_t now_ms;
    MappingSkipReason_t skip_reason = MAPPING_SKIP_REASON_NONE;

    /* 空指针保护，避免异常消息导致线程崩溃。 */
    if (scan_msg == NULL) {
        return;
    }

    /*
     * 小车转弯时暂停建图，因为一整帧雷达扫描跨越了一段时间；
     * 如果用同一个位姿解释整帧扫描，墙体会被拉弯、障碍会变厚。
     */
    turning_detected = ControlLogic_ShouldPauseMappingForTurn(scan_msg->pose_snapshot.theta_deg,
                                                              g_pid_angle.setpoint,
                                                              base_car_speed,
                                                              g_left_speed,
                                                              g_right_speed,
                                                              (uint8_t)g_relative_move_state);
    now_ms = HAL_GetTick();

    /* 如果已收到过雷达帧，就比较两帧之间的里程计差值。 */
    if (g_localizationStats.update_count > 0U) {
        odom_delta_x_m = scan_msg->pose_snapshot.x_m - g_localizationStats.last_pose.x_m;
        odom_delta_y_m = scan_msg->pose_snapshot.y_m - g_localizationStats.last_pose.y_m;
        odom_delta_translation_m = sqrtf((odom_delta_x_m * odom_delta_x_m) +
                                         (odom_delta_y_m * odom_delta_y_m));
        odom_delta_theta_deg = fabsf(localization_wrap_angle_deg(scan_msg->pose_snapshot.theta_deg -
                                                                 g_localizationStats.last_pose.theta_deg));
    }

    /* 转弯刚结束时短暂等待，让底盘和雷达扫描几何关系稳定下来。 */
    if ((g_lastMappingTurnDetected != 0U) && (turning_detected == 0U)) {
        g_mapSettleUntilMs = now_ms + LOCALIZATION_MAP_SETTLE_MS;
    }

    /* 判断当前是否仍处在转弯后的稳定等待时间内。 */
    if ((turning_detected == 0U) && ((int32_t)(g_mapSettleUntilMs - now_ms) > 0)) {
        settle_active = 1U;
    }

    /* 把本帧里程计跳变量写入消息，供建图线程和上位机显示。 */
    scan_msg->odom_delta_theta_deg = odom_delta_theta_deg;
    scan_msg->odom_delta_translation_m = odom_delta_translation_m;

    /* 建图线程只在 map_update_allowed 为 1 时真正写栅格。 */
    if (turning_detected != 0U) {
        skip_reason = MAPPING_SKIP_REASON_TURNING;
    } else if (settle_active != 0U) {
        skip_reason = MAPPING_SKIP_REASON_SETTLE;
    } else if ((odom_delta_translation_m > LOCALIZATION_MAP_MAX_ODOM_DELTA_XY_M) ||
               (odom_delta_theta_deg > LOCALIZATION_MAP_MAX_ODOM_DELTA_THETA_DEG) ||
               (scan_msg->quality.usable_point_count < LOCALIZATION_MAP_MIN_USABLE_POINTS)) {
        skip_reason = MAPPING_SKIP_REASON_QUALITY;
    }

    scan_msg->turning_detected = turning_detected;
    scan_msg->map_skip_reason = (uint8_t)skip_reason;
    scan_msg->map_update_allowed = (skip_reason == MAPPING_SKIP_REASON_NONE) ? 1U : 0U;

    /* 记录本帧状态，下一帧用于判断转弯结束和里程计跳变。 */
    g_lastMappingTurnDetected = turning_detected;
}

void LocalizationTask_UpdatePose(const SlamPose2D_t *odom_pose)
{
    /* 外部传入新的编码器里程计位姿时，同步更新定位快照。 */
    if (odom_pose == NULL) {
        return;
    }

    localization_lock();
    g_localizationStats.current_pose = *odom_pose;
    g_localizationStats.initialized = 1U;
    localization_unlock();
}

void LocalizationTask_GetPoseSnapshot(SlamPose2D_t *pose)
{
    /* 给其他模块提供当前位姿的线程安全快照。 */
    if (pose == NULL) {
        return;
    }

    localization_lock();
    if (g_localizationStats.initialized != 0U) {
        *pose = g_localizationStats.current_pose;
    } else {
        (void)memset(pose, 0, sizeof(*pose));
    }
    localization_unlock();
}



static void localization_update_stats(const LidarScanMsg_t *scan_msg)
{
    /* 更新定位统计信息，供遥测协议和上位机显示当前定位状态。 */
    localization_lock();
    g_localizationStats.update_count++;
    g_localizationStats.initialized = 1U;
    g_localizationStats.last_pose = scan_msg->pose_snapshot;
    g_localizationStats.last_odom_delta_theta_deg = scan_msg->odom_delta_theta_deg;
    g_localizationStats.last_odom_delta_translation_m = scan_msg->odom_delta_translation_m;
    g_localizationStats.last_map_update_allowed = scan_msg->map_update_allowed;
    g_localizationStats.last_map_skip_reason = scan_msg->map_skip_reason;
    g_localizationStats.last_turning_detected = scan_msg->turning_detected;
    localization_unlock();
}

void StartLocalizationTask(void *argument)
{
    LidarScanMsg_t scan_msg;
    (void)argument;
    (void)memset(&scan_msg, 0, sizeof(scan_msg));
    LocalizationTask_Reset();

    for (;;) {
        if (osMessageQueueGet(g_lidarResultQueue, &scan_msg, NULL, osWaitForever) != osOK) {
            continue;
        }

        // 1. 线程安全地更新发布位姿
        localization_lock();
        g_localizationStats.current_pose = scan_msg.pose_snapshot;
        g_localizationStats.initialized = 1U;
        localization_unlock();

        // 2. 运行建图门控（计算转弯、跳变，动态给 map_update_allowed 赋值）
        localization_update_mapping_gate(&scan_msg);

        // 3. 更新统计并发送给建图任务
        localization_update_stats(&scan_msg);

        if (g_localizedScanQueue != NULL) {
            (void)osMessageQueuePut(g_localizedScanQueue, &scan_msg, 0U, osWaitForever);
        } else {
            uint8_t free_idx = scan_msg.scan_index;
            g_lidarScanBuf[free_idx].point_count = 0U;
            (void)osMessageQueuePut(g_lidarFreeQueue, &free_idx, 0U, osWaitForever);
        }
    }
}
void LocalizationTask_Reset(void)
{
    /* 清空定位统计和建图门控历史。 */
    localization_lock();
    (void)memset(&g_localizationStats, 0, sizeof(g_localizationStats));
    g_lastMappingTurnDetected = 0U;
    g_mapSettleUntilMs = 0U;
    localization_unlock();
}

void LocalizationTask_GetStatsSnapshot(LocalizationTaskStats_t *stats)
{
    /* 复制一份定位统计快照，避免调用方长时间占用互斥锁。 */
    if (stats == NULL) {
        return;
    }

    localization_lock();
    *stats = g_localizationStats;
    localization_unlock();
}
