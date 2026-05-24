#include <math.h>

#include "encoder.h"
#include "system.h"

#define PULSE_TO_SPEED_FACTOR   (PI * DIAMETER / ENCODER_PULSES_PER_REV / ENCODER_SAMPLING_PERIOD)
#define MAX_REASONABLE_SPEED    0.8f

/* 里程计位置估计，坐标单位为米。通过 Odometry_GetPoseSnapshot() 对外读取。 */
static float s_odom_x = 0.0f;
static float s_odom_y = 0.0f;
/* 连续航向角，单位为度，不限制在 [-180, 180] 范围内。 */
static float s_odom_theta_deg = 0.0f;

volatile float g_left_speed = 0.0f;
volatile float g_right_speed = 0.0f;

/* 两次 Odometry_Update() 之间累计的左右轮位移增量，单位为米。 */
static float s_left_delta_acc_m = 0.0f;
static float s_right_delta_acc_m = 0.0f;
/* 内部积分 x/y 时使用的包角航向，保持在 [-180, 180] 范围附近。 */
static float s_heading_wrapped_deg = 0.0f;
/* 左右轮正反方向的里程计标定比例系数。 */
// static float s_encoder_left_forward_scale = 0.957f;
// static float s_encoder_left_reverse_scale = 0.992f;
// static float s_encoder_right_forward_scale = 0.896f;
// static float s_encoder_right_reverse_scale = 1.030f;
static float s_encoder_left_forward_scale = 1;
static float s_encoder_left_reverse_scale = 1;
static float s_encoder_right_forward_scale = 1;
static float s_encoder_right_reverse_scale = 1;
/* 上一次采样的编码器计数，用于计算下一次脉冲增量。 */
static int16_t last_left_count = 0;
static int16_t last_right_count = 0;
/* encoder_Reset() 之后左右轮累计行驶距离，单位为米，用于调试/标定。 */
static float g_left_distance_total_m = 0.0f;
static float g_right_distance_total_m = 0.0f;
/* 最近一次编码器采样快照，通过 Encoder_GetDebugSnapshot() 对外读取。 */
static EncoderDebugSnapshot_t s_encoder_debug;

static float encoder_apply_odometry_scale(float wheel_speed, float forward_scale, float reverse_scale)
{
    if (wheel_speed >= 0.0f) {
        return wheel_speed * forward_scale;
    }

    return wheel_speed * reverse_scale;
}

void encoder_init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    last_left_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    last_right_count = -(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

void encoder_update_speed(void)
{
    int16_t current_left_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t current_right_count = -(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t left_pulse_delta = current_left_count - last_left_count;
    int16_t right_pulse_delta = current_right_count - last_right_count;
    float raw_left_speed;
    float raw_right_speed;
    float odom_left_speed;
    float odom_right_speed;

    last_left_count = current_left_count;
    last_right_count = current_right_count;

    raw_left_speed = left_pulse_delta * PULSE_TO_SPEED_FACTOR;
    raw_right_speed = right_pulse_delta * PULSE_TO_SPEED_FACTOR;

    if (fabsf(raw_left_speed) > MAX_REASONABLE_SPEED) {
        raw_left_speed = g_left_speed;
    }

    if (fabsf(raw_right_speed) > MAX_REASONABLE_SPEED) {
        raw_right_speed = g_right_speed;
    }

    g_left_speed = raw_left_speed;
    g_right_speed = raw_right_speed;

    odom_left_speed = encoder_apply_odometry_scale(raw_left_speed,
                                                   s_encoder_left_forward_scale,
                                                   s_encoder_left_reverse_scale);
    odom_right_speed = encoder_apply_odometry_scale(raw_right_speed,
                                                    s_encoder_right_forward_scale,
                                                    s_encoder_right_reverse_scale);

    s_left_delta_acc_m += odom_left_speed * ENCODER_SAMPLING_PERIOD;
    s_right_delta_acc_m += odom_right_speed * ENCODER_SAMPLING_PERIOD;
    g_left_distance_total_m += odom_left_speed * ENCODER_SAMPLING_PERIOD;
    g_right_distance_total_m += odom_right_speed * ENCODER_SAMPLING_PERIOD;

    s_encoder_debug.left_pulse_delta = left_pulse_delta;
    s_encoder_debug.right_pulse_delta = right_pulse_delta;
    s_encoder_debug.left_counter_raw = current_left_count;
    s_encoder_debug.right_counter_raw = current_right_count;
    s_encoder_debug.raw_left_speed_mps = raw_left_speed;
    s_encoder_debug.raw_right_speed_mps = raw_right_speed;
    s_encoder_debug.odom_left_speed_mps = odom_left_speed;
    s_encoder_debug.odom_right_speed_mps = odom_right_speed;
}

void encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    last_left_count = 0;
    last_right_count = 0;
    g_left_speed = 0.0f;
    g_right_speed = 0.0f;
    s_left_delta_acc_m = 0.0f;
    s_right_delta_acc_m = 0.0f;
    g_left_distance_total_m = 0.0f;
    g_right_distance_total_m = 0.0f;
    s_encoder_debug.left_pulse_delta = 0;
    s_encoder_debug.right_pulse_delta = 0;
    s_encoder_debug.left_counter_raw = 0;
    s_encoder_debug.right_counter_raw = 0;
    s_encoder_debug.raw_left_speed_mps = 0.0f;
    s_encoder_debug.raw_right_speed_mps = 0.0f;
    s_encoder_debug.odom_left_speed_mps = 0.0f;
    s_encoder_debug.odom_right_speed_mps = 0.0f;
}

void Odometry_ResetPose(void)
{
    s_odom_x = 0.0f;
    s_odom_y = 0.0f;
    s_odom_theta_deg = 0.0f;
    s_heading_wrapped_deg = 0.0f;
    s_left_delta_acc_m = 0.0f;
    s_right_delta_acc_m = 0.0f;
}

uint32_t encoder_left_get_count(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

uint32_t encoder_right_get_count(void)
{
    return __HAL_TIM_GET_COUNTER(&htim1);
}

void Odometry_Update(float dt)
{
    float dl = s_left_delta_acc_m;
    float dr = s_right_delta_acc_m;
    float ds;
    float dth = 0.0f;
    float theta_mid_deg = s_heading_wrapped_deg;

    s_left_delta_acc_m = 0.0f;
    s_right_delta_acc_m = 0.0f;

    ds = (dl + dr) / 2.0f;
    if (fabsf(g_gyro_data.gz) > 1.0f) {
        dth = g_gyro_data.gz * dt;
        theta_mid_deg = s_heading_wrapped_deg + dth * 0.5f;
        s_odom_theta_deg += dth;
        s_heading_wrapped_deg += dth;
    }

    if (s_heading_wrapped_deg > 180.0f) {
        s_heading_wrapped_deg -= 360.0f;
    }

    if (s_heading_wrapped_deg < -180.0f) {
        s_heading_wrapped_deg += 360.0f;
    }

    s_odom_x += ds * cosf(theta_mid_deg * PI / 180.0f);
    s_odom_y += ds * sinf(theta_mid_deg * PI / 180.0f);
}

void Odometry_GetPoseSnapshot(SlamPose2D_t *pose)
{
    if (pose == NULL) {
        return;
    }

    pose->x_m = s_odom_x;
    pose->y_m = s_odom_y;
    pose->theta_deg = s_odom_theta_deg;
    pose->timestamp_ms = HAL_GetTick();
}

void Encoder_GetTravelSnapshot(float *left_distance_m,
                               float *right_distance_m,
                               int16_t *left_counter_raw,
                               int16_t *right_counter_raw)
{
    if (left_distance_m != NULL) {
        *left_distance_m = g_left_distance_total_m;
    }

    if (right_distance_m != NULL) {
        *right_distance_m = g_right_distance_total_m;
    }

    if (left_counter_raw != NULL) {
        *left_counter_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    }

    if (right_counter_raw != NULL) {
        *right_counter_raw = -(int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    }
}

void Encoder_GetCalibration(float *left_forward, float *left_reverse, float *right_forward, float *right_reverse)
{
    if (left_forward != NULL) {
        *left_forward = s_encoder_left_forward_scale;
    }

    if (left_reverse != NULL) {
        *left_reverse = s_encoder_left_reverse_scale;
    }

    if (right_forward != NULL) {
        *right_forward = s_encoder_right_forward_scale;
    }

    if (right_reverse != NULL) {
        *right_reverse = s_encoder_right_reverse_scale;
    }
}

uint8_t Encoder_SetCalibration(float left_forward, float left_reverse, float right_forward, float right_reverse)
{
    if ((left_forward <= 0.0f) ||
        (left_reverse <= 0.0f) ||
        (right_forward <= 0.0f) ||
        (right_reverse <= 0.0f)) {
        return 0U;
    }

    s_encoder_left_forward_scale = left_forward;
    s_encoder_left_reverse_scale = left_reverse;
    s_encoder_right_forward_scale = right_forward;
    s_encoder_right_reverse_scale = right_reverse;
    return 1U;
}

void Encoder_GetDebugSnapshot(EncoderDebugSnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    *snapshot = s_encoder_debug;
}
