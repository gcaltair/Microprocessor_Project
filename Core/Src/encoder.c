#include <math.h>

#include "encoder.h"
#include "system.h"

#define PULSE_TO_SPEED_FACTOR   (PI * DIAMETER / ENCODER_PULSES_PER_REV / ENCODER_SAMPLING_PERIOD)
#define MAX_REASONABLE_SPEED    1.2f
#define SPEED_FILTER_ALPHA      0.8f

volatile float g_dl_acc = 0.0f;
volatile float g_dr_acc = 0.0f;
volatile float g_x = 0.0f;
volatile float g_y = 0.0f;
volatile float g_th_continuous = 0.0f;
volatile float g_th = 0.0f;

volatile float g_left_speed = 0.0f;
volatile float g_right_speed = 0.0f;
volatile float g_encoder_left_forward_scale = 1.0f;
volatile float g_encoder_left_reverse_scale = 1.0f;
volatile float g_encoder_right_forward_scale = 1.0f;
volatile float g_encoder_right_reverse_scale = 1.0f;

static const float delta_t = 0.01f;
static int16_t last_left_count = 0;
static int16_t last_right_count = 0;
static float g_left_distance_total_m = 0.0f;
static float g_right_distance_total_m = 0.0f;

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

    g_left_speed = (1.0f - SPEED_FILTER_ALPHA) * g_left_speed + SPEED_FILTER_ALPHA * raw_left_speed;
    g_right_speed = (1.0f - SPEED_FILTER_ALPHA) * g_right_speed + SPEED_FILTER_ALPHA * raw_right_speed;

    odom_left_speed = encoder_apply_odometry_scale(raw_left_speed,
                                                   g_encoder_left_forward_scale,
                                                   g_encoder_left_reverse_scale);
    odom_right_speed = encoder_apply_odometry_scale(raw_right_speed,
                                                    g_encoder_right_forward_scale,
                                                    g_encoder_right_reverse_scale);

    g_dl_acc += odom_left_speed * delta_t;
    g_dr_acc += odom_right_speed * delta_t;
    g_left_distance_total_m += odom_left_speed * delta_t;
    g_right_distance_total_m += odom_right_speed * delta_t;
}

void encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    last_left_count = 0;
    last_right_count = 0;
    g_left_speed = 0.0f;
    g_right_speed = 0.0f;
    g_dl_acc = 0.0f;
    g_dr_acc = 0.0f;
    g_left_distance_total_m = 0.0f;
    g_right_distance_total_m = 0.0f;
}

void Odometry_ResetPose(void)
{
    g_x = 0.0f;
    g_y = 0.0f;
    g_th_continuous = 0.0f;
    g_th = 0.0f;
    g_dl_acc = 0.0f;
    g_dr_acc = 0.0f;
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
    float dl = g_dl_acc;
    float dr = g_dr_acc;
    float ds;
    float dth = 0.0f;
    float theta_mid_deg = g_th;

    g_dl_acc = 0.0f;
    g_dr_acc = 0.0f;

    ds = (dl + dr) / 2.0f;
    if (fabsf(g_gyro_data.gz) > 1.0f) {
        dth = g_gyro_data.gz * dt;
        theta_mid_deg = g_th + dth * 0.5f;
        g_th_continuous += dth;
        g_th += dth;
    }

    if (g_th > 180.0f) {
        g_th -= 360.0f;
    }

    if (g_th < -180.0f) {
        g_th += 360.0f;
    }

    g_x += ds * cosf(theta_mid_deg * PI / 180.0f);
    g_y += ds * sinf(theta_mid_deg * PI / 180.0f);
}

void Odometry_GetPoseSnapshot(SlamPose2D_t *pose)
{
    if (pose == NULL) {
        return;
    }

    pose->x_m = g_x;
    pose->y_m = g_y;
    pose->theta_deg = g_th_continuous;
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
        *left_forward = g_encoder_left_forward_scale;
    }

    if (left_reverse != NULL) {
        *left_reverse = g_encoder_left_reverse_scale;
    }

    if (right_forward != NULL) {
        *right_forward = g_encoder_right_forward_scale;
    }

    if (right_reverse != NULL) {
        *right_reverse = g_encoder_right_reverse_scale;
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

    g_encoder_left_forward_scale = left_forward;
    g_encoder_left_reverse_scale = left_reverse;
    g_encoder_right_forward_scale = right_forward;
    g_encoder_right_reverse_scale = right_reverse;
    return 1U;
}
