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

static const float delta_t = 0.01f;
static int16_t last_left_count = 0;
static int16_t last_right_count = 0;

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

    g_dl_acc += g_left_speed * delta_t;
    g_dr_acc += g_right_speed * delta_t;
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
}

uint32_t encoder_left_get_count(void)
{
    return __HAL_TIM_GET_COUNTER(&htim1);
}

uint32_t encoder_right_get_count(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

void Odometry_Update(float dt)
{
    float dl = g_dl_acc;
    float dr = g_dr_acc;
    float ds;
    float dth = 0.0f;

    g_dl_acc = 0.0f;
    g_dr_acc = 0.0f;

    ds = (dl + dr) / 2.0f;
    if (fabsf(g_gyro_data.gz) > 1.0f) {
        dth = g_gyro_data.gz * dt;
        g_th_continuous += dth;
        g_th += dth;
    }

    if (g_th > 180.0f) {
        g_th -= 180.0f;
    }

    if (g_th < -180.0f) {
        g_th += 180.0f;
    }

    g_x += ds * cosf((g_th + dth / 2.0f) * PI / 180.0f);
    g_y += ds * sinf((g_th + dth / 2.0f) * PI / 180.0f);
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
