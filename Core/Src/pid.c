#include <math.h>

#include "../Inc/control_logic.h"
#include "freertos_app.h"
#include "pid.h"
#include "system.h"

#define ANGLE_TOLERANCE_FOR_MOVING  3.0f
#define ANGLE_CONTROL_DEADBAND_ENTER_DEG  1.0f
#define ANGLE_CONTROL_DEADBAND_EXIT_DEG   2.0f
#define TURN_OUTPUT_LIMIT_MOVING    0.10f
#define TURN_OUTPUT_LIMIT_INPLACE   0.08f
#define TURN_OUTPUT_MOVING_BASE_RATIO     0.80f
#define POSITION_SLOWDOWN_DISTANCE_M      0.15f
#define POSITION_TERMINAL_MAX_SPEED_MPS   0.08f
#define POSITION_TERMINAL_MIN_SPEED_MPS   0.015f

volatile PID_Controller g_pid_speed_left;
volatile PID_Controller g_pid_speed_right;
volatile PID_Controller g_pid_angle;
volatile PID_Controller g_pid_position;

volatile float base_car_speed = 0.0f;
volatile int pwm_output_left = 0;
volatile int pwm_output_right = 0;

volatile float g_target_x = 0.0f;
volatile float g_target_y = 0.0f;
volatile RelativeMoveState g_relative_move_state = RELATIVE_MOVE_IDLE;
volatile ControlMode g_control_mode = CONTROL_MODE_MANUAL;
volatile float g_relative_move_target_distance_m = 0.0f;
volatile float g_relative_move_progress_m = 0.0f;
volatile float g_relative_move_remaining_m = 0.0f;
volatile float g_control_angle_error_deg = 0.0f;
volatile float g_control_turn_output_mps = 0.0f;
volatile float g_control_position_error_m = 0.0f;
volatile float g_control_left_speed_setpoint_mps = 0.0f;
volatile float g_control_right_speed_setpoint_mps = 0.0f;

static float s_target_distance = 0.0f;
static float s_initial_x = 0.0f;
static float s_initial_y = 0.0f;
static float s_drive_direction = 1.0f;
static float s_drive_axis_x = 1.0f;
static float s_drive_axis_y = 0.0f;
static float s_move_start_left_distance_m = 0.0f;
static float s_move_start_right_distance_m = 0.0f;
static float s_last_move_left_distance_m = 0.0f;
static float s_last_move_right_distance_m = 0.0f;
static float s_last_move_command_distance_m = 0.0f;
static float s_last_move_progress_distance_m = 0.0f;
static ControlDebugSnapshot_t s_last_move_control_snapshot;
static ControlDebugSnapshot_t s_last_turn_control_snapshot;
static uint8_t s_last_move_snapshot_valid = 0U;
static uint8_t s_last_turn_snapshot_valid = 0U;
static uint8_t s_angle_control_active = 0U;

static float pid_normalize_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static float pid_calculate_from_error(PID_Controller *pid, float error, float dt)
{
    float p_out;
    float i_out;
    float d_out;
    float output_float;
    float derivative;

    if (dt <= 0.00001f) {
        return 0.0f;
    }

    p_out = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    i_out = pid->Ki * pid->integral;
    derivative = (error - pid->last_error) / dt;
    d_out = pid->Kd * derivative;
    output_float = p_out + i_out + d_out;

    if (output_float > pid->output_max) {
        output_float = pid->output_max;
    } else if (output_float < pid->output_min) {
        output_float = pid->output_min;
    }

    pid->last_error = error;

    return output_float;
}

static float pid_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static float pid_apply_magnitude_limit(float value, float max_magnitude)
{
    if (max_magnitude < 0.0f) {
        max_magnitude = 0.0f;
    }

    if (value > max_magnitude) {
        return max_magnitude;
    }

    if (value < -max_magnitude) {
        return -max_magnitude;
    }

    return value;
}

static float pid_terminal_speed_limit_from_distance(float distance_error_m)
{
    float ramp;

    if (distance_error_m <= POSITION_REACHED_THRESHOLD) {
        return 0.0f;
    }

    if (distance_error_m >= POSITION_SLOWDOWN_DISTANCE_M) {
        return POSITION_TERMINAL_MAX_SPEED_MPS;
    }

    ramp = (distance_error_m - POSITION_REACHED_THRESHOLD) /
           (POSITION_SLOWDOWN_DISTANCE_M - POSITION_REACHED_THRESHOLD);

    return POSITION_TERMINAL_MIN_SPEED_MPS +
           ramp * (POSITION_TERMINAL_MAX_SPEED_MPS - POSITION_TERMINAL_MIN_SPEED_MPS);
}

static void pid_set_drive_axis_from_heading_deg(float heading_deg)
{
    float heading_rad = heading_deg * PI / 180.0f;

    s_drive_axis_x = cosf(heading_rad);
    s_drive_axis_y = sinf(heading_rad);
}

static void pid_fill_control_snapshot(ControlDebugSnapshot_t *snapshot, float position_error_m)
{
    if (snapshot == NULL) {
        return;
    }

    snapshot->position_error_m = position_error_m;
    snapshot->angle_error_deg = g_control_angle_error_deg;
    snapshot->turn_output_mps = g_control_turn_output_mps;
    snapshot->base_speed_mps = base_car_speed;
    snapshot->left_speed_setpoint_mps = g_control_left_speed_setpoint_mps;
    snapshot->right_speed_setpoint_mps = g_control_right_speed_setpoint_mps;
    snapshot->pwm_left = pwm_output_left;
    snapshot->pwm_right = pwm_output_right;
}

static void pid_capture_last_move_control_snapshot(float position_error_m)
{
    pid_fill_control_snapshot(&s_last_move_control_snapshot, position_error_m);
}

static void pid_capture_last_turn_control_snapshot(void)
{
    pid_fill_control_snapshot(&s_last_turn_control_snapshot, 0.0f);
    s_last_turn_snapshot_valid = 1U;
}

static void pid_get_odometry_pose_snapshot(SlamPose2D_t *pose)
{
    if (pose == NULL) {
        return;
    }

    Odometry_GetPoseSnapshot(pose);
    if (pose->timestamp_ms == 0U) {
        pose->x_m = g_x;
        pose->y_m = g_y;
        pose->theta_deg = g_th_continuous;
        pose->timestamp_ms = HAL_GetTick();
    }
}

static void lock_control_and_pid(void)
{
    if (g_controlMutex != NULL) {
        (void)osMutexAcquire(g_controlMutex, osWaitForever);
    }

    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }
}

static void unlock_pid_and_control(void)
{
    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }

    if (g_controlMutex != NULL) {
        (void)osMutexRelease(g_controlMutex);
    }
}

static void lock_odom_control_and_pid(void)
{
    if (g_odomMutex != NULL) {
        (void)osMutexAcquire(g_odomMutex, osWaitForever);
    }

    lock_control_and_pid();
}

static void unlock_pid_control_and_odom(void)
{
    unlock_pid_and_control();

    if (g_odomMutex != NULL) {
        (void)osMutexRelease(g_odomMutex);
    }
}

static PID_Controller *pid_select_controller(char loop_id)
{
    switch (loop_id) {
        case 'A':
        case 'a':
            return (PID_Controller *)&g_pid_angle;
        case 'L':
        case 'l':
            return (PID_Controller *)&g_pid_speed_left;
        case 'R':
        case 'r':
            return (PID_Controller *)&g_pid_speed_right;
        case 'P':
        case 'p':
            return (PID_Controller *)&g_pid_position;
        default:
            return NULL;
    }
}

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;

    if (pid->Ki > 0.0001f) {
        pid->integral_max = (pid->output_max * 0.5f) / pid->Ki;
    } else {
        pid->integral_max = 0.0f;
    }
}

float PID_Calculate(PID_Controller *pid, float current_value, float dt)
{
    float error;
    float p_out;
    float i_out;
    float d_out;
    float output_float;
    float derivative;

    if (dt <= 0.00001f) {
        return 0.0f;
    }

    if (pid->setpoint == 0.0f) {
        pid->integral = 0.0f;
    }

    error = pid->setpoint - current_value;
    p_out = pid->Kp * error;

    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    i_out = pid->Ki * pid->integral;
    derivative = (error - pid->last_error) / dt;
    d_out = pid->Kd * derivative;
    output_float = p_out + i_out + d_out;

    if (output_float > pid->output_max) {
        output_float = pid->output_max;
    } else if (output_float < pid->output_min) {
        output_float = pid->output_min;
    }

    pid->last_error = error;

    return output_float;
}

void Speed_Control_Loop(float dt)
{
    int raw_pwm_left = (int)PID_Calculate((PID_Controller *)&g_pid_speed_left, g_left_speed, dt);
    int raw_pwm_right = (int)PID_Calculate((PID_Controller *)&g_pid_speed_right, g_right_speed, dt);

    pwm_output_left = raw_pwm_left;
    pwm_output_right = raw_pwm_right;

    if (pwm_output_left > 0) {
        Motor_Control(MOTOR_LEFT, MOTOR_FORWARD, (uint16_t)(pwm_output_left + MOTOR_DEAD_ZONE));
    } else if (pwm_output_left < 0) {
        Motor_Control(MOTOR_LEFT, MOTOR_BACKWARD, (uint16_t)(-pwm_output_left + MOTOR_DEAD_ZONE));
    } else {
        Motor_Control(MOTOR_LEFT, MOTOR_STOP, 0);
    }

    if (pwm_output_right > 0) {
        Motor_Control(MOTOR_RIGHT, MOTOR_FORWARD, (uint16_t)(pwm_output_right + MOTOR_DEAD_ZONE));
    } else if (pwm_output_right < 0) {
        Motor_Control(MOTOR_RIGHT, MOTOR_BACKWARD, (uint16_t)(-pwm_output_right + MOTOR_DEAD_ZONE));
    } else {
        Motor_Control(MOTOR_RIGHT, MOTOR_STOP, 0);
    }
}

void Angle_Speed_Cascade_Control(float angle_current, float base_speed, float dt)
{
    float turn_output = 0.0f;
    float error = pid_normalize_angle_deg(g_pid_angle.setpoint - angle_current);
    float turn_limit = (fabsf(base_speed) > 0.001f) ? TURN_OUTPUT_LIMIT_MOVING : TURN_OUTPUT_LIMIT_INPLACE;
    float abs_base_speed = fabsf(base_speed);
    float abs_error = fabsf(error);

    if (abs_base_speed > 0.001f) {
        float moving_limit = abs_base_speed * TURN_OUTPUT_MOVING_BASE_RATIO;
        if (turn_limit > moving_limit) {
            turn_limit = moving_limit;
        }
    }

    if (s_angle_control_active != 0U) {
        if (abs_error <= ANGLE_CONTROL_DEADBAND_ENTER_DEG) {
            s_angle_control_active = 0U;
        }
    } else if (abs_error >= ANGLE_CONTROL_DEADBAND_EXIT_DEG) {
        s_angle_control_active = 1U;
    }

    if (s_angle_control_active != 0U) {
        turn_output = pid_calculate_from_error((PID_Controller *)&g_pid_angle, error, dt);
        turn_output = pid_clamp_float(turn_output, -turn_limit, turn_limit);
    } else {
        g_pid_angle.integral = 0.0f;
        g_pid_angle.last_error = 0.0f;
    }

    g_pid_speed_left.setpoint = base_speed - turn_output;
    g_pid_speed_right.setpoint = base_speed + turn_output;
    g_control_angle_error_deg = error;
    g_control_turn_output_mps = turn_output;
    g_control_left_speed_setpoint_mps = g_pid_speed_left.setpoint;
    g_control_right_speed_setpoint_mps = g_pid_speed_right.setpoint;

    Speed_Control_Loop(dt);
    if ((abs_base_speed <= 0.001f) && (fabsf(turn_output) > 0.0005f)) {
        pid_capture_last_turn_control_snapshot();
    }
}

uint8_t Control_GetPidTunings(char loop_id, float *kp, float *ki, float *kd)
{
    PID_Controller *pid = pid_select_controller(loop_id);

    if ((pid == NULL) || (kp == NULL) || (ki == NULL) || (kd == NULL)) {
        return 0U;
    }

    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    *kp = pid->Kp;
    *ki = pid->Ki;
    *kd = pid->Kd;

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }

    return 1U;
}

uint8_t Control_SetPidTunings(char loop_id, float kp, float ki, float kd)
{
    PID_Controller *pid = pid_select_controller(loop_id);

    if ((pid == NULL) ||
        (isfinite(kp) == 0) ||
        (isfinite(ki) == 0) ||
        (isfinite(kd) == 0) ||
        (kp < 0.0f) ||
        (ki < 0.0f) ||
        (kd < 0.0f)) {
        return 0U;
    }

    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    if (pid->Ki > 0.0001f) {
        pid->integral_max = (pid->output_max * 0.5f) / pid->Ki;
    } else {
        pid->integral_max = 0.0f;
    }

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }

    return 1U;
}

void Control_SetManualCommand(float command_base_speed, float angle_setpoint)
{
    lock_control_and_pid();

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = command_base_speed;
    g_pid_angle.setpoint = ControlLogic_WrapAngleDeg(angle_setpoint);
    g_relative_move_target_distance_m = 0.0f;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = 0.0f;
    g_target_x = 0.0f;
    g_target_y = 0.0f;
    s_last_move_snapshot_valid = 0U;
    s_last_turn_snapshot_valid = 0U;
    s_angle_control_active = 0U;

    unlock_pid_and_control();
}

void Control_SetManualDrive(float command_base_speed)
{
    SlamPose2D_t pose;

    lock_odom_control_and_pid();
    pid_get_odometry_pose_snapshot(&pose);

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = command_base_speed;
    g_pid_angle.setpoint = pose.theta_deg;
    g_relative_move_target_distance_m = 0.0f;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = 0.0f;
    g_target_x = 0.0f;
    g_target_y = 0.0f;
    s_last_move_snapshot_valid = 0U;
    s_last_turn_snapshot_valid = 0U;
    s_angle_control_active = 0U;

    unlock_pid_control_and_odom();
}

void Control_SetRelativeTurn(float delta_angle)
{
    SlamPose2D_t pose;

    lock_odom_control_and_pid();
    pid_get_odometry_pose_snapshot(&pose);

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = 0.0f;
    g_pid_angle.setpoint = ControlLogic_ResolveAbsoluteSetpointFromCurrentHeading(pose.theta_deg,
                                                                                  delta_angle);
    g_relative_move_target_distance_m = 0.0f;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = 0.0f;
    g_target_x = 0.0f;
    g_target_y = 0.0f;
    s_last_move_snapshot_valid = 0U;
    s_last_turn_snapshot_valid = 0U;
    s_angle_control_active = 0U;

    unlock_pid_control_and_odom();
}

void Control_SetBaseSpeed(float command_base_speed)
{
    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    base_car_speed = command_base_speed;

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }
}

void Control_StopCommand(void)
{
    SlamPose2D_t pose;

    lock_odom_control_and_pid();
    pid_get_odometry_pose_snapshot(&pose);

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = 0.0f;
    s_drive_direction = 1.0f;
    pid_set_drive_axis_from_heading_deg(pose.theta_deg);
    g_pid_angle.setpoint = pose.theta_deg;
    g_relative_move_target_distance_m = 0.0f;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = 0.0f;
    g_target_x = 0.0f;
    g_target_y = 0.0f;
    s_last_move_snapshot_valid = 0U;
    s_last_turn_snapshot_valid = 0U;
    s_angle_control_active = 0U;

    unlock_pid_control_and_odom();
}

void Start_Relative_Move(float dx, float dy)
{
    SlamPose2D_t pose;
    float target_heading_deg;

    lock_odom_control_and_pid();
    pid_get_odometry_pose_snapshot(&pose);

    if (g_relative_move_state != RELATIVE_MOVE_IDLE) {
        unlock_pid_control_and_odom();
        return;
    }

    s_target_distance = sqrtf(dx * dx + dy * dy);
    if (s_target_distance < 0.01f) {
        unlock_pid_control_and_odom();
        return;
    }

    g_target_x = dx;
    g_target_y = dy;
    g_relative_move_target_distance_m = s_target_distance;
    g_relative_move_progress_m = 0.0f;
    g_relative_move_remaining_m = s_target_distance;
    s_last_move_snapshot_valid = 0U;

    target_heading_deg = atan2f(dy, dx) * 180.0f / PI;
    s_drive_direction = 1.0f;
    if (target_heading_deg > 90.0f) {
        target_heading_deg -= 180.0f;
        s_drive_direction = -1.0f;
    } else if (target_heading_deg < -90.0f) {
        target_heading_deg += 180.0f;
        s_drive_direction = -1.0f;
    }

    g_pid_angle.setpoint = pose.theta_deg + target_heading_deg;
    s_initial_x = pose.x_m;
    s_initial_y = pose.y_m;
    pid_set_drive_axis_from_heading_deg(g_pid_angle.setpoint);
    g_pid_position.integral = 0.0f;
    g_pid_position.last_error = 0.0f;
    base_car_speed = 0.0f;
    g_control_mode = CONTROL_MODE_POSITION;
    g_relative_move_state = RELATIVE_MOVE_TURNING;
    g_control_position_error_m = s_target_distance;
    s_angle_control_active = 0U;

    unlock_pid_control_and_odom();
}

void Update_Relative_Move_PID(float dt, const SlamPose2D_t *pose)
{
    float current_angle_deg;
    float current_x_m;
    float current_y_m;
    float last_driving_position_error_m = 0.0f;
    uint8_t capture_driving_control = 0U;

    if (pose != NULL) {
        current_angle_deg = pose->theta_deg;
    } else {
        current_angle_deg = g_th_continuous;
    }

    current_x_m = g_x;
    current_y_m = g_y;

    switch (g_relative_move_state)
    {
        case RELATIVE_MOVE_IDLE:
            base_car_speed = 0.0f;
            g_control_position_error_m = 0.0f;
            break;

        case RELATIVE_MOVE_TURNING:
        {
            float angle_error;

            base_car_speed = 0.0f;
            angle_error = pid_normalize_angle_deg(g_pid_angle.setpoint - current_angle_deg);
            g_control_position_error_m = s_target_distance;

            if (fabsf(angle_error) < ANGLE_TOLERANCE_FOR_MOVING) {
                s_initial_x = current_x_m;
                s_initial_y = current_y_m;
                Encoder_GetTravelSnapshot(&s_move_start_left_distance_m,
                                          &s_move_start_right_distance_m,
                                          NULL,
                                          NULL);
                pid_set_drive_axis_from_heading_deg(current_angle_deg);
                g_relative_move_progress_m = 0.0f;
                g_relative_move_remaining_m = s_target_distance;
                g_pid_position.integral = 0.0f;
                g_pid_position.last_error = 0.0f;
                g_relative_move_state = RELATIVE_MOVE_DRIVING;
            }
            break;
        }

        case RELATIVE_MOVE_DRIVING:
        {
            float dx_traveled = current_x_m - s_initial_x;
            float dy_traveled = current_y_m - s_initial_y;
            float distance_progress = (dx_traveled * s_drive_axis_x + dy_traveled * s_drive_axis_y) * s_drive_direction;
            float distance_error;

            if (distance_progress < 0.0f) {
                distance_progress = 0.0f;
            }

            /*
             * Use commanded-path progress instead of start-to-current Euclidean distance.
             * When the robot drives while trimming heading, arc motion makes straight-line
             * displacement smaller than real path progress and causes consistent overshoot.
             */
            distance_error = s_target_distance - distance_progress;
            g_relative_move_progress_m = distance_progress;
            g_relative_move_remaining_m = (distance_error > 0.0f) ? distance_error : 0.0f;
            g_control_position_error_m = distance_error;

            if (distance_error < POSITION_REACHED_THRESHOLD) {
                float move_end_left_distance_m = 0.0f;
                float move_end_right_distance_m = 0.0f;

                Encoder_GetTravelSnapshot(&move_end_left_distance_m,
                                          &move_end_right_distance_m,
                                          NULL,
                                          NULL);
                s_last_move_left_distance_m = move_end_left_distance_m - s_move_start_left_distance_m;
                s_last_move_right_distance_m = move_end_right_distance_m - s_move_start_right_distance_m;
                s_last_move_command_distance_m = s_drive_direction * s_target_distance;
                s_last_move_progress_distance_m = distance_progress;
                s_last_move_snapshot_valid = 1U;
                g_relative_move_state = RELATIVE_MOVE_IDLE;
                g_control_mode = CONTROL_MODE_MANUAL;
                base_car_speed = 0.0f;
                s_drive_direction = 1.0f;
                g_pid_position.integral = 0.0f;
                g_pid_position.last_error = 0.0f;
                pid_set_drive_axis_from_heading_deg(current_angle_deg);
                g_pid_angle.setpoint = current_angle_deg;
                g_relative_move_remaining_m = 0.0f;
                g_control_position_error_m = 0.0f;
                s_angle_control_active = 0U;
                break;
            }

            g_pid_position.setpoint = 0.0f;
            base_car_speed = s_drive_direction * PID_Calculate((PID_Controller *)&g_pid_position, -distance_error, dt);

            if (base_car_speed > MAX_BASE_SPEED) {
                base_car_speed = MAX_BASE_SPEED;
            } else if (base_car_speed < -MAX_BASE_SPEED) {
                base_car_speed = -MAX_BASE_SPEED;
            }

            if (distance_error <= POSITION_SLOWDOWN_DISTANCE_M) {
                float terminal_speed_limit = pid_terminal_speed_limit_from_distance(distance_error);
                base_car_speed = pid_apply_magnitude_limit(base_car_speed, terminal_speed_limit);
            } else {
                if ((base_car_speed > 0.0f) && (base_car_speed < MIN_BASE_SPEED)) {
                    base_car_speed = MIN_BASE_SPEED;
                } else if ((base_car_speed < 0.0f) && (base_car_speed > -MIN_BASE_SPEED)) {
                    base_car_speed = -MIN_BASE_SPEED;
                }
            }
            last_driving_position_error_m = distance_error;
            capture_driving_control = 1U;
            break;
        }
    }

    Angle_Speed_Cascade_Control(current_angle_deg, base_car_speed, dt);
    if (capture_driving_control != 0U) {
        pid_capture_last_move_control_snapshot(last_driving_position_error_m);
    }
}

uint8_t Control_GetLastRelativeMoveTravelSnapshot(float *left_distance_m,
                                                  float *right_distance_m,
                                                  float *command_distance_m,
                                                  float *progress_distance_m)
{
    if (s_last_move_snapshot_valid == 0U) {
        return 0U;
    }

    if (left_distance_m != NULL) {
        *left_distance_m = s_last_move_left_distance_m;
    }

    if (right_distance_m != NULL) {
        *right_distance_m = s_last_move_right_distance_m;
    }

    if (command_distance_m != NULL) {
        *command_distance_m = s_last_move_command_distance_m;
    }

    if (progress_distance_m != NULL) {
        *progress_distance_m = s_last_move_progress_distance_m;
    }

    return 1U;
}

uint8_t Control_GetLastRelativeMoveControlSnapshot(ControlDebugSnapshot_t *snapshot)
{
    if ((s_last_move_snapshot_valid == 0U) || (snapshot == NULL)) {
        return 0U;
    }

    *snapshot = s_last_move_control_snapshot;
    return 1U;
}

uint8_t Control_GetLastTurnControlSnapshot(ControlDebugSnapshot_t *snapshot)
{
    if ((s_last_turn_snapshot_valid == 0U) || (snapshot == NULL)) {
        return 0U;
    }

    *snapshot = s_last_turn_control_snapshot;
    return 1U;
}
