#include <math.h>

#include "freertos_app.h"
#include "pid.h"
#include "system.h"

#define ANGLE_TOLERANCE_FOR_MOVING  3.0f

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

static float s_target_distance = 0.0f;
static float s_initial_x = 0.0f;
static float s_initial_y = 0.0f;

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
    float error = g_pid_angle.setpoint - angle_current;

    if (fabsf(error) > ANGLE_CONTROL_DEADBAND) {
        turn_output = PID_Calculate((PID_Controller *)&g_pid_angle, angle_current, dt);
    }

    g_pid_speed_left.setpoint = base_speed - turn_output;
    g_pid_speed_right.setpoint = base_speed + turn_output;

    Speed_Control_Loop(dt);
}

void Control_SetManualCommand(float command_base_speed, float angle_setpoint)
{
    lock_control_and_pid();

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = command_base_speed;
    g_pid_angle.setpoint = angle_setpoint;

    unlock_pid_and_control();
}

void Control_SetRelativeTurn(float delta_angle)
{
    lock_control_and_pid();

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = 0.0f;
    g_pid_angle.setpoint += delta_angle;

    unlock_pid_and_control();
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
    lock_odom_control_and_pid();

    g_relative_move_state = RELATIVE_MOVE_IDLE;
    g_control_mode = CONTROL_MODE_MANUAL;
    base_car_speed = 0.0f;
    g_pid_angle.setpoint = g_th_continuous;

    unlock_pid_control_and_odom();
}

void Start_Relative_Move(float dx, float dy)
{
    lock_odom_control_and_pid();

    if (g_relative_move_state != RELATIVE_MOVE_IDLE) {
        unlock_pid_control_and_odom();
        return;
    }

    s_target_distance = sqrtf(dx * dx + dy * dy);
    if (s_target_distance < 0.01f) {
        unlock_pid_control_and_odom();
        return;
    }

    g_pid_angle.setpoint = g_th_continuous + atan2f(dy, dx) * 180.0f / PI;
    s_initial_x = g_x;
    s_initial_y = g_y;
    base_car_speed = 0.0f;
    g_control_mode = CONTROL_MODE_POSITION;
    g_relative_move_state = RELATIVE_MOVE_TURNING;

    unlock_pid_control_and_odom();
}

void Update_Relative_Move_PID(float dt)
{
    float current_angle_deg = g_th_continuous;

    switch (g_relative_move_state)
    {
        case RELATIVE_MOVE_IDLE:
            base_car_speed = 0.0f;
            break;

        case RELATIVE_MOVE_TURNING:
        {
            float angle_error;

            base_car_speed = 0.0f;
            angle_error = g_pid_angle.setpoint - current_angle_deg;

            while (angle_error > 180.0f) {
                angle_error -= 360.0f;
            }

            while (angle_error < -180.0f) {
                angle_error += 360.0f;
            }

            if (fabsf(angle_error) < ANGLE_TOLERANCE_FOR_MOVING) {
                g_relative_move_state = RELATIVE_MOVE_DRIVING;
            }
            break;
        }

        case RELATIVE_MOVE_DRIVING:
        {
            float dx_traveled = g_x - s_initial_x;
            float dy_traveled = g_y - s_initial_y;
            float distance_traveled = sqrtf(dx_traveled * dx_traveled + dy_traveled * dy_traveled);
            float distance_error = s_target_distance - distance_traveled;

            if (distance_error < POSITION_REACHED_THRESHOLD) {
                g_relative_move_state = RELATIVE_MOVE_IDLE;
                g_control_mode = CONTROL_MODE_MANUAL;
                base_car_speed = 0.0f;
                break;
            }

            g_pid_position.setpoint = 0.0f;
            base_car_speed = PID_Calculate((PID_Controller *)&g_pid_position, -distance_error, dt);

            if (base_car_speed > MAX_BASE_SPEED) {
                base_car_speed = MAX_BASE_SPEED;
            }

            if (base_car_speed < MIN_BASE_SPEED) {
                base_car_speed = MIN_BASE_SPEED;
            }
            break;
        }
    }

    Angle_Speed_Cascade_Control(current_angle_deg, base_car_speed, dt);
}
