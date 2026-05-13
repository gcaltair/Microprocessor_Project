#ifndef __PID_H
#define __PID_H

#include "slam_types.h"

#define SPEED_ERROR_DEADBAND             0.01f
#define ANGLE_CONTROL_DEADBAND           1.0f
#define MOTOR_DEAD_ZONE                  686
#define POSITION_REACHED_THRESHOLD       0.05f
#define MAX_BASE_SPEED                   0.22f
#define MIN_BASE_SPEED                   0.04f
#define SPEED_TEST_MAX_SETPOINT_MPS      0.12f

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float setpoint;

    float integral;
    float last_error;

    float output_min;
    float output_max;
    float integral_max;
} PID_Controller;

typedef struct
{
    float position_error_m;
    float angle_error_deg;
    float turn_output_mps;
    float base_speed_mps;
    float left_speed_setpoint_mps;
    float right_speed_setpoint_mps;
    int pwm_left;
    int pwm_right;
} ControlDebugSnapshot_t;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
float PID_Calculate(PID_Controller *pid, float current_value, float dt);
void Speed_Control_Loop(float dt);
void Angle_Speed_Cascade_Control(float angle_current, float base_speed, float dt);
void Control_UpdateWheelSpeedTest(float dt);
void Update_Relative_Move_PID(float dt, const SlamPose2D_t *pose);
void Start_Relative_Move(float dx, float dy);
void Control_SetManualDrive(float base_speed);

void Control_SetManualCommand(float base_speed, float angle_setpoint);
void Control_SetRelativeTurn(float delta_angle);
void Control_SetBaseSpeed(float base_speed);
uint8_t Control_SetWheelSpeedTest(float left_speed_mps, float right_speed_mps);
void Control_StopCommand(void);
uint8_t Control_GetPidTunings(char loop_id, float *kp, float *ki, float *kd);
uint8_t Control_SetPidTunings(char loop_id, float kp, float ki, float kd);
uint8_t Control_GetLastRelativeMoveTravelSnapshot(float *left_distance_m,
                                                  float *right_distance_m,
                                                  float *command_distance_m,
                                                  float *progress_distance_m);
uint8_t Control_GetLastRelativeMoveControlSnapshot(ControlDebugSnapshot_t *snapshot);
uint8_t Control_GetLastTurnControlSnapshot(ControlDebugSnapshot_t *snapshot);

#endif /* __PID_H */
