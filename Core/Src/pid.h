#ifndef __PID_H
#define __PID_H

#include "slam_types.h"

#define SPEED_ERROR_DEADBAND             0.01f
#define ANGLE_CONTROL_DEADBAND           1.0f
#define MOTOR_DEAD_ZONE                  686
#define POSITION_REACHED_THRESHOLD       0.05f
#define MAX_BASE_SPEED                   0.22f
#define MIN_BASE_SPEED                   0.04f

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

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
float PID_Calculate(PID_Controller *pid, float current_value, float dt);
void Speed_Control_Loop(float dt);
void Angle_Speed_Cascade_Control(float angle_current, float base_speed, float dt);
void Update_Relative_Move_PID(float dt, const SlamPose2D_t *pose);
void Start_Relative_Move(float dx, float dy);
void Control_SetManualDrive(float base_speed);

void Control_SetManualCommand(float base_speed, float angle_setpoint);
void Control_SetRelativeTurn(float delta_angle);
void Control_SetBaseSpeed(float base_speed);
void Control_StopCommand(void);

#endif /* __PID_H */
