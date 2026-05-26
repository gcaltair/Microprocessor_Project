//
// Created by G on 25-7-1.
//

#ifndef FINAL_FINA_MOTOR_H
#define FINAL_FINA_MOTOR_H

#include "main.h"

// 电机方向定义
#define MOTOR_FORWARD     0
#define MOTOR_BACKWARD    1
#define MOTOR_STOP        2

// 电机编号定义
#define MOTOR_RIGHT       1
#define MOTOR_LEFT        0

// 电机控制函数声明
void Car_test(uint8_t speed);
void Motor_Init(void);
void Motor_SetSpeed(uint8_t motor, uint16_t speed);
void Motor_SetDirection(uint8_t motor, uint8_t direction);
void Motor_Control(uint8_t motor, uint8_t direction, int speed);
void Motor_Stop(uint8_t motor);
void Motor_StopAll(void);
void Motor_SetEmergencyStop(uint8_t active);
void Motor_EmergencyStopFromIsr(void);
uint8_t Motor_IsEmergencyStopped(void);

// 小车运动控制函数
void Car_Forward(uint16_t speed);
void Car_Backward(uint16_t speed);
void Car_TurnLeft(uint16_t speed);
void Car_TurnRight(uint16_t speed);
void Car_Stop(void);

#endif //FINAL_FINA_MOTOR_H
