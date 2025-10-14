//
// Created by G on 25-7-1.
//

#include "motor.h"

// 外部变量声明
extern TIM_HandleTypeDef htim3;

// 电机引脚映射（根据实际硬件连接修改）
// 假设：
// 左电机使用 TIM3 的 CH1(PWM1) 和 CH2(PWM2)
// 右电机使用 TIM3 的 CH3(PWM3) 和 CH4(PWM4)

/**
 * @brief  初始化电机
 * @param  None
 * @retval None
 */
void Motor_Init(void)
{
    // 确保 PWM 通道已启动
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    
    // 初始时停止所有电机
    Motor_StopAll();
}

/**
 * @brief  设置电机速度
 * @param  motor: 电机选择 (MOTOR_LEFT 或 MOTOR_RIGHT)
 * @param  speed: 速度值 (0-100)
 * @retval None
 */
void Motor_SetSpeed(uint8_t motor, uint8_t speed)
{
    // 限制速度范围
    if (speed > 100)
        speed = 100;
    
    if (motor == MOTOR_LEFT)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
    }
    else if (motor == MOTOR_RIGHT)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
    }
}

/**
 * @brief  设置电机方向
 * @param  motor: 电机选择 (MOTOR_LEFT 或 MOTOR_RIGHT)
 * @param  direction: 方向 (MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_STOP)
 * @retval None
 */
void Motor_SetDirection(uint8_t motor, uint8_t direction)
{
    if (motor == MOTOR_LEFT)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                break;
            case MOTOR_BACKWARD:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                break;
            case MOTOR_STOP:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                break;
        }
    }
    else if (motor == MOTOR_RIGHT)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                break;
            case MOTOR_BACKWARD:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                break;
            case MOTOR_STOP:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                break;
        }
    }
}

/**
 * @brief  控制电机速度和方向
 * @param  motor: 电机选择 (MOTOR_LEFT 或 MOTOR_RIGHT)
 * @param  direction: 方向 (MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_STOP)
 * @param  speed: 速度值 (0-100)
 * @retval None
 */
void Motor_Control(uint8_t motor, uint8_t direction, int speed)
{
//左电机CHANNEL4是IN1 CHANEEL3 是IN2

    // 限制速度范围
    if (speed > 10000)
        speed = 10000;
    
    if (motor == MOTOR_RIGHT)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                break;
            case MOTOR_BACKWARD:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                break;
            case MOTOR_STOP:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                break;
        }
    }
    else if (motor == MOTOR_LEFT)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:
                // 修改为实际前进对应的PWM通道
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
                break;
            case MOTOR_BACKWARD:
                // 修改为实际后退对应的PWM通道
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
                break;
            case MOTOR_STOP:
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
                break;
        }
    }
}

/**
 * @brief  停止指定电机
 * @param  motor: 电机选择 (MOTOR_LEFT 或 MOTOR_RIGHT)
 * @retval None
 */
void Motor_Stop(uint8_t motor)
{
    Motor_Control(motor, MOTOR_STOP, 0);
}

/**
 * @brief  停止所有电机
 * @param  None
 * @retval None
 */
void Motor_StopAll(void)
{
    Motor_Stop(MOTOR_LEFT);
    Motor_Stop(MOTOR_RIGHT);
}

/**
 * @brief  小车前进
 * @param  speed: 速度值 (0-100)
 * @retval None
 */
void Car_Forward(uint16_t speed)
{
    Motor_Control(MOTOR_LEFT, MOTOR_FORWARD, speed);
    Motor_Control(MOTOR_RIGHT, MOTOR_FORWARD, speed);
}

/**
 * @brief  小车后退
 * @param  speed: 速度值 (0-100)
 * @retval None
 */
void Car_Backward(uint8_t speed)
{


    Motor_Control(MOTOR_LEFT, MOTOR_BACKWARD, speed);
    Motor_Control(MOTOR_RIGHT, MOTOR_BACKWARD, speed);
}

/**
 * @brief  小车左转
 * @param  speed: 速度值 (0-100)
 * @retval None
 */
void Car_TurnLeft(uint8_t speed)
{
    Motor_Control(MOTOR_LEFT, MOTOR_STOP, 0); // 左电机前进
    Motor_Control(MOTOR_RIGHT, MOTOR_FORWARD, speed);  // 右电机停止
}

/**
 * @brief  小车右转
 * @param  speed: 速度值 (0-100)
 * @retval None
 */
void Car_TurnRight(uint8_t speed)
{

    // 修改为实际左转对应的电机控制
    Motor_Control(MOTOR_LEFT, MOTOR_FORWARD, speed);  // 左电机停止
    Motor_Control(MOTOR_RIGHT, MOTOR_STOP, 0); // 右电机前进

}

/**
 * @brief  小车停止
 * @param  None
 * @retval None
 */
void Car_Stop(void)
{
    Motor_StopAll();
}
void Car_test(uint8_t speed)
{
    // 测试小车运动
    Car_Forward(speed);    // 前进
    HAL_Delay(2000);

    Car_Stop();           // 停止
    HAL_Delay(1000);

    Car_Backward(speed);   // 后退
    HAL_Delay(2000);

    Car_Stop();           // 停止
    HAL_Delay(1000);

    Car_TurnLeft(speed);   // 左转
    HAL_Delay(2000);

    Car_Stop();           // 停止
    HAL_Delay(1000);

    Car_TurnRight(speed);  // 右转
    HAL_Delay(2000);

    Car_Stop();           // 停止
    HAL_Delay(1000);
}
