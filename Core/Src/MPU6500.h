#ifndef __MPU6500_H__
#define __MPU6500_H__
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "i2c.h"

// MPU6500 I2C地址 (AD0=GND: 0x68, AD0=VCC: 0x69)
#define MPU6500_I2C_ADDR        0x68
#define MPU6500_I2C_ADDR_WRITE  (MPU6500_I2C_ADDR << 1)       // 0xD0
#define MPU6500_I2C_ADDR_READ   ((MPU6500_I2C_ADDR << 1) | 1)  // 0xD1

// MPU6500寄存器地址定义
#define PWR_MGMT_1      0x6B    // 电源管理1寄存器
#define WHO_AM_I_REG    0x75    // 器件ID寄存器
#define ACCEL_XOUT_H    0x3B    // 加速度X高字节
#define GYRO_XOUT_H     0x43    // 陀螺仪X高字节
#define SMPLRT_DIV      0x19    // 采样率分频寄存器
#define CONFIG         0x1A     // 配���寄存器
#define GYRO_CONFIG    0x1B     // 陀螺仪配置寄存器
#define ACCEL_CONFIG   0x1C     // 加速度计配置寄存器

// 陀螺仪量程配置
#define GYRO_RANGE_250DPS   0x00  // ±250°/s
#define GYRO_RANGE_500DPS   0x01  // ±500°/s
#define GYRO_RANGE_1000DPS  0x02  // ±1000°/s
#define GYRO_RANGE_2000DPS  0x03  // ±2000°/s

// 加速度计量程配置
#define ACCEL_RANGE_2G      0x00  // ±2g
#define ACCEL_RANGE_4G      0x01  // ±4g
#define ACCEL_RANGE_8G      0x02  // ±8g
#define ACCEL_RANGE_16G     0x03  // ±16g

// 陀螺仪数据结构体，单位为°/s
typedef struct {
    float gx; // X轴角速度 (°/s)
    float gy; // Y轴角速度 (°/s)
    float gz; // Z轴角速度 (°/s)
} GyroData;

// 加速度计数据结构体，单位为g
typedef struct {
    float ax; // X轴加速度 (g)
    float ay; // Y轴加速度 (g)
    float az; // Z轴加速度 (g)
} AccelData;

// MPU6500初始化，返回1成功，0失败
uint8_t MPU6500_Init(void);

// 读取加速度计数据，结果存入accel结构体
void MPU6500_Read_Accel(AccelData *accel);

// 读取陀螺仪数据，结果存入gyro结构体
void MPU6500_Read_Gyro(GyroData *gyro);

// 读取MPU6500寄存器的一个字节
uint8_t MPU_Read_Byte(uint8_t reg);

// 向MPU6500寄存器写入一个字节
void MPU_Write_Byte(uint8_t reg, uint8_t data);

void MPU6500_PrintAccelData(AccelData* accelData);

void MPU6500_PrintGyroData(GyroData* gyroData);
void MPU_update();


#endif
