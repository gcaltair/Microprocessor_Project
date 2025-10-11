#include "system.h"

AccelData g_accel_data;
GyroData g_gyro_data;
// 读取MPU6500寄存器的一个字节数据
// reg: 要读取的寄存器地址
// 返回值: 读取到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t tx = reg | 0x80; // 读操作时，寄存器地址最高位需为1
    uint8_t rx = 0;
    MPU6500_CS_LOW(); // 片选拉低，开始SPI通信
    HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY); // 发送寄存器地址
    HAL_SPI_Receive(&hspi2, &rx, 1, HAL_MAX_DELAY);  // 接收数据
    MPU6500_CS_HIGH(); // 片选拉高，结束SPI通信
    return rx; // 返回读取到的数据
}


void MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg & 0x7F; // 写操作时，寄存器地址最高位需为0
    buf[1] = data;       // 要写入的数据
    MPU6500_CS_LOW(); // 片选拉低，开始SPI通信
    HAL_SPI_Transmit(&hspi2, buf, 2, HAL_MAX_DELAY); // 发送寄存器地址和数据
    MPU6500_CS_HIGH(); // 片选拉高，结束SPI通信
}

// 初始化MPU6500传感器
// 返回值: 1表示初始化成功，0表示失败
uint8_t MPU6500_Init(void)
{
    // 复位MPU6500
    MPU_Write_Byte(PWR_MGMT_1, 0x80); // 向电源管理寄存器写入0x80，进行复位
    HAL_Delay(100); // 延时100ms等待复位完成

    // 唤醒MPU6500，设置时钟源为PLL
    MPU_Write_Byte(PWR_MGMT_1, 0x01); // 设置时钟源为X轴陀螺仪PLL
    HAL_Delay(100); // 延时100ms等待稳定

    // 设置采样率分频，0x00表示不分频，采样率最大
    MPU_Write_Byte(SMPLRT_DIV, 0x00);

    // 设置数字低通滤波器，0x01对应带宽184Hz
    MPU_Write_Byte(CONFIG, 0x01);

    // 设置陀螺仪量程为±1000dps
    MPU_Write_Byte(GYRO_CONFIG, GYRO_RANGE_1000DPS << 3);

    // 设置加速度计量程为±2g
    MPU_Write_Byte(ACCEL_CONFIG, ACCEL_RANGE_2G << 3);

    // 读取芯片ID，判断MPU6500是否正常
    uint8_t id = MPU_Read_Byte(WHO_AM_I_REG); // 读取WHO_AM_I寄存器
    if (id == 0x70 || id == 0x68) // 0x70/0x68为MPU6500的合法ID
        return 1; // 初始化成功
    return 0; // 初始化失败
}

// 读取加速度计数据（单位:g），结果存入accel结构体
// accel: 存放加速度计数据的结构体指针
void MPU6500_Read_Accel(AccelData *accel)
{
    transmit("request\r\n");
    uint8_t buf[6]; // 存放读取的6字节原始数据
    MPU6500_CS_LOW(); // 片选拉低，开始SPI通信
    uint8_t reg = ACCEL_XOUT_H | 0x80; // 读操作，寄存器地址最高位为1
    HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY); // 发送寄存器地址
    HAL_SPI_Receive(&hspi2, buf, 6, HAL_MAX_DELAY);   // 连续读取6字节数据
    MPU6500_CS_HIGH(); // 片选拉高，结束SPI通信

    // 合成16位原始加速度数据
    int16_t ax_raw = (int16_t)(buf[0] << 8 | buf[1]); // X轴原始数据
    int16_t ay_raw = (int16_t)(buf[2] << 8 | buf[3]); // Y轴原始数据
    int16_t az_raw = (int16_t)(buf[4] << 8 | buf[5]); // Z轴原始数据

    char str[64];
    // 转换为g单位（±2g时，16384 LSB/g）
    accel->ax = ax_raw / 16384.0f;
    accel->ay = ay_raw / 16384.0f;
    accel->az = az_raw / 16384.0f;


}


void MPU6500_Read_Gyro(GyroData *gyro)
{
    uint8_t buf[6]; // 存放读取的6字节原始数据
    MPU6500_CS_LOW(); // 片选拉低，开始SPI通信
    uint8_t reg = GYRO_XOUT_H | 0x80; // 读操作，寄存器地址最高位为1
    HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY); // 发送寄存器地址
    HAL_SPI_Receive(&hspi2, buf, 6, HAL_MAX_DELAY);   // 连续读取6字节数据
    MPU6500_CS_HIGH(); // 片选拉高，结束SPI通信

    // 合成16位原始陀螺仪数据
    int16_t gx_raw = (int16_t)(buf[0] << 8 | buf[1]); // X轴原始数据
    int16_t gy_raw = (int16_t)(buf[2] << 8 | buf[3]); // Y轴原始数据
    int16_t gz_raw = (int16_t)(buf[4] << 8 | buf[5]); // Z轴原始数据

    // 转换为°/s（±1000dps时，32.8 LSB/(°/s)）
    gyro->gx = gx_raw / 32.8f;
    gyro->gy = gy_raw / 32.8f;
    gyro->gz = gz_raw / 32.8f;
}

// 打印加速度计原始数据到串口（单位:g）
// ax, ay, az: 三轴原始加速度数据（int16_t）
void MPU6500_PrintAccelData(AccelData* accelData) {
    char str[64];
    // 格式化输出字符串
    snprintf(str, sizeof(str), "ACC X:%.2fg Y:%.2fg Z:%.2fg\r\n", accelData->ax, accelData->ay, accelData->az);

    // 通过串口2发送数据
    HAL_UART_Transmit(&huart5, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

// 打印陀螺仪原始数据到串口（单位:°/s）
// gx, gy, gz: 三轴原始角速度数据（int16_t）
void MPU6500_PrintGyroData(GyroData* gyroData) {
    char str[64];

    // 格式化输出字符串
    snprintf(str, sizeof(str), "GYRO X:%.2f /s Y:%.2f /s Z:%.2f /s\r\n", gyroData->gx, gyroData->gy, gyroData->gz);

    // 通过串口2发送数据
    HAL_UART_Transmit(&huart5, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}