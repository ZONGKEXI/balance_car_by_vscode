#ifndef APP_MPU6050_H
#define APP_MPU6050_H
#include "stm32f1xx_hal.h"

// 复位传感器并配置加速度、陀螺仪量程。
void APP_MPU6050_Init(void);
// 按 5ms 计划更新传感器数据和姿态互补滤波。
void APP_MPU6050_Proc(void);
// 读取寄存器并更新加速度、温度和角速度。
void APP_MPU6050_Update(void);
// 读取X 轴加速度（g）。
float APP_MPU6050_GetAx(void);
// 读取Y 轴加速度（g）。
float APP_MPU6050_GetAy(void);
// 读取Z 轴加速度（g）。
float APP_MPU6050_GetAz(void);
// 读取X 轴角速度（度/s）。
float APP_MPU6050_GetGx(void);
// 读取Y 轴角速度（度/s）。
float APP_MPU6050_GetGy(void);
// 读取Z 轴角速度（度/s）。
float APP_MPU6050_GetGz(void);
// 读取温度（℃）。
float APP_MPU6050_GetTemperature(void);
// 读取偏航角（度）。
float APP_MPU6050_GetYaw(void);
// 读取俯仰角（度）。
float APP_MPU6050_GetPitch(void);
// 读取横滚角（度）。
float APP_MPU6050_GetRoll(void);
// 向 reg 寄存器写入 value，返回通信状态。
HAL_StatusTypeDef APP_MPU6050_WriteReg(uint8_t reg, uint8_t value);
// 读取 reg 寄存器到 value，返回通信状态。
HAL_StatusTypeDef APP_MPU6050_ReadReg(uint8_t reg, uint8_t *value);
#endif