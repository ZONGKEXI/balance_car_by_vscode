#include "app_mpu6050.h"
#include "i2c.h"
#include <stdint.h>
#include <math.h>
#include "qmath.h"

#define MPU6050_ADDR (0x68u << 1) // AD0 接低电平时的地址，左移后供 HAL 使用

static float ax, ay, az; // X/Y/Z 轴加速度（g）
static float gx, gy, gz; // X/Y/Z 轴角速度（度/s）
static float temperature; // 按当前换算公式得到的温度（℃）
static float yaw, pitch, roll; // 偏航角、俯仰角、横滚角（度）

// 复位并配置 MPU6050 的量程；I2C1 须已初始化，写入失败进入错误处理。
void APP_MPU6050_Init(void)
{
    // 1. 触发设备复位，并等待复位完成。
    if (APP_MPU6050_WriteReg(0x6B, 0x80) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_Delay(100);

    // 2. 清除睡眠位，唤醒设备。
    if (APP_MPU6050_WriteReg(0x6B, 0x00) != HAL_OK)
    {
        Error_Handler();
    }

    // 3. 将陀螺仪量程设为 ±2000 度/s。
    if (APP_MPU6050_WriteReg(0x1B, 0x18) != HAL_OK)
    {
        Error_Handler();
    }

    // 4. 将加速度计量程设为 ±2g。
    if (APP_MPU6050_WriteReg(0x1C, 0x00) != HAL_OK)
    {
        Error_Handler();
    }
}

// 按 5ms 计划更新姿态：陀螺仪积分后与加速度倾角做互补滤波。
void APP_MPU6050_Proc(void)
{
    // 1. 到达计划时刻后读取传感器，并推进下一次处理时刻。
    static uint32_t nxt = 0; // 下一次姿态处理的计划时刻（ms）
    uint32_t now = HAL_GetTick(); // 当前系统时刻（ms）
    if (now < nxt)
        return;
    nxt = nxt + 5;
    APP_MPU6050_Update();

    // 2. 按 5ms 积分角速度；pitch 使用 X 轴，roll 使用负 Y 轴。
    float yaw_g = yaw + gz *0.005f; // Z 轴积分得到的偏航角（度）
    float pitch_g = pitch + gx *0.005f; // X 轴积分得到的俯仰角（度）
    float roll_g = roll - gy *0.005f; // 负 Y 轴积分得到的横滚角（度）

    // 3. 用加速度的 Y/Z 和 X/Z 分量计算俯仰、横滚倾角。
    float pitch_a = qatan2(ay, az) / 3.1415926f * 180.0f; // 加速度俯仰角（度）
    float roll_a = qatan2(ax, az) / 3.1415926f * 180.0f; // 加速度横滚角（度）

    // 4. 偏航角保留积分结果，俯仰和横滚融合两种估计。
    yaw = yaw_g;
    pitch = 0.95238 * pitch_g + 0.04762 * pitch_a;
    roll = 0.95238 * roll_g + 0.04762 * roll_a;
}

// 读取加速度、温度和角速度寄存器，并换算为对应物理量。
void APP_MPU6050_Update(void)
{
    // 1. 逐字节读取 X/Y/Z 轴加速度。
    uint8_t ax_high, ax_low; // X 轴加速度的高/低字节
    uint8_t ay_high, ay_low; // Y 轴加速度的高/低字节
    uint8_t az_high, az_low; // Z 轴加速度的高/低字节
    APP_MPU6050_ReadReg(0x3B, &ax_high); 
    APP_MPU6050_ReadReg(0x3C, &ax_low);
    APP_MPU6050_ReadReg(0x3D, &ay_high);
    APP_MPU6050_ReadReg(0x3E, &ay_low);
    APP_MPU6050_ReadReg(0x3F, &az_high);
    APP_MPU6050_ReadReg(0x40, &az_low);
    // 2. 合并为有符号原始值，再按 ±2g 量程换算加速度。
    int16_t ax_raw = (int16_t)((uint16_t)ax_high << 8 | (uint16_t)ax_low); // X 轴原始计数
    int16_t ay_raw = (int16_t)((uint16_t)ay_high << 8 | (uint16_t)ay_low); // Y 轴原始计数
    int16_t az_raw = (int16_t)((uint16_t)az_high << 8 | (uint16_t)az_low); // Z 轴原始计数

    ax = (float)ax_raw * 6.1035e-5f;
    ay = (float)ay_raw * 6.1035e-5f;
    az = (float)az_raw * 6.1035e-5f;

    // 3. 读取温度原始值，并按当前公式换算温度。
    uint8_t temp_high, temp_low; // 温度寄存器的高/低字节
    APP_MPU6050_ReadReg(0x41, &temp_high);
    APP_MPU6050_ReadReg(0x42, &temp_low);
    int16_t temperature_raw = (int16_t)((uint16_t)temp_high << 8 | (uint16_t)temp_low); // 有符号温度原始计数
    temperature = (float)temperature_raw / 333.87f + 21.0f;

    // 4. 逐字节读取 X/Y/Z 轴角速度，并合并有符号原始值。
    uint8_t gx_high, gx_low; // X 轴角速度的高/低字节
    uint8_t gy_high, gy_low; // Y 轴角速度的高/低字节
    uint8_t gz_high, gz_low; // Z 轴角速度的高/低字节
    APP_MPU6050_ReadReg(0x43, &gx_high);
    APP_MPU6050_ReadReg(0x44, &gx_low);
    APP_MPU6050_ReadReg(0x45, &gy_high);
    APP_MPU6050_ReadReg(0x46, &gy_low);
    APP_MPU6050_ReadReg(0x47, &gz_high);
    APP_MPU6050_ReadReg(0x48, &gz_low);
    int16_t gx_raw = (int16_t)((uint16_t)gx_high << 8 | (uint16_t)gx_low); // X 轴原始计数
    int16_t gy_raw = (int16_t)((uint16_t)gy_high << 8 | (uint16_t)gy_low); // Y 轴原始计数
    int16_t gz_raw = (int16_t)((uint16_t)gz_high << 8 | (uint16_t)gz_low); // Z 轴原始计数
    // 5. 按 ±2000 度/s 量程换算角速度。
    gx = (float)gx_raw * 6.1035e-2f;
    gy = (float)gy_raw * 6.1035e-2f;
    gz = (float)gz_raw * 6.1035e-2f;

}

// 读取最近一次更新的X 轴加速度（g）。
float APP_MPU6050_GetAx(void)
{
    return ax;
}

// 读取最近一次更新的Y 轴加速度（g）。
float APP_MPU6050_GetAy(void)
{
    return ay;
}

// 读取最近一次更新的Z 轴加速度（g）。
float APP_MPU6050_GetAz(void)
{
    return az;
}

// 读取最近一次更新的X 轴角速度（度/s）。
float APP_MPU6050_GetGx(void)
{
    return gx;
}

// 读取最近一次更新的Y 轴角速度（度/s）。
float APP_MPU6050_GetGy(void)
{
    return gy;
}

// 读取最近一次更新的Z 轴角速度（度/s）。
float APP_MPU6050_GetGz(void)
{
    return gz;
}

// 读取最近一次更新的温度（℃）。
float APP_MPU6050_GetTemperature(void)
{
    return temperature;
}

// 读取最近一次更新的偏航角（度）。
float APP_MPU6050_GetYaw(void)
{
    return yaw;
}

// 读取最近一次更新的俯仰角（度）。
float APP_MPU6050_GetPitch(void)
{
    return pitch;
}

// 读取最近一次更新的横滚角（度）。
float APP_MPU6050_GetRoll(void)
{
    return roll;
}



// 向 reg 指定的寄存器写入 value，返回 I2C 通信状态。
HAL_StatusTypeDef APP_MPU6050_WriteReg(uint8_t reg, uint8_t value)
{

    // 1. 使用 I2C1 向设备的 8 位寄存器地址写入一字节，超时 20ms。
    return HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        20
    );
}

// 从 reg 指定的寄存器读取一字节到 value，返回 I2C 通信状态。
HAL_StatusTypeDef APP_MPU6050_ReadReg(uint8_t reg, uint8_t *value)
{
    // 1. 使用 I2C1 从设备的 8 位寄存器地址读取一字节，超时 20ms。
    return HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        20
    );
}