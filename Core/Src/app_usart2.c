#include "app_usart2.h"
#include "app_bat.h"
#include "app_encoder.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "app_mpu6050.h"
#include "app_motor.h"
#include "main.h"
#include "pid.h"


// 阻塞发送 str 指向的字符串，不发送结尾的空字符。
void App_USART2_SendString(const char *str)
{
    // 1. 按字符串长度阻塞发送，超时参数为 100 ms。
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)str,
        strlen(str),
        100
    );
}


// 输出电池电压，格式为 BAT: x.xxx V，并以回车换行结束。
void App_USART2_PrintBatVoltage(void)
{
    char buffer[64]; // 本次串口输出的文本缓冲区。

    // 1. 读取电池电压估算值。
    float voltage = App_Bat_GetVoltage(); // 电池电压估算值，V。

    // 2. 转为四舍五入的整数 mV，便于用整数格式化。
    uint32_t voltage_mv = // 四舍五入后的电池电压，mV。
        (uint32_t)(voltage * 1000.0f + 0.5f);

    // 3. 拆成伏的整数和三位小数部分。
    snprintf(
        buffer,
        sizeof(buffer),
        "BAT: %lu.%03lu V\r\n",
        (unsigned long)(voltage_mv / 1000),
        (unsigned long)(voltage_mv % 1000)
    );

    // 4. 发送格式化后的电压文本。
    App_USART2_SendString(buffer);
}

// 输出左右轮累计角度，格式为 ENC: L=x.xx, R=x.xx deg。
void App_USART2_PrintEncoderCounts(void)
{
    char buffer[64]; // 本次串口输出的文本缓冲区。

    // 1. 读取左右轮累计角度。
    float pos_l = App_Encoder_GetPos_L(); // 左轮累计角度，deg。
    float pos_r = App_Encoder_GetPos_R(); // 右轮累计角度，deg。


    // 2. 格式化左右角度，保留两位小数。
    snprintf(
        buffer,
        sizeof(buffer),
        "ENC: L=%.2f, R=%.2f deg\r\n",
        pos_l,
        pos_r
    );

    // 3. 发送以回车换行结束的角度文本。
    App_USART2_SendString(buffer);
}

// 输出 ax、ay、az、gx、gy、gz、temp，带字段名并保留三位小数。
void App_USART2_PrintMPU6050Data(void)
{
    char buffer[128]; // 本次串口输出的文本缓冲区。

    // 1. 读取已保存的加速度、角速度和温度。
    float ax = APP_MPU6050_GetAx(); // X 轴加速度，g。
    float ay = APP_MPU6050_GetAy(); // Y 轴加速度，g。
    float az = APP_MPU6050_GetAz(); // Z 轴加速度，g。
    float gx = APP_MPU6050_GetGx(); // X 轴角速度，deg/s。
    float gy = APP_MPU6050_GetGy(); // Y 轴角速度，deg/s。
    float gz = APP_MPU6050_GetGz(); // Z 轴角速度，deg/s。
    float temperature = APP_MPU6050_GetTemperature(); // 温度换算值，摄氏度。

    // 2. 按 ax、ay、az、gx、gy、gz、temp 顺序格式化。
    snprintf(
        buffer,
        sizeof(buffer),
        "MPU6050: ax=%.3f, ay=%.3f, az=%.3f, gx=%.3f, gy=%.3f, gz=%.3f, temp=%.3f\r\n",
        ax, ay, az, gx, gy, gz, temperature
    );

    // 3. 发送以回车换行结束的传感器文本。
    App_USART2_SendString(buffer);
}

// 定时发送十列 MPU6050 CSV 数据，保留三位小数并以回车换行结束。
void App_USART2_PrintMPU6050Data_Proc(void)
{
    static uint32_t nxt = 0; // 下次发送的目标时间，ms。
    uint32_t now = HAL_GetTick(); // 当前系统时间，ms。
    // 1. 未到目标时间则返回；到期后将目标时间推进 10 ms。
    if (now < nxt)
        return;
    nxt = nxt + 10;

    char buffer[128]; // 本次串口输出的文本缓冲区。
    // 2. 读取已保存的传感器值和姿态角。
    float ax = APP_MPU6050_GetAx(); // X 轴加速度，g。
    float ay = APP_MPU6050_GetAy(); // Y 轴加速度，g。
    float az = APP_MPU6050_GetAz(); // Z 轴加速度，g。
    float gx = APP_MPU6050_GetGx(); // X 轴角速度，deg/s。
    float gy = APP_MPU6050_GetGy(); // Y 轴角速度，deg/s。
    float gz = APP_MPU6050_GetGz(); // Z 轴角速度，deg/s。
    float temperature = APP_MPU6050_GetTemperature(); // 温度换算值，摄氏度。
    float yaw = APP_MPU6050_GetYaw(); // 偏航角，deg。
    float pitch = APP_MPU6050_GetPitch(); // 俯仰角，deg。
    float roll = APP_MPU6050_GetRoll(); // 横滚角，deg。

    // 3. CSV 列顺序：ax、ay、az、gx、gy、gz、temp、yaw、pitch、roll。
    snprintf(
        buffer,
        sizeof(buffer),
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
        ax, ay, az,
        gx, gy, gz,
        temperature,
        yaw, pitch, roll
    );
    // 4. 发送格式化后的 CSV 文本。
    App_USART2_SendString(buffer);
}


// 定时发送左右轮实测速度与右轮目标速度，CSV 保留两位小数。
void App_USART2_PrintMotorOmega(void)
{
    static uint32_t nxt = 0; // 下次发送的目标时间，ms。
    uint32_t now = HAL_GetTick(); // 当前系统时间，ms。
    // 1. 未到目标时间则返回；到期后将目标时间推进 10 ms。
    if (now < nxt)
        return;
    nxt = nxt + 10;
    char buffer[64]; // 本次串口输出的文本缓冲区。
    // 2. 读取实测速度和右轮目标速度。
    float omega_l = App_Encoder_GetSpeed_L(); // 左轮实测角速度，rad/s。
    float omega_r = App_Encoder_GetSpeed_R(); // 右轮实测角速度，rad/s。
    float set_r = App_Motor_GetOmegaSet_R(); // 右轮目标角速度，rad/s。

    // 3. CSV 列顺序：左轮实测、右轮实测、右轮目标，均为 rad/s。
    snprintf(
        buffer,
        sizeof(buffer),
        "%.2f,%.2f,%.2f\r\n",
        omega_l,
        omega_r,
        set_r
    );

    // 4. 发送以回车换行结束的 CSV 文本。
    App_USART2_SendString(buffer);
}