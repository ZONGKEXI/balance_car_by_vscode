// 编码器独立串口测试；外设初始化后单独调用，测试循环不返回。
#include "test_encoder.h"
#include "app_encoder.h"
#include "app_usart2.h"
#include <stdio.h>

// 循环输出左右轮累计角度（度）；阻塞且不返回。
void Encoder_Test(void)
{
    // 1. 提示测试开始，清零累计计数。
    App_USART2_SendString("Encoder Test Start!\r\n");
    App_Encoder_Init();

    while (1)
    {
        // 2. 读取角度，正负号表示转动方向。
        float pos_l = App_Encoder_GetPos_L(); // 左轮累计角度，度。
        float pos_r = App_Encoder_GetPos_R(); // 右轮累计角度，度。
        char buffer[64]; // 串口输出缓冲区。

        // 3. 输出“左角度,右角度”，每次发送后等待 50ms。
        snprintf(buffer, sizeof(buffer), "%.2f,%.2f\r\n", pos_l, pos_r);
        App_USART2_SendString(buffer);
        HAL_Delay(50);
    }
}

static float last_pos_l = 0.0f; // 上次采样的左轮累计角度，度。
static float last_pos_r = 0.0f; // 上次采样的右轮累计角度，度。

// 用相邻角度差和固定 1ms 间隔估算角速度；阻塞且不返回。
void Encoder_M_Method_Test(void)
{
    // 1. 提示测试开始，初始化编码器。
    App_USART2_SendString("Encoder M method Test Start!\r\n");
    App_Encoder_Init();

    while (1)
    {
        // 2. 读取累计角度，计算本轮角度增量。
        float pos_l = App_Encoder_GetPos_L(); // 左轮累计角度，度。
        float pos_r = App_Encoder_GetPos_R(); // 右轮累计角度，度。

        float M_l = pos_l - last_pos_l; // 左轮角度增量，度。
        float M_r = pos_r - last_pos_r; // 右轮角度增量，度。

        // 3. 按 1ms 换算角速度；实际间隔还包含读取和发送耗时。
        float omega_l = M_l / 0.001f; // 左轮估算角速度，度/s。
        float omega_r = M_r / 0.001f; // 右轮估算角速度，度/s。

        // 4. 保存角度，并输出“左角速度,右角速度”（度/s）。
        last_pos_l = pos_l;
        last_pos_r = pos_r;
        char buffer[64]; // 串口输出缓冲区。

        snprintf(buffer, sizeof(buffer), "%.2f,%.2f\r\n", omega_l, omega_r);
        App_USART2_SendString(buffer);
        HAL_Delay(1);
    }
}


// 循环输出编码器接口提供的左右轮角速度；阻塞且不返回。
void Encoder_T_Method_Test(void)
{
    // 1. 提示测试开始，初始化编码器。
    App_USART2_SendString("Encoder T method Test Start!\r\n");
    App_Encoder_Init();
    while (1)
    {
    // 2. 读取左右轮角速度，单位为 rad/s。
    float omega_l = App_Encoder_GetSpeed_L(); // 左轮角速度。
    float omega_r = App_Encoder_GetSpeed_R(); // 右轮角速度。
    char buffer[64]; // 串口输出缓冲区。
    // 3. 发送“左角速度,右角速度”，每次发送后等待 1ms。
    snprintf(buffer, sizeof(buffer), "%.2f,%.2f\r\n", omega_l, omega_r);
    App_USART2_SendString(buffer);
    HAL_Delay(1);
    }
}
