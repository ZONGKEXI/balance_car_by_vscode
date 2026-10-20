#include "test_mpu6050.h"
#include "app_usart2.h"
#include "app_mpu6050.h"

// 初始化传感器并循环输出加速度、角速度和温度；阻塞且不返回。
void Test_MPU6050_Init(void)
{
    // 1. 提示测试开始，并初始化传感器。
    App_USART2_SendString("MPU6050 Test Start!\r\n");
    APP_MPU6050_Init();
    while(1)
    {
        // 2. 读取并换算本轮传感器数据。
        APP_MPU6050_Update();
        // 3. 发送传感器数据，每次发送后等待 500ms。
        App_USART2_PrintMPU6050Data();  
        HAL_Delay(500);
    }
}

// 初始化传感器并循环输出传感器值和欧拉角；阻塞且不返回。
void Test_EularAngleTest(void)
{
    // 1. 初始化传感器并提示测试开始。
    APP_MPU6050_Init();
    App_USART2_SendString("MPU6050 Eular Angle Test Start!\r\n");
    while(1)
    {
        // 2. 执行姿态更新，内部按固定 5ms 积分。
        APP_MPU6050_Proc(); 
        // 3. 发送传感器值和欧拉角；本测试每轮等待 500ms。
        App_USART2_PrintMPU6050Data_Proc();
        HAL_Delay(500);
    }
}