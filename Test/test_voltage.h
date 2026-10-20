#ifndef TEST_VOLTAGE_H
#define TEST_VOLTAGE_H

// MX 初始化和 App_Bat_Init() 完成后使用。
#ifndef TEST_VOLTAGE_INTERVAL_MS
#define TEST_VOLTAGE_INTERVAL_MS 500U // 电压输出间隔，ms。
#endif

void Test_Voltage_Init(void); // 初始化计时并打印配置。
void Test_Voltage_Process(void); // 主循环调用，定时打印电压。

#endif
