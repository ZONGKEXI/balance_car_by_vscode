#ifndef __APP_BAT_H
#define __APP_BAT_H

#include "main.h"

// 电压换算使用的参考电压，V。
#define BAT_ADC_VREF          3.3f

// 12 位 ADC 满量程码值。
#define BAT_ADC_FULL_SCALE    4095.0f

// 配置的分压换算系数：(R1 + R2) / R2，用于估算电池电压。
#define BAT_DIVIDER_RATIO     2.5f

// 主循环启动注入采样的间隔，ms。
#define BAT_SAMPLE_INTERVAL   10U

// 电池电压分级阈值，单位 V。

// 大于 7.9 V：三灯亮。
#define BAT_VOLTAGE_FULL          7.9f

// 7.4 V < 电压 <= 7.9 V：两灯亮。
#define BAT_VOLTAGE_75            7.4f

// 7.0 V < 电压 <= 7.4 V：一灯亮。
#define BAT_VOLTAGE_50            7.0f

// 6.5 V < 电压 <= 7.0 V：全灭；<= 6.5 V：闪烁。
#define BAT_VOLTAGE_LOW           6.5f


// 电量 LED 定义。

// LED1、LED2、LED3 分别使用 PA4、PA5、PA6。


// 当前配置为高电平点亮、低电平熄灭。
#define BAT_LED_ON                GPIO_PIN_SET
#define BAT_LED_OFF               GPIO_PIN_RESET

// 低压报警状态翻转间隔，ms；完整闪烁周期约 200 ms。
#define BAT_LED_BLINK_INTERVAL    100U

extern uint16_t g_bat_adc_raw; // 最近一次 ADC 原始读数。
extern float g_bat_adc_voltage; // PB0 输入电压换算值，V。
extern float g_bat_voltage; // 电池电压估算值，V。
extern volatile uint8_t g_bat_data_valid; // 1：已有一次采样；不表示新鲜度或硬件状态。


// 校准 ADC 并立即启动一次软件触发的电池采样。
void App_Bat_Init(void);

// 主循环按 10 ms 间隔启动采样，转换完成前不重复启动。
void App_Bat_Proc(void);

// 返回最近一次 ADC 原始值。
uint16_t App_Bat_GetRaw(void);

// 返回 PB0 电压换算值，V。
float App_Bat_GetADCVoltage(void);

// 返回电池电压估算值，V。
float App_Bat_GetVoltage(void);

// 按电压分级更新电量 LED，低压时闪烁。
void App_Bat_LED_Process(void);

#endif
