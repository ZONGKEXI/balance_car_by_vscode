#include "test_voltage.h"
#include "app_bat.h"
#include "app_usart2.h"
#include <stdio.h>

static uint32_t last_print_ms; // 上次输出时间，ms。

// 将电压四舍五入为毫伏，便于整数格式输出。
static uint32_t VoltageToMillivolts(float voltage)
{
    return (uint32_t)(voltage * 1000.0f + 0.5f);
}

// 初始化输出计时，并打印参考电压、分压比和输出间隔。
void Test_Voltage_Init(void)
{
    // 1. 将配置转换为整数，记录计时起点。
    char message[112]; // 串口输出缓冲区。
    uint32_t reference_mv = VoltageToMillivolts(BAT_ADC_VREF); // ADC 参考电压，mV。
    uint32_t divider_x1000 = (uint32_t)(BAT_DIVIDER_RATIO * 1000.0f + 0.5f); // 分压比的千倍值。
    last_print_ms = HAL_GetTick();
    // 2. 格式化配置并发送。
    snprintf(message, sizeof(message),
             "VOLT: configured VREF=%lumV divider=%lu.%03lu; interval=%lums\r\n",
             (unsigned long)reference_mv,
             (unsigned long)(divider_x1000 / 1000U),
             (unsigned long)(divider_x1000 % 1000U),
             (unsigned long)TEST_VOLTAGE_INTERVAL_MS);
    App_USART2_SendString(message);
}

// 主循环定时输出 ADC 原始值、采样端电压和电池电压。
void Test_Voltage_Process(void)
{
    // 1. 达到输出间隔后再读取数据。
    uint32_t now = HAL_GetTick(); // 当前时刻，ms。
    if ((uint32_t)(now - last_print_ms) < TEST_VOLTAGE_INTERVAL_MS)
    {
        return;
    }
    last_print_ms = now;

    // 2. 短暂关中断，取得 ADC 回调更新的同一组数据。
    uint32_t saved_mask = __get_PRIMASK(); // 原中断屏蔽状态。
    __disable_irq();
    uint8_t ready = g_bat_data_valid; // 是否已有采样。
    uint16_t raw = App_Bat_GetRaw(); // ADC 原始计数。
    float adc_voltage = App_Bat_GetADCVoltage(); // ADC 输入端电压，V。
    float battery_voltage = App_Bat_GetVoltage(); // 分压还原后的估算电池电压，V。
    __set_PRIMASK(saved_mask);

    // 3. 恢复中断后，提示等待采样或格式化并发送电压。
    if (ready == 0U)
    {
        App_USART2_SendString("VOLT: waiting for ADC injected sample\r\n");
        return;
    }
    uint32_t adc_mv = VoltageToMillivolts(adc_voltage); // ADC 输入端电压，mV。
    uint32_t battery_mv = VoltageToMillivolts(battery_voltage); // 电池电压，mV。
    char message[112]; // 串口输出缓冲区。
    snprintf(message, sizeof(message),
             "VOLT: raw=%u ADC=%lu.%03luV BAT=%lu.%03luV\r\n",
             (unsigned int)raw,
             (unsigned long)(adc_mv / 1000U),
             (unsigned long)(adc_mv % 1000U),
             (unsigned long)(battery_mv / 1000U),
             (unsigned long)(battery_mv % 1000U));
    App_USART2_SendString(message);
}
