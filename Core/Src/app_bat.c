#include "app_bat.h"
#include "adc.h"


uint16_t g_bat_adc_raw = 0; // 最近一次 12 位 ADC 读数，0~4095。

float g_bat_adc_voltage = 0.0f; // PB0 输入电压换算值，V。

float g_bat_voltage = 0.0f; // 电池电压估算值，V。

volatile uint8_t g_bat_data_valid = 0; // 1：已有采样；不表示新鲜度或硬件状态。

static volatile uint8_t s_bat_conversion_active = 0; // 回调完成前禁止重复启动。
static uint32_t s_bat_last_start_tick = 0; // 最近一次成功启动采样的时间，ms。

static void App_Bat_StartSample(uint32_t now)
{
    // 先置位，允许转换中断在 HAL 启动函数返回前完成。
    s_bat_conversion_active = 1;
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) == HAL_OK)
    {
        s_bat_last_start_tick = now;
    }
    else
    {
        s_bat_conversion_active = 0; // 失败时留给后续主循环重试。
    }
}

// 按 led1、led2、led3 的电平设置 PA4、PA5、PA6。
static void App_Bat_LED_Set(
    GPIO_PinState led1, // LED1（PA4）输出电平。
    GPIO_PinState led2, // LED2（PA5）输出电平。
    GPIO_PinState led3) // LED3（PA6）输出电平。
{
    // 1. 依次更新三个 LED 引脚。
    HAL_GPIO_WritePin(
        BAT_LED1_GPIO_Port,
        BAT_LED1_Pin,
        led1
    );

    HAL_GPIO_WritePin(
        BAT_LED2_GPIO_Port,
        BAT_LED2_Pin,
        led2
    );

    HAL_GPIO_WritePin(
        BAT_LED3_GPIO_Port,
        BAT_LED3_Pin,
        led3
    );
}

// 关闭电量灯，校准 ADC 并立即启动一次软件触发的电池采样。
void App_Bat_Init(void)
{
    
    // 1. 上电先关闭三个 LED。
    App_Bat_LED_Set(
        BAT_LED_OFF,
        BAT_LED_OFF,
        BAT_LED_OFF
    );

    // 2. 校准 ADC1。
    HAL_ADCEx_Calibration_Start(&hadc1);

    // 3. 立即采样；后续由主循环按 10 ms 间隔启动。
    s_bat_last_start_tick = HAL_GetTick();
    App_Bat_StartSample(s_bat_last_start_tick);
}

// 主循环调度采样；转换未完成时跳过，避免 HAL 清除待处理的结果。
void App_Bat_Proc(void)
{
    uint32_t now = HAL_GetTick();

    if ((s_bat_conversion_active == 0)
        && ((uint32_t)(now - s_bat_last_start_tick) >= BAT_SAMPLE_INTERVAL))
    {
        App_Bat_StartSample(now);
    }
}


// ADC1 注入转换完成后，保存采样值并换算电池电压。
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) // hadc：完成注入转换的 ADC 句柄。
{
    // 1. 仅处理 ADC1 的电池采样。
    if (hadc->Instance == ADC1)
    {
        // 2. 读取注入组 Rank 1 的原始结果。
        g_bat_adc_raw =
            HAL_ADCEx_InjectedGetValue(
                hadc,
                ADC_INJECTED_RANK_1
            );

        // 3. 按配置的参考电压换算 PB0 电压，单位 V。
        g_bat_adc_voltage =
            ((float)g_bat_adc_raw / BAT_ADC_FULL_SCALE)
            * BAT_ADC_VREF;

        // 4. 按配置的分压系数估算电池电压，单位 V。
        g_bat_voltage =
            g_bat_adc_voltage
            * BAT_DIVIDER_RATIO;

        // 5. 标记已经获得一次采样。
        g_bat_data_valid = 1;

        s_bat_conversion_active = 0;
    }
}



// 返回最近一次 ADC 原始值。
uint16_t App_Bat_GetRaw(void)
{
    return g_bat_adc_raw;
}


// 返回换算后的 PB0 输入电压，单位 V。
float App_Bat_GetADCVoltage(void)
{
    return g_bat_adc_voltage;
}


// 返回按分压系数估算的电池电压，单位 V。
float App_Bat_GetVoltage(void)
{
    return g_bat_voltage;
}

// 根据电池电压显示电量，低压时以 100 ms 间隔翻转三灯。
void App_Bat_LED_Process(void)
{
    static uint8_t stage = 0; // 低压闪烁状态，0：熄灭；1：点亮。


    static uint32_t lastTime = 0; // 上次状态重置或翻转时间，ms。


    // 1. 尚未取得采样时关闭 LED，避免把初始 0 V 当作低压。
    if (g_bat_data_valid == 0)
    {
        App_Bat_LED_Set(
            BAT_LED_OFF,
            BAT_LED_OFF,
            BAT_LED_OFF
        );

        return;
    }


    // 2. 读取已保存的电池电压。
    float vbat = App_Bat_GetVoltage(); // 本次用于分级的电池电压，V。


    // 3. 按电压区间显示：大于 7.9 V 时点亮三灯。
    if (vbat > BAT_VOLTAGE_FULL)
    {
        App_Bat_LED_Set(
            BAT_LED_ON,
            BAT_LED_ON,
            BAT_LED_ON
        );

        // 复位闪烁状态和计时。
        stage = 0;

        lastTime = HAL_GetTick();
    }


    // 7.4 V < 电压 <= 7.9 V：点亮两灯。
    else if (vbat > BAT_VOLTAGE_75)
    {
        App_Bat_LED_Set(
            BAT_LED_ON,
            BAT_LED_ON,
            BAT_LED_OFF
        );

        stage = 0;

        lastTime = HAL_GetTick();
    }


    // 7.0 V < 电压 <= 7.4 V：点亮一灯。
    else if (vbat > BAT_VOLTAGE_50)
    {
        App_Bat_LED_Set(
            BAT_LED_ON,
            BAT_LED_OFF,
            BAT_LED_OFF
        );

        stage = 0;

        lastTime = HAL_GetTick();
    }


    // 6.5 V < 电压 <= 7.0 V：三灯熄灭。
    else if (vbat > BAT_VOLTAGE_LOW)
    {
        App_Bat_LED_Set(
            BAT_LED_OFF,
            BAT_LED_OFF,
            BAT_LED_OFF
        );

        stage = 0;

        lastTime = HAL_GetTick();
    }


    // 4. 电压 <= 6.5 V 时，三灯同时闪烁。
    else
    {
        uint32_t now = HAL_GetTick(); // 当前系统时间，ms。


        // 达到 100 ms 翻转间隔后切换灯状态。
        if ((now - lastTime) >= BAT_LED_BLINK_INTERVAL)
        {
            switch (stage)
            {
                // 三灯从熄灭切换为点亮。
                case 0:
                {
                    App_Bat_LED_Set(
                        BAT_LED_ON,
                        BAT_LED_ON,
                        BAT_LED_ON
                    );

                    stage = 1;

                    break;
                }


                // 三灯从点亮切换为熄灭。
                case 1:
                {
                    App_Bat_LED_Set(
                        BAT_LED_OFF,
                        BAT_LED_OFF,
                        BAT_LED_OFF
                    );

                    stage = 0;

                    break;
                }


                // 未知状态恢复为三灯熄灭。
                default:
                {
                    App_Bat_LED_Set(
                        BAT_LED_OFF,
                        BAT_LED_OFF,
                        BAT_LED_OFF
                    );

                    stage = 0;

                    break;
                }
            }

            // 记录本次翻转时间。
            lastTime = now;
        }
    }
}
