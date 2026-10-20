#include "app_button.h"
#include "app_pwm.h"
#include "app_motor.h"
#include "app_control.h"

// PA11 为上拉输入：低电平按下，高电平松开。
#define USER_KEY_PRESSED       GPIO_PIN_RESET
#define USER_KEY_RELEASED      GPIO_PIN_SET

// 按键电平至少稳定 20ms 后才确认状态变化。
#define BUTTON_DEBOUNCE_TIME   20U

static uint8_t pwm_on = 0; // 按键记录的电机使能状态：0 关闭，1 开启。

static GPIO_PinState stable_state = GPIO_PIN_SET; // 消抖后确认的稳定电平。

static GPIO_PinState last_raw_state = GPIO_PIN_SET; // 上一次采样的原始电平。

static uint32_t debounce_tick = 0; // 最近一次原始电平变化时间，单位 ms。

// 功能：单击时复位平衡控制，并切换电机使能；clicks 为点击次数。
static void OnUserKey_Clicked(uint8_t clicks)
{
    if (clicks == 1)
    {
        // 1. 清除旧的平衡控制历史和轮速目标。
        App_Control_Reset();

        // 2. 翻转使能状态，再由电机模块处理启停。
        pwm_on = !pwm_on;

        App_Motor_Cmd(pwm_on);
    }
}

// 功能：记录按键初始电平和消抖时间，初始化使能状态为关闭。
void App_Button_Init(void)
{
    // 1. 读取 PA11，并建立原始状态、稳定状态和计时起点。
    stable_state =
        HAL_GPIO_ReadPin(
            USER_KEY_GPIO_Port,
            USER_KEY_Pin
        );

    last_raw_state = stable_state;

    debounce_tick = HAL_GetTick();

    // 2. 初始化按键模块保存的使能状态。
    pwm_on = 0;
}

// 功能：在主循环中扫描按键，完成消抖并触发单击回调。
void App_Button_Proc(void)
{
    GPIO_PinState raw_state; // 本次采样的原始电平。

    uint32_t now = HAL_GetTick(); // 当前时间，单位 ms。

    // 1. 读取按键当前电平。
    raw_state =
        HAL_GPIO_ReadPin(
            USER_KEY_GPIO_Port,
            USER_KEY_Pin
        );

    // 2. 电平变化时更新采样状态，重新开始消抖计时。
    if (raw_state != last_raw_state)
    {
        last_raw_state = raw_state;

        debounce_tick = now;
    }

    // 3. 电平稳定至少 20ms 后确认变化；仅在确认按下时触发一次回调。
    if ((now - debounce_tick) >= BUTTON_DEBOUNCE_TIME)
    {
        if (stable_state != raw_state)
        {
            stable_state = raw_state;

            if (stable_state == USER_KEY_PRESSED)
            {
                // 当前扫描逻辑只产生单击事件。
                OnUserKey_Clicked(1);
            }
        }
    }
}

// 功能：读取按键模块记录的电机使能状态，0 关闭、1 开启。
uint8_t App_Button_GetMotorEnable(void)
{
    return pwm_on;
}
