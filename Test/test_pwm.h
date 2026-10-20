#ifndef TEST_PWM_H
#define TEST_PWM_H

#include <stdint.h>

// 测试占空比使用整数百分比。
#ifndef TEST_PWM_DUTY_PERCENT
#define TEST_PWM_DUTY_PERCENT 20U // 测试占空比，%。
#endif
#ifndef TEST_PWM_RUN_TIME_MS
#define TEST_PWM_RUN_TIME_MS 1500U // 每次电机驱动时间，ms。
#endif
#ifndef TEST_PWM_PAUSE_TIME_MS
#define TEST_PWM_PAUSE_TIME_MS 500U // 两次驱动之间的待机时间，ms。
#endif
#ifndef TEST_PWM_DEBOUNCE_MS
#define TEST_PWM_DEBOUNCE_MS 30U // 按键消抖时间，ms。
#endif

#if TEST_PWM_DUTY_PERCENT < 1U || TEST_PWM_DUTY_PERCENT > 100U
#error "TEST_PWM_DUTY_PERCENT must be an integer between 1 and 100"
#endif
#if TEST_PWM_RUN_TIME_MS == 0U || TEST_PWM_PAUSE_TIME_MS == 0U || TEST_PWM_DEBOUNCE_MS == 0U
#error "PWM test intervals must be greater than zero"
#endif

// MX_GPIO/TIM/UART 初始化和 App_PWM_Init() 完成后使用。
void Test_PWM_Init(void); // 初始化状态，等待按键启动。
void Test_PWM_Process(void); // 主循环调用，处理按键和步骤切换。
// 成功启动返回 1；已运行或方向引脚检查失败返回 0。
uint8_t Test_PWM_Start(void);
void Test_PWM_Stop(void); // 停止测试并关闭电机输出。
uint8_t Test_PWM_IsRunning(void); // 获取测试运行状态。

#endif
