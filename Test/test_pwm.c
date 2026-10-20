#include "test_pwm.h"
#include "app_pwm.h"
#include "app_usart2.h"
#include <stdio.h>

// 单个电机测试步骤的输出和持续时间。
typedef struct
{
    float left_duty; // 左轮有符号占空比，%。
    float right_duty; // 右轮有符号占空比，%。
    uint32_t duration_ms; // 步骤持续时间，ms。
    const char *name; // 串口显示的步骤名称。
} PWM_TestStep;

// 依次测试左轮、右轮和双轮的正负驱动；实际转向需实测。
static const PWM_TestStep steps[] =
{
    { (float)TEST_PWM_DUTY_PERCENT, 0.0f, TEST_PWM_RUN_TIME_MS, "LEFT positive" },
    { 0.0f, 0.0f, TEST_PWM_PAUSE_TIME_MS, "standby" },
    { -(float)TEST_PWM_DUTY_PERCENT, 0.0f, TEST_PWM_RUN_TIME_MS, "LEFT negative" },
    { 0.0f, 0.0f, TEST_PWM_PAUSE_TIME_MS, "standby" },
    { 0.0f, (float)TEST_PWM_DUTY_PERCENT, TEST_PWM_RUN_TIME_MS, "RIGHT positive" },
    { 0.0f, 0.0f, TEST_PWM_PAUSE_TIME_MS, "standby" },
    { 0.0f, -(float)TEST_PWM_DUTY_PERCENT, TEST_PWM_RUN_TIME_MS, "RIGHT negative" },
    { 0.0f, 0.0f, TEST_PWM_PAUSE_TIME_MS, "standby" },
    { (float)TEST_PWM_DUTY_PERCENT, (float)TEST_PWM_DUTY_PERCENT,
      TEST_PWM_RUN_TIME_MS, "BOTH positive" },
    { 0.0f, 0.0f, TEST_PWM_PAUSE_TIME_MS, "standby" },
    { -(float)TEST_PWM_DUTY_PERCENT, -(float)TEST_PWM_DUTY_PERCENT,
      TEST_PWM_RUN_TIME_MS, "BOTH negative" },
    { 0.0f, 0.0f, TEST_PWM_PAUSE_TIME_MS, "standby" }
};

static uint8_t running; // 测试序列是否运行。
static uint32_t step_index; // 当前测试步骤索引。
static uint32_t step_started_ms; // 当前步骤开始时间，ms。
static GPIO_PinState key_raw; // 最近一次读取的按键电平。
static GPIO_PinState key_stable; // 消抖后确认的按键电平。
static uint32_t key_changed_ms; // 原始按键电平变化时间，ms。

// 关闭电机驱动，再清零左右轮占空比。
static void StopOutputs(void)
{
    App_PWM_Cmd(0U);
    App_PWM_SetDutyCycle_L(0.0f);
    App_PWM_SetDutyCycle_R(0.0f);
}

// 驱动关闭时检查方向引脚；正负命令均互补且翻转时返回 1。
static uint8_t CheckDirection(void (*set_duty)(float),
                              GPIO_TypeDef *port1, uint16_t pin1,
                              GPIO_TypeDef *port2, uint16_t pin2)
{
    // 1. 分别写入正负命令，读取两组方向电平。
    set_duty((float)TEST_PWM_DUTY_PERCENT);
    GPIO_PinState positive1 = HAL_GPIO_ReadPin(port1, pin1); // 正命令时 IN1 电平。
    GPIO_PinState positive2 = HAL_GPIO_ReadPin(port2, pin2); // 正命令时 IN2 电平。
    set_duty(-(float)TEST_PWM_DUTY_PERCENT);
    GPIO_PinState negative1 = HAL_GPIO_ReadPin(port1, pin1); // 负命令时 IN1 电平。
    GPIO_PinState negative2 = HAL_GPIO_ReadPin(port2, pin2); // 负命令时 IN2 电平。
    // 2. 恢复零占空比，并检查两引脚的互补和翻转关系。
    set_duty(0.0f);
    return (positive1 != positive2 && negative1 != negative2 &&
            positive1 != negative1 && positive2 != negative2) ? 1U : 0U;
}

// 应用当前步骤，记录开始时间并打印输出命令。
static void ApplyStep(void)
{
    // 1. 先关闭输出，再设置左右轮命令。
    const PWM_TestStep *step = &steps[step_index]; // 当前步骤配置。
    StopOutputs();
    App_PWM_SetDutyCycle_L(step->left_duty);
    App_PWM_SetDutyCycle_R(step->right_duty);
    // 2. 有非零占空比时启用驱动，零命令保持待机。
    if (step->left_duty != 0.0f || step->right_duty != 0.0f)
    {
        App_PWM_Cmd(1U);
    }
    // 3. 开始计时，并发送步骤信息。
    step_started_ms = HAL_GetTick();
    char message[112]; // 串口输出缓冲区。
    snprintf(message, sizeof(message),
             "PWM: %s L=%d%% R=%d%% duration=%lums\r\n",
             step->name, (int)step->left_duty, (int)step->right_duty,
             (unsigned long)step->duration_ms);
    App_USART2_SendString(message);
}

// 初始化测试状态和按键消抖状态，等待按键启动。
void Test_PWM_Init(void)
{
    // 1. 清空运行状态，关闭电机输出。
    running = 0U;
    step_index = 0U;
    StopOutputs();
    // 2. 记录按键初始电平和时间，并打印操作提示。
    key_raw = HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin);
    key_stable = key_raw;
    key_changed_ms = HAL_GetTick();
    App_USART2_SendString("PWM: idle; press USER to start one sequence, press again to stop\r\n");
}

// 检查方向引脚并启动一次测试序列；成功返回 1。
uint8_t Test_PWM_Start(void)
{
    // 1. 已运行时拒绝重复启动。
    if (running != 0U)
    {
        return 0U;
    }
    StopOutputs();
    // 2. 驱动关闭时检查左右轮方向引脚，失败则结束。
    if (CheckDirection(App_PWM_SetDutyCycle_L,
                       MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin,
                       MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin) == 0U)
    {
        StopOutputs();
        App_USART2_SendString("PWM: FAIL left direction outputs; driver remains disabled\r\n");
        return 0U;
    }
    if (CheckDirection(App_PWM_SetDutyCycle_R,
                       MOTOR_R_IN1_GPIO_Port, MOTOR_R_IN1_Pin,
                       MOTOR_R_IN2_GPIO_Port, MOTOR_R_IN2_Pin) == 0U)
    {
        StopOutputs();
        App_USART2_SendString("PWM: FAIL right direction outputs; driver remains disabled\r\n");
        return 0U;
    }
    // 3. 通过检查后，从第一步开始运行。
    running = 1U;
    step_index = 0U;
    ApplyStep();
    return 1U;
}

// 结束测试并关闭电机输出。
void Test_PWM_Stop(void)
{
    running = 0U;
    StopOutputs();
    App_USART2_SendString("PWM: stopped\r\n");
}

// 返回测试序列的运行状态。
uint8_t Test_PWM_IsRunning(void)
{
    return running;
}

// 主循环处理按键消抖，并按时间推进测试序列。
void Test_PWM_Process(void)
{
    // 1. 跟踪按键原始电平变化，达到消抖时间后确认状态。
    uint32_t now = HAL_GetTick(); // 当前时刻，ms。
    GPIO_PinState raw = HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin); // 本次按键电平。
    if (raw != key_raw)
    {
        key_raw = raw;
        key_changed_ms = now;
    }
    if (key_raw != key_stable &&
        (uint32_t)(now - key_changed_ms) >= TEST_PWM_DEBOUNCE_MS)
    {
        key_stable = key_raw;
        // 2. 按键按下时切换启动和停止。
        if (key_stable == GPIO_PIN_RESET)
        {
            if (running != 0U) { Test_PWM_Stop(); }
            else { (void)Test_PWM_Start(); }
        }
    }
    // 3. 仅在运行中、当前步骤时间到后推进序列。
    if (running == 0U)
    {
        return;
    }
    now = HAL_GetTick();
    if ((uint32_t)(now - step_started_ms) < steps[step_index].duration_ms)
    {
        return;
    }
    // 4. 最后一步结束则停止，否则应用下一步。
    step_index++;
    if (step_index >= (uint32_t)(sizeof(steps) / sizeof(steps[0])))
    {
        Test_PWM_Stop();
        App_USART2_SendString("PWM: complete; press USER to run again\r\n");
        return;
    }
    ApplyStep();
}
