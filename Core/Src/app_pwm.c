#include "app_pwm.h"
#include "main.h"
#include "tim.h"
#include "math.h"

// 功能：让 TB6612 默认待机，并启动左右轮 PWM 通道。
void App_PWM_Init(void)
{
    // 1. 拉低共享的 STBY，使电机驱动保持待机。
    HAL_GPIO_WritePin(
        TB6612_STBY_GPIO_Port,
        TB6612_STBY_Pin,
        GPIO_PIN_RESET
    );

    // 2. 启动左轮 TIM1_CH1 和右轮 TIM4_CH1。
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

}

// 功能：控制 TB6612 总使能；on 为 0 时待机，非零时工作。
void App_PWM_Cmd(uint8_t on)
{
    if (on == 0)
    {
        // STBY 低电平：驱动待机。
        HAL_GPIO_WritePin(
            TB6612_STBY_GPIO_Port,
            TB6612_STBY_Pin,
            GPIO_PIN_RESET
        );
    }
    else
    {
        // STBY 高电平：允许驱动电机。
        HAL_GPIO_WritePin(
            TB6612_STBY_GPIO_Port,
            TB6612_STBY_Pin,
            GPIO_PIN_SET
        );
    }
}

// 功能：设置左轮方向和占空比；duty_cycle 范围 -100%～100%。
void App_PWM_SetDutyCycle_L(float duty_cycle)
{
    // 1. 将输入限制在有效范围内。
    float sign; // 方向符号：+1 正向，-1 反向。
    if (duty_cycle > 100.0f)
    {
        duty_cycle = 100.0f;
    }
    else if (duty_cycle < -100.0f)
    {
        duty_cycle = -100.0f;
    }

    // 2. 分离方向符号与占空比大小。
    if (duty_cycle >= 0.0f)
    {
        sign = 1.0f;
    }
    else
    {
        sign = -1.0f;
    }

    float DutyCycle = fabsf(duty_cycle); // 占空比绝对值，单位 %。

    // 3. 根据方向设置 AIN1、AIN2 电平。
    if (sign > 0.0f)
    {
        // 正向：AIN1(PA9)=0，AIN2(PA10)=1。
        HAL_GPIO_WritePin(
            MOTOR_L_IN1_GPIO_Port,
            MOTOR_L_IN1_Pin,
            GPIO_PIN_RESET
        );
        HAL_GPIO_WritePin(
            MOTOR_L_IN2_GPIO_Port,
            MOTOR_L_IN2_Pin,
            GPIO_PIN_SET
        );
    }
    else
    {
        // 反向：AIN1(PA9)=1，AIN2(PA10)=0。
        HAL_GPIO_WritePin(
            MOTOR_L_IN1_GPIO_Port,
            MOTOR_L_IN1_Pin,
            GPIO_PIN_SET
        );
        HAL_GPIO_WritePin(
            MOTOR_L_IN2_GPIO_Port,
            MOTOR_L_IN2_Pin,
            GPIO_PIN_RESET
        );
    }
    // 4. 将百分比换算为比较值 CCR，并更新左轮 PWM。
    uint32_t ccr_value = (uint32_t)(DutyCycle / 100.0f * (htim1.Init.Period + 1)); // PWM 比较值。
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_value);

}
// 功能：设置右轮方向和占空比；duty_cycle 范围 -100%～100%。
void App_PWM_SetDutyCycle_R(float duty_cycle)
{
    // 1. 将输入限制在有效范围内。
    float sign; // 方向符号：+1 正向，-1 反向。
    if (duty_cycle > 100.0f)
    {
        duty_cycle = 100.0f;
    }
    else if (duty_cycle < -100.0f)
    {
        duty_cycle = -100.0f;
    }

    // 2. 分离方向符号与占空比大小。
    if (duty_cycle >= 0.0f)
    {
        sign = 1.0f;
    }
    else
    {
        sign = -1.0f;
    }

    float DutyCycle = fabsf(duty_cycle); // 占空比绝对值，单位 %。

    // 3. 根据方向设置 BIN1、BIN2 电平。
    if (sign > 0.0f)
    {
        // 正向：BIN1(PB5)=1，BIN2(PB7)=0。
        HAL_GPIO_WritePin(
            MOTOR_R_IN1_GPIO_Port,
            MOTOR_R_IN1_Pin,
            GPIO_PIN_SET
        );
        HAL_GPIO_WritePin(
            MOTOR_R_IN2_GPIO_Port,
            MOTOR_R_IN2_Pin,
            GPIO_PIN_RESET
        );
    }
    else
    {
        // 反向：BIN1(PB5)=0，BIN2(PB7)=1。
        HAL_GPIO_WritePin(
            MOTOR_R_IN1_GPIO_Port,
            MOTOR_R_IN1_Pin,
            GPIO_PIN_RESET
        );
        HAL_GPIO_WritePin(
            MOTOR_R_IN2_GPIO_Port,
            MOTOR_R_IN2_Pin,
            GPIO_PIN_SET
        );
    }
    // 4. 将百分比换算为比较值 CCR，并更新右轮 PWM。
    uint32_t ccr_value = (uint32_t)(DutyCycle / 100.0f * (htim4.Init.Period + 1)); // PWM 比较值。
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr_value);

}
