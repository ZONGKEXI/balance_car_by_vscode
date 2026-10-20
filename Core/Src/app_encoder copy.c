//
// @简介：左右轮编码器的位置读取与 T 法角速度测量。
// @说明：A 相双边沿触发 EXTI，读取 A/B 电平判断方向。
//        TIM3 每 1us 计数，HAL_GetTick() 用于判断长时间无事件。
// @前提：CubeMX 已完成 GPIO/EXTI 配置，TIM3 已启动。
//
#include "app_encoder.h"
#include "tim.h"
#include <stdint.h>

// 本阶段使用的测速超时阈值：50ms。
#define ENCODER_STOP_TIMEOUT_MS  50u
#define ENCODER_STOP_TIMEOUT_US  (ENCODER_STOP_TIMEOUT_MS * 1000u)

// 沿用当前换算参数，每次有效计数对应的车轮角度。
// 22 应为当前 A 相双边沿方式下，电机轴每圈的有效计数。
#define ENCODER_DEGREES_PER_COUNT \
    (360.0f / 22.0f / (30613.0f / 1500.0f))

// 左右轮累计计数。
static volatile int32_t encoder_count_l = 0;
static volatile int32_t encoder_count_r = 0;

// 方向状态：
// +1：正向；-1：反向。
// +2：刚切换到正向；-2：刚切换到反向。
// ±2 时暂不使用跨换向的时间间隔计算速度。
static volatile int8_t direction_l = 1;
static volatile int8_t direction_r = 1;

// t0：最近一次事件时间；t1：前一次事件时间。
// 存储 TIM3 的 16 位计数，单位为 us。
static volatile uint16_t t0_l = 0, t1_l = 0;
static volatile uint16_t t0_r = 0, t1_r = 0;

// 采样状态：
// 0：尚无事件。
// 1：已建立时间起点，等待下一次同向事件。
// 2：已取得两个同向、未过期的事件。
static volatile uint8_t sample_count_l = 0;
static volatile uint8_t sample_count_r = 0;

// 最近一次事件的毫秒时间，用于识别停转和长事件间隔。
static volatile uint32_t last_edge_ms_l = 0;
static volatile uint32_t last_edge_ms_r = 0;

//
// @简介：清零左右轮累计位置，并复位全部测速状态。
// @说明：初始化后，需要收到两个同向、未过期的事件才能测速。
//
void App_Encoder_Init(void)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();

    encoder_count_l = 0;
    direction_l = 1;
    t0_l = 0;
    t1_l = 0;
    sample_count_l = 0;
    last_edge_ms_l = 0;

    encoder_count_r = 0;
    direction_r = 1;
    t0_r = 0;
    t1_r = 0;
    sample_count_r = 0;
    last_edge_ms_r = 0;

    __set_PRIMASK(irq_state);
}

//
// @简介：读取左轮累计旋转角度，单位为度。
// @说明：角度可以超过一圈，正负号由累计计数决定。
//
float App_Encoder_GetPos_L(void)
{
    return (float)encoder_count_l * ENCODER_DEGREES_PER_COUNT;
}

//
// @简介：读取右轮累计旋转角度，单位为度。
//
float App_Encoder_GetPos_R(void)
{
    return (float)encoder_count_r * ENCODER_DEGREES_PER_COUNT;
}

//
// @简介：估算左轮角速度，单位为度/s。
// @说明：采用最近事件间隔与当前等待时间中的较大值。
//        采样不足、换向、超时或时间间隔为零时返回零。
//
float App_Encoder_GetSpeed_L(void)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();

    // 在同一个短临界区内取得完整的数据副本。
    int8_t direction_copy = direction_l;
    uint8_t sample_count_copy = sample_count_l;
    uint16_t t0_copy = t0_l;
    uint16_t t1_copy = t1_l;
    uint32_t last_edge_ms_copy = last_edge_ms_l;

    uint16_t now_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint32_t now_ms = HAL_GetTick();

    __set_PRIMASK(irq_state);

    // 尚未取得两个有效事件，或刚发生换向。
    if (sample_count_copy < 2u ||
        direction_copy == 2 || direction_copy == -2)
    {
        return 0.0f;
    }

    // 用毫秒时间识别长时间无事件，避免 TIM3 回绕后假速度重现。
    uint32_t age_ms = (uint32_t)(now_ms - last_edge_ms_copy);

    if (age_ms >= ENCODER_STOP_TIMEOUT_MS)
    {
        return 0.0f;
    }

    // 按 16 位回绕计算时间差，单位为 us。
    uint16_t interval_us = (uint16_t)(t0_copy - t1_copy);
    uint16_t wait_us = (uint16_t)(now_us - t0_copy);

    // 零间隔不能作为除数；超出本阶段范围也不计算。
    if (interval_us == 0u ||
        interval_us >= ENCODER_STOP_TIMEOUT_US)
    {
        return 0.0f;
    }

    // 等待时间尚未超过上次间隔时，保持最近的速度估计。
    // 超过后，随着等待时间增加，速度估计逐渐降低。
    uint16_t T_us = (interval_us > wait_us) ? interval_us : wait_us;

    if (T_us >= ENCODER_STOP_TIMEOUT_US)
    {
        return 0.0f;
    }

    // interval_us > 0，因此取较大值后的 T_us 也一定大于零。
    float T_s = (float)T_us * 1.0e-6f;

    return (float)direction_copy * ENCODER_DEGREES_PER_COUNT / T_s;
}

//
// @简介：估算右轮角速度，单位为度/s。
// @说明：处理方法与左轮相同，使用右轮自己的数据。
//
float App_Encoder_GetSpeed_R(void)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();

    int8_t direction_copy = direction_r;
    uint8_t sample_count_copy = sample_count_r;
    uint16_t t0_copy = t0_r;
    uint16_t t1_copy = t1_r;
    uint32_t last_edge_ms_copy = last_edge_ms_r;

    uint16_t now_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    uint32_t now_ms = HAL_GetTick();

    __set_PRIMASK(irq_state);

    if (sample_count_copy < 2u ||
        direction_copy == 2 || direction_copy == -2)
    {
        return 0.0f;
    }

    uint32_t age_ms = (uint32_t)(now_ms - last_edge_ms_copy);

    if (age_ms >= ENCODER_STOP_TIMEOUT_MS)
    {
        return 0.0f;
    }

    uint16_t interval_us = (uint16_t)(t0_copy - t1_copy);
    uint16_t wait_us = (uint16_t)(now_us - t0_copy);

    if (interval_us == 0u ||
        interval_us >= ENCODER_STOP_TIMEOUT_US)
    {
        return 0.0f;
    }

    uint16_t T_us = (interval_us > wait_us) ? interval_us : wait_us;

    if (T_us >= ENCODER_STOP_TIMEOUT_US)
    {
        return 0.0f;
    }

    float T_s = (float)T_us * 1.0e-6f;

    return (float)direction_copy * ENCODER_DEGREES_PER_COUNT / T_s;
}

//
// @简介：处理左右轮 A 相中断，记录时间、更新位置与方向。
// @说明：左轮 A/B 不同时计数增加；右轮采用相反的符号约定。
//        首次事件、换向或长时间无事件后，重新建立测速起点。
//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MOTOR_L_ENCA_Pin)
    {
        uint16_t edge_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        uint32_t edge_ms = HAL_GetTick();

        // 必须在更新 last_edge_ms_l 前计算本次事件间隔。
        uint32_t gap_ms = (uint32_t)(edge_ms - last_edge_ms_l);

        t1_l = t0_l;
        t0_l = edge_us;

        GPIO_PinState a = HAL_GPIO_ReadPin(
            MOTOR_L_ENCA_GPIO_Port, MOTOR_L_ENCA_Pin);

        GPIO_PinState b = HAL_GPIO_ReadPin(
            MOTOR_L_ENCB_GPIO_Port, MOTOR_L_ENCB_Pin);

        // A/B 电平不同：左轮正向。
        if ((a == GPIO_PIN_SET && b == GPIO_PIN_RESET) ||
            (a == GPIO_PIN_RESET && b == GPIO_PIN_SET))
        {
            if (direction_l < 0)
            {
                direction_l = 2;
            }
            else
            {
                direction_l = 1;
            }

            encoder_count_l++;
        }
        else
        {
            // A/B 电平相同：左轮反向。
            if (direction_l > 0)
            {
                direction_l = -2;
            }
            else
            {
                direction_l = -1;
            }

            encoder_count_l--;
        }

        // 首次采样、刚换向、间隔过长：本次只建立时间起点。
        if (sample_count_l == 0u ||
            direction_l == 2 || direction_l == -2 ||
            gap_ms >= ENCODER_STOP_TIMEOUT_MS)
        {
            sample_count_l = 1u;
        }
        else if (sample_count_l < 2u)
        {
            sample_count_l++;
        }

        last_edge_ms_l = edge_ms;
    }
    else if (GPIO_Pin == MOTOR_R_ENCA_Pin)
    {
        uint16_t edge_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
        uint32_t edge_ms = HAL_GetTick();
        uint32_t gap_ms = (uint32_t)(edge_ms - last_edge_ms_r);

        t1_r = t0_r;
        t0_r = edge_us;

        GPIO_PinState a = HAL_GPIO_ReadPin(
            MOTOR_R_ENCA_GPIO_Port, MOTOR_R_ENCA_Pin);

        GPIO_PinState b = HAL_GPIO_ReadPin(
            MOTOR_R_ENCB_GPIO_Port, MOTOR_R_ENCB_Pin);

        // A/B 电平不同：右轮反向。
        if ((a == GPIO_PIN_SET && b == GPIO_PIN_RESET) ||
            (a == GPIO_PIN_RESET && b == GPIO_PIN_SET))
        {
            if (direction_r > 0)
            {
                direction_r = -2;
            }
            else
            {
                direction_r = -1;
            }

            encoder_count_r--;
        }
        else
        {
            // A/B 电平相同：右轮正向。
            if (direction_r < 0)
            {
                direction_r = 2;
            }
            else
            {
                direction_r = 1;
            }

            encoder_count_r++;
        }

        if (sample_count_r == 0u ||
            direction_r == 2 || direction_r == -2 ||
            gap_ms >= ENCODER_STOP_TIMEOUT_MS)
        {
            sample_count_r = 1u;
        }
        else if (sample_count_r < 2u)
        {
            sample_count_r++;
        }

        last_edge_ms_r = edge_ms;
    }
}