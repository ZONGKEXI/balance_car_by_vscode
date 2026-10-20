// 通过 A 相双边沿计数测位置，并用 TIM3 的微秒计时测角速度。
#include "app_encoder.h"
#include "tim.h"
#include <stdint.h>

#define ENCODER_STOP_TIMEOUT_MS  50u // 无边沿达到此时长后速度归零（ms）
#define ENCODER_STOP_TIMEOUT_US  (ENCODER_STOP_TIMEOUT_MS * 1000u) // 微秒超时阈值

// 每个计数对应的车轮角度（度）。
#define ENCODER_DEGREES_PER_COUNT \
    (360.0f / 22.0f / (30613.0f / 1500.0f))

// 每个计数对应的车轮角度（rad）。
#define ENCODER_RADIANS_PER_COUNT \
    (2.0f * 3.14159265f / 22.0f / (30613.0f / 1500.0f))

static volatile int32_t encoder_count_l = 0; // 左轮累计计数，可正可负
static volatile int32_t encoder_count_r = 0; // 右轮累计计数，可正可负

static volatile int8_t direction_l = 1; // 左轮方向：正负表示正反向，±2 表示刚换向
static volatile int8_t direction_r = 1; // 右轮方向：正负表示正反向，±2 表示刚换向

static volatile uint16_t t0_l = 0, t1_l = 0; // 左轮最近/前次边沿计时（us，16 位回绕）
static volatile uint16_t t0_r = 0, t1_r = 0; // 右轮最近/前次边沿计时（us，16 位回绕）

static volatile uint8_t sample_count_l = 0; // 左轮采样状态：0 无样本，1 有起点，2 有同向间隔
static volatile uint8_t sample_count_r = 0; // 右轮采样状态：0 无样本，1 有起点，2 有同向间隔

static volatile uint32_t last_edge_ms_l = 0; // 左轮最近边沿时刻（ms），用于判断超时
static volatile uint32_t last_edge_ms_r = 0; // 右轮最近边沿时刻（ms），用于判断超时

// 清零左右轮位置，并复位测速所需的方向、时间和采样状态。
void App_Encoder_Init(void)
{
    // 1. 保存中断状态并关中断，防止复位过程中被边沿中断打断。
    uint32_t irq_state = __get_PRIMASK(); // 进入临界区前的中断屏蔽状态
    __disable_irq();

    // 2. 复位左轮累计计数和测速状态。
    encoder_count_l = 0;
    direction_l = 1;
    t0_l = 0;
    t1_l = 0;
    sample_count_l = 0;
    last_edge_ms_l = 0;

    // 3. 复位右轮累计计数和测速状态。
    encoder_count_r = 0;
    direction_r = 1;
    t0_r = 0;
    t1_r = 0;
    sample_count_r = 0;
    last_edge_ms_r = 0;

    // 4. 恢复进入函数前的中断状态。
    __set_PRIMASK(irq_state);
}

// 将左轮累计计数换算为旋转角度（度，可超过一圈）。
float App_Encoder_GetPos_L(void)
{
    return (float)encoder_count_l * ENCODER_DEGREES_PER_COUNT;
}

// 将右轮累计计数换算为旋转角度（度，可超过一圈）。
float App_Encoder_GetPos_R(void)
{
    return (float)encoder_count_r * ENCODER_DEGREES_PER_COUNT;
}

// 根据边沿间隔估算左轮角速度（rad/s），无有效采样时返回零。
float App_Encoder_GetSpeed_L(void)
{
    // 1. 关中断并复制测速状态，保证读取的数据属于同一次采样。
    uint32_t irq_state = __get_PRIMASK(); // 进入临界区前的中断屏蔽状态
    __disable_irq();

    int8_t direction_copy = direction_l; // 方向状态副本
    uint8_t sample_count_copy = sample_count_l; // 采样状态副本
    uint16_t t0_copy = t0_l; // 最近边沿计时（us）
    uint16_t t1_copy = t1_l; // 前次边沿计时（us）
    uint32_t last_edge_ms_copy = last_edge_ms_l; // 最近边沿时刻（ms）

    uint16_t now_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3); // 当前微秒计时
    uint32_t now_ms = HAL_GetTick(); // 当前毫秒时刻

    __set_PRIMASK(irq_state);

    // 2. 排除采样不足、刚换向和长时间没有边沿的情况。
    if (sample_count_copy < 2u ||
        direction_copy == 2 || direction_copy == -2)
    {
        return 0.0f;
    }

    uint32_t age_ms = (uint32_t)(now_ms - last_edge_ms_copy); // 距离最近边沿的时长（ms）

    if (age_ms >= ENCODER_STOP_TIMEOUT_MS)
    {
        return 0.0f;
    }

    // 3. 按 16 位计时回绕计算边沿间隔，并排除零间隔及超时。
    uint16_t interval_us = (uint16_t)(t0_copy - t1_copy); // 最近两次边沿间隔（us）
    uint16_t wait_us = (uint16_t)(now_us - t0_copy); // 最近边沿后已等待的时间（us）

    if (interval_us == 0u ||
        interval_us >= ENCODER_STOP_TIMEOUT_US)
    {
        return 0.0f;
    }

    // 4. 取间隔和等待时间的较大值，使速度在无新边沿时逐渐降低。
    uint16_t T_us = (interval_us > wait_us) ? interval_us : wait_us; // 测速周期（us）

    if (T_us >= ENCODER_STOP_TIMEOUT_US)
    {
        return 0.0f;
    }

    // 5. 将周期换算为秒，按方向计算角速度。
    float T_s = (float)T_us * 1.0e-6f; // 测速周期（s）

    return (float)direction_copy * ENCODER_RADIANS_PER_COUNT / T_s;
}

// 根据边沿间隔估算右轮角速度（rad/s），无有效采样时返回零。
float App_Encoder_GetSpeed_R(void)
{
    // 1. 关中断并复制测速状态，保证读取的数据属于同一次采样。
    uint32_t irq_state = __get_PRIMASK(); // 进入临界区前的中断屏蔽状态
    __disable_irq();

    int8_t direction_copy = direction_r; // 方向状态副本
    uint8_t sample_count_copy = sample_count_r; // 采样状态副本
    uint16_t t0_copy = t0_r; // 最近边沿计时（us）
    uint16_t t1_copy = t1_r; // 前次边沿计时（us）
    uint32_t last_edge_ms_copy = last_edge_ms_r; // 最近边沿时刻（ms）

    uint16_t now_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3); // 当前微秒计时
    uint32_t now_ms = HAL_GetTick(); // 当前毫秒时刻

    __set_PRIMASK(irq_state);

    // 2. 排除采样不足、刚换向和长时间没有边沿的情况。
    if (sample_count_copy < 2u ||
        direction_copy == 2 || direction_copy == -2)
    {
        return 0.0f;
    }

    uint32_t age_ms = (uint32_t)(now_ms - last_edge_ms_copy); // 距离最近边沿的时长（ms）

    if (age_ms >= ENCODER_STOP_TIMEOUT_MS)
    {
        return 0.0f;
    }

    // 3. 按 16 位计时回绕计算边沿间隔，并排除零间隔及超时。
    uint16_t interval_us = (uint16_t)(t0_copy - t1_copy); // 最近两次边沿间隔（us）
    uint16_t wait_us = (uint16_t)(now_us - t0_copy); // 最近边沿后已等待的时间（us）

    if (interval_us == 0u ||
        interval_us >= ENCODER_STOP_TIMEOUT_US)
    {
        return 0.0f;
    }

    // 4. 取间隔和等待时间的较大值，使速度在无新边沿时逐渐降低。
    uint16_t T_us = (interval_us > wait_us) ? interval_us : wait_us; // 测速周期（us）

    if (T_us >= ENCODER_STOP_TIMEOUT_US)
    {
        return 0.0f;
    }

    // 5. 将周期换算为秒，按方向计算角速度。
    float T_s = (float)T_us * 1.0e-6f; // 测速周期（s）

    return (float)direction_copy * ENCODER_RADIANS_PER_COUNT / T_s;
}

// 根据 GPIO_Pin 处理左右轮 A 相边沿，更新位置、方向和测速采样状态。
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MOTOR_L_ENCA_Pin)
    {
        // 1. 记录左轮边沿时间，保留前次计时并计算毫秒间隔。
        uint16_t edge_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3); // 本次边沿计时（us）
        uint32_t edge_ms = HAL_GetTick(); // 本次边沿时刻（ms）

        uint32_t gap_ms = (uint32_t)(edge_ms - last_edge_ms_l); // 与前次边沿的间隔（ms）

        t1_l = t0_l;
        t0_l = edge_us;

        // 2. 读取 A/B 电平，更新方向状态和累计计数。
        GPIO_PinState a = HAL_GPIO_ReadPin( // A 相电平
            MOTOR_L_ENCA_GPIO_Port, MOTOR_L_ENCA_Pin);

        GPIO_PinState b = HAL_GPIO_ReadPin( // B 相电平
            MOTOR_L_ENCB_GPIO_Port, MOTOR_L_ENCB_Pin);

        // 左轮 A/B 不同为反向，相同为正向；±2 表示本次刚换向。
        if ((a == GPIO_PIN_SET && b == GPIO_PIN_RESET) ||
            (a == GPIO_PIN_RESET && b == GPIO_PIN_SET))
        {
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
        else
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

        // 3. 首次边沿、换向或间隔超时后重建起点，否则补齐同向采样。
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

        // 4. 保存本次边沿的毫秒时刻，供下一次中断和超时判断使用。
        last_edge_ms_l = edge_ms;
    }
    else if (GPIO_Pin == MOTOR_R_ENCA_Pin)
    {
        // 1. 记录右轮边沿时间，保留前次计时并计算毫秒间隔。
        uint16_t edge_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3); // 本次边沿计时（us）
        uint32_t edge_ms = HAL_GetTick(); // 本次边沿时刻（ms）
        uint32_t gap_ms = (uint32_t)(edge_ms - last_edge_ms_r); // 与前次边沿的间隔（ms）

        t1_r = t0_r;
        t0_r = edge_us;

        // 2. 读取 A/B 电平，更新方向状态和累计计数。
        GPIO_PinState a = HAL_GPIO_ReadPin( // A 相电平
            MOTOR_R_ENCA_GPIO_Port, MOTOR_R_ENCA_Pin);

        GPIO_PinState b = HAL_GPIO_ReadPin( // B 相电平
            MOTOR_R_ENCB_GPIO_Port, MOTOR_R_ENCB_Pin);

        // 右轮 A/B 不同为正向，相同为反向；±2 表示本次刚换向。
        if ((a == GPIO_PIN_SET && b == GPIO_PIN_RESET) ||
            (a == GPIO_PIN_RESET && b == GPIO_PIN_SET))
        {
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
        else
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

        // 3. 首次边沿、换向或间隔超时后重建起点，否则补齐同向采样。
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

        // 4. 保存本次边沿的毫秒时刻，供下一次中断和超时判断使用。
        last_edge_ms_r = edge_ms;
    }
}
