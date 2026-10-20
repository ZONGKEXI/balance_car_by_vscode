#include "app_rgb.h"
#include "main.h"
#include "tim.h"
#include <string.h>

// TIM2时钟72MHz，PSC=0、ARR=89，一位持续1.25us。
#define RGB_CPU_HZ             72000000u
#define RGB_T0H_CYCLES         22u
#define RGB_T1H_CYCLES         47u
#define RGB_RESET_SLOTS        402u  // 留出CCR预装延迟，确保低电平超过500us。
#define RGB_DATA_SLOTS         (RGB_LED_COUNT * 24u)
#define RGB_DMA_SLOTS          (RGB_RESET_SLOTS * 2u + RGB_DATA_SLOTS)
#define RGB_DMA_TIMEOUT_MS     3u

// 呼吸参数：每20ms刷新，2秒完成一次亮暗循环，峰值32。
#define RGB_BREATH_REFRESH_MS  20u
#define RGB_BREATH_PERIOD_MS   2000u
#define RGB_BREATH_MAX         32u

static uint8_t rgb_data[RGB_LED_COUNT][3]; // 每颗灯的G、R、B缓存。
// 静态数组初始为0；只写中间的数据位，首尾始终保持锁存低电平。
static uint16_t rgb_wave[RGB_DMA_SLOTS] __attribute__((aligned(4)));
static uint8_t rgb_ready;
static volatile uint8_t rgb_busy;    // DMA正在读取波形缓存。
static volatile uint8_t rgb_pending; // 有最新颜色等待发送。
static uint32_t rgb_started_ms;      // 本帧启动时间，用于异常恢复。
static uint32_t rgb_last_update_ms;
static uint32_t rgb_phase_ms;        // 呼吸相位：0~1999ms。

// 五颗灯分别使用红、绿、蓝、黄、紫。
static const uint8_t rgb_colors[RGB_LED_COUNT][3] = {
    {1u, 0u, 0u}, {0u, 1u, 0u}, {0u, 0u, 1u},
    {1u, 1u, 0u}, {1u, 0u, 1u}
};

// 停止本次传输，将CCR1的预装值和有效值恢复为0。
static void RGB_StopTransfer(void)
{
    HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
    rgb_busy = 0u;
}

// 外设配置完成后调用；成功返回1，配置不符返回0。
uint8_t App_RGB_Init(void)
{
    rgb_ready = 0u;
    if (HAL_RCC_GetHCLKFreq() != RGB_CPU_HZ ||
        htim2.Instance != TIM2 || htim2.Init.Prescaler != 0u ||
        htim2.Init.Period != 89u || htim2.hdma[TIM_DMA_ID_CC1] == NULL ||
        __get_IPSR() != 0u || __get_PRIMASK() != 0u)
        return 0u;

    if (rgb_busy != 0u)
        RGB_StopTransfer();

    rgb_phase_ms = 0u;
    rgb_last_update_ms = HAL_GetTick();
    rgb_ready = 1u;
    App_RGB_Clear();
    return 1u;
}

// 只更新颜色缓存，不修改正在发送的DMA波形。
void App_RGB_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= RGB_LED_COUNT)
        return;
    rgb_data[index][0] = g;
    rgb_data[index][1] = r;
    rgb_data[index][2] = b;
}

// 将颜色编码成波形并启动DMA；忙时保留最新请求，不等待发送完成。
void App_RGB_Show(void)
{
    rgb_pending = 1u;
    if (rgb_ready == 0u || rgb_busy != 0u ||
        __get_IPSR() != 0u || __get_PRIMASK() != 0u)
        return;

    // 1. 按G、R、B顺序编码，从每个字节的最高位开始。
    uint32_t n = RGB_RESET_SLOTS;
    for (uint32_t led = 0u; led < RGB_LED_COUNT; ++led)
    {
        for (uint32_t color = 0u; color < 3u; ++color)
        {
            uint8_t value = rgb_data[led][color];
            for (uint8_t mask = 0x80u; mask != 0u; mask >>= 1u)
            {
                rgb_wave[n++] = (value & mask)
                    ? RGB_T1H_CYCLES : RGB_T0H_CYCLES;
            }
        }
    }

    // 2. 清除旧计数和输出，保证本帧从低电平开始。
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COUNTER(&htim2, 0u);
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE | TIM_FLAG_CC1);

    // 3. 先标记忙再启动DMA；失败时释放HAL状态，留给主循环重试。
    rgb_busy = 1u;
    rgb_pending = 0u;
    rgb_started_ms = HAL_GetTick();
    if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1,
            (const uint32_t *)rgb_wave, RGB_DMA_SLOTS) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(htim2.hdma[TIM_DMA_ID_CC1], DMA_IT_HT);
    }
    else
    {
        RGB_StopTransfer();
        rgb_pending = 1u;
    }
}

// 请求发送全零颜色，完成后五颗灯熄灭。
void App_RGB_Clear(void)
{
    memset(rgb_data, 0, sizeof(rgb_data));
    App_RGB_Show();
}

// 主循环任务：更新时间/亮度，启动DMA后立即继续其他任务。
void App_RGB_Proc(void)
{
    if (rgb_ready == 0u)
        return;
    uint32_t now = HAL_GetTick();

    // 1. 异常未收到完成中断时，停止并准备重试。
    if (rgb_busy != 0u &&
        (uint32_t)(now - rgb_started_ms) >= RGB_DMA_TIMEOUT_MS)
    {
        RGB_StopTransfer();
        rgb_pending = 1u;
    }

    // 2. 每20ms计算同步呼吸亮度，处理毫秒计数回绕。
    uint32_t elapsed = (uint32_t)(now - rgb_last_update_ms);
    if (elapsed >= RGB_BREATH_REFRESH_MS)
    {
        rgb_last_update_ms = now;
        rgb_phase_ms = (rgb_phase_ms + elapsed % RGB_BREATH_PERIOD_MS)
                       % RGB_BREATH_PERIOD_MS;
        uint32_t half_period = RGB_BREATH_PERIOD_MS / 2u;
        uint32_t ramp = (rgb_phase_ms <= half_period)
                      ? rgb_phase_ms : RGB_BREATH_PERIOD_MS - rgb_phase_ms;
        uint8_t brightness = (uint8_t)(ramp * RGB_BREATH_MAX / half_period);

        for (uint8_t led = 0u; led < RGB_LED_COUNT; ++led)
        {
            App_RGB_SetColor(led,
                rgb_colors[led][0] * brightness,
                rgb_colors[led][1] * brightness,
                rgb_colors[led][2] * brightness);
        }
        rgb_pending = 1u;
    }

    // 3. DMA空闲时发送最新颜色，不等待传输完成。
    if (rgb_pending != 0u)
        App_RGB_Show();
}

// 尾部已输出足够低电平，完成中断中结束本帧。
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        RGB_StopTransfer();
}

// DMA错误只影响彩灯，下次主循环尝试重新发送。
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        RGB_StopTransfer();
        rgb_pending = 1u;
    }
}
