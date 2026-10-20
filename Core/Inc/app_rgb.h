#ifndef APP_RGB_H
#define APP_RGB_H

#include <stdint.h>

#define RGB_LED_COUNT  5u

// 初始化TIM2/DMA彩灯应用，成功返回1，配置检查失败返回0。
uint8_t App_RGB_Init(void);

// 修改颜色缓存：index为0~4，r、g、b为0~255；调用Show后发送。
void App_RGB_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

// 请求异步发送颜色；DMA忙时保留最新请求，立即返回。
void App_RGB_Show(void);

// 请求熄灯；继续调用Proc时，呼吸任务会重新更新颜色。
void App_RGB_Clear(void);

// 主循环任务：每20ms更新五灯呼吸，覆盖手动颜色，并发送待处理数据。
void App_RGB_Proc(void);

#endif
