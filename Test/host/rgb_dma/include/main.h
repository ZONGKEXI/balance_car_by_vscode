#ifndef RGB_DMA_HOST_MAIN_H
#define RGB_DMA_HOST_MAIN_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef struct { volatile uint32_t BSRR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CTRL, CYCCNT; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2;
    volatile uint32_t CCER, CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4;
    volatile uint32_t BDTR, DCR, DMAR;
} TIM_TypeDef;
typedef struct { volatile uint32_t CCR, CNDTR, CPAR, CMAR; } DMA_Channel_TypeDef;
typedef struct { uint32_t Direction, PeriphInc, MemInc, PeriphDataAlignment;
    uint32_t MemDataAlignment, Mode, Priority; } DMA_InitTypeDef;
typedef struct { DMA_Channel_TypeDef *Instance; DMA_InitTypeDef Init;
    uint32_t State, ErrorCode; void *Parent; } DMA_HandleTypeDef;
typedef struct { uint32_t Prescaler, CounterMode, Period, ClockDivision,
    RepetitionCounter, AutoReloadPreload; } TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init;
    uint32_t Channel; DMA_HandleTypeDef *hdma[7]; uint32_t State; } TIM_HandleTypeDef;
typedef struct { uint32_t ClockType, SYSCLKSource, AHBCLKDivider,
    APB1CLKDivider, APB2CLKDivider; } RCC_ClkInitTypeDef;

extern GPIO_TypeDef host_gpio;
extern DWT_Type host_dwt;
extern CoreDebug_Type host_debug;
extern TIM_TypeDef host_tim2;

#define RGB_GPIO_Port (&host_gpio)
#define RGB_Pin 1u
#define DWT (&host_dwt)
#define CoreDebug (&host_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24u)
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define TIM2 (&host_tim2)
#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 4u
#define HAL_TIM_ACTIVE_CHANNEL_1 1u
#define HAL_TIM_ACTIVE_CHANNEL_2 2u
#define HAL_TIM_ACTIVE_CHANNEL_CLEARED 0u
#define TIM_DMA_ID_CC1 1u
#define TIM_FLAG_UPDATE 1u
#define TIM_FLAG_CC1 2u
#define TIM_EVENTSOURCE_UPDATE 1u
#define TIM_CR1_CEN 1u
#define TIM_DIER_CC1DE (1u << 9u)
#define DMA_IT_HT (1u << 2u)
#define RCC_HCLK_DIV1 0u
#define RCC_HCLK_DIV2 0x400u

uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clock, uint32_t *latency);
uint32_t __get_PRIMASK(void);
uint32_t __get_IPSR(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void HAL_Delay(uint32_t ms);
static inline void __NOP(void) { }
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *timer,
    uint32_t channel, const uint32_t *data, uint16_t length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *dma);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *timer);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *timer);
void Host_TIM_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer, uint32_t event);
void Host_DMA_DisableIT(DMA_HandleTypeDef *dma, uint32_t interrupt);

#define __HAL_TIM_SET_COMPARE(timer, channel, value) Host_TIM_SetCompare((timer), (channel), (value))
#define __HAL_TIM_GET_COMPARE(timer, channel) ((void)(channel), (timer)->Instance->CCR1)
#define __HAL_TIM_SET_COUNTER(timer, value) ((timer)->Instance->CNT = (value))
#define __HAL_TIM_GET_COUNTER(timer) ((timer)->Instance->CNT)
#define __HAL_TIM_CLEAR_FLAG(timer, flags) ((timer)->Instance->SR &= ~(flags))
#define __HAL_TIM_DISABLE(timer) ((timer)->Instance->CR1 &= ~TIM_CR1_CEN)
#define __HAL_DMA_DISABLE_IT(dma, interrupt) Host_DMA_DisableIT((dma), (interrupt))

#endif
