#ifndef RGB_BATTERY_HOST_HAL_H
#define RGB_BATTERY_HOST_HAL_H

#include <stdint.h>

/* Only the HAL boundary is simulated; app_bat.c and adc.c are compiled unchanged. */
typedef enum { GPIO_PIN_RESET = 0U, GPIO_PIN_SET = 1U } GPIO_PinState;
typedef enum { HAL_OK = 0U, HAL_ERROR = 1U, HAL_BUSY = 2U, HAL_TIMEOUT = 3U } HAL_StatusTypeDef;
typedef struct { uint32_t ODR; } GPIO_TypeDef;
typedef struct { uint32_t unused; } ADC_TypeDef;
typedef struct { uint32_t unused; } TIM_TypeDef;
typedef struct { uint32_t unused; } DMA_HandleTypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct {
    uint32_t ScanConvMode;
    uint32_t ContinuousConvMode;
    uint32_t DiscontinuousConvMode;
    uint32_t ExternalTrigConv;
    uint32_t DataAlign;
    uint32_t NbrOfConversion;
} ADC_InitTypeDef;
typedef struct { ADC_TypeDef *Instance; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
typedef struct {
    uint32_t InjectedChannel;
    uint32_t InjectedRank;
    uint32_t InjectedNbrOfConversion;
    uint32_t InjectedSamplingTime;
    uint32_t ExternalTrigInjecConv;
    uint32_t AutoInjectedConv;
    uint32_t InjectedDiscontinuousConvMode;
    uint32_t InjectedOffset;
} ADC_InjectionConfTypeDef;
typedef struct { uint32_t Pin; uint32_t Mode; } GPIO_InitTypeDef;

extern GPIO_TypeDef host_gpio_a;
extern GPIO_TypeDef host_gpio_b;
extern ADC_TypeDef host_adc_1;
extern ADC_TypeDef host_adc_2;
#define GPIOA (&host_gpio_a)
#define GPIOB (&host_gpio_b)
#define ADC1 (&host_adc_1)
#define ADC2 (&host_adc_2)
#define GPIO_PIN_0 0x0001U
#define GPIO_PIN_1 0x0002U
#define GPIO_PIN_3 0x0008U
#define GPIO_PIN_4 0x0010U
#define GPIO_PIN_5 0x0020U
#define GPIO_PIN_6 0x0040U
#define GPIO_PIN_7 0x0080U
#define GPIO_PIN_9 0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U
#define GPIO_MODE_ANALOG 0U
#define ADC1_2_IRQn 18U
#define DISABLE 0U
#define ADC_SCAN_DISABLE 0U
#define ADC_SOFTWARE_START 1U
#define ADC_DATAALIGN_RIGHT 0U
#define ADC_CHANNEL_8 8U
#define ADC_INJECTED_RANK_1 1U
#define ADC_SAMPLETIME_1CYCLE_5 0U
#define ADC_INJECTED_SOFTWARE_START 2U
#define ADC_EXTERNALTRIGINJECCONV_T2_TRGO 3U
#define __HAL_RCC_ADC1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_ADC1_CLK_DISABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)

uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *adc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *adc);
uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef *adc, uint32_t rank);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *timer);
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *adc);
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *adc, ADC_InjectionConfTypeDef *config);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);
void HAL_GPIO_DeInit(GPIO_TypeDef *port, uint32_t pins);
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority);
void HAL_NVIC_EnableIRQ(uint32_t irq);
void HAL_NVIC_DisableIRQ(uint32_t irq);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *adc);

#endif
