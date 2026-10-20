#ifndef TEST_HOST_STM32F1XX_HAL_H
#define TEST_HOST_STM32F1XX_HAL_H

#include <stdint.h>

/* Host-only HAL boundary. This directory is never in the firmware include path. */
typedef enum { GPIO_PIN_RESET = 0U, GPIO_PIN_SET = 1U } GPIO_PinState;
typedef enum { HAL_OK = 0U, HAL_ERROR = 1U } HAL_StatusTypeDef;
typedef struct { uint32_t Instance; } UART_HandleTypeDef;
typedef struct __DMA_HandleTypeDef DMA_HandleTypeDef;
typedef struct { uint32_t IDR; uint32_t ODR; } GPIO_TypeDef;
typedef struct { uint32_t CCR1; uint32_t CNT; } TIM_TypeDef;
typedef struct { uint32_t Period; } TIM_Base_InitTypeDef;
typedef struct {
    TIM_TypeDef *Instance;
    TIM_Base_InitTypeDef Init;
} TIM_HandleTypeDef;

extern GPIO_TypeDef host_gpio_a;
extern GPIO_TypeDef host_gpio_b;
#define GPIOA (&host_gpio_a)
#define GPIOB (&host_gpio_b)
#define GPIO_PIN_0  ((uint16_t)0x0001U)
#define GPIO_PIN_1  ((uint16_t)0x0002U)
#define GPIO_PIN_2  ((uint16_t)0x0004U)
#define GPIO_PIN_3  ((uint16_t)0x0008U)
#define GPIO_PIN_4  ((uint16_t)0x0010U)
#define GPIO_PIN_5  ((uint16_t)0x0020U)
#define GPIO_PIN_6  ((uint16_t)0x0040U)
#define GPIO_PIN_7  ((uint16_t)0x0080U)
#define GPIO_PIN_8  ((uint16_t)0x0100U)
#define GPIO_PIN_9  ((uint16_t)0x0200U)
#define GPIO_PIN_10 ((uint16_t)0x0400U)
#define GPIO_PIN_11 ((uint16_t)0x0800U)
#define GPIO_PIN_12 ((uint16_t)0x1000U)
#define GPIO_PIN_13 ((uint16_t)0x2000U)
#define GPIO_PIN_14 ((uint16_t)0x4000U)
#define GPIO_PIN_15 ((uint16_t)0x8000U)
#define TIM_CHANNEL_1 0U
#define __HAL_TIM_SET_COMPARE(handle, channel, value) \
    ((void)(channel), (handle)->Instance->CCR1 = (value))
#define __HAL_TIM_GET_COUNTER(handle) ((handle)->Instance->CNT)

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
void HAL_GPIO_EXTI_Callback(uint16_t pin);

#endif
