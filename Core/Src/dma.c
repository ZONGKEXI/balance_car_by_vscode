#include "dma.h"

// 开启DMA时钟；彩灯传输完成中断低于编码器、ADC和串口。
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}
