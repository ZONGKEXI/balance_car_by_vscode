#include "app_rc.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "app_control.h"

static char intBuf[64];   // 中断中逐字节收集命令。
static char transBuf[64]; // 最近一条完整命令。
static char procBuf[64];  // 主循环解析时使用的副本。

static volatile uint8_t lineReceivedFlag = 0; // 有完整命令等待处理。
static uint16_t intBufCursor = 0; // 下一字节的存放位置。
static uint8_t lineOverflow = 0; // 超长行丢弃到下一个结束符。
static uint8_t rx_buffer = 0; // HAL 持续使用的接收变量。

// 功能：复位组帧状态，启动 USART3 的蓝牙接收。
void App_RC_Init(void)
{
    // 1. 复位组帧状态。
    intBufCursor = 0;
    lineReceivedFlag = 0;
    lineOverflow = 0;

    // 2. 首次启动 USART3 一字节中断接收。
    if (HAL_UART_Receive_IT(&huart3, &rx_buffer, 1) != HAL_OK)
    {
        Error_Handler();
    }
}

// 功能：解析 move 转向值 前后值，更新控制环的目标速度。
void App_RC_Proc(void)
{
    // 1. 每 5ms 检查是否有完整命令。
    static uint32_t nxt = 0;        // 下一次计划执行时间，单位 ms。
    uint32_t now = HAL_GetTick();   // 当前时间，单位 ms。
    if (now < nxt)
        return;
    nxt = nxt + 5;

    if (lineReceivedFlag) {
        // 2. 短暂保护完整行副本，避免复制时被接收中断改写。
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        strcpy(procBuf, transBuf);
        lineReceivedFlag = 0;
        __set_PRIMASK(primask);

        // 3. 解析整数指令，换算为 m/s 和 rad/s；float 接口保留小数。
        int turnSpeed, moveSpeed;
        if (sscanf(procBuf, "move %d %d", &turnSpeed, &moveSpeed) == 2) {
            // 按手机按钮的实际方向约定，使用指令本身的正负号。
            App_Control_SetMoveSpeed(moveSpeed * 0.01f * 0.7f);
            App_Control_SetTurnSpeed(turnSpeed * 0.01f * 15.0f);
        }
    }
}

// 功能：逐字节组帧，支持 CR、LF、CRLF，完整行交给主循环。
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart3)
        return;

    // 1. 遇到结束符提交有效行；CRLF 中第二个结束符作为空行忽略。
    if (rx_buffer == '\r' || rx_buffer == '\n') {
        if (!lineOverflow && intBufCursor > 0U) {
            intBuf[intBufCursor] = '\0';
            strcpy(transBuf, intBuf);
            lineReceivedFlag = 1;
        }
        intBufCursor = 0;
        lineOverflow = 0;
    } else if (!lineOverflow) {
        // 2. 存放普通字符，并为字符串结束符保留一个字节。
        if (intBufCursor < sizeof(intBuf) - 1U) {
            intBuf[intBufCursor++] = rx_buffer;
        } else {
            lineOverflow = 1;
        }
    }

    // 3. 当前字节处理完，再启动下一字节接收。
    HAL_UART_Receive_IT(&huart3, &rx_buffer, 1);
}
