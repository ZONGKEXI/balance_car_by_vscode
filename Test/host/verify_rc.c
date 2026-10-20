#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Compile the real source so the test can inspect private PID targets and buffers. */
#include "../../Core/Src/app_control.c"
#include "../../Core/Src/app_rc.c"

UART_HandleTypeDef huart2 = { 2U };
UART_HandleTypeDef huart3 = { 3U };
static UART_HandleTypeDef *armed_uart;
static uint8_t *armed_data;
static uint32_t tick, irq_mask;
static float target_l, target_r;
static unsigned checks, failures, arm_calls;

#define CHECK(condition, message) do { \
    checks++; \
    if (!(condition)) { fprintf(stderr, "FAIL: %s\n", message); failures++; } \
} while (0)

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
void App_Motor_SetOmega_L(float omega) { target_l = omega; }
void App_Motor_SetOmega_R(float omega) { target_r = omega; }
float App_Encoder_GetSpeed_L(void) { return 0.0f; }
float App_Encoder_GetSpeed_R(void) { return 0.0f; }
float APP_MPU6050_GetGx(void) { return 0.0f; }
float APP_MPU6050_GetGz(void) { return 0.0f; }
float APP_MPU6050_GetPitch(void) { return 0.0f; }
void Error_Handler(void) { fputs("Unexpected Error_Handler\n", stderr); exit(2); }

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    CHECK(size == 1U, "receiver must request one byte");
    armed_uart = uart;
    armed_data = data;
    arm_calls++;
    return HAL_OK;
}

static int Near(float actual, float expected)
{
    return fabsf(actual - expected) < 0.00001f;
}

static void Start(void)
{
    tick += 10U;
    irq_mask = 0U;
    App_Control_Init();
    App_RC_Init();
}

static void Send(UART_HandleTypeDef *uart, const char *text)
{
    while (*text != '\0') {
        *armed_data = (uint8_t)*text++;
        HAL_UART_RxCpltCallback(uart);
    }
}

static void Process(void)
{
    tick += 10U;
    App_RC_Proc();
}

int main(void)
{
    Start();
    CHECK(armed_uart == &huart3, "Bluetooth reception must start on USART3");

    App_Control_SetMoveSpeed(0.14f);
    App_Control_SetTurnSpeed(0.75f);
    CHECK(Near(PID_GetSetPoint(&pid_velocity), 0.14f), "move interface must preserve 0.14 m/s");
    CHECK(Near(PID_GetSetPoint(&pid_turn), 0.75f), "turn interface must preserve 0.75 rad/s");

    Start();
    Send(&huart3, "move 0 -20\n");
    Process();
    CHECK(Near(PID_GetSetPoint(&pid_velocity), -0.14f), "negative move command must set negative movement");
    CHECK(Near(PID_GetSetPoint(&pid_turn), 0.0f), "straight command must clear turning");

    Start();
    Send(&huart3, "move -5 0\r");
    Process();
    CHECK(Near(PID_GetSetPoint(&pid_turn), -0.75f), "negative turn command with CR must set negative turning");
    App_Control_Proc();
    CHECK(Near(target_l - target_r, 1.5f), "negative turn command must make the left wheel faster");

    Start();
    Send(&huart3, "move 5 20\r\n");
    Process();
    CHECK(Near(PID_GetSetPoint(&pid_velocity), 0.14f), "positive move command with CRLF must set positive movement");
    CHECK(Near(PID_GetSetPoint(&pid_turn), 0.75f), "positive turn command with CRLF must set positive turning");
    App_Control_Proc();
    CHECK(Near(target_r - target_l, 1.5f), "positive turn command must make the right wheel faster");

    Start();
    Send(&huart3, "move 0 -20");
    Process();
    CHECK(Near(PID_GetSetPoint(&pid_velocity), 0.0f), "partial line must not change targets");
    Send(&huart3, "\n");
    irq_mask = 1U;
    Process();
    CHECK(Near(PID_GetSetPoint(&pid_velocity), -0.14f), "terminator must release a buffered command");
    CHECK(irq_mask == 1U, "processing must preserve a previously disabled IRQ state");
    irq_mask = 0U;

    Send(&huart3, "\r\n\ninvalid\nmove 0\n");
    Process();
    CHECK(Near(PID_GetSetPoint(&pid_velocity), -0.14f), "empty and invalid lines must retain the last target");

    Start();
    App_Control_SetMoveSpeed(0.14f);
    unsigned before = arm_calls;
    Send(&huart2, "move 0 0\n");
    Process();
    CHECK(arm_calls == before, "USART2 callback must not rearm the Bluetooth receiver");
    CHECK(Near(PID_GetSetPoint(&pid_velocity), 0.14f), "USART2 must not overwrite the Bluetooth target");

    Start();
    Send(&huart3, "move 0 -20");
    for (unsigned i = 0U; i < 64U; i++) Send(&huart3, " ");
    CHECK(intBufCursor < sizeof(intBuf), "long input must stay inside the receive buffer");
    if (intBufCursor < sizeof(intBuf)) {
        Send(&huart3, "\n");
        Process();
        CHECK(Near(PID_GetSetPoint(&pid_velocity), 0.0f), "an overlong command must be discarded entirely");
        Send(&huart3, "move 0 -20\n");
        Process();
        CHECK(Near(PID_GetSetPoint(&pid_velocity), -0.14f), "valid command after an overlong line must recover");
    } else {
        /* Avoid invoking the old, unsafe strcpy after a detected overflow. */
        App_RC_Init();
    }

    Send(&huart3, "move 0 0\n");
    Process();
    CHECK(Near(PID_GetSetPoint(&pid_velocity), 0.0f), "stop command must clear movement");
    CHECK(Near(PID_GetSetPoint(&pid_turn), 0.0f), "stop command must clear turning");
    CHECK(armed_uart == &huart3, "continuous reception must remain on USART3");

    printf("RC checks: %u; failures: %u\n", checks, failures);
    return failures == 0U ? 0 : 1;
}
