#include "app_pwm.h"
#include "app_encoder.h"
#include "app_bat.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>
#include <setjmp.h>

#ifndef DRIVER_REGRESSION_ONLY
#include "test_voltage.h"
#include "test_encoder.h"
#include "test_pwm.h"
static jmp_buf encoder_test_exit;
static unsigned encoder_delay_count;
#endif

GPIO_TypeDef host_gpio_a;
GPIO_TypeDef host_gpio_b;
static TIM_TypeDef host_tim1, host_tim2, host_tim3, host_tim4;
TIM_HandleTypeDef htim1 = { &host_tim1, { 999U } };
TIM_HandleTypeDef htim2 = { &host_tim2, { 9999U } };
TIM_HandleTypeDef htim3 = { &host_tim3, { 65535U } };
TIM_HandleTypeDef htim4 = { &host_tim4, { 999U } };
uint16_t g_bat_adc_raw;
float g_bat_adc_voltage, g_bat_voltage;
volatile uint8_t g_bat_data_valid;

static uint32_t tick, irq_mask;
static unsigned failures;
static uint8_t stuck_right_in1, changed_direction_while_enabled;
static char serial_output[16384];

#define CHECK(condition, message) do { \
    if (!(condition)) { fprintf(stderr, "FAIL: %s\n", message); failures++; } \
} while (0)

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __set_PRIMASK(uint32_t value) { irq_mask = value; }

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (port->IDR & pin) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    uint32_t before = port->ODR;
    if (stuck_right_in1 && port == MOTOR_R_IN1_GPIO_Port && pin == MOTOR_R_IN1_Pin)
        state = GPIO_PIN_RESET;
    if (state == GPIO_PIN_SET) { port->ODR |= pin; port->IDR |= pin; }
    else { port->ODR &= ~(uint32_t)pin; port->IDR &= ~(uint32_t)pin; }
    uint8_t direction_pin =
        (port == GPIOA && (pin & (MOTOR_L_IN1_Pin | MOTOR_L_IN2_Pin)) != 0U) ||
        (port == GPIOB && (pin & (MOTOR_R_IN1_Pin | MOTOR_R_IN2_Pin)) != 0U);
    if (direction_pin && before != port->ODR &&
        (GPIOA->ODR & TB6612_STBY_Pin) != 0U)
        changed_direction_while_enabled = 1U;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    (void)timer; (void)channel; return HAL_OK;
}

void App_USART2_SendString(const char *text)
{
    size_t used = strlen(serial_output);
    snprintf(serial_output + used, sizeof(serial_output) - used, "%s", text);
}
uint16_t App_Bat_GetRaw(void) { return g_bat_adc_raw; }
float App_Bat_GetADCVoltage(void) { return g_bat_adc_voltage; }
float App_Bat_GetVoltage(void) { return g_bat_voltage; }

static void Reset(void)
{
    memset(&host_gpio_a, 0, sizeof(host_gpio_a));
    memset(&host_gpio_b, 0, sizeof(host_gpio_b));
    memset(&host_tim1, 0, sizeof(host_tim1));
    memset(&host_tim3, 0, sizeof(host_tim3));
    memset(&host_tim4, 0, sizeof(host_tim4));
    host_gpio_a.IDR = USER_KEY_Pin;
    tick = 0U; irq_mask = 0U;
    stuck_right_in1 = 0U; changed_direction_while_enabled = 0U;
    g_bat_data_valid = 0U;
    serial_output[0] = '\0';
    App_PWM_Init();
    App_Encoder_Init();
}

static void CheckRightReversal(void)
{
    Reset();
    App_PWM_SetDutyCycle_R(20.0f);
    GPIO_PinState positive1 = HAL_GPIO_ReadPin(MOTOR_R_IN1_GPIO_Port, MOTOR_R_IN1_Pin);
    GPIO_PinState positive2 = HAL_GPIO_ReadPin(MOTOR_R_IN2_GPIO_Port, MOTOR_R_IN2_Pin);
    App_PWM_SetDutyCycle_R(-20.0f);
    CHECK(positive1 != HAL_GPIO_ReadPin(MOTOR_R_IN1_GPIO_Port, MOTOR_R_IN1_Pin),
          "right PWM IN1 must reverse for negative duty");
    CHECK(positive2 != HAL_GPIO_ReadPin(MOTOR_R_IN2_GPIO_Port, MOTOR_R_IN2_Pin),
          "right PWM IN2 must reverse for negative duty");
    CHECK(host_tim4.CCR1 == 200U, "reversal preserves absolute PWM duty");
}

#ifndef DRIVER_REGRESSION_ONLY
static uint8_t Enabled(void)
{
    return HAL_GPIO_ReadPin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin) == GPIO_PIN_SET;
}

static void CheckVoltage(void)
{
    Reset();
    Test_Voltage_Init();
    serial_output[0] = '\0';
    tick = 499U; Test_Voltage_Process();
    CHECK(serial_output[0] == '\0', "voltage print must be rate limited");
    tick = 500U; Test_Voltage_Process();
    CHECK(strstr(serial_output, "waiting") != NULL, "no sample must not report a false zero battery");
    serial_output[0] = '\0';
    g_bat_adc_raw = 3000U; g_bat_adc_voltage = 2.418f;
    g_bat_voltage = 6.045f; g_bat_data_valid = 1U;
    irq_mask = 1U;
    tick = 1000U; Test_Voltage_Process();
    CHECK(strstr(serial_output, "raw=3000") != NULL, "voltage must include raw ADC");
    CHECK(strstr(serial_output, "ADC=2.418V") != NULL, "voltage must report the configured ADC result");
    CHECK(strstr(serial_output, "BAT=6.045V") != NULL, "voltage must report the configured battery result");
    CHECK(irq_mask == 1U, "voltage snapshot must preserve a previously disabled IRQ state");

    tick = UINT32_MAX - 100U; Test_Voltage_Init(); serial_output[0] = '\0';
    tick += 500U; Test_Voltage_Process();
    CHECK(strstr(serial_output, "raw=3000") != NULL, "voltage timer must survive tick wrap");
}

static void SetEncoder(GPIO_TypeDef *port, uint16_t a_pin, uint16_t b_pin, uint8_t a, uint8_t b)
{
    port->IDR &= ~(uint32_t)(a_pin | b_pin);
    if (a) port->IDR |= a_pin;
    if (b) port->IDR |= b_pin;
    HAL_GPIO_EXTI_Callback(a_pin);
}

void HAL_Delay(uint32_t delay)
{
    CHECK(delay == 50U, "simple encoder test must wait 50ms between lines");
    tick += delay;
    encoder_delay_count++;
    if (encoder_delay_count == 3U) { longjmp(encoder_test_exit, 1); }
    if (encoder_delay_count == 1U)
    {
        SetEncoder(GPIOB, MOTOR_L_ENCA_Pin, MOTOR_L_ENCB_Pin, 1U, 0U);
        SetEncoder(GPIOB, MOTOR_R_ENCA_Pin, MOTOR_R_ENCB_Pin, 1U, 1U);
    }
    else
    {
        SetEncoder(GPIOB, MOTOR_L_ENCA_Pin, MOTOR_L_ENCB_Pin, 0U, 0U);
        SetEncoder(GPIOB, MOTOR_L_ENCA_Pin, MOTOR_L_ENCB_Pin, 1U, 1U);
        SetEncoder(GPIOB, MOTOR_R_ENCA_Pin, MOTOR_R_ENCB_Pin, 0U, 1U);
        SetEncoder(GPIOB, MOTOR_R_ENCA_Pin, MOTOR_R_ENCB_Pin, 1U, 0U);
    }
}

static void CheckEncoder(void)
{
    Reset();
    SetEncoder(GPIOB, MOTOR_L_ENCA_Pin, MOTOR_L_ENCB_Pin, 1U, 0U);
    encoder_delay_count = 0U;
    if (setjmp(encoder_test_exit) == 0) { Encoder_Test(); }
    CHECK(strcmp(serial_output, "Encoder Test Start!\r\n0.00,0.00\r\n-0.80,-0.80\r\n0.80,0.80\r\n") == 0,
          "encoder test initializes position and prints two signed degree columns");
    CHECK(encoder_delay_count == 3U, "encoder test repeats output continuously");
}

static void CheckPWMIdleAndStop(void)
{
    Reset(); Test_PWM_Init();
    CHECK(!Test_PWM_IsRunning() && !Enabled(), "PWM init must not start motors");
    CHECK(Test_PWM_Start() != 0U && Test_PWM_IsRunning(), "PWM explicit start begins one sequence");
    CHECK(Enabled() && host_tim1.CCR1 == 200U && host_tim4.CCR1 == 0U,
          "first PWM phase only drives the left motor");
    Test_PWM_Stop();
    CHECK(!Test_PWM_IsRunning() && !Enabled(), "PWM stop must disable STBY");
    CHECK(host_tim1.CCR1 == 0U && host_tim4.CCR1 == 0U, "PWM stop zeros both outputs");
}

static void CheckPWMSequence(void)
{
    Reset(); tick = UINT32_MAX - 100U; Test_PWM_Init();
    CHECK(Test_PWM_Start() != 0U, "PWM sequence starts near tick wrap");
    for (unsigned phase = 0U; phase < 6U; phase++)
    {
        CHECK(Enabled(), "each running phase enables the driver");
        if (phase < 2U) CHECK(host_tim1.CCR1 == 200U && host_tim4.CCR1 == 0U, "left-only phase");
        else if (phase < 4U) CHECK(host_tim1.CCR1 == 0U && host_tim4.CCR1 == 200U, "right-only phase");
        else CHECK(host_tim1.CCR1 == 200U && host_tim4.CCR1 == 200U, "both-motor phase");
        tick += TEST_PWM_RUN_TIME_MS; Test_PWM_Process();
        CHECK(!Enabled() && host_tim1.CCR1 == 0U && host_tim4.CCR1 == 0U,
              "each run is followed by a disabled zero-duty interval");
        tick += TEST_PWM_PAUSE_TIME_MS; Test_PWM_Process();
    }
    CHECK(!Test_PWM_IsRunning() && !Enabled(), "PWM sequence ends instead of repeating");
    CHECK(!changed_direction_while_enabled, "PWM direction changes occur with STBY disabled");
    CHECK(strstr(serial_output, "complete") != NULL, "completed sequence is reported");
}

static void CheckPWMButton(void)
{
    Reset(); Test_PWM_Init();
    host_gpio_a.IDR &= ~(uint32_t)USER_KEY_Pin;
    tick = 1U; Test_PWM_Process();
    tick = TEST_PWM_DEBOUNCE_MS; Test_PWM_Process();
    CHECK(!Test_PWM_IsRunning(), "short key transitions do not start motors");
    tick = TEST_PWM_DEBOUNCE_MS + 1U; Test_PWM_Process();
    CHECK(Test_PWM_IsRunning(), "a debounced press starts PWM");
    tick += 50U; Test_PWM_Process();
    CHECK(Test_PWM_IsRunning(), "holding the key must not toggle repeatedly");
    host_gpio_a.IDR |= USER_KEY_Pin; tick += 1U; Test_PWM_Process();
    tick += TEST_PWM_DEBOUNCE_MS; Test_PWM_Process();
    host_gpio_a.IDR &= ~(uint32_t)USER_KEY_Pin; tick += 1U; Test_PWM_Process();
    tick += TEST_PWM_DEBOUNCE_MS; Test_PWM_Process();
    CHECK(!Test_PWM_IsRunning() && !Enabled(), "a second debounced press stops PWM");

    Reset(); host_gpio_a.IDR &= ~(uint32_t)USER_KEY_Pin; Test_PWM_Init();
    tick = 100U; Test_PWM_Process();
    CHECK(!Test_PWM_IsRunning(), "booting with the key held must not start motors");
}

static void CheckPWMDirectionFault(void)
{
    Reset(); Test_PWM_Init(); stuck_right_in1 = 1U;
    CHECK(Test_PWM_Start() == 0U, "direction output fault must reject motor start");
    CHECK(!Enabled() && !Test_PWM_IsRunning(), "direction fault leaves the driver disabled");
    CHECK(host_tim1.CCR1 == 0U && host_tim4.CCR1 == 0U, "direction fault zeros both duties");
}
#endif

int main(void)
{
    CheckRightReversal();
#ifndef DRIVER_REGRESSION_ONLY
    CheckVoltage(); CheckEncoder(); CheckPWMIdleAndStop();
    CheckPWMSequence(); CheckPWMButton(); CheckPWMDirectionFault();
#endif
    if (failures != 0U) { fprintf(stderr, "%u behavioral checks failed\n", failures); return 1; }
    puts("PASS: all behavioral checks");
    return 0;
}
