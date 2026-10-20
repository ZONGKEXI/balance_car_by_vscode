#include <math.h>
#include <stdio.h>
#include "app_bat.h"
#include "adc.h"
#include "tim.h"

GPIO_TypeDef host_gpio_a;
GPIO_TypeDef host_gpio_b;
ADC_TypeDef host_adc_1;
ADC_TypeDef host_adc_2;
TIM_HandleTypeDef htim2;

static uint32_t tick;
static uint32_t raw;
static unsigned start_calls;
static unsigned calibration_calls;
static unsigned timer_calls;
static unsigned failures;
static unsigned error_calls;
static uint8_t complete_during_start;
static HAL_StatusTypeDef start_result;
static ADC_InjectionConfTypeDef injected_config;

#define CHECK(condition, message) do { \
    if (!(condition)) { fprintf(stderr, "FAIL: %s (line %d)\n", message, __LINE__); failures++; } \
} while (0)

uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) port->ODR |= pin;
    else port->ODR &= ~(uint32_t)pin;
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *adc)
{
    CHECK(adc == &hadc1, "calibrate ADC1");
    calibration_calls++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *adc)
{
    CHECK(adc == &hadc1, "start ADC1");
    CHECK(calibration_calls != 0U, "calibrate before starting");
    start_calls++;
    if ((start_result == HAL_OK) && complete_during_start)
        HAL_ADCEx_InjectedConvCpltCallback(adc);
    return start_result;
}
uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef *adc, uint32_t rank)
{
    CHECK(adc == &hadc1, "read ADC1");
    CHECK(rank == ADC_INJECTED_RANK_1, "read injected rank 1");
    return raw;
}
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *timer)
{
    (void)timer;
    timer_calls++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *adc) { (void)adc; return HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *adc, ADC_InjectionConfTypeDef *config)
{
    CHECK(adc == &hadc1, "configure ADC1");
    injected_config = *config;
    return HAL_OK;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config) { (void)port; (void)config; }
void HAL_GPIO_DeInit(GPIO_TypeDef *port, uint32_t pins) { (void)port; (void)pins; }
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority)
{ (void)irq; (void)priority; (void)subpriority; }
void HAL_NVIC_EnableIRQ(uint32_t irq) { (void)irq; }
void HAL_NVIC_DisableIRQ(uint32_t irq) { (void)irq; }
void Error_Handler(void) { error_calls++; }

static void begin(uint32_t now)
{
    tick = now;
    raw = 2048U;
    start_calls = calibration_calls = timer_calls = 0U;
    complete_during_start = 0U;
    start_result = HAL_OK;
    g_bat_data_valid = 0U;
    host_gpio_a.ODR = BAT_LED1_Pin | BAT_LED2_Pin | BAT_LED3_Pin;
    App_Bat_Init();
}

static void test_init(void)
{
    begin(100U);
    CHECK(calibration_calls == 1U, "init calibrates once");
    CHECK(start_calls == 1U, "init immediately starts one conversion");
    CHECK(timer_calls == 0U, "battery no longer starts TIM2");
    CHECK(host_gpio_a.ODR == 0U, "init switches off battery LEDs");
}

static void test_configuration(void)
{
    MX_ADC1_Init();
    CHECK(error_calls == 0U, "ADC init succeeds");
    CHECK(injected_config.ExternalTrigInjecConv == ADC_INJECTED_SOFTWARE_START,
          "injected group uses software trigger");
    CHECK(injected_config.InjectedChannel == ADC_CHANNEL_8, "battery remains on channel 8");
    CHECK(injected_config.InjectedNbrOfConversion == 1U, "one injected conversion per start");
}

static void test_interval(void)
{
    begin(100U);
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    tick = 109U;
    App_Bat_Proc();
    CHECK(start_calls == 1U, "no conversion before 10 ms");
    tick = 110U;
    App_Bat_Proc();
    CHECK(start_calls == 2U, "start at 10 ms");
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    App_Bat_Proc();
    CHECK(start_calls == 2U, "same tick cannot start twice");
    tick = 120U;
    App_Bat_Proc();
    CHECK(start_calls == 3U, "start next 10 ms sample");
}

static void test_pending_conversion(void)
{
    ADC_HandleTypeDef other = { .Instance = ADC2 };
    begin(0U);
    for (tick = 10U; tick <= 100U; tick += 10U) App_Bat_Proc();
    CHECK(start_calls == 1U, "pending conversion is never restarted");
    HAL_ADCEx_InjectedConvCpltCallback(&other);
    App_Bat_Proc();
    CHECK(start_calls == 1U, "another ADC callback cannot release the battery conversion");
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    App_Bat_Proc();
    CHECK(start_calls == 2U, "ADC1 callback releases pending conversion");
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    App_Bat_Proc();
    CHECK(start_calls == 2U, "late polling does not burst catch-up samples");
}

static void test_failed_start_retry(void)
{
    begin(0U);
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    tick = 10U;
    start_result = HAL_BUSY;
    App_Bat_Proc();
    CHECK(start_calls == 2U, "attempt due conversion");
    start_result = HAL_ERROR;
    App_Bat_Proc();
    CHECK(start_calls == 3U, "busy return leaves conversion available for retry");
    start_result = HAL_OK;
    App_Bat_Proc();
    CHECK(start_calls == 4U, "error return can retry next loop without waiting");
    App_Bat_Proc();
    CHECK(start_calls == 4U, "successful retry marks conversion active");
}

static void test_tick_wraparound(void)
{
    begin(UINT32_MAX - 5U);
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    tick = 3U;
    App_Bat_Proc();
    CHECK(start_calls == 1U, "wraparound at 9 ms still waits");
    tick = 4U;
    App_Bat_Proc();
    CHECK(start_calls == 2U, "unsigned tick difference reaches 10 ms across wraparound");
}

static void test_callback_during_start(void)
{
    begin(0U);
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    complete_during_start = 1U;
    tick = 10U;
    App_Bat_Proc();
    tick = 20U;
    App_Bat_Proc();
    CHECK(start_calls == 3U, "completion before start returns cannot leave active stuck");
}

static void test_values_and_leds(void)
{
    begin(0U);
    raw = 2048U;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    CHECK(App_Bat_GetRaw() == raw, "retain raw ADC value");
    CHECK(fabsf(App_Bat_GetADCVoltage() - ((float)raw / 4095.0f * 3.3f)) < 0.00001f,
          "retain ADC voltage conversion");
    CHECK(fabsf(App_Bat_GetVoltage() - ((float)raw / 4095.0f * 3.3f * 2.5f)) < 0.00001f,
          "retain divider voltage conversion");
    CHECK(g_bat_data_valid == 1U, "callback marks data valid");
    g_bat_voltage = 8.0f;
    App_Bat_LED_Process();
    CHECK(host_gpio_a.ODR == (BAT_LED1_Pin | BAT_LED2_Pin | BAT_LED3_Pin), "full voltage lights three LEDs");
    g_bat_voltage = 7.5f;
    App_Bat_LED_Process();
    CHECK(host_gpio_a.ODR == (BAT_LED1_Pin | BAT_LED2_Pin), "75 percent lights two LEDs");
    g_bat_voltage = 7.1f;
    App_Bat_LED_Process();
    CHECK(host_gpio_a.ODR == BAT_LED1_Pin, "50 percent lights one LED");
    g_bat_voltage = 6.8f;
    App_Bat_LED_Process();
    CHECK(host_gpio_a.ODR == 0U, "low battery turns LEDs off");
    g_bat_voltage = 6.5f;
    tick = 100U;
    App_Bat_LED_Process();
    CHECK(host_gpio_a.ODR == (BAT_LED1_Pin | BAT_LED2_Pin | BAT_LED3_Pin), "low alarm toggles after 100 ms");
    tick = 200U;
    App_Bat_LED_Process();
    CHECK(host_gpio_a.ODR == 0U, "low alarm toggles off after another 100 ms");
    g_bat_data_valid = 0U;
    host_gpio_a.ODR = BAT_LED1_Pin;
    App_Bat_LED_Process();
    CHECK(host_gpio_a.ODR == 0U, "invalid sample switches off LEDs");
}

int main(void)
{
    hadc1.Instance = ADC1;
    test_init();
    test_configuration();
    test_interval();
    test_pending_conversion();
    test_failed_start_retry();
    test_tick_wraparound();
    test_callback_during_start();
    test_values_and_leds();
    if (failures) {
        fprintf(stderr, "%u battery check(s) failed\n", failures);
        return 1;
    }
    puts("PASS: 8 battery scenarios (software trigger, timing, active guard, retry, wraparound, callback race, values and LEDs)");
    return 0;
}
