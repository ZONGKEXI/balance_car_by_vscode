#include "app_rgb.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>

/* Weak fallbacks keep the legacy driver runnable for the initial RED check.
 * A DMA implementation supplies strong callbacks and App_RGB_Proc. */
void App_RGB_Proc(void);
__attribute__((weak)) void App_RGB_Proc(void) { }
__attribute__((weak)) void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *timer)
{ (void)timer; }
__attribute__((weak)) void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *timer)
{ (void)timer; }

#define RESET_SLOTS 402u
#define DATA_SLOTS (RGB_LED_COUNT * 24u)
#define FRAME_SLOTS (2u * RESET_SLOTS + DATA_SLOTS)
#define MAX_STARTS 256u

GPIO_TypeDef host_gpio;
DWT_Type host_dwt;
CoreDebug_Type host_debug;
TIM_TypeDef host_tim2;
TIM_HandleTypeDef htim2;
static DMA_Channel_TypeDef dma_channel;
static DMA_HandleTypeDef dma;
static uint32_t tick, hclk, primask, ipsr;
static unsigned failures, test_count, start_count, stop_count, delay_count, irq_count;
static unsigned tick_calls, bad_start_state, concurrent_starts, ht_disable_count;
static unsigned early_buffer_changes, ug_count;
static uint8_t hardware_busy, hal_channel_busy;
static const uint16_t *active_buffer;
static uint16_t active_copy[FRAME_SLOTS], frames[MAX_STARTS][FRAME_SLOTS];
static uint16_t lengths[MAX_STARTS];
static HAL_StatusTypeDef next_start_status, next_stop_status;
static const char *current_test;

#define CHECK(condition, message) do { \
    if (!(condition)) { fprintf(stderr, "FAIL [%s]: %s (line %d)\n", \
        current_test, message, __LINE__); failures++; } \
} while (0)
#define REQUIRE(condition, message) do { \
    if (!(condition)) { CHECK(condition, message); return; } \
} while (0)

uint32_t HAL_GetTick(void) { tick_calls++; return tick; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return hclk; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return hclk / 2u; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clock, uint32_t *latency)
{ memset(clock, 0, sizeof(*clock)); clock->APB1CLKDivider = RCC_HCLK_DIV2; *latency = 2u; }
uint32_t __get_PRIMASK(void) { return primask; }
uint32_t __get_IPSR(void) { return ipsr; }
void __disable_irq(void) { irq_count++; primask = 1u; }
void __set_PRIMASK(uint32_t mask) { primask = mask; }
void HAL_Delay(uint32_t ms) { (void)ms; delay_count++; }

static void CheckActiveBuffer(void)
{
    if (hardware_busy && active_buffer != NULL &&
        memcmp(active_buffer, active_copy, sizeof(active_copy)) != 0)
        early_buffer_changes++;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *timer,
    uint32_t channel, const uint32_t *data, uint16_t length)
{
    HAL_StatusTypeDef result = hal_channel_busy ? HAL_BUSY : next_start_status;
    next_start_status = HAL_OK;
    CheckActiveBuffer();
    if (hardware_busy) concurrent_starts++;
    if (timer != &htim2 || channel != TIM_CHANNEL_1 ||
        timer->Instance->CCR1 != 0u || timer->Instance->CNT != 0u ||
        (timer->Instance->EGR & TIM_EVENTSOURCE_UPDATE) == 0u ||
        (timer->Instance->SR & (TIM_FLAG_UPDATE | TIM_FLAG_CC1)) != 0u)
        bad_start_state++;
    if (start_count < MAX_STARTS) {
        lengths[start_count] = length;
        if (length == FRAME_SLOTS)
            memcpy(frames[start_count], data, sizeof(frames[start_count]));
    }
    start_count++;
    /* The shipped STM32 HAL leaves channel state BUSY when DMA_Start_IT fails.
     * Retrying alone cannot recover; Stop_DMA must first release that state. */
    hal_channel_busy = 1u;
    if (result == HAL_OK) {
        hardware_busy = 1u;
        active_buffer = (const uint16_t *)data;
        if (length == FRAME_SLOTS) memcpy(active_copy, data, sizeof(active_copy));
        timer->Instance->CR1 |= TIM_CR1_CEN;
        timer->Instance->DIER |= TIM_DIER_CC1DE;
        dma_channel.CCR |= DMA_IT_HT;
    }
    return result;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *timer, uint32_t channel)
{
    HAL_StatusTypeDef result = next_stop_status;
    next_stop_status = HAL_OK;
    CheckActiveBuffer();
    CHECK(timer == &htim2 && channel == TIM_CHANNEL_1, "stop must target TIM2 channel 1");
    stop_count++;
    hardware_busy = 0u;
    hal_channel_busy = 0u;
    timer->Instance->CR1 &= ~TIM_CR1_CEN;
    timer->Instance->DIER &= ~TIM_DIER_CC1DE;
    return result;
}

HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *handle)
{ CHECK(handle == &dma, "abort must target RGB DMA"); hardware_busy = 0u; return HAL_OK; }
void Host_TIM_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{ CHECK(channel == TIM_CHANNEL_1, "RGB compare must target channel 1"); timer->Instance->CCR1 = value; }
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer, uint32_t event)
{
    timer->Instance->EGR = event;
    if ((event & TIM_EVENTSOURCE_UPDATE) != 0u) timer->Instance->SR |= TIM_FLAG_UPDATE;
    ug_count++;
    return HAL_OK;
}
void Host_DMA_DisableIT(DMA_HandleTypeDef *handle, uint32_t interrupt)
{ CHECK(handle == &dma, "interrupt control must target RGB DMA");
  handle->Instance->CCR &= ~interrupt; if ((interrupt & DMA_IT_HT) != 0u) ht_disable_count++; }

static void Reset(uint32_t start_tick)
{
    memset(&host_gpio, 0, sizeof(host_gpio));
    memset(&host_dwt, 0, sizeof(host_dwt));
    memset(&host_debug, 0, sizeof(host_debug));
    memset(&host_tim2, 0, sizeof(host_tim2));
    memset(&htim2, 0, sizeof(htim2));
    memset(&dma, 0, sizeof(dma));
    memset(&dma_channel, 0, sizeof(dma_channel));
    memset(frames, 0, sizeof(frames));
    memset(lengths, 0, sizeof(lengths));
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0u;
    htim2.Init.Period = 89u;
    htim2.hdma[TIM_DMA_ID_CC1] = &dma;
    dma.Instance = &dma_channel;
    dma.Parent = &htim2;
    tick = start_tick; hclk = 72000000u; primask = 0u; ipsr = 0u;
    start_count = stop_count = delay_count = irq_count = tick_calls = 0u;
    bad_start_state = concurrent_starts = ht_disable_count = 0u;
    early_buffer_changes = ug_count = 0u;
    hardware_busy = hal_channel_busy = 0u; active_buffer = NULL;
    next_start_status = next_stop_status = HAL_OK;
}

static void Complete(void)
{
    htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
    HAL_TIM_PWM_PulseFinishedCallback(&htim2);
    htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
}

static uint8_t DecodeByte(unsigned frame, unsigned offset)
{
    uint8_t value = 0u;
    for (unsigned bit = 0u; bit < 8u; bit++) {
        uint16_t slot = frames[frame][RESET_SLOTS + offset * 8u + bit];
        CHECK(slot == 22u || slot == 47u, "each data bit must use the 0/1 PWM pulse width");
        value = (uint8_t)((value << 1u) | (slot == 47u));
    }
    return value;
}

static void CheckColor(unsigned frame, unsigned led, uint8_t r, uint8_t g, uint8_t b)
{
    CHECK(DecodeByte(frame, led * 3u) == g, "GRB green byte differs");
    CHECK(DecodeByte(frame, led * 3u + 1u) == r, "GRB red byte differs");
    CHECK(DecodeByte(frame, led * 3u + 2u) == b, "GRB blue byte differs");
}

static void CheckWave(unsigned frame)
{
    REQUIRE(frame < start_count && frame < MAX_STARTS, "expected DMA frame was not submitted");
    REQUIRE(lengths[frame] == FRAME_SLOTS, "DMA frame must contain 402 reset slots + 120 data slots + 402 reset slots");
    for (unsigned n = 0u; n < RESET_SLOTS; n++) {
        CHECK(frames[frame][n] == 0u, "frame prefix must hold low");
        CHECK(frames[frame][RESET_SLOTS + DATA_SLOTS + n] == 0u, "frame suffix must hold low");
    }
}

static void CheckPalette(unsigned frame, uint8_t brightness)
{
    CheckWave(frame);
    CheckColor(frame, 0u, brightness, 0u, 0u);
    CheckColor(frame, 1u, 0u, brightness, 0u);
    CheckColor(frame, 2u, 0u, 0u, brightness);
    CheckColor(frame, 3u, brightness, brightness, 0u);
    CheckColor(frame, 4u, brightness, 0u, brightness);
}

static int InitAndComplete(uint32_t start_tick)
{
    Reset(start_tick);
    if (App_RGB_Init() != 1u || start_count != 1u) {
        CHECK(0, "valid initialization must enqueue one black DMA frame");
        return 0;
    }
    Complete();
    return 1;
}

static void TestInit(void)
{
    Reset(0u);
    host_tim2.CCR1 = 47u; host_tim2.CNT = 88u;
    host_tim2.SR = TIM_FLAG_UPDATE | TIM_FLAG_CC1;
    uint8_t initialized = App_RGB_Init();
    REQUIRE(start_count == 1u, "initialization must enqueue a black DMA frame without DWT timing");
    CHECK(initialized == 1u, "valid timer/DMA configuration must initialize successfully");
    CheckPalette(0u, 0u);
    CHECK(bad_start_state == 0u, "DMA launch must reset compare/counter and load the low level before starting");
    CHECK(ht_disable_count == 1u && (dma_channel.CCR & DMA_IT_HT) == 0u,
        "unused DMA half-transfer interrupt must be disabled");
    CHECK(delay_count == 0u && irq_count == 0u, "initialization must not wait or globally mask interrupts");
    Complete();
    CHECK(stop_count == 1u && host_tim2.CCR1 == 0u && ug_count >= 2u,
        "completion must stop DMA and load a low compare value");
}

static void TestPrerequisites(void)
{
    Reset(0u); hclk = 64000000u;
    CHECK(App_RGB_Init() == 0u && start_count == 0u, "wrong clock must reject initialization");
    Reset(0u); htim2.Instance = NULL;
    CHECK(App_RGB_Init() == 0u && start_count == 0u, "missing TIM2 must reject initialization");
    Reset(0u); htim2.Init.Prescaler = 1u;
    CHECK(App_RGB_Init() == 0u && start_count == 0u, "wrong timer prescaler must reject initialization");
    Reset(0u); htim2.Init.Period = 90u;
    CHECK(App_RGB_Init() == 0u && start_count == 0u, "wrong timer period must reject initialization");
    Reset(0u); htim2.hdma[TIM_DMA_ID_CC1] = NULL;
    CHECK(App_RGB_Init() == 0u && start_count == 0u, "missing channel DMA must reject initialization");
}

static void TestManualGRB(void)
{
    if (!InitAndComplete(0u)) return;
    App_RGB_SetColor(0u, 0x81u, 0x42u, 0x24u);
    App_RGB_SetColor(4u, 0xffu, 0x55u, 0xaau);
    App_RGB_SetColor(RGB_LED_COUNT, 0xffu, 0xffu, 0xffu);
    App_RGB_SetColor(255u, 0xffu, 0xffu, 0xffu);
    CHECK(start_count == 1u, "SetColor must only modify the cache");
    App_RGB_Show();
    REQUIRE(start_count == 2u, "Show must enqueue a manual frame");
    CheckWave(1u);
    CheckColor(1u, 0u, 0x81u, 0x42u, 0x24u);
    CheckColor(1u, 4u, 0xffu, 0x55u, 0xaau);
    for (unsigned led = 1u; led < 4u; led++) CheckColor(1u, led, 0u, 0u, 0u);
}

static void TestFailedReinitialization(void)
{
    if (!InitAndComplete(0u)) return;
    hclk = 48000000u;
    CHECK(App_RGB_Init() == 0u, "changed clock must reject reinitialization");
    App_RGB_SetColor(0u, 21u, 22u, 23u);
    App_RGB_Show();
    tick = 1000u; App_RGB_Proc();
    CHECK(start_count == 1u && !hardware_busy,
        "failed reinitialization must disable subsequent Show and Proc submissions");
    hclk = 72000000u;
    REQUIRE(App_RGB_Init() == 1u && start_count == 2u,
        "restoring the valid clock must permit initialization and a new black frame");
    CheckPalette(1u, 0u); Complete();
    tick = 1500u; App_RGB_Proc();
    REQUIRE(start_count == 3u, "breathing must resume after valid reinitialization");
    CheckPalette(2u, 16u);
}

static void TestBusyCoalescing(void)
{
    if (!InitAndComplete(0u)) return;
    App_RGB_SetColor(0u, 1u, 2u, 3u); App_RGB_Show();
    for (unsigned n = 4u; n < 10u; n++) {
        App_RGB_SetColor(0u, (uint8_t)n, (uint8_t)(n + 1u), (uint8_t)(n + 2u));
        App_RGB_Show(); App_RGB_Proc(); CheckActiveBuffer();
    }
    CHECK(start_count == 2u, "busy Show requests must coalesce until the current DMA frame finishes");
    CHECK(early_buffer_changes == 0u && concurrent_starts == 0u,
        "DMA source must remain unchanged while transmission is active");
    Complete();
    CHECK(start_count == 2u, "interrupt completion must leave the next DMA start to the main loop");
    App_RGB_Proc();
    REQUIRE(start_count == 3u, "Proc must send the latest queued cache after completion");
    CheckColor(2u, 0u, 9u, 10u, 11u);
    Complete(); App_RGB_Proc();
    CHECK(start_count == 3u, "coalesced requests must not leave duplicate frames queued");
}

static void TestClearWhileBusy(void)
{
    if (!InitAndComplete(0u)) return;
    for (uint8_t led = 0u; led < RGB_LED_COUNT; led++) App_RGB_SetColor(led, 255u, 255u, 255u);
    App_RGB_Show(); App_RGB_Clear(); CheckActiveBuffer();
    CHECK(start_count == 2u && early_buffer_changes == 0u, "Clear must queue black without changing active DMA data");
    Complete(); App_RGB_Proc();
    REQUIRE(start_count == 3u, "queued Clear must send after completion");
    CheckPalette(2u, 0u);
}

static void TestCallbackSelection(void)
{
    if (!InitAndComplete(0u)) return;
    App_RGB_Show();
    TIM_TypeDef other_instance = {0};
    TIM_HandleTypeDef other = {0}; other.Instance = &other_instance;
    other.Channel = HAL_TIM_ACTIVE_CHANNEL_1;
    unsigned before = stop_count;
    HAL_TIM_PWM_PulseFinishedCallback(&other); HAL_TIM_ErrorCallback(&other);
    htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_2; HAL_TIM_PWM_PulseFinishedCallback(&htim2);
    CHECK(stop_count == before && hardware_busy, "unrelated timers/channels must not end RGB transfer");
    App_RGB_Show(); CHECK(start_count == 2u, "ignored callbacks must keep the original DMA busy");
    Complete(); App_RGB_Proc();
    CHECK(start_count == 3u && concurrent_starts == 0u, "channel 1 completion must release the queued frame");
}

static void TestRefreshInterval(void)
{
    if (!InitAndComplete(0u)) return;
    tick = 19u; App_RGB_Proc(); CHECK(start_count == 1u, "breathing refresh must wait until 20ms");
    tick = 20u; App_RGB_Proc(); REQUIRE(start_count == 2u, "breathing must refresh at 20ms");
    CheckPalette(1u, 0u); Complete();
    tick = 39u; App_RGB_Proc(); CHECK(start_count == 2u, "refresh must not run faster than 20ms");
    tick = 40u; App_RGB_Proc(); CHECK(start_count == 3u, "second 20ms update must be submitted");
}

static void TestSynchronousBreathing(void)
{
    if (!InitAndComplete(0u)) return;
    const uint32_t moments[] = {500u, 1000u, 1500u, 2000u, 2500u};
    const uint8_t brightness[] = {16u, 32u, 16u, 0u, 16u};
    for (unsigned n = 0u; n < sizeof(moments) / sizeof(moments[0]); n++) {
        tick = moments[n]; App_RGB_Proc();
        REQUIRE(start_count == n + 2u, "each elapsed breathing interval must enqueue exactly one frame");
        CheckPalette(n + 1u, brightness[n]); Complete();
    }
}

static void TestTickRollover(void)
{
    const uint32_t origin = UINT32_MAX - 249u;
    if (!InitAndComplete(origin)) return;
    tick = origin + 500u; App_RGB_Proc();
    REQUIRE(start_count == 2u, "refresh must survive HAL tick rollover");
    CheckPalette(1u, 16u); Complete();
    tick = origin + 1000u; App_RGB_Proc();
    REQUIRE(start_count == 3u, "phase must continue after HAL tick rollover");
    CheckPalette(2u, 32u);
}

static void TestLargeElapsed(void)
{
    if (!InitAndComplete(0u)) return;
    tick = 1980u; App_RGB_Proc(); Complete();
    tick = 1979u; App_RGB_Proc();
    REQUIRE(start_count == 3u, "large unsigned elapsed interval must still refresh once");
    /* 1980 + (2^32 - 1) ms is phase 1275 of the 2000ms cycle. */
    CheckPalette(2u, 23u);
}

static void TestStartFailureRetry(void)
{
    if (!InitAndComplete(0u)) return;
    App_RGB_SetColor(0u, 17u, 18u, 19u);
    next_start_status = HAL_BUSY; App_RGB_Show();
    CHECK(start_count == 2u && !hardware_busy, "failed HAL start must not mark hardware busy");
    App_RGB_Proc();
    REQUIRE(start_count == 3u && hardware_busy, "main loop must retry a failed DMA start");
    CheckColor(2u, 0u, 17u, 18u, 19u); Complete();
    next_start_status = HAL_ERROR; App_RGB_Show(); App_RGB_Proc();
    CHECK(start_count == 5u && hardware_busy, "HAL error start must also recover");
}

static void TestErrorRecovery(void)
{
    if (!InitAndComplete(0u)) return;
    App_RGB_SetColor(0u, 7u, 8u, 9u); App_RGB_Show();
    htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_1; HAL_TIM_ErrorCallback(&htim2);
    CHECK(!hardware_busy && host_tim2.CCR1 == 0u, "DMA error must stop transfer and return the output to low");
    App_RGB_Proc();
    REQUIRE(start_count == 3u, "DMA error must schedule a retry in the main loop");
    CheckColor(2u, 0u, 7u, 8u, 9u);
}

static void TestTimeoutRecovery(void)
{
    if (!InitAndComplete(UINT32_MAX - 1u)) return;
    unsigned stops_before = stop_count;
    App_RGB_SetColor(0u, 11u, 12u, 13u); App_RGB_Show();
    tick = 0u; App_RGB_Proc();
    CHECK(start_count == 2u && stop_count == stops_before, "2ms transfer must not trigger the 3ms timeout");
    tick = 1u; App_RGB_Proc();
    REQUIRE(stop_count == stops_before + 1u && start_count == 3u, "3ms timeout across tick rollover must abort and retry");
    CheckColor(2u, 0u, 11u, 12u, 13u);
    CHECK(concurrent_starts == 0u && early_buffer_changes == 0u, "timeout recovery must stop the old DMA before rebuilding");
    Complete();
    App_RGB_SetColor(0u, 21u, 22u, 23u); App_RGB_Show();
    CHECK(start_count == 4u, "normal Show must remain usable after timeout recovery");
}

static void TestContextAndNonblocking(void)
{
    if (!InitAndComplete(0u)) return;
    App_RGB_SetColor(0u, 1u, 2u, 3u);
    ipsr = 1u; App_RGB_Show();
    CHECK(start_count == 1u, "Show must not launch DMA from interrupt context");
    ipsr = 0u; primask = 1u; App_RGB_Show();
    CHECK(start_count == 1u, "Show must not launch DMA while interrupts are globally masked");
    primask = 0u; App_RGB_Show();
    CHECK(start_count == 2u, "Show must work again in the main loop");
    unsigned before_ticks = tick_calls;
    for (unsigned n = 0u; n < 32u; n++) { App_RGB_Show(); App_RGB_Proc(); }
    CHECK(start_count == 2u && concurrent_starts == 0u, "busy calls must return without another DMA start");
    CHECK(tick_calls - before_ticks <= 128u, "busy APIs must not poll the clock waiting for DMA completion");
    CHECK(irq_count == 0u && delay_count == 0u, "RGB APIs must never globally mask interrupts or delay");
}

static void Run(const char *name, void (*test)(void))
{
    current_test = name; test_count++; test();
}

int main(int argc, char **argv)
{
    Run("initial black DMA frame", TestInit);
    if (argc < 2 || strcmp(argv[1], "--init-only") != 0) {
        Run("initialization prerequisites", TestPrerequisites);
        Run("manual GRB encoding and index bounds", TestManualGRB);
        Run("failed reinitialization disables output", TestFailedReinitialization);
        Run("busy frame coalescing", TestBusyCoalescing);
        Run("Clear while busy", TestClearWhileBusy);
        Run("timer callback routing", TestCallbackSelection);
        Run("20ms refresh interval", TestRefreshInterval);
        Run("synchronized 2s breathing", TestSynchronousBreathing);
        Run("HAL tick rollover", TestTickRollover);
        Run("large elapsed phase", TestLargeElapsed);
        Run("failed HAL start retry", TestStartFailureRetry);
        Run("DMA error recovery", TestErrorRecovery);
        Run("3ms timeout recovery", TestTimeoutRecovery);
        Run("nonblocking foreground calls", TestContextAndNonblocking);
    }
    if (failures != 0u) {
        fprintf(stderr, "%u failure(s) in %u RGB DMA test(s)\n", failures, test_count);
        return 1;
    }
    printf("PASS: %u RGB DMA host tests\n", test_count);
    return 0;
}
