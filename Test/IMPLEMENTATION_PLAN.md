# Hardware Test Modules Implementation Plan

> **For agentic workers:** Execute inline with executing-plans and test-driven-development. The user requested complete implementation in the current folder.

**Goal:** Provide complete voltage, encoder and PWM motor tests in `Test`, selectable in `main.c`.

**Architecture:** Reuse `app_bat`, `app_encoder`, `app_pwm` and USART2. Poll test processes in the main loop. Select exactly one mode; PWM mode also polls encoder feedback and owns the USER button.

**Tech Stack:** STM32F103 HAL, C11, CMake/Ninja, MinGW GCC for hardware-independent checks.

## Constraints and decisions

- Work directly in the named workspace. It is not a Git repository; no worktree or Git commits are created.
- Default mode is ENCODER. PWM initializes with STBY low; USER starts or stops a single sequence.
- Preserve the current right motor positive direction. Invert only its erroneous negative branch.
- Print configured battery scaling, not a claim that the divider was measured.
- Encoder feedback is counts and counts per actual sample interval; no unspecified PPR or gear ratio is assumed.
- HAL and serial transport are simulated for host checks; actual PWM and encoder application code is compiled into those checks.

## Task 1: Driver regression and host checks

Files: `Test/host/include/stm32f1xx_hal.h`, `Test/host/verify_tests.c`, `Test/host/run.ps1`.

- [x] Reproduce the right-direction defect with actual `App_PWM_SetDutyCycle_R`:

```c
App_PWM_SetDutyCycle_R(20.0f);
GPIO_PinState positive = HAL_GPIO_ReadPin(MOTOR_R_IN1_GPIO_Port, MOTOR_R_IN1_Pin);
App_PWM_SetDutyCycle_R(-20.0f);
CHECK(positive != HAL_GPIO_ReadPin(MOTOR_R_IN1_GPIO_Port, MOTOR_R_IN1_Pin));
```

- [x] Run `Test/host/run.ps1 -RegressionOnly`; expect failure because both branches set the same direction.
- [x] Add behavioral checks before the new modules: voltage rate and pending sample, encoder delta and tick wrap, PWM idle/start/stop, button debounce, complete sequence, direction fault abort.

## Task 2: Six requested files

Files: `Test/test_voltage.c/.h`, `Test/test_encoder.c/.h`, `Test/test_pwm.c/.h`.

- [x] Implement voltage and encoder interfaces:

```c
void Test_Voltage_Init(void);
void Test_Voltage_Process(void);
void Test_Encoder_Init(void);
void Test_Encoder_Process(void);
```

Voltage: every 500ms, snapshot the ready flag, raw ADC, ADC voltage and battery voltage with interrupt state restored, then print outside the snapshot.
Encoder: every 100ms, snapshot both cumulative counts and print signed changes plus actual elapsed milliseconds; unsigned tick arithmetic handles wrap.

- [x] Implement the PWM interface:

```c
void Test_PWM_Init(void);
void Test_PWM_Process(void);
uint8_t Test_PWM_Start(void);
void Test_PWM_Stop(void);
uint8_t Test_PWM_IsRunning(void);
```

Sequence: left positive, left negative, right positive, right negative, both positive, both negative; each runs at 20% for 1500ms, followed by 500ms standby. Disable STBY and zero both CCRs before changing direction. Debounce the USER key for 30ms. Check complementary and reversible direction outputs before enabling STBY. End or stop with STBY low and zero duty.

## Task 3: Integration and verification

Files: `Core/Inc/app_bat.h`, `Core/Src/app_pwm.c`, `Core/Src/main.c`, `CMakeLists.txt`, `Test/README.md`.

- [x] Declare existing `extern volatile uint8_t g_bat_data_valid` in the battery header.
- [x] Correct right negative direction to IN1 high, IN2 low; retain positive IN1 low, IN2 high.
- [x] Add all three test source files and `Test` include path to CMake.
- [x] Select modes in a CubeMX USER CODE block:

```c
#define TEST_MODE_VOLTAGE 1
#define TEST_MODE_ENCODER 2
#define TEST_MODE_PWM 3
#ifndef APP_TEST_MODE
#define APP_TEST_MODE TEST_MODE_ENCODER
#endif
```

- [x] Run `Test/host/run.ps1`; expect all behavioral checks to pass.
- [x] Build the STM32 Debug target for all three mode values; expect compile and link success. Restore the default ENCODER mode.
- [x] Review the entire change, especially stop behavior, right reversal and initial button state.

## Execution record

- Context checked: current drivers and generated pin mapping; right PWM direction defect confirmed in source.
- Driver regression RED: `Test/host/run.ps1 -RegressionOnly` reproduced two right-direction failures.
- Full behavioral RED before fixing the driver: 20 failed checks because the PWM test correctly rejected the non-reversing right output; voltage/encoder checks did not report failures.
- Six requested files and integration implemented; full verification pending.

- Verification GREEN: host behavioral checks passed under GCC with -Wall -Wextra -Werror.
- STM32 Debug compile/link passed for VOLTAGE, PWM and the restored default ENCODER mode; build logs contain no warning/error diagnostics.
- CMake override restored to empty; main.c remains in ENCODER mode.
- Independent final review found no material issues in sequence stop/start, polarity, snapshots, wrap handling, tests or integration.
- No physical board flashing or hardware execution was performed. ADC accuracy, waveforms and mechanical direction remain board validation.
