# 小车硬件测试

三组测试代码已加入 CMake。编码器测试采用单函数独立入口。

| 文件 | 功能 |
|---|---|
| `test_voltage.c/.h` | 每 500ms 输出原始 ADC、ADC 引脚电压、电池换算电压；尚无首个样本时提示等待 |
| `test_encoder.c/.h` | `Encoder_Test()` 循环输出左右角度，每次延时 50ms |
| `test_pwm.c/.h` | 按键启动一轮左右及双轮正负占空比测试，再按一次停止 |

## 选择测试

当前 `main.c` 保持电量 LED 和按键主循环。测试编码器时，在初始化区取消这一行的注释：

```c
Encoder_Test();
```

该函数初始化编码器后进入自己的 `while (1)`，持续执行编码器测试。
USART2 由 `MX_USART2_UART_Init()` 初始化，保持工程配置的 **921600 波特率、8N1**。

电压测试使用 `Test_Voltage_Init()` 和 `Test_Voltage_Process()`；PWM 测试使用 `Test_PWM_Init()` 和 `Test_PWM_Process()`。Init 在初始化区调用，Process 在主循环调用。PWM 测试的 USER 按键由 `Test_PWM_Process()` 管理。

## 电压测试

输出示例：

```text
VOLT: configured VREF=3300mV divider=2.500; interval=500ms
VOLT: raw=3000 ADC=2.418V BAT=6.045V
```

`BAT` 沿用 `app_bat` 当前换算结果。分压系数和参考电压是软件配置值，需要按实际硬件核对，测试代码不修改它们。
`test_voltage.h` 的 `TEST_VOLTAGE_INTERVAL_MS` 控制输出周期。

## 编码器测试

电机保持待机，手动转轮观察。每行两列为左、右角度，单位为度，可供 VOFA 解析：

```text
0.00,0.00
12.83,12.83
-8.02,-8.02
```

测试直接调用当前 `App_Encoder_GetPos_L/R()`；它们返回的是角度。
静止时数值保持不变，转一侧仅改变对应一侧；按车体前进方向转轮时，两侧数值应都增加。
当前引脚：左 A=PB14、B=PB15；右 A=PB3、B=PB4。A 相双边沿计数。
`Encoder_Test()` 中的 `HAL_Delay(50)` 控制两次输出之间的延时。工程已开启 `printf` 浮点格式化支持。

## PWM 电机转动测试

测试时将车轮架空。进入 PWM 模式后初始待机，按下并松开 USER 启动一轮：

1. 左轮 +20%，停机，左轮 -20%，停机。
2. 右轮 +20%，停机，右轮 -20%，停机。
3. 双轮 +20%，停机，双轮 -20%，停机并结束。

每次运行 1500ms，停机间隔 500ms。运行中再次按下 USER，在按键确认后停止并关闭 STBY。整轮结束不会自动重复；上电一直按着按键也不会启动。

启动前在 STBY 低电平下检查 IN1/IN2 是否互补、正负命令是否换向；失败时输出 `FAIL` 并保持待机。
每次改变方向前先关闭 STBY、清零两轮占空比。

在 `test_pwm.h` 可修改整数占空比 `TEST_PWM_DUTY_PERCENT`、运行时间 `TEST_PWM_RUN_TIME_MS`、停机时间 `TEST_PWM_PAUSE_TIME_MS` 和消抖时间 `TEST_PWM_DEBOUNCE_MS`。
正负命令对应的车体方向需实际验证。Process 必须在主循环中持续调用，时间控制依赖主循环及时运行。

已修正 `app_pwm.c` 右轮负占空比分支：保留正向 IN1=0、IN2=1，负向改为 IN1=1、IN2=0。

## 自动验证

在项目根目录运行：

```powershell
.\Test\host\run.ps1
```

需要 PATH 中有 MinGW GCC。输出 `PASS: all behavioral checks` 表示软件行为检查通过。
检查编译真实 PWM 和编码器应用代码，模拟 GPIO、计时、ADC 数据源及串口传输；覆盖编码器 CSV 输出及 50ms 延时、电压输出节拍、计时回绕、换向、停机、消抖、整轮完成与方向故障。
实际 ADC 精度、PWM 波形和电机机械方向需要上板验证。
