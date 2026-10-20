#include "app_motor.h"
#include "pid.h"
#include "app_encoder.h"
#include "app_pwm.h"
#include "app_bat.h"

static PID_TypeDef motor_pid_l;   // 左轮速度 PID：输出目标电压，单位 V。
static PID_TypeDef motor_pid_r;   // 右轮速度 PID：输出目标电压，单位 V。
static uint8_t motor_enabled = 0; // 速度环使能状态：0 关闭，1 开启。

// 功能：配置左右轮速度 PI 的参数和输出电压限幅。
void App_Motor_Init(void) {
    PID_Init(&motor_pid_l, 0.5f, 7.0f, 0.0f);
    PID_LimitConfig(&motor_pid_l, -8.4f, +8.4f);
    PID_Init(&motor_pid_r, 0.5f, 7.0f, 0.0f);
    PID_LimitConfig(&motor_pid_r, -8.4f, +8.4f);
}


// 功能：用编码器速度反馈计算电机电压，再换算为 PWM 占空比。
void App_Motor_Proc(void) {
    // 1. 按标称 1ms 周期调度；关闭电机时不更新速度 PID。
    static uint32_t nxt = 0;       // 下一次计划执行时间，单位 ms。
    uint32_t now = HAL_GetTick();  // 当前时间，单位 ms。
    if (now < nxt)
        return;
    nxt = nxt + 1;

    if (motor_enabled == 0)
    {
        return;
    }

    // 2. 尚未取得电池采样或电压非正时，清零输出和 PID 历史。
    float vbat = App_Bat_GetVoltage(); // 当前电池电压，单位 V。
    if(g_bat_data_valid == 0 || vbat <= 0.0f )
    {
        App_PWM_SetDutyCycle_L(0.0f);
        App_PWM_SetDutyCycle_R(0.0f);

        PID_Reset(&motor_pid_l);
        PID_Reset(&motor_pid_r);
        return;
    }

    // 3. 读取左右轮速度，并通过速度 PID 算出目标电压。
    float omega_l = App_Encoder_GetSpeed_L(); // 左轮实际速度，单位 rad/s。
    float omega_r = App_Encoder_GetSpeed_R(); // 右轮实际速度，单位 rad/s。

    float output_l = PID_Compute(&motor_pid_l, omega_l); // 左轮目标电压，单位 V。
    float output_r = PID_Compute(&motor_pid_r, omega_r); // 右轮目标电压，单位 V。



    // 4. 目标电压除以电池电压，换算为有符号的占空比并输出。
    float duty_l = output_l / vbat * 100.0f; // 左轮占空比，单位 %。
    float duty_r = output_r / vbat * 100.0f; // 右轮占空比，单位 %。
    
    App_PWM_SetDutyCycle_L(duty_l);
    App_PWM_SetDutyCycle_R(duty_r);
}

// 功能：设置左轮目标速度，omega 单位为 rad/s。
void App_Motor_SetOmega_L(float omega)
{
    PID_SetPoint(&motor_pid_l, omega);
}
// 功能：设置右轮目标速度，omega 单位为 rad/s。
void App_Motor_SetOmega_R(float omega)
{
    PID_SetPoint(&motor_pid_r, omega); 
}
// 功能：读取左轮目标速度，单位 rad/s。
float App_Motor_GetOmegaSet_L(void)
{
    return PID_GetSetPoint(&motor_pid_l);
}

// 功能：读取右轮目标速度，单位 rad/s。
float App_Motor_GetOmegaSet_R(void)
{
    return PID_GetSetPoint(&motor_pid_r);
}

// 功能：启停电机，并在切换时清零 PWM 和速度 PID 历史；on 非零为开启。
void App_Motor_Cmd(uint8_t on)
{
    motor_enabled = (on != 0);

    // 1. 先让驱动待机，再清零左右轮 PWM 输出。
    App_PWM_Cmd(0);
    App_PWM_SetDutyCycle_L(0.0f);
    App_PWM_SetDutyCycle_R(0.0f);

    // 2. 清除速度 PID 的旧积分、误差和输出。
    PID_Reset(&motor_pid_l);
    PID_Reset(&motor_pid_r);

    // 3. 根据新的使能状态控制 STBY。
    App_PWM_Cmd(motor_enabled);
}
