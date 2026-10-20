#include "app_control.h"
#include "app_mpu6050.h"
#include "pid.h"
#include "qmath.h"
#include "app_motor.h"
#include "stm32f1xx_hal.h"
#include "app_encoder.h"
#include "app_rc.h"

static PID_TypeDef pid_velocity;     // 车身速度环：输出目标车身加速度。
static PID_TypeDef pid_theta;     // 车身倾角环：输出目标倾斜角速度。
static PID_TypeDef pid_theta_dot; // 车身角速度环：输出目标角加速度。
static PID_TypeDef pid_turn;     // 车身转向环：输出目标转向角速度。
static const float g = 9.8f;      // 重力加速度，单位 m/s^2。
static const float lp = 0.062;   // 倒立摆模型使用的长度参数，单位 m。
static const float r = 0.032;    // 轮胎半径，单位 m。
static float omega_ref = 0.0f;   // 积分得到的左右轮共同目标速度，单位 rad/s。
static uint32_t last_time = 0;   // 上一次控制计算时间，单位 ms。

// 功能：配置倾角环和角速度环的 PID 参数，并清除控制历史。
void App_Control_Init(void)
{
    // 1. 配置速度环PI、倾角 P 控制和角速度 PI 控制的参数、输出限幅。
    PID_Init(&pid_theta, 4.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_theta, -12.57f, 12.57f);
    PID_Init(&pid_theta_dot, 10.0f, 10.0f, 0.0f);
    PID_LimitConfig(&pid_theta_dot, -125.7f, 125.7f);
    PID_Init(&pid_velocity, 10.0f, 1.0f, 0.0f);
    PID_LimitConfig(&pid_velocity, -0.5f * g, 0.5f * g);
    PID_Init(&pid_turn, 1.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_turn, -15.0f, 15.0f);
    // 2. 将目标值、历史状态和计时起点复位。
    App_Control_Reset();
}

// 功能：清除累计轮速、平衡 PID 历史和左右轮目标，供启停时调用。
void App_Control_Reset(void)
{
    // 1. 清除累计目标轮速和两级平衡 PID 的历史。
    omega_ref = 0.0f;

    PID_SetPoint(&pid_velocity, 0.0f);
    PID_SetPoint(&pid_theta, 0.0f);
    PID_SetPoint(&pid_theta_dot, 0.0f);

    PID_Reset(&pid_velocity);
    PID_Reset(&pid_theta);
    PID_Reset(&pid_theta_dot);

    // 2. 清零左右轮目标，并重新建立控制计时起点。
    App_Motor_SetOmega_L(0.0f);
    App_Motor_SetOmega_R(0.0f);

    last_time = HAL_GetTick();
}

// 功能：根据车身倾角和角速度，计算平衡所需的左右轮目标速度。
void App_Control_Proc(void)
{
    // 1. 按标称 5ms 周期执行控制计算。
    static uint32_t nxt = 0;        // 下一次计划执行时间，单位 ms。
    uint32_t now = HAL_GetTick();   // 当前时间，单位 ms。
    if (now < nxt)
        return;
    nxt = nxt + 5;

    // 5. 速度环：根据目标直线加速度，计算目标车轮速度增量。
    //PID_SetPoint(&pid_velocity, 0);//如果不用遥控器，这个是保持原地不动的速度指令
    float omega = (App_Encoder_GetSpeed_L() + App_Encoder_GetSpeed_R()) * 0.5f; // 当前车轮平均角速度，rad/s。
    float theta_dot = APP_MPU6050_GetGx() * 0.01745329f; // 当前车身角速度，rad/s。
    float omega2 = -theta_dot * (lp + r) / r;//倒立摆反方向角速度rad/s（减去速度悖论）
    float omega1 = omega - omega2;//理论车辆角速度，rad/s。
    float x_dot = omega1 * r; // 目标车轮速度，m/s。
    float theta_ref = qtan(PID_Compute(&pid_velocity, x_dot) / g); // 目标车轮速度增量，m/s。
  
    // 2. 倾角环：以 0rad 为目标，计算目标车身角速度。
    PID_SetPoint(&pid_theta, theta_ref); 
    float theta = APP_MPU6050_GetPitch() * 0.01745329f; // 当前倾角，单位 rad。
    float pid_theta_ref = PID_Compute(&pid_theta, theta); // 目标车身角速度，rad/s。

    // 3. 角速度环：根据陀螺仪反馈，计算目标车身角加速度。
    PID_SetPoint(&pid_theta_dot, pid_theta_ref);

    float theta_dot_dot_ref = PID_Compute(&pid_theta_dot, theta_dot); // 目标角加速度，rad/s^2。

    // 4. 倒立摆逆解算：将目标角加速度换算为车轮轴的目标直线加速度。
    float x_dot_dot_ref = (g * qsin(theta) - theta_dot_dot_ref * lp) / qcos(theta); // 目标直线加速度，m/s^2。



    // . 按实际时间间隔积分，得到目标轮速。
    if(last_time != 0)
    {
        float delta_t = (now - last_time) * 0.001f; // 两次控制计算的时间差，单位 s。
        omega_ref += 1 / r * x_dot_dot_ref * delta_t;
    }
    last_time = now;

    // 6. 转向环：根据目标转向角速度，计算目标转向角加速度。
    float gz = APP_MPU6050_GetGz() * 0.01745329f; // 当前车身转向角速度，rad/s。
    float omega_diff = PID_Compute(&pid_turn, gz); // 目标转向角加速度，rad/s^2。

    // . 将共同目标轮速交给左右电机的速度环。
    App_Motor_SetOmega_L(omega_ref - omega_diff);
    App_Motor_SetOmega_R(omega_ref + omega_diff);
}


void App_Control_SetMoveSpeed(float moveSpeed) // 设置目标直线速度，m/s。
{
    PID_SetPoint(&pid_velocity, moveSpeed);
}

void App_Control_SetTurnSpeed(float turnSpeed) // 设置目标转向角速度，rad/s。
{
    // 将遥控目标交给转向环，保留小数。
    PID_SetPoint(&pid_turn, turnSpeed);
}
