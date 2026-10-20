#include "pid.h"

// 初始化 PID 系数、目标值和历史状态。
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd) {
    // 1. 设置比例、积分和微分系数。
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    // 2. 清空目标值和历史状态，记录起始时间。
    pid->setpoint = 0.0f;
    pid->last_update_time = HAL_GetTick();
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->error = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->previous_integral = 0.0f;
    // 3. 默认不限制输出幅度。
    pid->Upperlimit = +3.4e+38f;
    pid->Lowerlimit = -3.4e+38f;
}

// 配置总输出及积分项输出的上下限。
void PID_LimitConfig(PID_TypeDef *pid, float lower, float upper) {
    pid->Lowerlimit = lower;
    pid->Upperlimit = upper;
}

// 修改目标值，保留历史积分和误差。
void PID_SetPoint(PID_TypeDef *pid, float setpoint) {
    pid->setpoint = setpoint;
}

// 获取当前目标值。
float PID_GetSetPoint(PID_TypeDef *pid) {
    return pid->setpoint;
}

// 根据反馈值计算位置式 PID 输出，并保存本次状态。
float PID_Compute(PID_TypeDef *pid, float feedback) {
    // 1. 计算采样间隔；同一毫秒内调用时返回上次输出。
    uint32_t now = HAL_GetTick(); // 当前时刻，ms。
    float dt = (float)(now - pid->last_update_time) / 1000.0f; // 采样间隔，s。
    if (dt <= 0.0f)
        return pid->output;

    // 2. 求目标与反馈的误差，作为 P 项输入。
    pid->error = pid->setpoint - feedback;

    if(pid->last_update_time != 0)
    {
    // 3. 求 D 项误差变化率和 I 项梯形积分。
    pid->derivative = (pid->error - pid->previous_error) / dt;

    // 两次误差的平均值乘以采样间隔，再累加到历史积分。
    pid->integral = pid->previous_integral + (pid->error + pid->previous_error) * dt * 0.5f;
    }

    // 4. 限制 ki * integral 的幅度，避免积分持续累积。
    if (pid->ki != 0.0f) {
        float integral_output = pid->ki * pid->integral; // I 项输出。
        if (integral_output > pid->Upperlimit) {
            pid->integral = pid->Upperlimit / pid->ki;
        } else if (integral_output < pid->Lowerlimit) {
            pid->integral = pid->Lowerlimit / pid->ki;
        }
    } else {
        pid->integral = 0.0f;
    }

    // 5. 合成 P、I、D 三项并限制总输出。
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative;
    
    if (pid->output > pid->Upperlimit) {
        pid->output = pid->Upperlimit;
    } else if (pid->output < pid->Lowerlimit) {
        pid->output = pid->Lowerlimit;
    }
    
    // 6. 保存误差、积分和时间，供下次计算使用。
    pid->previous_error = pid->error;
    pid->previous_integral = pid->integral;
    pid->last_update_time = now;
    
    return pid->output;
}

// 清除历史状态，保留系数、目标值和限幅配置。
void PID_Reset(PID_TypeDef *pid)
{
    pid->error = 0.0f;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->previous_integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->last_update_time = HAL_GetTick();
}
