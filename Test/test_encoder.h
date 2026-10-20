// 编码器测试入口；外设初始化后选择一个调用，均阻塞且不返回。
#ifndef TEST_ENCODER_H
#define TEST_ENCODER_H

// 输出左右轮累计角度，发送后延时 50ms。
void Encoder_Test(void);

// 根据两次角度采样差和固定 1ms 间隔估算角速度。
void Encoder_M_Method_Test(void);

// 输出编码器接口提供的左右轮角速度。
void Encoder_T_Method_Test(void);

#endif
