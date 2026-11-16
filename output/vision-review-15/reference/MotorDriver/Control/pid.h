#ifndef PID_H
#define PID_H
#include <stdint.h>

/* 误差采用RPM，时间采用秒，输出采用PWM千分比。 */
typedef struct {
    float kp, ki, kd;
    float integral, previous_error;
    float output_limit, integral_limit;
    uint8_t has_previous;
} Pid;

/* 保留参数，只清除上一次控制过程的历史。 */
void Pid_Reset(Pid *pid);
/* 传入有效对象；增益和限幅由配置初始化，不在每次更新时逐项校验。
 * 成功返回1；周期异常返回0。调用者检查输入与输出数值。支持Ki=0和Kd=0。 */
uint8_t Pid_Update(Pid *pid, float target, float measured,
                   float feedforward, float dt, float *output);
#endif
