#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H
#include <stdint.h>

typedef struct {
    float target[2];       /* 斜率后的左右轮目标RPM，按车体前进为正。 */
    float measured[2];     /* 同一坐标下的一阶滤波RPM。 */
    int16_t pwm[2];        /* 同一坐标下的PWM；写驱动时再换成硬件方向。 */
} WheelStatus;

void Wheel_Init(void);
void Wheel_Stop(void);
/* 同向目标可更新，换向须先停稳；两轮超限时共同缩小。失败时停车。 */
uint8_t Wheel_SetTarget(float left_rpm, float right_rpm);
/* 纯几何换算：v单位mm/s，omega单位mrad/s，输出RPM。 */
uint8_t Wheel_Differential(float v, float omega, float *left, float *right);
/* 车体目标沿用原协议单位：mm/s、mrad/s、mm/s²；加减速度分别生效。 */
uint8_t Wheel_SetMotion(float v, float omega, float accel, float decel);
/* raw_*沿用第六章原始编码器RPM；每约20ms调用，停车时也更新滤波。 */
uint8_t Wheel_Update(float raw_left, float raw_right, float dt, uint8_t run);
void Wheel_Get(WheelStatus *status);
#endif
