//关节运动状态和轨迹计算
#ifndef AT32_MOTION_H
#define AT32_MOTION_H
#include "common.h"

//---------运动状态区---------
typedef struct
{
  //当前运动状态和故障
  motion_state_t state;
  motor_fault_t fault;
  // 运动故障细分：1实际越界，2参考越界，3轨迹超时，4跟随，5停车终点越界。
  uint8_t fault_detail;
  //机械零点是否建立
  uint8_t referenced;
  //是否到达命令位置
  uint8_t arrived;
  //失联后是否禁止新的运动
  uint8_t interlocked;
  //反馈速度是否满足停稳条件
  uint8_t stationary;
  //输出轴位置、速度和命令目标
  float position;
  float velocity;
  float requested;
} motion_t;

//---------轨迹数据区---------
typedef struct
{
  //前台读取的运动结果，由运动模块更新
  motion_t status;
  //本次运动的输出轴速度上限，单位度/秒
  float speed_limit;
  //轨迹参考位置和速度，输出轴单位
  // 多圈长行程不能用float逐毫秒累加：误差会累计成跟随故障。
  double ref_pos;
  float ref_vel;
  float manual_speed;
  float brake_acceleration;
  uint8_t manual_velocity;
  //机械零点对应的电机计数和输出轴角度
  int64_t zero_counts;
  float zero_degrees;
  //本段运动已经运行的时间，单位秒
  double elapsed_s;
  //本段运动允许的最长时间，单位秒
  double timeout_s;
  //连续满足到位条件的时间，单位毫秒
  uint32_t settle_ms;
} motion_control_t;
extern motion_control_t Joint;

//---------机械参考区---------
int Motion_setZero(int64_t counts, float degrees);
//将输出轴参考角换算为电机累计计数
double Motion_calculateMotorTarget(void);

//---------运动命令区---------
//固定配置初始化有效后受理目标角度和速度
int Motion_moveTo(float degrees, float speed);
int Motion_runVelocity(float degrees_per_second);
int Motion_setBrakeAcceleration(float degrees_per_second_squared);
float Motion_brakeAcceleration(void);
int Motion_stop(int link_lost);
void Motion_release(void);

//---------轨迹更新区---------
//主循环仅在时间前进时调用，elapsed_ms为正毫秒数
void Motion_update(int64_t counts, uint32_t elapsed_ms, float measured_velocity);

//---------故障处理区---------
void Motion_clearFault(void);
void Motion_trip(motor_fault_t reason);

#endif
