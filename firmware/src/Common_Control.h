//保存各环输入、积分和输出，计算函数依次更新这些数据
#ifndef COMMON_CONTROL_H
#define COMMON_CONTROL_H
#include <stdint.h>
#include "control_timing.h"
//---------时基参数区---------
//三环控制周期，单位秒；速度Ki在初始化时乘此周期
#define CONTROL_PERIOD_SECONDS (1.0 / CONTROL_CURRENT_LOOP_FREQUENCY_HZ)

//---------位置环数据区---------
typedef struct
{
  //位置比例增益
  float KP;
  //目标位置和反馈位置，编码器计数
  int32_t targetValue;
  int32_t currentValue;
  //速度给定上限和计算结果，计数/秒
  float limit;
  float out;
} Position_Struct;

//---------速度环数据区---------
typedef struct
{
  //速度环比例增益和每周期积分增益
  float KP;
  float KI;
  //给定速度和反馈速度，计数/秒
  float targetValue;
  float currentValue;
  //积分电流、电流上限和电流给定，mA
  float sumError;
  float limit;
  float out;
} Speed_Struct;

//---------电流环数据区---------
typedef struct
{
  //电流环比例增益和每周期积分增益
  float KP;
  float KI;
  //电流给定和电流反馈，mA
  float targetValue;
  float currentValue;
  //积分电压和输出电压，mV
  float sumError;
  float out;
} PI_Struct;

typedef struct
{
  //d轴和q轴电流环
  PI_Struct d;
  PI_Struct q;
  //电压矢量上限，mV
  int32_t voltageLimit;
  //两相输出电压，mV
  float outA;
  float outB;
} Current_Struct;

//---------组合数据区---------
typedef struct
{
  Position_Struct position;
  Speed_Struct speed;
  Current_Struct current;
} Motor_Control_Struct;

//---------控制计算接口区---------
//两相电流转为d/q反馈，电流单位mA，角度单位rad
void Common_Park_Compute(Current_Struct *current, int32_t a, int32_t b, float angle);
//位置反馈和目标位置生成速度给定
void Common_Position_Compute(Position_Struct *position);
//速度反馈和给定速度生成电流给定
void Common_Speed_Compute(Speed_Struct *speed);
//电流反馈和给定电流生成两相电压，角度单位rad
void Common_Current_Compute(Current_Struct *current, float angle, int32_t bus);

#endif
