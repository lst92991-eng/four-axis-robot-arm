/**
 * @file common.h
 * @brief 定义模块之间交换的参数、状态及单位。
 */
#ifndef AT32_APPLICATION_COMMON_H
#define AT32_APPLICATION_COMMON_H
#include <stdint.h>
#include "config.h"

//---------运动状态区---------
typedef enum
{
  //关闭运动
  MOTION_OFF,
  //保持位置
  MOTION_HOLD,
  //运动到
  MOTION_MOVE,
  //制动
  MOTION_BRAKE,
  //故障锁定
  MOTION_FAULT,
  //手动持续速度运行，不设置机械角度终点。
  MOTION_VELOCITY
} motion_state_t;

//---------故障分类区---------
typedef enum
{
  // 无故障
  FAULT_NONE,
  // 编码器反馈异常
  FAULT_SENSOR,
  // 电流采样或过流异常
  FAULT_CURRENT,
  // 母线电压异常
  FAULT_VOLTAGE,
  // 控制时序或执行异常
  FAULT_CONTROL,
  // 行程、跟踪或停止异常
  FAULT_MOTION
} motor_fault_t;

//---------反馈数据区---------
typedef struct
{
  //编码器多圈计数
  int64_t encoder_counts;
  // J1 调试帧使用 mA 和 mV。
  int32_t trace_iq_ref, trace_iq, trace_vq;
  // 标定监督量：相电流使用 ADC counts，d 轴反馈换回同一单位。
  int32_t phase_a_counts, phase_b_counts, phase_a_mean, phase_b_mean, id_counts;
  uint32_t bus_mv;
  //编码器速度，计数/秒
  float velocity_counts_s;
  //反馈是否有效
  uint8_t encoder_valid;
  //功率输出是否使能
  volatile uint8_t power;
  //驱动故障
  volatile motor_fault_t fault;
} motor_status_t;

//---------系统时间区---------
//系统毫秒计数
extern volatile uint32_t system_ms;

#endif
