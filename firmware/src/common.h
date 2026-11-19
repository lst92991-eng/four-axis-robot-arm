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

// SENSOR细分：1/2/4/8来自MT6816。突跳和采样超时同时出现时为48。
enum { SENSOR_COUNT_JUMP=16, SENSOR_SAMPLE_GAP=32, SENSOR_COUNT_RANGE=64, SENSOR_FEEDBACK_STALE=65 };

enum
{
  CONTROL_TRAJECTORY_GAP = 1, CONTROL_REFERENCE_STATE, CONTROL_REFERENCE_FEEDBACK,
  CONTROL_REFERENCE_BUS, CONTROL_REFERENCE_RANGE, CONTROL_ADC_MAIN_GAP,
  CONTROL_TICK_FEEDBACK, CONTROL_TICK_MAIN_GAP, CONTROL_CALIBRATION, CONTROL_CPU
};

//---------反馈数据区---------
typedef struct
{
  //编码器多圈计数
  int64_t encoder_counts;
  // J1 调试帧使用 mA 和 mV。
  int32_t trace_iq_ref, trace_iq, trace_vq;
  int16_t trace_vd;
  uint8_t trace_voltage_clipped;
  // 标定监督量：相电流使用 ADC counts，d 轴反馈换回同一单位。
  int32_t phase_a_counts, phase_b_counts, phase_a_mean, phase_b_mean, id_counts;
  uint16_t phase_a_peak_counts, phase_b_peak_counts;
  uint32_t bus_mv;
  uint16_t bus_trip_mv, bus_peak_mv;
  //编码器速度，计数/秒
  float velocity_counts_s;
  //反馈是否有效
  uint8_t encoder_valid;
  //功率输出是否使能
  volatile uint8_t power;
  //驱动故障
  volatile motor_fault_t fault;
  uint8_t fault_detail;
} motor_status_t;

//---------系统时间区---------
//系统毫秒计数
extern volatile uint32_t system_ms;

#endif
