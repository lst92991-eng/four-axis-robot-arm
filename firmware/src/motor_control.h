//电机采样、三环计算和功率控制接口
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "common.h"
#include "Common_Control.h"

//---------配置状态区---------
//初始化时确认的固定配置有效性，运行阶段只读
extern uint8_t motor_configured;

//---------初始化区---------
//初始化三环参数
void Motor_initialize(void);

//---------前台交接区---------
//命令受理和实际使能前检查配置、故障、反馈与供电
int Motor_canRun(void);
//记录主循环存活并读取同一次反馈，返回系统毫秒数
uint32_t Motor_readFeedback(motor_status_t *out);
//关中断交付位置目标和速度上限，供采样中断使用
int Motor_applyReference(void);
// 基本标定专用固定电角电流模式；不开放给正常运动命令。
int Motor_calibrationDrive(int enabled, int32_t angle_mdeg, int32_t current_ma);
void Motor_readCurrentFault(int32_t values[12]);
int Motor_setSpeedGains(int32_t kp_milli, int32_t ki_milli);
void Motor_readSpeedGains(int32_t values[3]);

//---------采样回调区---------
//电流采样驱动调用此入口
void Motor_control_callback(int32_t a, int32_t b);

//---------关断收尾区---------
//在控制中断或前台临界区内关断，保存首次故障并清除积分
void Motor_shutdown(motor_fault_t reason);
//满足失能、反馈和供电条件后清除故障
int Motor_clearFault(void);

#endif
