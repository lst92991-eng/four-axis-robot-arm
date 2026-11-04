//初始化控制参数，交接前台数据，执行采样闭环并统一关断
#include "motor_control.h"
#include "motion.h"
#include "at32m412_416_conf.h"
#include "current_sense.h"
#include "current_limits.h"
#include "pwm_output.h"
#include "mt6816.h"
#include <math.h>
#include <limits.h>
#include <stdlib.h>
//---------运行数据区---------
//采样和三环运行数据由本模块保存
static struct
{
  motor_status_t state;
  Motor_Control_Struct control;
  //滤波后的电机速度，编码器计数/秒
  float speed;
  //最近一次ADC采样、编码器采样和前台执行时刻，毫秒
  volatile uint32_t last_adc_ms;
  volatile uint32_t last_encoder_ms;
  volatile uint32_t last_main_ms;
  //四次ADC采样执行一次三环计算
  uint8_t divider;
} Drive;

uint8_t motor_configured;

//---------初始化区---------
//初始化：确认固定配置，装入三环参数
void Motor_initialize(void)
{
  //固定配置在启动时确认，后续设零和运动共用结果
  int zero_valid = CONFIG_ENCODER_ZERO >= 0 && CONFIG_ENCODER_ZERO < 16384;
  int poles_valid = CONFIG_ENCODER_POLES > 0 && CONFIG_ENCODER_POLES <= 100;
  int direction_valid = CONFIG_ENCODER_DIR == 1 || CONFIG_ENCODER_DIR == -1;
  int acceleration_valid = isfinite(CONFIG_ACCELERATION) && CONFIG_ACCELERATION > 0;
  motor_configured = zero_valid && poles_valid && direction_valid && acceleration_valid;
  //位置环比例增益
  Drive.control.position.KP = CONFIG_POSITION_KP;
  //把输出轴速度增益换算到电机编码器计数单位
  float scale = 16384 * CONFIG_RATIO / 360;
  Drive.control.speed.KP = CONFIG_SPEED_KP / scale;
  Drive.control.speed.KI = CONFIG_SPEED_KI / scale * CONTROL_PERIOD_SECONDS;
  Drive.control.speed.limit = CONFIG_CURRENT_MA;
  //d轴和q轴使用同一组电流环参数
  Drive.control.current.d.KP = CONFIG_CURRENT_KP;
  Drive.control.current.q.KP = CONFIG_CURRENT_KP;
  Drive.control.current.d.KI = CONFIG_CURRENT_KI;
  Drive.control.current.q.KI = CONFIG_CURRENT_KI;
  Drive.control.current.voltageLimit = CONFIG_VOLTAGE_MV;
}

//---------前台交接区---------
//前台交接：先检查采样时效，再读取反馈和交付参考
//检查已有样本的更新时间
static int feedbackFresh(void)
{
  uint32_t now = system_ms;
  if (!Drive.state.encoder_valid)
    return 0;
  return now - Drive.last_adc_ms <= 2 && now - Drive.last_encoder_ms <= 2;
}

//命令受理和参考交付前，确认配置、反馈和供电条件
int Motor_canRun(void)
{
  uint32_t bus = current_sense_get_bus_mv();
  //配置和已有故障决定能否接受运动
  if (!motor_configured || Drive.state.fault)
    return 0;
  //采样必须持续更新
  if (!feedbackFresh())
    return 0;
  return bus >= 10000 && bus <= 30000;
}

//读取同一组反馈，并刷新主循环存活时刻
uint32_t Motor_readFeedback(motor_status_t *out)
{
  //记录主循环时刻，供毫秒中断检查
  Drive.last_main_ms = system_ms;
  //关中断复制整组反馈，避免多圈计数读取到一半被更新
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  //重启TMR6计时，采样中断通过溢出标志检查主循环超时
  TMR6->cval = 0;
  tmr_flag_clear(TMR6, TMR_OVF_FLAG);
  *out = Drive.state;
  out->encoder_valid = feedbackFresh();
  uint32_t now = system_ms;
  out->velocity_counts_s = Drive.speed * CONFIG_ENCODER_DIR;
  __set_PRIMASK(irq);
  return now;
}

//把参考位置、速度上限和使能状态一起交给采样中断
int Motor_applyReference(void)
{
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  motion_state_t state = Joint.status.state;
  if (state < MOTION_HOLD || state > MOTION_BRAKE)
  {
    if (Drive.state.power)
      Motor_shutdown(FAULT_NONE);
    __set_PRIMASK(irq);
    return 1;
  }
  double tick = Motion_calculateMotorTarget();
  //目标计数需留在32位位置环的范围内
  int outside = tick < INT32_MIN + 1.0 || tick > INT32_MAX - 1.0;
  if (!Motor_canRun() || outside)
  {
    Motor_shutdown(FAULT_CONTROL);
    __set_PRIMASK(irq);
    return 0;
  }
  float scale = 16384 * CONFIG_RATIO / 360;
  Drive.control.position.targetValue = (int32_t)llround(tick) * CONFIG_ENCODER_DIR;
  Drive.control.position.limit = Joint.speed_limit * scale;
  Drive.state.power = 1;
  __set_PRIMASK(irq);
  return 1;
}

//---------采样中断区---------
//采样中断：由两相ADC计数依次得到反馈、三环输出和PWM
void Motor_control_callback(int32_t a, int32_t b)
{
  Drive.last_adc_ms = system_ms;
  int window = pwm_output_pwm_sample_begin();
  if (Drive.state.power)
  {
    //先判主循环超时，再判相电流和采样窗口
    int main_late = (TMR6->ists & TMR_OVF_FLAG) != 0;
    if (main_late)
    {
      Motor_shutdown(FAULT_CONTROL);
      return;
    }
    //先扩展到64位再取绝对值，覆盖负数最小值
    int current_valid = llabs((long long)a) <= CURRENT_RUNTIME_PHASE_TRIP &&
                        llabs((long long)b) <= CURRENT_RUNTIME_PHASE_TRIP;
    if (!current_valid || !window)
    {
      Motor_shutdown(FAULT_CURRENT);
      return;
    }
  }
  //PWM采样25kHz，三环每4次采样执行一次
  if (++Drive.divider < 4)
  {
    pwm_output_pwm_update_service();
    return;
  }
  Drive.divider = 0;
  //读取编码器原始角度
  uint16_t raw;
  if (!mt6816_read_angle(&raw))
  {
    Drive.state.encoder_valid = 0;
    Motor_shutdown(FAULT_SENSOR);
    return;
  }
  //跨越单圈边界时修正增量，展开为多圈位置
  int32_t delta = raw - (int32_t)((uint64_t)Drive.state.encoder_counts & 16383U);
  if (delta > 8192)
    delta -= 16384;
  if (delta < -8192)
    delta += 16384;
  if (Drive.state.encoder_valid && (abs(delta) > 4096 || system_ms - Drive.last_encoder_ms > 3))
  {
    Drive.state.encoder_valid = 0;
    Motor_shutdown(FAULT_SENSOR);
    return;
  }
  //由计数增量计算速度，再进行一阶低通滤波
  float measured = Drive.state.encoder_valid ? delta * 6250.0f * CONFIG_ENCODER_DIR : 0;
  Drive.speed += 0.1f * (measured - Drive.speed);
  Drive.state.encoder_counts = Drive.state.encoder_valid ? Drive.state.encoder_counts + delta : raw;
  Drive.state.encoder_valid = 1;
  Drive.last_encoder_ms = system_ms;
  int64_t count = Drive.state.encoder_counts * CONFIG_ENCODER_DIR;
  if (count < INT32_MIN + 1LL || count > INT32_MAX - 1LL)
  {
    Motor_shutdown(FAULT_SENSOR);
    return;
  }
  //按电角零点、方向和极对数计算电角度
  int32_t phase =
      ((int32_t)raw - CONFIG_ENCODER_ZERO) * CONFIG_ENCODER_DIR * CONFIG_ENCODER_POLES % 16384;
  float angle = phase * (6.28318530718f / 16384);
  int32_t counts = (int32_t)count;
  //把两相电流换算到转子坐标系
  Common_Park_Compute(&Drive.control.current, current_counts_to_ma(a), current_counts_to_ma(b), angle);
  uint32_t bus = current_sense_get_bus_mv();
  //失能时保留反馈采样，结束本轮功率计算
  if (!Drive.state.power)
  {
    pwm_output_pwm_update_service();
    return;
  }
  //母线电压有效后执行三环控制
  if (bus < 10000 || bus > 30000)
  {
    Motor_shutdown(FAULT_VOLTAGE);
    return;
  }
  Drive.control.position.currentValue = counts;
  Drive.control.speed.currentValue = Drive.speed;
  //位置环生成速度给定，速度环生成电流给定
  Common_Position_Compute(&Drive.control.position);
  Drive.control.speed.targetValue = Drive.control.position.out;
  Common_Speed_Compute(&Drive.control.speed);
  Drive.control.current.q.targetValue = Drive.control.speed.out;
  //电流环生成两相电压，再换算成PWM占空比
  Common_Current_Compute(&Drive.control.current, angle, (int32_t)bus);
  float da = Drive.control.current.outA * CONTROL_PWM_TIMER_COUNTS / bus;
  float db = Drive.control.current.outB * CONTROL_PWM_TIMER_COUNTS / bus;
  float peak = fmaxf(fabsf(da), fabsf(db));
  if (peak > CONTROL_PWM_SAMPLE_SAFE_COMMAND)
  {
    da *= CONTROL_PWM_SAMPLE_SAFE_COMMAND / peak;
    db *= CONTROL_PWM_SAMPLE_SAFE_COMMAND / peak;
  }
  pwm_output_set_signed_phase_duty((int16_t)lroundf(da), (int16_t)lroundf(db));
  pwm_output_pwm_update_service();
}

//---------时钟中断区---------
//毫秒中断检查采样与主循环是否持续执行
void SysTick_Handler(void)
{
  ++system_ms;
  if (!Drive.state.power)
    return;
  if (!feedbackFresh() || system_ms - Drive.last_main_ms > 5)
    Motor_shutdown(FAULT_CONTROL);
}

//---------关断收尾区---------
//关断收尾：清除输出与积分，保持反馈采样
void Motor_shutdown(motor_fault_t reason)
{
  //保存首次故障，正常失能保留已有故障
  if (Drive.state.fault == FAULT_NONE)
    Drive.state.fault = reason;
  //关闭功率并清除各环积分
  Drive.state.power = 0;
  Drive.control.current.q.targetValue = 0;
  Drive.control.current.d.sumError = 0;
  Drive.control.current.q.sumError = 0;
  Drive.control.speed.sumError = 0;
  pwm_output_disable();
  //失能后继续采样，供设零和恢复使用
  pwm_output_enable_sampling();
}

//前台请求清故障，恢复为等待新反馈的失能状态
int Motor_clearFault(void)
{
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  uint32_t bus = current_sense_get_bus_mv();
  //先确认功率关闭和反馈有效，再确认供电范围
  int ok = 0;
  if (!Drive.state.power && feedbackFresh())
    ok = bus >= 10000 && bus <= 30000;
  if (ok)
  {
    Drive.state.fault = FAULT_NONE;
    Drive.state.encoder_valid = 0;
    Drive.speed = 0;
    Motor_shutdown(FAULT_NONE);
  }
  __set_PRIMASK(irq);
  return ok;
}

//CPU异常时执行统一关断并停留
void HardFault_Handler(void)
{
  //HardFault固定优先级高于普通中断，关断后停留
  Motor_shutdown(FAULT_CONTROL);
  while (1)
  {
  }
}
