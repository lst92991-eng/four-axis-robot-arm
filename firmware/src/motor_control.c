//初始化控制参数，交接前台数据，执行采样闭环并统一关断
#include "motor_control.h"
#include "motion.h"
#include "at32m412_416_conf.h"
#include "current_sense.h"
#include "current_limits.h"
#include "pwm_output.h"
#include "mt6816.h"
#include "calibration.h"
#include "speed_estimator.h"
#include "position_speed_limits.h"
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
  uint8_t calibrating;
  float calibration_angle, calibration_ma;
  int32_t sum_a, sum_b;
  uint8_t mean_samples;
  int32_t current_fault[12];
  speed_estimator_t speed_estimator;
  uint32_t last_speed_ms;
  float speed_kp, speed_ki;
} Drive;

uint8_t motor_configured;

//---------初始化区---------
//初始化：确认固定配置，装入三环参数
void Motor_initialize(void)
{
  Calibration_init();
  speed_estimator_init(&Drive.speed_estimator,4096,64);
  //固定配置在启动时确认，后续设零和运动共用结果
  int zero_valid = CONFIG_ENCODER_ZERO >= 0 && CONFIG_ENCODER_ZERO < 16384;
  int poles_valid = CONFIG_ENCODER_POLES > 0 && CONFIG_ENCODER_POLES <= 100;
  int direction_valid = CONFIG_ENCODER_DIR == 1 || CONFIG_ENCODER_DIR == -1;
  int acceleration_valid = isfinite(CONFIG_ACCELERATION) && CONFIG_ACCELERATION > 0;
  motor_configured = ((zero_valid && poles_valid && direction_valid) || Calibration_ready()) && acceleration_valid;
  //位置环比例增益
  Drive.control.position.KP = CONFIG_POSITION_KP;
  //把输出轴速度增益换算到电机编码器计数单位
  float scale = 16384 * CONFIG_RATIO / 360;
  Drive.speed_kp=CONFIG_SPEED_KP;Drive.speed_ki=CONFIG_SPEED_KI;
  Drive.control.speed.KP = CONFIG_SPEED_KP / scale;
  Drive.control.speed.KI = CONFIG_SPEED_KI / scale * 0.001f;
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
  if (!motor_configured || Drive.state.fault || Calibration_active())
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
  out->bus_mv = current_sense_get_bus_mv();
  out->id_counts = (int32_t)(Drive.control.current.d.currentValue * 819.0f / 3300.0f);
  out->encoder_valid = feedbackFresh();
  uint32_t now = system_ms;
  __set_PRIMASK(irq);
  // 编码器每格约0.022度，不能在160us内直接差分后当作低速反馈。
  // 复用六轴量化区间观测器，仅在毫秒时刻前进时更新，64位计算留在主循环。
  if (out->encoder_valid && out->encoder_counts>=INT32_MIN+1LL && out->encoder_counts<=INT32_MAX-1LL)
    speed_estimator_update_interval(&Drive.speed_estimator,(int32_t)out->encoder_counts,now);
  else
    speed_estimator_reset(&Drive.speed_estimator);
  float raw_speed=speed_estimator_get_mdeg_s(&Drive.speed_estimator)*(16384.0f/360000.0f);
  __disable_irq();
  Drive.speed=raw_speed*Calibration_direction();
  out->velocity_counts_s=raw_speed;
  __set_PRIMASK(irq);
  return now;
}

//把参考位置、速度上限和使能状态一起交给采样中断
int Motor_applyReference(void)
{
  if (Calibration_active())
    return 1;
  uint32_t irq = __get_PRIMASK();
  __disable_irq();
  uint32_t dt_ms=system_ms-Drive.last_speed_ms;
  Drive.last_speed_ms=system_ms;
  motion_state_t state = Joint.status.state;
  if ((state < MOTION_HOLD || state > MOTION_BRAKE) && state!=MOTION_VELOCITY)
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
  Drive.control.position.targetValue = (int32_t)llround(tick) * Calibration_direction();
  Drive.control.position.limit = Joint.speed_limit * scale;
  float speed_reference_limit=Drive.control.position.limit;
#if CONFIG_NODE == 1
  position_speed_limits_t limits=position_speed_limits(Joint.speed_limit,CONFIG_MAX_VELOCITY,
    CONFIG_RATIO,Drive.control.position.KP,CONFIG_FOLLOWING_ERROR>0?CONFIG_FOLLOWING_ERROR:2.5f);
  Drive.control.position.limit=limits.correction*scale;
  speed_reference_limit=limits.total*scale;
#endif
  Drive.control.position.currentValue=(int32_t)Drive.state.encoder_counts*Calibration_direction();
  Drive.control.speed.currentValue=Drive.speed;
  Common_Position_Compute(&Drive.control.position);
  float feedforward=Joint.ref_vel*scale*Calibration_direction()/CONFIG_DIRECTION;
  Drive.control.speed.targetValue=position_speed_reference(
    feedforward,Drive.control.position.out,speed_reference_limit);
  // 普通位置/速度环以主循环1kHz更新，电流环保持原6250Hz。
  Drive.control.speed.KI=Drive.speed_ki/scale*((dt_ms>=1 && dt_ms<=3)?dt_ms:1)*0.001f;
  Common_Speed_Compute(&Drive.control.speed);
  Drive.control.current.q.targetValue=Drive.control.speed.out;
  Drive.state.power = 1;
  __set_PRIMASK(irq);
  return 1;
}

int Motor_calibrationDrive(int enabled, int32_t angle_mdeg, int32_t current_ma)
{
  uint32_t irq=__get_PRIMASK();
  __disable_irq();
  if (!enabled)
  {
    if (Drive.calibrating) Motor_shutdown(FAULT_NONE);
    __set_PRIMASK(irq);
    return 1;
  }
  uint32_t bus=current_sense_get_bus_mv();
  int ok=Calibration_active() && Joint.status.state==MOTION_OFF && !Drive.state.fault &&
         feedbackFresh() && bus>=10000 && bus<=30000 &&
         current_ma>=0 && current_ma<=1000 && current_ma<=CONFIG_CURRENT_MA &&
         angle_mdeg>=0 && angle_mdeg<360000;
  if (ok)
  {
    Drive.calibration_angle=angle_mdeg*(6.28318530718f/360000.0f);
    Drive.calibration_ma=(float)current_ma;
    Drive.calibrating=1;
    Drive.state.power=1;
  }
  else Motor_shutdown(FAULT_CONTROL);
  __set_PRIMASK(irq);
  return ok;
}

void Motor_readCurrentFault(int32_t values[12])
{
  uint32_t irq=__get_PRIMASK();
  __disable_irq();
  for(unsigned i=0;i<12;i++)values[i]=Drive.current_fault[i];
  __set_PRIMASK(irq);
}

int Motor_setSpeedGains(int32_t kp_milli,int32_t ki_milli)
{
  if(CONFIG_NODE!=1 || kp_milli<100 || kp_milli>20000 || ki_milli<0 || ki_milli>300000)
    return 0;
  uint32_t irq=__get_PRIMASK();__disable_irq();
  int ok=Joint.status.state==MOTION_OFF && !Drive.state.power && !Drive.state.fault &&
         !Calibration_active() && feedbackFresh();
  if(ok)
  {
    float scale=16384*CONFIG_RATIO/360;
    Drive.speed_kp=kp_milli*0.001f;Drive.speed_ki=ki_milli*0.001f;
    Drive.control.speed.KP=Drive.speed_kp/scale;
    Drive.control.speed.KI=Drive.speed_ki/scale*0.001f;
    Drive.control.speed.sumError=0;
  }
  __set_PRIMASK(irq);return ok;
}
void Motor_readSpeedGains(int32_t values[3])
{
  values[0]=(int32_t)lroundf(Drive.speed_kp*1000);
  values[1]=(int32_t)lroundf(Drive.speed_ki*1000);
  values[2]=(int32_t)CONFIG_CURRENT_MA;
}

//---------采样中断区---------
//采样中断：由两相ADC计数依次得到反馈、三环输出和PWM
void Motor_control_callback(int32_t a, int32_t b)
{
  Drive.last_adc_ms = system_ms;
  Drive.state.phase_a_counts=a;
  Drive.state.phase_b_counts=b;
  Drive.sum_a+=a; Drive.sum_b+=b;
  if (++Drive.mean_samples>=30)
  {
    Drive.state.phase_a_mean=Drive.sum_a/Drive.mean_samples;
    Drive.state.phase_b_mean=Drive.sum_b/Drive.mean_samples;
    Drive.sum_a=Drive.sum_b=0; Drive.mean_samples=0;
  }
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
    if (Drive.calibrating)
      current_valid = current_valid && llabs((long long)a)<=CURRENT_CAL_PHASE_TRIP &&
                                      llabs((long long)b)<=CURRENT_CAL_PHASE_TRIP;
    if (!current_valid || !window)
    {
      if (!Drive.state.fault)
      {
        Drive.current_fault[0]=(!current_valid?1:0)|(!window?2:0);
        Drive.current_fault[1]=(int32_t)system_ms;
        Drive.current_fault[2]=a;Drive.current_fault[3]=b;
        Drive.current_fault[4]=Drive.calibrating;
        Drive.current_fault[5]=(int32_t)Drive.calibration_ma;
        Drive.current_fault[6]=(int32_t)Drive.control.current.d.currentValue;
        Drive.current_fault[7]=(int32_t)Drive.control.current.q.currentValue;
        Drive.current_fault[8]=(int32_t)Drive.control.current.outA;
        Drive.current_fault[9]=(int32_t)Drive.control.current.outB;
        Drive.current_fault[10]=(int32_t)Drive.state.encoder_counts;
        Drive.current_fault[11]=window;
      }
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
  Drive.state.encoder_counts = Drive.state.encoder_valid ? Drive.state.encoder_counts + delta : raw;
  Drive.state.encoder_valid = 1;
  Drive.last_encoder_ms = system_ms;
  int64_t count = Drive.state.encoder_counts * Calibration_direction();
  if (count < INT32_MIN + 1LL || count > INT32_MAX - 1LL)
  {
    Motor_shutdown(FAULT_SENSOR);
    return;
  }
  //按电角零点、方向和极对数计算电角度
  float angle = Drive.calibrating ? Drive.calibration_angle : Calibration_angle(raw);
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
  if (Drive.calibrating)
  {
    Drive.control.current.d.targetValue=Drive.calibration_ma;
    Drive.control.current.q.targetValue=0;
  }
  else
  {
    Drive.control.current.d.targetValue=0;
  }
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
  Drive.calibrating=0;
  Drive.control.current.d.targetValue=0;
  Drive.control.current.q.targetValue = 0;
  Drive.control.current.d.sumError = 0;
  Drive.control.current.q.sumError = 0;
  Drive.control.speed.sumError = 0;
  Drive.control.current.d.out=Drive.control.current.q.out=0;
  Drive.control.current.outA=Drive.control.current.outB=0;
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
    for(unsigned i=0;i<12;i++)Drive.current_fault[i]=0;
    Drive.state.encoder_valid = 0;
    Drive.speed = 0;
    speed_estimator_reset(&Drive.speed_estimator);
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
