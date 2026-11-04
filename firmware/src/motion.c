//按反馈更新轨迹参考，停止后保持当前位置
#include "motion.h"
#include <math.h>

//---------运行数据区---------
motion_control_t Joint;

//当前状态是否需要闭环控制
static int active(void);

//---------机械参考区---------
int Motion_setZero(int64_t counts, float degrees)
{
  //失能后才能建立新的机械零点
  if (Joint.status.state != MOTION_OFF)
    return 0;
  //有限行程区间同时排除NaN和无穷值
  if (!(degrees >= CONFIG_MINIMUM && degrees <= CONFIG_MAXIMUM))
    return 0;
  Joint.zero_counts = counts;
  Joint.zero_degrees = degrees;
  Joint.status.position = degrees;
  Joint.status.velocity = 0;
  Joint.status.requested = degrees;
  Joint.ref_pos = degrees;
  Joint.ref_vel = 0;
  Joint.settle_ms = 0;
  Joint.status.stationary = 0;
  Joint.status.referenced = 1;
  Joint.status.arrived = 0;
  Joint.status.interlocked = 0;
  return 1;
}

double Motion_calculateMotorTarget(void)
{
  return (double)Joint.zero_counts + (Joint.ref_pos - Joint.zero_degrees) * 16384.0 *
                                    CONFIG_RATIO / (360 * CONFIG_DIRECTION);
}

//---------运动命令区---------
int Motion_moveTo(float degrees, float speed)
{
  //确认可以接受新的目标
  int accepting = Joint.status.state == MOTION_OFF || Joint.status.state == MOTION_HOLD;
  if (!accepting || Joint.status.interlocked)
    return 0;
  //机械参考有效且反馈停稳后开始新一段运动
  if (!Joint.status.referenced || !Joint.status.stationary)
    return 0;

  //检查目标角度和速度
  int target_valid = degrees >= CONFIG_MINIMUM && degrees <= CONFIG_MAXIMUM;
  int speed_valid = speed > 0 && speed <= 30;
  int motor_speed_valid = speed * CONFIG_RATIO <= 1440;
  if (!target_valid || !speed_valid)
    return 0;
  if (!motor_speed_valid)
    return 0;

  //确认实际起点与轨迹起点一致
  if (Joint.status.position < CONFIG_MINIMUM || Joint.status.position > CONFIG_MAXIMUM)
    return 0;
  float start = Joint.status.state == MOTION_OFF ? Joint.status.position : Joint.ref_pos;
  if (fabsf(start - Joint.status.position) > 0.1f)
    return 0;
  float duration = fabsf(degrees - start) / speed + 2 * speed / CONFIG_ACCELERATION + 10;
  if (duration > 3600)
    return 0;

  //记录本段运动的目标和时限
  Joint.ref_pos = start;
  Joint.ref_vel = 0;
  Joint.speed_limit = speed;
  Joint.status.requested = degrees;
  Joint.status.arrived = 0;
  Joint.status.state = MOTION_MOVE;
  Joint.elapsed_s = 0;
  Joint.timeout_s = duration;
  Joint.settle_ms = 0;
  return 1;
}

int Motion_stop(int link_lost)
{
  if (!active())
    return 0;
  if (link_lost)
    Joint.status.interlocked = 1;
  if (Joint.status.state == MOTION_BRAKE)
    return 1;
  float v = Joint.ref_vel;
  float distance = v * fabsf(v) / (2 * CONFIG_ACCELERATION);
  float end = Joint.ref_pos + distance;
  if (!(end >= CONFIG_MINIMUM && end <= CONFIG_MAXIMUM))
  {
    Motion_trip(FAULT_MOTION);
    return 0;
  }
  Joint.elapsed_s = 0;
  Joint.timeout_s = fabsf(v) / CONFIG_ACCELERATION + 2;
  Joint.settle_ms = 0;
  Joint.status.arrived = 0;
  Joint.status.state = MOTION_BRAKE;
  return 1;
}

void Motion_release(void)
{
  Joint.status.arrived = 0;
  if (Joint.status.state != MOTION_FAULT)
    Joint.status.state = MOTION_OFF;
}

//---------轨迹更新区---------
void Motion_update(int64_t counts, uint32_t elapsed_ms, float measured_velocity)
{
  //机械零点建立后才计算输出轴位置
  if (!Joint.status.referenced)
    return;
  if (elapsed_ms > 3)
  {
    if (active())
      Motion_trip(FAULT_CONTROL);
    else
      Joint.status.referenced = 0;
    return;
  }
  float dt = elapsed_ms * 0.001f;
  float quantum = 360.0f / (16384 * CONFIG_RATIO);

  //更新反馈位置、速度和本次停稳条件
  Joint.status.position =
      Joint.zero_degrees + (float)(counts - Joint.zero_counts) * quantum * CONFIG_DIRECTION;
  Joint.status.velocity = measured_velocity;
  Joint.status.stationary = fabsf(measured_velocity) <= 0.05f;
  if (!active())
    return;

  //检查实际位置的机械行程
  if (Joint.status.position < CONFIG_MINIMUM - quantum ||
      Joint.status.position > CONFIG_MAXIMUM + quantum)
  {
    Motion_trip(FAULT_MOTION);
    return;
  }
  //按本轮状态选择参考速度，制动的目标速度为零
  motion_state_t state = Joint.status.state;
  float target_speed = 0;
  if (state == MOTION_MOVE)
  {
    //按剩余距离减速，留出编码器量化和一个控制周期的余量
    float error = Joint.status.requested - Joint.ref_pos;
    float distance = fmaxf(0, fabsf(error) - 4 * quantum);
    float braking_speed = sqrtf(2 * CONFIG_ACCELERATION * distance);
    braking_speed = fmaxf(0, braking_speed - CONFIG_ACCELERATION * dt);
    target_speed = copysignf(fminf(Joint.speed_limit, braking_speed), error);
  }

  //保持沿用原参考，运动和制动共用加速度积分
  if (state != MOTION_HOLD)
  {
    //按加速度限制更新参考速度和位置
    float old_speed = Joint.ref_vel;
    float step = CONFIG_ACCELERATION * dt;
    float change = fmaxf(-step, fminf(step, target_speed - old_speed));
    Joint.ref_vel += change;
    Joint.ref_pos += (old_speed + Joint.ref_vel) * 0.5f * dt;
    //低速末段在四个编码器量化步内对齐目标
    if (state == MOTION_MOVE &&
        fabsf(Joint.status.requested - Joint.ref_pos) <= 4 * quantum && fabsf(old_speed) <= step)
    {
      Joint.ref_pos = Joint.status.requested;
      Joint.ref_vel = 0;
    }
    Joint.elapsed_s += dt;
    if (Joint.ref_pos < CONFIG_MINIMUM - quantum || Joint.ref_pos > CONFIG_MAXIMUM + quantum ||
        Joint.elapsed_s > Joint.timeout_s)
    {
      Motion_trip(FAULT_MOTION);
      return;
    }
  }

  //检查实际反馈是否跟上参考位置
  if (fabsf(Joint.ref_pos - Joint.status.position) > fmaxf(2.5f, Joint.speed_limit * 2.5f))
  {
    Motion_trip(FAULT_MOTION);
    return;
  }
  //累计连续满足位置、反馈速度和参考速度条件的时间
  float goal = state == MOTION_BRAKE ? Joint.ref_pos : Joint.status.requested;
  if (fabsf(goal - Joint.status.position) <= 0.1f && Joint.status.stationary && Joint.ref_vel == 0)
    Joint.settle_ms += elapsed_ms;
  else
    Joint.settle_ms = 0;

  //连续停稳250毫秒后进入保持，运动完成时记录到位
  if (Joint.settle_ms >= 250)
  {
    Joint.settle_ms = 250;
    if (state == MOTION_MOVE)
      Joint.status.arrived = 1;
    Joint.status.state = MOTION_HOLD;
  }
  //反馈偏离原命令目标时撤销到位标志
  if (fabsf(Joint.status.requested - Joint.status.position) > 0.1f)
    Joint.status.arrived = 0;
}

//---------故障处理区---------
void Motion_clearFault(void)
{
  Joint.status.state = MOTION_OFF;
  Joint.status.fault = FAULT_NONE;
  Joint.status.referenced = 0;
}

void Motion_trip(motor_fault_t reason)
{
  if (!Joint.status.fault)
    Joint.status.fault = reason;
  Joint.status.state = MOTION_FAULT;
  Joint.status.arrived = 0;
  if (reason == FAULT_SENSOR || reason == FAULT_CONTROL)
    Joint.status.referenced = 0;
}

//---------状态判断区---------

static int active(void)
{
  return Joint.status.state >= MOTION_HOLD && Joint.status.state <= MOTION_BRAKE;
}
