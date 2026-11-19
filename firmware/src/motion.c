//按反馈更新轨迹参考，停止后保持当前位置
#include "motion.h"
#include <math.h>

//---------运行数据区---------
motion_control_t Joint;

typedef struct
{
  double acceleration;
  double low_edge_s;
  double high_edge_s;
  double duration_s;
  double travel_deg;
} speed_ramp_t;

static struct
{
  double start;
  double peak_speed;
  double cruise_s;
  speed_ramp_t rise;
  speed_ramp_t brake;
} j1_profile;

//当前状态是否需要闭环控制
static int active(void);

static int withinPositionLimits(double degrees, float margin)
{
  return joint_config->minimum_deg == joint_config->maximum_deg ||
         (degrees >= joint_config->minimum_deg - margin &&
          degrees <= joint_config->maximum_deg + margin);
}

//---------机械参考区---------
int Motion_setZero(int64_t counts, float degrees)
{
  //失能后才能建立新的机械零点
  if (Joint.status.state != MOTION_OFF)
    return 0;
  // 仅限制协议数值可表示范围；手动设参考不依赖未测量的机械行程。
  if (!isfinite(degrees) || fabsf(degrees)>2000000.0f)
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
  Joint.manual_velocity=0;
  return 1;
}

double Motion_calculateMotorTarget(void)
{
  return (double)Joint.zero_counts + (Joint.ref_pos - Joint.zero_degrees) * 16384.0 *
                                    joint_config->ratio / (360 * joint_config->joint_direction);
}

//---------运动命令区---------
//以速度由低到高计算。制动反向采样，低速端较长，减速力在到零前逐步撤掉。
static speed_ramp_t speedRamp(double speed, double acceleration, double low_blend_s)
{
  const double high_blend_s = 0.060;
  double transition_speed = 0.5 * acceleration * (low_blend_s + high_blend_s);
  double scale = speed < transition_speed ? sqrt(speed / transition_speed) : 1;
  speed_ramp_t ramp;
  ramp.acceleration = acceleration * scale;
  ramp.low_edge_s = low_blend_s * scale;
  ramp.high_edge_s = high_blend_s * scale;
  double flat_s = fmax(0, (speed - transition_speed) / acceleration);
  ramp.duration_s = ramp.low_edge_s + flat_s + ramp.high_edge_s;
  double entry_speed = 0.5 * ramp.acceleration * ramp.low_edge_s;
  ramp.travel_deg = entry_speed * ramp.low_edge_s / 3
                  + (entry_speed + 0.5 * ramp.acceleration * flat_s) * flat_s
                  + (speed - ramp.acceleration * ramp.high_edge_s / 6) * ramp.high_edge_s;
  return ramp;
}

static void planJ1Move(double start, double distance, double speed, double acceleration, double braking)
{
  const double start_blend_s = 0.060;
  const double stop_blend_s = 0.150;
  speed_ramp_t rise = speedRamp(speed, acceleration, start_blend_s);
  speed_ramp_t brake = speedRamp(speed, braking, stop_blend_s);
  double ramp_distance = rise.travel_deg + brake.travel_deg;
  if (ramp_distance > distance)
  {
    //短行程先算能加速并制动到终点的峰速。
    double low = 0;
    double high = speed;
    for (int i = 0; i < 24; ++i)
    {
      double candidate = (low + high) * 0.5;
      rise = speedRamp(candidate, acceleration, start_blend_s);
      brake = speedRamp(candidate, braking, stop_blend_s);
      if (rise.travel_deg + brake.travel_deg <= distance)
        low = candidate;
      else
        high = candidate;
    }
    speed = low;
    rise = speedRamp(speed, acceleration, start_blend_s);
    brake = speedRamp(speed, braking, stop_blend_s);
    ramp_distance = rise.travel_deg + brake.travel_deg;
  }
  j1_profile.start = start;
  j1_profile.peak_speed = speed;
  j1_profile.rise = rise;
  j1_profile.brake = brake;
  j1_profile.cruise_s = speed > 0 ? (distance - ramp_distance) / speed : 0;
}

//J1扩大指令范围，生成轨迹前仍按电机侧速度上限夹紧。
static float limitCommandSpeed(float speed)
{
  if (!(speed > 0 && speed <= joint_config->max_speed_deg_s && joint_config->ratio > 0))
    return 0;
  float motor_limit = joint_config->can_node == 1 ? J1_MOTOR_SPEED_LIMIT_DEG_S : MOTOR_SPEED_LIMIT_DEG_S;
  if (speed * joint_config->ratio <= motor_limit)
    return speed;
  return joint_config->can_node == 1 ? motor_limit / joint_config->ratio : 0;
}

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
  int target_valid = isfinite(degrees) && fabsf(degrees) <= 2000000.0f &&
                     withinPositionLimits(degrees, 0);
  speed = limitCommandSpeed(speed);
  if (!target_valid || speed == 0)
    return 0;

  //确认实际起点与轨迹起点一致
  if (!withinPositionLimits(Joint.status.position, 0))
    return 0;
  double start = Joint.status.state == MOTION_OFF ? Joint.status.position : Joint.ref_pos;
  if (fabs(start - Joint.status.position) > 0.1)
    return 0;
  float acceleration = Motion_startAcceleration();
  float braking = joint_config->can_node == 1 ? Motion_brakeAcceleration() : acceleration;
  double duration = fabs((double)degrees - start) / speed + speed / acceleration + speed / braking + 10;
  if (duration > 3600)
    return 0;

  //记录本段运动的目标和时限
  if (joint_config->can_node == 1)
    planJ1Move(start, fabs((double)degrees - start), speed, acceleration, braking);
  Joint.ref_pos = start;
  Joint.manual_velocity=0;
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

int Motion_runVelocity(float velocity)
{
  float speed=limitCommandSpeed(fabsf(velocity));
  if(speed==0 ||
     Joint.status.interlocked || !Joint.status.referenced ||
     !Joint.status.stationary || (Joint.status.state!=MOTION_OFF && Joint.status.state!=MOTION_HOLD))
    return 0;
  double start=(Joint.status.state==MOTION_OFF || joint_config->can_node!=1)?
    Joint.status.position:Joint.ref_pos;
  if(fabs(start-Joint.status.position)>.1)return 0;
  Joint.ref_pos=start;Joint.ref_vel=0;Joint.speed_limit=speed;
  Joint.manual_speed=copysignf(speed,velocity);Joint.manual_velocity=1;
  Joint.status.requested=(float)start;Joint.status.arrived=0;
  Joint.status.state=MOTION_VELOCITY;Joint.elapsed_s=0;Joint.timeout_s=0;Joint.settle_ms=0;
  return 1;
}

float Motion_startAcceleration(void)
{
  return Joint.start_acceleration > 0 ? Joint.start_acceleration : joint_config->acceleration_deg_s2;
}

float Motion_brakeAcceleration(void)
{
  return Joint.brake_acceleration > 0 ? Joint.brake_acceleration : joint_config->brake_deg_s2;
}

int Motion_setRamps(float brake_deg_s2, float start_deg_s2)
{
  int stopped = Joint.status.state == MOTION_OFF ||
                (Joint.status.state == MOTION_HOLD && Joint.status.stationary);
  int brake_valid = brake_deg_s2 >= 10 && brake_deg_s2 <= J1_MAX_RAMP_DEG_S2;
  int start_valid = start_deg_s2 == 0 || (start_deg_s2 >= 10 && start_deg_s2 <= J1_MAX_RAMP_DEG_S2);
  if (joint_config->can_node != 1 || !stopped || !brake_valid || !start_valid)
    return 0;
  Joint.brake_acceleration = brake_deg_s2;
  if (start_deg_s2 != 0)
    Joint.start_acceleration = start_deg_s2;
  return 1;
}

int Motion_stop(int link_lost)
{
  if (!active())
    return 0;
  if (link_lost)
    Joint.status.interlocked = 1;
  if (Joint.status.state == MOTION_HOLD && joint_config->can_node != 1)
    return 1;
  if (Joint.status.state == MOTION_BRAKE)
    return 1;
  float v = Joint.ref_vel;
  float distance = v * fabsf(v) / (2 * Motion_brakeAcceleration());
  double end = Joint.ref_pos + distance;
  if (!Joint.manual_velocity && !withinPositionLimits(end, 0))
  {
    Joint.status.fault_detail=5;
    Motion_trip(FAULT_MOTION);
    return 0;
  }
  Joint.elapsed_s = 0;
  Joint.timeout_s = fabsf(v) / Motion_brakeAcceleration() + 2;
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
//求由停稳到峰速时的实时位置和速度；制动时倒着用同一段曲线。
static void sampleRamp(speed_ramp_t ramp, double peak, double t, double *position, double *velocity)
{
  double remaining = ramp.duration_s - t;
  double accel = ramp.acceleration;
  if (t < ramp.low_edge_s)
  {
    *velocity = 0.5 * accel * t * (t / ramp.low_edge_s);
    *position = *velocity * t / 3;
  }
  else if (remaining > ramp.high_edge_s)
  {
    double flat = t - ramp.low_edge_s;
    double entry_speed = 0.5 * accel * ramp.low_edge_s;
    *velocity = entry_speed + accel * flat;
    *position = entry_speed * ramp.low_edge_s / 3 + (entry_speed + *velocity) * 0.5 * flat;
  }
  else
  {
    double speed_drop = 0.5 * accel * remaining * (remaining / ramp.high_edge_s);
    *velocity = peak - speed_drop;
    *position = ramp.travel_deg - (peak - speed_drop / 3) * remaining;
  }
}

static void updateJ1Reference(double t)
{
  double peak = j1_profile.peak_speed;
  double rise_s = j1_profile.rise.duration_s;
  double cruise_end = rise_s + j1_profile.cruise_s;
  double finish = cruise_end + j1_profile.brake.duration_s;
  if (t >= finish)
  {
    Joint.ref_pos = Joint.status.requested;
    Joint.ref_vel = 0;
    return;
  }

  double position;
  double speed = peak;
  if (t < rise_s)
    sampleRamp(j1_profile.rise, peak, t, &position, &speed);
  else if (t < cruise_end)
    position = j1_profile.rise.travel_deg + peak * (t - rise_s);
  else
  {
    double remaining_position;
    sampleRamp(j1_profile.brake, peak, finish - t, &remaining_position, &speed);
    position = fabs((double)Joint.status.requested - j1_profile.start) - remaining_position;
  }

  double direction = Joint.status.requested >= j1_profile.start ? 1 : -1;
  Joint.ref_pos = j1_profile.start + direction * position;
  Joint.ref_vel = (float)(direction * speed);
}

float Motion_speedKiScale(void)
{
  double brake_start = j1_profile.rise.duration_s + j1_profile.cruise_s;
  if (joint_config->can_node == 1 && Joint.status.state == MOTION_MOVE &&
      Joint.status.requested < j1_profile.start && Joint.elapsed_s >= brake_start &&
      Joint.descent_brake_ki_permille > 0)
    return Joint.descent_brake_ki_permille * .001f;
  return 1;
}

void Motion_update(int64_t counts, uint32_t elapsed_ms, float measured_velocity)
{
  //机械零点建立后才计算输出轴位置
  if (!Joint.status.referenced)
    return;
  if (elapsed_ms > 3)
  {
    if (active())
    {
      Joint.status.fault_detail = CONTROL_TRAJECTORY_GAP;
      Motion_trip(FAULT_CONTROL);
    }
    else
      Joint.status.referenced = 0;
    return;
  }
  float dt = elapsed_ms * 0.001f;
  float quantum = 360.0f / (16384 * joint_config->ratio);

  //更新反馈位置、速度和本次停稳条件
  Joint.status.position =
      Joint.zero_degrees + (float)(counts - Joint.zero_counts) * quantum * joint_config->joint_direction;
  Joint.status.velocity = measured_velocity;
  Joint.status.stationary = fabsf(measured_velocity) <= 0.05f;
  if (!active())
    return;

  //检查实际位置的机械行程
  if (!isfinite(Joint.status.position) || fabsf(Joint.status.position)>2000000.0f ||
      (!Joint.manual_velocity && !withinPositionLimits(Joint.status.position, quantum)))
  {
    Joint.status.fault_detail=1;
    Motion_trip(FAULT_MOTION);
    return;
  }
  //按本轮状态选择参考速度，制动的目标速度为零
  motion_state_t state = Joint.status.state;
  const int rotary_jog = Joint.manual_velocity && joint_config->can_node != 1;
  const int smooth_move = state == MOTION_MOVE && joint_config->can_node == 1;
  float acceleration = Motion_startAcceleration();
  float rate = state == MOTION_BRAKE ? Motion_brakeAcceleration() : acceleration;
  float target_speed = 0;
  if(state==MOTION_VELOCITY)target_speed=Joint.manual_speed;
  if (state == MOTION_MOVE && !smooth_move)
  {
    //按剩余距离减速，留出编码器量化和一个控制周期的余量
    float error = Joint.status.requested - Joint.ref_pos;
    float distance = fmaxf(0, fabsf(error) - 4 * quantum);
    float braking_speed = fmaxf(0, sqrtf(2 * acceleration * distance) - acceleration * dt);
    target_speed = copysignf(fminf(Joint.speed_limit, braking_speed), error);
  }

  //保持沿用原参考，运动和制动共用加速度积分
  if (state != MOTION_HOLD)
  {
    double interval_s = elapsed_ms * 0.001;
    if (smooth_move)
      updateJ1Reference(Joint.elapsed_s + interval_s);
    else
    {
      //按加速度限制更新参考速度和位置
      float old_speed = Joint.ref_vel;
      float step = rate * dt;
      float change = fmaxf(-step, fminf(step, target_speed - old_speed));
      Joint.ref_vel += change;
      if (rotary_jog)
      {
        // 手动速度运行以当前编码器位置为参考。松手减速到零后在实际位置保持。
        Joint.ref_pos = Joint.status.position;
        Joint.status.requested = Joint.status.position;
        if (state == MOTION_BRAKE && Joint.ref_vel == 0)
        {
          Joint.status.state = MOTION_HOLD;
          Joint.manual_velocity = 0;
        }
        return;
      }
      Joint.ref_pos += ((double)old_speed + Joint.ref_vel) * 0.5 * interval_s;
      //低速末段在四个编码器量化步内对齐目标
      if (state == MOTION_MOVE &&
          fabs(Joint.status.requested - Joint.ref_pos) <= 4 * quantum && fabsf(old_speed) <= step)
      {
        Joint.ref_pos = Joint.status.requested;
        Joint.ref_vel = 0;
      }
    }
    Joint.elapsed_s += interval_s;
    if (!isfinite(Joint.ref_pos) || fabs(Joint.ref_pos)>2000000.0 ||
        (!Joint.manual_velocity && !withinPositionLimits(Joint.ref_pos, quantum)) ||
        (state!=MOTION_VELOCITY && Joint.elapsed_s > Joint.timeout_s))
    {
      Joint.status.fault_detail=Joint.elapsed_s>Joint.timeout_s?3:2;
      Motion_trip(FAULT_MOTION);
      return;
    }
  }
  if(Joint.manual_velocity)Joint.status.requested=(float)Joint.ref_pos;

  //检查实际反馈是否跟上参考位置
  float following_limit = joint_config->following_error_deg > 0 ? joint_config->following_error_deg :
                          fmaxf(2.5f, Joint.speed_limit * 2.5f);
  double error = Joint.ref_pos - Joint.status.position;
  //J1的跟随误差只用于诊断；其他轴仍执行原停机判定。
  if (joint_config->can_node != 1 && fabs(error) > following_limit)
  {
    Joint.status.fault_detail=4;
    Motion_trip(FAULT_MOTION);
    return;
  }
  //累计连续满足位置、反馈速度和参考速度条件的时间
  double goal = state == MOTION_BRAKE ? Joint.ref_pos : Joint.status.requested;
  int j1_velocity_stop = state == MOTION_BRAKE && Joint.manual_velocity && joint_config->can_node == 1;
  double settle_error = j1_velocity_stop ? 0.75 : 0.10;
  if (fabs(goal - Joint.status.position) <= settle_error && Joint.status.stationary && Joint.ref_vel == 0)
    Joint.settle_ms += elapsed_ms;
  else
    Joint.settle_ms = 0;

  //连续停稳250毫秒后进入保持，运动完成时记录到位
  if (Joint.settle_ms >= 250)
  {
    Joint.settle_ms = 250;
    if (state == MOTION_MOVE)
      Joint.status.arrived = 1;
    //J1速度制动没有指定到位点，停稳后保持实际位置。
    if (j1_velocity_stop)
    {
      Joint.ref_pos = Joint.status.position;
      Joint.status.requested = Joint.status.position;
    }
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
  Joint.status.fault_detail=0;
  Joint.status.interlocked = 0;
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
  return (Joint.status.state >= MOTION_HOLD && Joint.status.state <= MOTION_BRAKE) ||
         Joint.status.state==MOTION_VELOCITY;
}
