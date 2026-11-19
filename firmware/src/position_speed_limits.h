#ifndef POSITION_SPEED_LIMITS_H
#define POSITION_SPEED_LIMITS_H
#include "config.h"
#include <math.h>

typedef struct { float correction, total; } position_speed_limits_t;

// Output-shaft deg/s. Position feedback needs headroom beyond the trajectory
// speed, but may never exceed the configured joint or motor speed ceiling.
static inline position_speed_limits_t position_speed_limits(
  float requested, float joint_max, float motor_max, float ratio, float position_kp, float error_bound)
{
  position_speed_limits_t limits={0,0};
  if(!(requested>0 && joint_max>0 && motor_max>0 && ratio>0 && position_kp>0 && error_bound>0))return limits;
  limits.correction=fminf(requested,position_kp*error_bound);
  limits.total=fminf(requested+limits.correction,fminf(joint_max,motor_max/ratio));
  return limits;
}
static inline float position_speed_reference(float feedforward,float correction,float limit)
{ return fmaxf(-limit,fminf(limit,feedforward+correction)); }

// 轨迹末段：对超过剩余轨迹距离的偏差，按可制动距离增加纠偏速度。
// 参数单位为输出轴度、度/秒和度/秒²；方向由位置环的有符号误差决定。
static inline float position_move_correction_limit(float baseline, float requested,
  float tracking_error, float remaining_travel, float reference_speed, float braking)
{
  float tail_error = fabsf(tracking_error) - fabsf(remaining_travel);
  if (!(tail_error > 0 && braking > 0))
    return baseline;
  float stopping_speed = sqrtf(2 * braking * tail_error);
  float available = stopping_speed - fabsf(reference_speed);
  return fminf(requested, fmaxf(baseline, available));
}
#endif
