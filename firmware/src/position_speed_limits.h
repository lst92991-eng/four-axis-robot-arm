#ifndef POSITION_SPEED_LIMITS_H
#define POSITION_SPEED_LIMITS_H
#include <math.h>

typedef struct { float correction, total; } position_speed_limits_t;

// Output-shaft deg/s. Position feedback needs headroom beyond the trajectory
// speed, but may never exceed the configured joint or motor speed ceiling.
static inline position_speed_limits_t position_speed_limits(
  float requested, float joint_max, float ratio, float position_kp, float error_bound)
{
  position_speed_limits_t limits={0,0};
  if(!(requested>0 && joint_max>0 && ratio>0 && position_kp>0 && error_bound>0))return limits;
  limits.correction=fminf(requested,position_kp*error_bound);
  limits.total=fminf(requested+limits.correction,fminf(joint_max,1440.0f/ratio));
  return limits;
}
static inline float position_speed_reference(float feedforward,float correction,float limit)
{ return fmaxf(-limit,fminf(limit,feedforward+correction)); }
#endif
