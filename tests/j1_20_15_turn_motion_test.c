#include "../firmware/src/config.h"
static const JointConfig profile = {
  .can_node = 1, .joint_direction = 1, .ratio = 1,
  .minimum_deg = (-3610.0f), .maximum_deg = 7210.0f,
  .max_speed_deg_s = 180.0f, .following_error_deg = 2.5f,
  .acceleration_deg_s2 = 10.0f, .brake_deg_s2 = 30,
};
const JointConfig *joint_config = &profile;
#include "../firmware/src/motion.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void follow(double goal,float speed)
{
  assert(Motion_moveTo((float)goal,speed));
  unsigned tick;
  for(tick=0;tick<150000 && Joint.status.state!=MOTION_HOLD;tick++)
  {
    int64_t count=10000+(int64_t)llround(Joint.ref_pos*16384/360);
    Motion_update(count,1,Joint.ref_vel);
    assert(!Joint.status.fault && fabsf(Joint.ref_vel)<=speed+.001f);
  }
  assert(tick<150000 && Joint.status.arrived && fabs(Joint.status.position-goal)<.1);
}
int main(void)
{
  memset(&Joint,0,sizeof(Joint));assert(Motion_setZero(10000,0));Joint.status.stationary=1;
  follow(7200,90);follow(1800,180);
  assert(!Motion_moveTo(7211,90));assert(!Motion_moveTo(0,181));
  puts("20/15 turns: PASS (90/180 deg/s, target, speed, bounds, final +5 turns)");
  return 0;
}
