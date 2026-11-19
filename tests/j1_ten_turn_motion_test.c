#include "../firmware/src/config.h"
static const JointConfig profile = {
  .can_node = 1, .joint_direction = 1, .ratio = 1,
  .minimum_deg = (-3600.0f), .maximum_deg = 3600.0f,
  .max_speed_deg_s = 60.0f, .following_error_deg = 2.5f,
  .acceleration_deg_s2 = 10.0f, .brake_deg_s2 = 30,
};
const JointConfig *joint_config = &profile;
#include "../firmware/src/motion.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void follow(float goal,float speed)
{
  assert(Motion_moveTo(goal,speed));
  unsigned tick;
  for(tick=0;tick<380000 && Joint.status.state!=MOTION_HOLD;tick++)
  {
    int64_t count=10000+(int64_t)llround(Joint.ref_pos*16384/360);
    Motion_update(count,1,Joint.ref_vel);
    assert(Joint.status.fault==0);
    assert(fabsf(Joint.ref_vel)<=speed+.001f);
  }
  assert(tick<380000 && Joint.status.arrived && fabsf(Joint.status.position-goal)<.1f);
}
int main(void)
{
  memset(&Joint,0,sizeof(Joint));assert(Motion_setZero(10000,0));Joint.status.stationary=1;
  follow(3600,10);follow(0,10);follow(3600,60);follow(0,60);
  assert(!Motion_moveTo(3601,10));
  puts("j1_ten_turn_motion: PASS (10 turns and return, speed/range/arrival)");
  return 0;
}
