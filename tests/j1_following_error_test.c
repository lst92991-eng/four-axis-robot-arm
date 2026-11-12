#include "../firmware/src/config.h"
static const JointConfig profile = {
  .can_node = 1, .joint_direction = 1, .ratio = 1,
  .minimum_deg = -180, .maximum_deg = 180,
  .max_speed_deg_s = 500.0f, .following_error_deg = 2.5f,
  .acceleration_deg_s2 = 30, .brake_deg_s2 = 30,
};
const JointConfig *joint_config = &profile;
#include "../firmware/src/motion.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
  const float speeds[]={1,5,10,30,60,90,180,500};
  for(unsigned i=0;i<sizeof(speeds)/sizeof(speeds[0]);i++)
  {
    memset(&Joint,0,sizeof(Joint));
    assert(Motion_setZero(10000,0));Joint.status.stationary=1;
    assert(Motion_moveTo(90,speeds[i]));
    Motion_update(10000-137,1,0); // About -3 degrees opposite the reference.
    assert(Joint.status.state==MOTION_FAULT && Joint.status.fault==FAULT_MOTION);
  }
  puts("j1_following_error: PASS (unchanged 2.5-degree protection through 500 deg/s)");
  return 0;
}
