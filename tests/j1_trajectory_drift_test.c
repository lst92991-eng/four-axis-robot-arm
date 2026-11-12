#include "../firmware/src/config.h"
static const JointConfig profile = {
  .can_node = 1, .joint_direction = 1, .ratio = 1,
  .minimum_deg = (-3600.0f), .maximum_deg = 3600.0f,
  .max_speed_deg_s = 30, .following_error_deg = 2.5f,
  .acceleration_deg_s2 = 10.0f, .brake_deg_s2 = 30,
};
const JointConfig *joint_config = &profile;
#include "../firmware/src/motion.c"
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
  memset(&Joint,0,sizeof(Joint));Motion_setZero(10000,0);Joint.status.stationary=1;
  if(!Motion_moveTo(3600,10))return 2;
  double max_drift=0;
  for(unsigned tick=1;tick<=355000;tick++)
  {
    double t=tick*.001;
    double ideal=t<=1?5*t*t:10*t-5;
    // An independently integrated plant, with one degree of tracking lag.
    // Following ref_pos itself would conceal numerical accumulation errors.
    double actual=fmax(0,ideal-1);
    int64_t count=10000+(int64_t)llround(actual*16384/360);
    Motion_update(count,1,t<1?(float)(10*t):10);
    double drift=fabs(Joint.ref_pos-ideal);
    if(drift>max_drift)max_drift=drift;
    if(Joint.status.fault)
    {
      printf("FAIL at %.3fs, reference=%.6f, ideal=%.6f, actual=%.6f, drift=%.6f\n",
             t,(double)Joint.ref_pos,ideal,(double)Joint.status.position,drift);
      return 1;
    }
  }
  printf("trajectory max numerical drift = %.9f deg\n",max_drift);
  return max_drift<.01?0:1;
}
