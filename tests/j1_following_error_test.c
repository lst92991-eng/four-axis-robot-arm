#include "../firmware/src/config.h"
#undef CONFIG_RATIO
#undef CONFIG_FOLLOWING_ERROR
#undef CONFIG_MAX_VELOCITY
#define CONFIG_RATIO 1.0f
#define CONFIG_FOLLOWING_ERROR 2.5f
#define CONFIG_MAX_VELOCITY 500.0f
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
