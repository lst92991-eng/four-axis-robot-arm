#include "../firmware/src/config.h"
#undef CONFIG_RATIO
#undef CONFIG_FOLLOWING_ERROR
#undef CONFIG_ACCELERATION
#undef CONFIG_MINIMUM
#undef CONFIG_MAXIMUM
#define CONFIG_RATIO 1.0f
#define CONFIG_FOLLOWING_ERROR 2.5f
#define CONFIG_ACCELERATION 10.0f
#define CONFIG_MINIMUM (-3610.0f)
#define CONFIG_MAXIMUM 3610.0f
#include "../firmware/src/motion.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void)
{
  memset(&Joint,0,sizeof(Joint));assert(Motion_setZero(10000,0));
  Joint.status.state=MOTION_HOLD;Joint.status.requested=3600;
  Joint.ref_pos=3600;Joint.speed_limit=10;
  Motion_update(10000+163840+2,1,0);
  assert(Joint.status.fault==0); // Two-count settling motion is not the hard boundary.
  Motion_update(10000+163840+200,1,0);
  assert(Joint.status.fault==FAULT_MOTION && Joint.status.fault_detail==4);
  Motion_clearFault();Joint.status.stationary=1;
  assert(!Motion_moveTo(3611,10));
  puts("endpoint: PASS (settling margin, unchanged 2.5-degree following protection)");
  return 0;
}
