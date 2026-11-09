#include "speed_estimator.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
  const int speeds[]={1000,5000,10000,30000,60000,90000,180000,-1000,-10000,-60000,-180000};
  for(unsigned j=0;j<sizeof(speeds)/sizeof(speeds[0]);j++)
  {
    speed_estimator_t e;assert(speed_estimator_init(&e,4096,64));
    int64_t sum=0;int maximum=0;
    for(unsigned ms=0;ms<=3000;ms++)
    {
      int32_t counts=(int32_t)llround((double)speeds[j]*ms*16384/(360000.0*1000));
      assert(speed_estimator_update_interval(&e,counts,ms));
      int v=speed_estimator_get_mdeg_s(&e);
      if(ms>2000){sum+=v;if(abs(v)>maximum)maximum=abs(v);}
    }
    assert(llabs(sum/1000-speeds[j])<150);
    // At 1 deg/s, quantized staircase observations still produce bounded
    // pulses (~5.1 deg/s here), not the former 13.7 deg/s single-count spike.
    assert(maximum<(abs(speeds[j])==1000?6000:abs(speeds[j])+1500));
    int32_t pos=(int32_t)llround((double)speeds[j]*3000*16384/(360000.0*1000));
    for(unsigned ms=3001;ms<3200;ms++)speed_estimator_update_interval(&e,pos,ms);
    assert(abs(speed_estimator_get_mdeg_s(&e))<100);
  }
  speed_estimator_t e;assert(speed_estimator_init(&e,4096,64));
  speed_estimator_update_interval(&e,0,0);
  speed_estimator_update_interval(&e,1,1);
  // A one-count observation must not create the old 13.7 deg/s impulse.
  assert(abs(speed_estimator_get_mdeg_s(&e))<3000);
  speed_estimator_update_interval(&e,100,30);
  assert(speed_estimator_get_mdeg_s(&e)==0);
  puts("j1_speed_interval: PASS (1/5/10/30 deg/s, reverse, stop, one-count and gap)");
  return 0;
}
