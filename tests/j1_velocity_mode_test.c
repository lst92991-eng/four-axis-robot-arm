#include "../firmware/src/config.h"
static const JointConfig profile = {
  .can_node = 1, .joint_direction = 1, .ratio = 1,
  .minimum_deg = -180, .maximum_deg = 180,
  .max_speed_deg_s = 500.0f, .following_error_deg = 2.5f,
  .acceleration_deg_s2 = 60.0f, .brake_deg_s2 = 360.0f,
};
const JointConfig *joint_config = &profile;
#include "../firmware/src/motion.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void step(void)
{
  int64_t count=10000+(int64_t)llround((Joint.ref_pos-Joint.zero_degrees)*16384/360);
  Motion_update(count,1,Joint.ref_vel);
}
int main(void)
{
  memset(&Joint,0,sizeof(Joint));assert(Motion_setZero(10000,8000));Joint.status.stationary=1;
  assert(!Motion_runVelocity(501));
  assert(Motion_runVelocity(500));
  for(unsigned i=0;i<27000;i++)
  {
    step();assert(!Joint.status.fault);
    if(i==999)assert(fabsf(Joint.ref_vel-60)<.02f);
  }
  assert(Joint.status.state==MOTION_VELOCITY && Joint.ref_vel==500 && Joint.ref_pos>9000);
  assert(!Motion_setBrakeAcceleration(120)); // Cannot retune braking while moving.
  assert(Motion_stop(0));unsigned stopped=0;
  while(Joint.status.state!=MOTION_HOLD && stopped<2000){step();stopped++;}
  assert(stopped<2000 && !Joint.status.fault && Joint.status.stationary);
  assert(Motion_setBrakeAcceleration(120));assert(Motion_runVelocity(-90));
  for(unsigned i=0;i<11000;i++)
  {
    step();
    if(i==999)assert(fabsf(Joint.ref_vel+60)<.02f);
  }
  assert(Motion_stop(1) && Joint.status.interlocked);
  for(unsigned i=0;i<1300;i++)step();
  assert(Joint.status.state==MOTION_HOLD && !Motion_runVelocity(10));
  Motion_release();assert(Motion_setZero(10000,0));Joint.status.stationary=1;
  assert(Motion_runVelocity(10));Motion_update(10000+200,1,0);
  assert(Joint.status.fault==FAULT_MOTION && Joint.status.fault_detail==4);
  puts("velocity: PASS (no position endpoint, accel/cruise/brake, watchdog interlock, following protection)");
  return 0;
}
