#include "../firmware/src/config.h"
static const JointConfig profile = {
  .can_node = 1, .joint_direction = 1, .ratio = 1,
  .minimum_deg = -180, .maximum_deg = 180,
  .max_speed_deg_s = 12000.0f, .following_error_deg = 2.5f,
  .acceleration_deg_s2 = 30, .brake_deg_s2 = 30,
};
const JointConfig *joint_config = &profile;
#include "../firmware/src/motion.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
  const float speeds[]={1,5,10,30,60,90,180,500,720,800,900,1000,1200};
  for(unsigned i=0;i<sizeof(speeds)/sizeof(speeds[0]);i++)
  {
    memset(&Joint,0,sizeof(Joint));
    assert(Motion_setZero(10000,0));Joint.status.stationary=1;
    assert(Motion_setRamps(1080,540));
    assert(Motion_moveTo(90,speeds[i]));
    Motion_update(10000-950,1,0);
    assert(Joint.status.state==MOTION_MOVE && Joint.status.fault==FAULT_NONE);
    assert(Motion_stop(1));
    Motion_update(10000-950,1,0);
    assert(Joint.status.state==MOTION_BRAKE && Joint.status.interlocked && !Joint.status.fault);

    memset(&Joint,0,sizeof(Joint));
    assert(Motion_setZero(10000,0));Joint.status.stationary=1;
    assert(Motion_setRamps(1080,540));
    assert(Motion_moveTo(-90,speeds[i]));
    Motion_update(10000+950,1,0);
    assert(Joint.status.state==MOTION_MOVE && Joint.status.fault==FAULT_NONE);
  }
  memset(&Joint,0,sizeof(Joint));
  assert(Motion_setZero(10000,0));Joint.status.stationary=1;
  assert(Motion_runVelocity(1000));
  Motion_update(10000-950,1,0);
  assert(Joint.status.state==MOTION_VELOCITY && Joint.status.fault==FAULT_NONE);
  assert(Motion_stop(0));
  Motion_update(10000-950,1,0);
  assert(Joint.status.fault==FAULT_NONE);
  for (int direction=-1;direction<=1;direction+=2)
  {
    memset(&Joint,0,sizeof(Joint));
    assert(Motion_setZero(10000,0));Joint.status.stationary=1;
    assert(Motion_setRamps(1080,540));
    assert(!Motion_moveTo(direction*90,12001));
    assert(Motion_moveTo(direction*90,12000));
    assert(Joint.speed_limit==J1_MOTOR_SPEED_LIMIT_DEG_S);
    Motion_release();
    Motion_update(10000,1,0);
    assert(!Motion_runVelocity(direction*12001));
    assert(Motion_runVelocity(direction*12000));
    assert(Joint.manual_speed==direction*J1_MOTOR_SPEED_LIMIT_DEG_S);
  }
  JointConfig rotary = profile;
  rotary.can_node = 2;
  joint_config = &rotary;
  memset(&Joint,0,sizeof(Joint));
  assert(Motion_setZero(10000,0));Joint.status.stationary=1;
  assert(Motion_moveTo(90,30));
  Motion_update(10000-950,1,0);
  assert(Joint.status.state==MOTION_FAULT && Joint.status.fault_detail==4);
  puts("j1_following_error: PASS (J1 diagnostic only; rotary trip and link-loss braking retained)");
  return 0;
}
