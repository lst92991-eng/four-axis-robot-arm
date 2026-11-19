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

static int64_t brakeToZero(int count_residual)
{
  memset(&Joint,0,sizeof(Joint));assert(Motion_setZero(10000,0));Joint.status.stationary=1;
  assert(Motion_runVelocity(90));
  for(unsigned i=0;i<2000;i++)step();
  assert(Motion_stop(0));
  while(Joint.ref_vel!=0)step();
  assert(Joint.status.state==MOTION_BRAKE && Joint.settle_ms==0);
  return 10000+(int64_t)llround(Joint.ref_pos*16384/360)+count_residual;
}

static void checkBrakeResidual(void)
{
  int64_t count=brakeToZero(-10); // 约0.22°，现场反复停稳但报超时的量级。
  for(unsigned i=0;i<249;i++)Motion_update(count,1,0);
  assert(Joint.status.state==MOTION_BRAKE && Joint.settle_ms==249);
  Motion_update(count,1,.06f);
  assert(Joint.settle_ms==0);
  for(unsigned i=0;i<250;i++)Motion_update(count,1,0);
  assert(Joint.status.state==MOTION_HOLD && !Joint.status.fault && !Joint.status.arrived);
  assert(Joint.ref_pos==Joint.status.position && Joint.status.requested==Joint.status.position);
  assert(Motion_runVelocity(-90));

  count=brakeToZero(-16); // 现场回读约0.36°；已停稳时也能正常接入保持。
  for(unsigned i=0;i<250;i++)Motion_update(count,1,0);
  assert(Joint.status.state==MOTION_HOLD && !Joint.status.fault);
  assert(Joint.ref_pos==Joint.status.position);

  count=brakeToZero(40); // 超出速度停车余量，仍按原超时判故障。
  for(unsigned i=0;i<2010;i++)Motion_update(count,1,0);
  assert(Joint.status.state==MOTION_FAULT && Joint.status.fault_detail==3);
  count=brakeToZero(130);
  assert(Motion_stop(1));
  for(unsigned i=0;i<2010;i++)Motion_update(count,1,0);
  assert(Joint.status.state==MOTION_FAULT && Joint.status.fault_detail==3);
}

static void checkRamps(void)
{
  memset(&Joint,0,sizeof(Joint));assert(Motion_setZero(10000,0));Joint.status.stationary=1;
  assert(Motion_startAcceleration()==60 && Motion_brakeAcceleration()==360);
  assert(Motion_setRamps(150,0) && Motion_startAcceleration()==60);
  assert(Motion_setRamps(360,120));
  assert(Motion_setRamps(J1_MAX_RAMP_DEG_S2,J1_MAX_RAMP_DEG_S2));
  assert(Motion_setRamps(360,120));
  assert(!Motion_setRamps(300,9) && !Motion_setRamps(300,J1_MAX_RAMP_DEG_S2+1)
         && !Motion_setRamps(J1_MAX_RAMP_DEG_S2+1,90));
  assert(Motion_brakeAcceleration()==360 && Motion_startAcceleration()==120);
  assert(Motion_setRamps(350,0) && Motion_startAcceleration()==120);
  Joint.status.state=MOTION_HOLD;Joint.status.stationary=0;
  assert(!Motion_setRamps(300,90));
  Joint.status.stationary=1;
  assert(Motion_setRamps(360,120) && Motion_runVelocity(150));
  for(unsigned i=0;i<500;i++)step();
  assert(fabsf(Joint.ref_vel-60)<.02f);
  assert(!Motion_setRamps(360,90));
  assert(Motion_stop(0) && !Motion_setRamps(360,90));
  while(Joint.status.state==MOTION_BRAKE)step();
  assert(Joint.status.state==MOTION_HOLD);

  assert(Motion_moveTo(-60,120));
  float peak_acceleration=0, peak_braking=0;
  for (unsigned i=0;i<5000 && Joint.status.state==MOTION_MOVE;i++)
  {
    float old_speed=fabsf(Joint.ref_vel);
    step();
    float change=(fabsf(Joint.ref_vel)-old_speed)*1000;
    peak_acceleration=fmaxf(peak_acceleration,change);
    peak_braking=fmaxf(peak_braking,-change);
  }
  assert(Joint.status.arrived && Joint.status.state==MOTION_HOLD && !Joint.status.fault);
  assert(fabsf(Joint.status.position+60)<=.1f);
  assert(fabsf(peak_acceleration-120)<.03f && fabsf(peak_braking-360)<.03f);
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
  assert(!Motion_setRamps(120,0)); // Cannot retune braking while moving.
  assert(Motion_stop(0));unsigned stopped=0;
  while(Joint.status.state!=MOTION_HOLD && stopped<2000){step();stopped++;}
  assert(stopped<2000 && !Joint.status.fault && Joint.status.stationary);
  assert(Motion_setRamps(120,0));assert(Motion_runVelocity(-90));
  for(unsigned i=0;i<11000;i++)
  {
    step();
    if(i==999)assert(fabsf(Joint.ref_vel+60)<.02f);
  }
  assert(Motion_stop(1) && Joint.status.interlocked);
  for(unsigned i=0;i<1300;i++)step();
  assert(Joint.status.state==MOTION_HOLD && !Motion_runVelocity(10));
  Motion_release();assert(Motion_setZero(10000,0));Joint.status.stationary=1;
  assert(Motion_runVelocity(-10));Motion_update(10000+200,1,0);
  assert(Joint.status.state==MOTION_VELOCITY && Joint.status.fault==FAULT_NONE);
  Motion_clearFault();assert(Motion_setZero(10000,0));Joint.status.stationary=1;
  assert(Motion_runVelocity(10));Motion_update(10000,4,0);
  assert(Joint.status.fault==FAULT_CONTROL && Joint.status.fault_detail==CONTROL_TRAJECTORY_GAP);
  assert(!Joint.status.referenced);
  checkBrakeResidual();
  memset(&Joint,0,sizeof(Joint));
  assert(Motion_setZero(10000,0));
  Motion_update(10000,1,0);
  assert(Motion_setRamps(900,450));
  assert(Motion_moveTo(-90,380));
  Motion_release();
  Motion_update(10000,1,0);
  assert(Motion_runVelocity(-380));
  Motion_release();
  Motion_update(10000,1,0);
  assert(Motion_moveTo(90,380));
  Motion_release();
  Motion_update(10000,1,0);
  assert(Motion_setRamps(360,180) && Motion_moveTo(-90,380));
  checkRamps();
  puts("velocity: PASS (J1 ramps, cruise/brake, settled residual, interlock, following protection)");
  return 0;
}
