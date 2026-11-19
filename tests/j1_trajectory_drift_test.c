#include "../firmware/src/config.h"
static const JointConfig profile = {
  .can_node = 1, .joint_direction = 1, .ratio = 1,
  .minimum_deg = (-3600.0f), .maximum_deg = 3600.0f,
  .max_speed_deg_s = 12000, .following_error_deg = 2.5f,
  .acceleration_deg_s2 = 10.0f, .brake_deg_s2 = 30,
};
const JointConfig *joint_config = &profile;
#include "../firmware/src/motion.c"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void checkCruiseDrift(void)
{
  memset(&Joint,0,sizeof(Joint));Motion_setZero(10000,0);Joint.status.stationary=1;
  assert(Motion_moveTo(3600,10));
  double max_drift=0;
  for(unsigned tick=1;tick<=355000;tick++)
  {
    double t=tick*.001;
    // 10°/s reached after 1 s plus the 60 ms transition.
    double ideal=10*(t-.53);
    // An independently integrated plant, with one degree of tracking lag.
    // Following ref_pos itself would conceal numerical accumulation errors.
    double actual=fmax(0,ideal-1);
    int64_t count=10000+(int64_t)llround(actual*16384/360);
    Motion_update(count,1,t<1.06?(float)(10*t):10);
    if(t>=1.06)
    {
      double drift=fabs(Joint.ref_pos-ideal);
      if(drift>max_drift)max_drift=drift;
    }
    assert(!Joint.status.fault);
  }
  printf("trajectory max numerical drift = %.9f deg\n",max_drift);
  assert(max_drift<.01);
}

static void checkSmoothMove(float target, unsigned dt_ms, double expected_peak)
{
  memset(&Joint,0,sizeof(Joint));
  assert(Motion_setZero(10000,0));
  Joint.status.stationary=1;
  assert(Motion_setRamps(3000,3000));
  assert(Motion_moveTo(target,2400));

  double old_position=0, old_speed=0, old_accel=0, peak=0;
  double dt=dt_ms*.001;
  int sign=target<0?-1:1;
  for(unsigned t=dt_ms;t<=3000 && Joint.status.state==MOTION_MOVE;t+=dt_ms)
  {
    int64_t counts=10000+(int64_t)llround(Joint.ref_pos*16384/360);
    Motion_update(counts,dt_ms,Joint.ref_vel);
    double speed=Joint.ref_vel*sign;
    double accel=(speed-old_speed)/dt;
    assert(speed>=0 && speed<=2400);
    assert(Joint.ref_pos*sign>=old_position*sign-1e-8);
    assert(Joint.ref_pos*sign<=fabs(target)+1e-8);
    assert(fabs(accel)<=3001);
    assert(fabs(accel-old_accel)<=51000*dt);
    peak=fmax(peak,speed);
    old_position=Joint.ref_pos;
    old_speed=speed;
    old_accel=accel;
  }
  assert(Joint.status.state==MOTION_HOLD && Joint.status.arrived && !Joint.status.fault);
  assert(Joint.ref_pos==target && Joint.ref_vel==0);
  assert(fabs(peak-expected_peak)<(expected_peak>10?2:.1));
}

static void checkBrakeAtZero(void)
{
  planJ1Move(0, 1679.525, 2400, 3000, 3000);
  double finish=j1_profile.rise.duration_s+j1_profile.cruise_s+j1_profile.brake.duration_s;
  Joint.status.requested=1679.525f;
  updateJ1Reference(finish-.15);
  assert(fabs(Joint.ref_vel-225)<.01);
  updateJ1Reference(finish-.06);
  assert(fabs(Joint.ref_vel-36)<.01);
  updateJ1Reference(finish-.01);
  assert(fabs(Joint.ref_vel-1)<.01);
  assert(fabs(Joint.status.requested-Joint.ref_pos-1.0/300)<1e-7);
  updateJ1Reference(finish);
  assert(Joint.ref_vel==0 && Joint.ref_pos==Joint.status.requested);
}

static void checkDescentKi(void)
{
  Joint.descent_brake_ki_permille=500;
  Joint.status.state=MOTION_MOVE;
  for (int sign=-1; sign<=1; sign+=2)
  {
    Joint.status.requested=sign*1679.525f;
    planJ1Move(0, 1679.525, 2400, 3000, 3000);
    double start=j1_profile.rise.duration_s+j1_profile.cruise_s;
    Joint.elapsed_s=start-.001;
    assert(Motion_speedKiScale()==1);
    Joint.elapsed_s=start+.001;
    assert(Motion_speedKiScale()==(sign<0?.5f:1));
  }
  Joint.status.requested=-1679.525f;
  Joint.status.state=MOTION_BRAKE;
  assert(Motion_speedKiScale()==1);
  Joint.status.state=MOTION_HOLD;
  assert(Motion_speedKiScale()==1);
}

int main(void)
{
  checkCruiseDrift();
  checkSmoothMove(1679.525f,1,2154.90);
  checkSmoothMove(-1679.525f,3,2154.90);
  checkSmoothMove(2350,3,2400);
  checkSmoothMove(-.02f,1,1.64);
  checkSmoothMove(0,3,0);
  checkBrakeAtZero();
  checkDescentKi();
  puts("J1 smooth move: PASS (travel and return, full speed, short move, acceleration continuity)");
  return 0;
}
