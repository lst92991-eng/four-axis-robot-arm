#include "motion.h"
#include <assert.h>
#include <stdio.h>

const JointConfig *joint_config = &four_axis_config[2];

int main(void)
{
  assert(Motion_setZero(0, -207));
  Motion_update(0, 1, 0);
  assert(Motion_runVelocity(3));
  for (int ms=0; ms<3000; ++ms)
    Motion_update(0, 1, 0);
  assert(Joint.status.state == MOTION_VELOCITY && Joint.ref_vel == 3);
  assert(Joint.status.fault == FAULT_NONE);

  assert(Motion_stop(0));
  for (int ms=0; ms<400; ++ms)
    Motion_update(10, 1, 0);
  assert(Joint.status.state == MOTION_HOLD && Joint.status.fault == FAULT_NONE);
  assert(Joint.ref_pos == Joint.status.position);
  assert(Motion_stop(1) && Joint.status.interlocked && Joint.status.state == MOTION_HOLD);

  Motion_trip(FAULT_MOTION);
  Motion_clearFault();
  assert(Joint.status.state == MOTION_OFF && Joint.status.referenced);
  assert(Joint.status.fault == FAULT_NONE && !Joint.status.interlocked);
  assert(Motion_moveTo(-235, 3));
  puts("rotary_teaching: PASS (velocity jog past old window, actual-position hold, reference recovery)");
}
