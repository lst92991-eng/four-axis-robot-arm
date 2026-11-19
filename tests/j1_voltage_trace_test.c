#include "z_trace.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

motion_control_t Joint;
static const JointConfig config = {.can_node = 1};
const JointConfig *joint_config = &config;
static uint32_t trace[16];
static uint32_t phase_trace[16];

float arm_sin_f32(float angle) { return sinf(angle); }
float arm_cos_f32(float angle) { return cosf(angle); }
int CAN_send(uint16_t identifier, const uint8_t data[64])
{
  if (identifier == 0x301)
  {
    uint32_t magic;
    memcpy(&magic,data,sizeof(magic));
    memcpy(magic==0x31434850u ? phase_trace : trace,data,sizeof(trace));
  }
  return 1;
}
void Motor_readCurrentFault(int32_t values[12]) { memset(values, 0, 12 * sizeof(*values)); }

static void checkCurrentReversal(float direction)
{
  Current_Struct original = {.voltageLimit = 20000};
  original.q.KP = 3;
  original.q.KI = .7f;
  original.q.targetValue = direction * 2000;
  Current_Struct tracking = original;
  //保持同一个饱和给定，比较随后制动时的剩余积分。
  for (int i = 0; i < 40; ++i)
  {
    Common_Current_Compute(&original, 0, 24000);
    Common_Current_Compute(&tracking, 0, 24000);
    Common_Current_TrackOutput(&tracking, 1, 24000);
  }
  assert(original.voltageClipped && tracking.voltageClipped);
  assert(original.q.out == tracking.q.out);
  assert(fabsf(tracking.q.sumError - direction * 14000) < .01f);

  original.q.targetValue = tracking.q.targetValue = direction * -500;
  Common_Current_Compute(&original, 0, 24000);
  Common_Current_Compute(&tracking, 0, 24000);
  assert(!tracking.voltageClipped);
  assert(fabsf((original.q.out - tracking.q.out) - direction * 6000) < .01f);
}

static void checkPwmLimit(void)
{
  Current_Struct current = {.voltageLimit = 20000};
  current.d.KP = current.q.KP = 2;
  current.d.targetValue = 1000;
  current.q.targetValue = -500;
  current.d.sumError = 8000;
  current.q.sumError = -9000;
  Common_Current_Compute(&current, 0, 24000);
  assert(!current.voltageClipped);
  Common_Current_TrackOutput(&current, 1, 24000);
  assert(current.d.sumError == 8000 && current.q.sumError == -9000);
  Common_Current_TrackOutput(&current, .8f, 24000);
  assert(current.d.sumError == 6000 && current.q.sumError == -7000);
}

int main(void)
{
  Current_Struct current = {.voltageLimit = 6000};
  current.d.KP = current.q.KP = 1;
  current.d.targetValue = -3000;
  current.q.targetValue = 4000;
  Common_Current_Compute(&current, 0, 24000);
  assert(!current.voltageClipped && current.d.out == -3000 && current.q.out == 4000);

  current.d.targetValue = -6000;
  current.q.targetValue = 8000;
  Common_Current_Compute(&current, 0, 24000);
  assert(current.voltageClipped && fabsf(current.d.out + 3600) < .01f &&
         fabsf(current.q.out - 4800) < .01f);

  motor_status_t status = {.encoder_valid = 1, .trace_voltage_clipped = current.voltageClipped,
                           .trace_vd = (int16_t)lroundf(current.d.out),
                           .trace_vq = (int32_t)lroundf(current.q.out),
                           .bus_trip_mv = 30940, .bus_peak_mv = 31000,
                           .phase_a_counts = -400, .phase_a_peak_counts = 490,
                           .phase_b_counts = 320, .phase_b_peak_counts = 480};
  z_trace(&status, 20);
  assert(trace[0] == 0x3147545a && trace[9] == 4800 && (trace[13] & 0xffff) == 3);
  assert((int16_t)(trace[13] >> 16) == -3600 && trace[14] == 30940 && trace[15] == 31000);
  assert(phase_trace[0]==0x31434850u && (int32_t)phase_trace[2]==-400 &&
         phase_trace[3]==320 && phase_trace[4]==490 && phase_trace[5]==480);
  status.encoder_valid = 0;
  z_trace(&status, 40);
  assert(trace[13] == 0);

  current.voltageLimit = 8000;
  current.d.targetValue = -4800;
  current.q.targetValue = 6400;
  Common_Current_Compute(&current, 0, 24000);
  assert(!current.voltageClipped && current.d.out == -4800 && current.q.out == 6400);

  current.voltageLimit = 10000;
  current.d.targetValue = -6600;
  current.q.targetValue = 8800;
  Common_Current_Compute(&current, 0, 24000);
  assert(current.voltageClipped && fabsf(current.d.out + 6000) < .01f &&
         fabsf(current.q.out - 8000) < .01f);
  status.encoder_valid = 1;
  status.power = 1;
  Joint.ref_pos = 30.124;
  Joint.status.position = 10;
  z_trace(&status, 60);
  assert(((trace[13] >> 2) & 0x3fff) == 5031);
  Joint.ref_pos = 8.752;
  z_trace(&status, 80);
  assert(((trace[13] >> 2) & 0x3fff) == ((uint32_t)-312 & 0x3fff));
  checkCurrentReversal(1);
  checkCurrentReversal(-1);
  checkPwmLimit();
  puts("j1_voltage_trace: PASS (d/q clipping, signed wire fields, both braking directions and PWM tracking)");
  return 0;
}
