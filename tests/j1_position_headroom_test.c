#include "position_speed_limits.h"
#include <assert.h>
#include <stdio.h>

static void check_move_tail(void)
{
  // 38°与47°分别对应升、降实测末段偏差。位置环仍以原来的Kp=8纠偏。
  const float target_speed = 2400;
  const float braking = 3000;
  const float baseline = 20;
  for (int sign = -1; sign <= 1; sign += 2)
  {
    float error = sign * (sign > 0 ? 38.0f : 47.0f);
    float cap = position_move_correction_limit(baseline, target_speed, error, 0, 0, braking);
    assert(position_speed_reference(0, 8 * error, cap) == 8 * error);
    assert(cap <= sqrtf(2 * braking * fabsf(error)));

    // 剩余轨迹还长、前馈还快时，均沿用普通纠偏。
    assert(position_move_correction_limit(baseline, target_speed, error, sign * 50, 0, braking) == baseline);
    assert(position_move_correction_limit(baseline, target_speed, error, 0, sign * 800, braking) == baseline);
    assert(position_move_correction_limit(baseline, target_speed, error, 0, 0, 0) == baseline);
  }
  // 刹车速度预算扣除前馈后逐步开放，穿过原20°/s边界时给定连续。
  float ramp = position_move_correction_limit(baseline, target_speed, 29, 5, 355, braking);
  assert(fabsf(ramp - (sqrtf(2 * braking * 24) - 355)) < .001f);
  float boundary = sqrtf(2 * braking * 24) - baseline;
  assert(fabsf(position_move_correction_limit(baseline, target_speed, 29, 5, boundary - .01f, braking)
               - baseline - .01f) < .001f);
  assert(position_move_correction_limit(.1f, .1f, 38, 0, 0, braking) == .1f);
  assert(position_move_correction_limit(baseline, target_speed, 100000, 0, 0, braking) == target_speed);
  // 小误差实际给定继续由Kp决定。
  float small_cap = position_move_correction_limit(baseline, target_speed, -.25f, 0, 0, braking);
  assert(position_speed_reference(0, -2, small_cap) == -2);
}

int main(void)
{
  position_speed_limits_t s=position_speed_limits(180,500,MOTOR_SPEED_LIMIT_DEG_S,1,8,2.5f);
  assert(s.correction==20 && s.total==200);
  assert(position_speed_reference(180,8*2,s.total)==196);
  assert(position_speed_reference(-180,-8*2,s.total)==-196);
  assert(position_speed_reference(180,-8*2,s.total)==164);
  s=position_speed_limits(490,500,MOTOR_SPEED_LIMIT_DEG_S,1,8,2.5f);
  assert(s.total==500 && position_speed_reference(490,20,s.total)==500);
  s=position_speed_limits(500,500,MOTOR_SPEED_LIMIT_DEG_S,1,8,2.5f);
  assert(s.total==500 && position_speed_reference(-500,-20,s.total)==-500);
  s=position_speed_limits(.1f,500,MOTOR_SPEED_LIMIT_DEG_S,1,8,2.5f);
  assert(s.correction==.1f && s.total==.2f);
  s=position_speed_limits(50,500,MOTOR_SPEED_LIMIT_DEG_S,25,8,2.5f);
  assert(fabsf(s.total-57.6f)<.001f);
  // The same calculation remains correct after conversion to encoder counts/s.
  float scale=16384.0f/360;
  s=position_speed_limits(180,500,MOTOR_SPEED_LIMIT_DEG_S,1,8,2.5f);
  assert(fabsf(position_speed_reference(180*scale,16*scale,s.total*scale)/scale-196)<.001f);
  float motor_max = J1_MOTOR_SPEED_LIMIT_DEG_S;
  s=position_speed_limits(motor_max,12000,motor_max,1,8,2.5f);
  assert(s.total==motor_max && position_speed_reference(-motor_max,-20,s.total)==-motor_max);
  check_move_tail();
  puts("headroom: PASS (both directions, terminal error, feedforward budget, low speed and motor cap)");
  return 0;
}
