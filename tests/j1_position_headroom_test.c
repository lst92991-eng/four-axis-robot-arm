#include "position_speed_limits.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
  position_speed_limits_t s=position_speed_limits(180,500,1,8,2.5f);
  assert(s.correction==20 && s.total==200);
  assert(position_speed_reference(180,8*2,s.total)==196);
  assert(position_speed_reference(-180,-8*2,s.total)==-196);
  assert(position_speed_reference(180,-8*2,s.total)==164);
  s=position_speed_limits(490,500,1,8,2.5f);
  assert(s.total==500 && position_speed_reference(490,20,s.total)==500);
  s=position_speed_limits(500,500,1,8,2.5f);
  assert(s.total==500 && position_speed_reference(-500,-20,s.total)==-500);
  s=position_speed_limits(.1f,500,1,8,2.5f);
  assert(s.correction==.1f && s.total==.2f);
  s=position_speed_limits(50,500,25,8,2.5f);
  assert(fabsf(s.total-57.6f)<.001f);
  // The same calculation remains correct after conversion to encoder counts/s.
  float scale=16384.0f/360;
  s=position_speed_limits(180,500,1,8,2.5f);
  assert(fabsf(position_speed_reference(180*scale,16*scale,s.total*scale)/scale-196)<.001f);
  puts("headroom: PASS (positive/negative catch-up, slowdown, low speed, 500 cap, motor cap)");
  return 0;
}
