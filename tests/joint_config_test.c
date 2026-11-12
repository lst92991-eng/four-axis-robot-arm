#include "config.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
  uint32_t image[132] = {0};
  uint32_t metadata[3] = {JOINT_APP_MAGIC, sizeof(image), 0};
  uint32_t *target = image + 128;
  assert(!JointConfig_fromImage(metadata, image));

  target[0] = JOINT_TARGET_MAGIC;
  target[1] = FOUR_AXIS_COUNT;
  for (uint32_t node = 1; node <= FOUR_AXIS_COUNT; ++node)
  {
    target[2] = node;
    target[3] = node;
    assert(JointConfig_fromImage(metadata, image) == &four_axis_config[node - 1]);
  }

  target[3] = 3;
  assert(!JointConfig_fromImage(metadata, image));
  target[2] = 0;
  assert(!JointConfig_fromImage(metadata, image));
  target[2] = target[3] = 5;
  target[1] = SIX_AXIS_COUNT;
  assert(!JointConfig_fromImage(metadata, image));
  metadata[1] = JOINT_APP_METADATA - JOINT_APP_ADDRESS + 4;
  assert(!JointConfig_fromImage(metadata, image));

  assert(four_axis_config[0].following_error_deg == 2.5f);
  assert(four_axis_config[1].ratio == 20);
  assert(four_axis_config[2].speed_kp == 1 && four_axis_config[3].speed_ki == 7.5f);
  assert(four_axis_config[2].driven_pulley_teeth == 3 * four_axis_config[2].drive_pulley_teeth);
  assert(six_axis_config[1].can_node == 3 && six_axis_config[2].can_node == 2);
  puts("joint_config: PASS (node selection, invalid target rejection, axis profiles)");
}
