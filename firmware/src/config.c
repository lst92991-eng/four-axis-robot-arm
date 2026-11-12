#include "config.h"

// 顺序为 J1 至 J4。单位见结构体字段，旋转轴速度使用配置后的关节角度。
const JointConfig four_axis_config[FOUR_AXIS_COUNT] = {
  { // J1，丝杆升降
    .can_node = 1, .ratio = 1, .joint_direction = 1,
    .pole_pairs = 50, .encoder_direction = 1, .encoder_zero = -1,
    .minimum_deg = -3610, .maximum_deg = 7210,
    .default_speed_deg_s = 10, .max_speed_deg_s = 500,
    .acceleration_deg_s2 = 60, .brake_deg_s2 = 360, .following_error_deg = 2.5f,
    .current_limit_ma = 1400, .voltage_limit_mv = 6000,
    .speed_kp = 1.6f, .speed_ki = 200, .position_kp = 8, .current_kp = 3, .current_ki = .7f,
  },
  { // J2，近端旋转
    .can_node = 2, .ratio = 20, .joint_direction = 1,
    .pole_pairs = 50, .encoder_direction = 1, .encoder_zero = -1,
    .default_speed_deg_s = 10, .max_speed_deg_s = 30,
    .acceleration_deg_s2 = 30, .brake_deg_s2 = 30, .following_error_deg = 0,
    .current_limit_ma = 1000, .voltage_limit_mv = 6000,
    .speed_kp = 20, .speed_ki = 150, .position_kp = 8, .current_kp = 3, .current_ki = .7f,
  },
  { // J3，中间旋转
    .can_node = 3, .ratio = 1, .joint_direction = 1,
    .pole_pairs = 50, .encoder_direction = 1, .encoder_zero = -1,
    .drive_pulley_teeth = 20, .driven_pulley_teeth = 60,
    .default_speed_deg_s = 10, .max_speed_deg_s = 30,
    .acceleration_deg_s2 = 30, .brake_deg_s2 = 30, .following_error_deg = 0,
    .current_limit_ma = 1000, .voltage_limit_mv = 6000,
    .speed_kp = 1, .speed_ki = 7.5f, .position_kp = 8, .current_kp = 3, .current_ki = .7f,
  },
  { // J4，末端旋转
    .can_node = 4, .ratio = 1, .joint_direction = 1,
    .pole_pairs = 50, .encoder_direction = 1, .encoder_zero = -1,
    .default_speed_deg_s = 10, .max_speed_deg_s = 30,
    .acceleration_deg_s2 = 30, .brake_deg_s2 = 30, .following_error_deg = 0,
    .current_limit_ma = 1000, .voltage_limit_mv = 6000,
    .speed_kp = 1, .speed_ki = 7.5f, .position_kp = 8, .current_kp = 3, .current_ki = .7f,
  },
};

// 顺序为 J1 至 J6。型号和标称比来自六轴 joint_workspace.py。
// J2 的节点、方向、限流来自 2026-10-20 实板记录，试车速度来自 R3 清单。
// J3 的节点为交接时的临时映射，25:1 是上位机预置值。
const SixAxisJointConfig six_axis_config[SIX_AXIS_COUNT] = {
  { .motor_length_mm = 48, .nominal_ratio = 25 },
  { .can_node = 3, .motor_length_mm = 60, .nominal_ratio = 20, .up_motor_direction = -1,
    .saved_iq_limit_counts = 694, .test_output_speed_mdeg_s = 270 },
  { .can_node = 2, .motor_length_mm = 34, .nominal_ratio = 25 },
  { .motor_length_mm = 34, .nominal_ratio = 10 },
  { .motor_length_mm = 40, .nominal_ratio = 1 },
  { .motor_length_mm = 40, .nominal_ratio = 1 },
};

const JointConfig *JointConfig_fromImage(const uint32_t metadata[3], const uint32_t *image)
{
  uint32_t size = metadata[1];
  if (metadata[0] != JOINT_APP_MAGIC || size < 512 + 4 * sizeof(uint32_t) ||
      size > JOINT_APP_METADATA - JOINT_APP_ADDRESS || size % sizeof(uint32_t))
    return 0;

  const uint32_t *target = image + size / sizeof(uint32_t) - 4;
  uint32_t joint = target[2];
  if (target[0] != JOINT_TARGET_MAGIC || target[1] != FOUR_AXIS_COUNT ||
      joint < 1 || joint > FOUR_AXIS_COUNT)
    return 0;

  const JointConfig *config = &four_axis_config[joint - 1];
  return target[3] == config->can_node ? config : 0;
}
