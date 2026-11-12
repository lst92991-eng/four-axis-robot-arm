#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H
#include <stdint.h>

#define JOINT_APP_ADDRESS 0x08004000u
#define JOINT_APP_METADATA 0x0801E000u
#define JOINT_APP_MAGIC 0x3141544fu
#define JOINT_TARGET_MAGIC 0x31465250u
#define FOUR_AXIS_COUNT 4u
#define SIX_AXIS_COUNT 6u

typedef struct
{
  uint8_t can_node;
  int8_t joint_direction, encoder_direction;
  uint8_t pole_pairs;
  int16_t encoder_zero;  // -1 要求使用本板 Flash 标定。
  uint8_t drive_pulley_teeth, driven_pulley_teeth;  // CAD 同步带级，0 表示未录入。
  float ratio;  // 编码器角与当前 CAN 逻辑角的换算比。
  float minimum_deg, maximum_deg;  // 软限位待示教。上下限相同表示未设置。
  float default_speed_deg_s, max_speed_deg_s;
  float acceleration_deg_s2, brake_deg_s2;
  float following_error_deg;  // 0 沿用旋转轴原有策略。
  float current_limit_ma;
  float speed_kp, speed_ki, position_kp, current_kp, current_ki;
  int32_t voltage_limit_mv;
} JointConfig;

// 六轴旧控制器记录。节点、方向及测试参数的 0 值表示未录入。
typedef struct
{
  uint8_t can_node, motor_length_mm, nominal_ratio;
  int8_t up_motor_direction;
  uint16_t saved_iq_limit_counts, test_output_speed_mdeg_s;
} SixAxisJointConfig;

extern const JointConfig four_axis_config[FOUR_AXIS_COUNT];
extern const SixAxisJointConfig six_axis_config[SIX_AXIS_COUNT];
extern const JointConfig *joint_config;

// 引导程序验证整镜像 CRC 后，应用按末尾记录选择本板参数。
const JointConfig *JointConfig_fromImage(const uint32_t metadata[3], const uint32_t *image);
#endif
