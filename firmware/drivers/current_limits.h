/** @file current_limits.h
 * @brief 定义构建电流上限和ADC计数换算。 */
#ifndef CURRENT_LIMITS_H
#define CURRENT_LIMITS_H
#include <stdint.h>
#include <limits.h>

//---------采样换算参数区---------
/* 电流上限由构建参数选择；ADC换算采用3.3V、10mΩ分流和20倍放大。 */
#define CURRENT_ADC_VREF_MV             3300L
#define CURRENT_ADC_FULL_SCALE         4095L
#define CURRENT_SHUNT_MOHM                10L
#define CURRENT_AMPLIFIER_GAIN            20L
//---------电流门限区---------
#ifndef STEPFOC_MOTOR_CURRENT_MA
#define STEPFOC_MOTOR_CURRENT_MA        5000L
#endif
#if STEPFOC_MOTOR_CURRENT_MA != 1500 && STEPFOC_MOTOR_CURRENT_MA != 2000 && \
    STEPFOC_MOTOR_CURRENT_MA != 2500 && STEPFOC_MOTOR_CURRENT_MA != 5000
#error Unsupported motor current ceiling
#endif
#define CURRENT_RUNTIME_REFERENCE_MAX ((STEPFOC_MOTOR_CURRENT_MA * 819L) / 3300L)
#define CURRENT_RUNTIME_PHASE_TRIP CURRENT_RUNTIME_REFERENCE_MAX

// 六轴短程电角标定器使用 ADC counts；上限不超过本项目 1 A。
#define CURRENT_CAL_REFERENCE_MIN 20L
#define CURRENT_CAL_REFERENCE_DEFAULT 74L
#define CURRENT_CAL_REFERENCE_MAX 248L
#define CURRENT_CAL_PHASE_TRIP 248L

//---------单位换算区---------
/* 使用64位中间值换算mA；整数除法向零截断，结果钳位到int32范围。 */
static inline int32_t current_counts_to_ma(int32_t counts)
{
  int64_t value = (int64_t)counts * CURRENT_ADC_VREF_MV * 1000L /
                  (CURRENT_ADC_FULL_SCALE * CURRENT_SHUNT_MOHM * CURRENT_AMPLIFIER_GAIN);
  if (value > INT32_MAX)
    return INT32_MAX;
  if (value < INT32_MIN)
    return INT32_MIN;
  return (int32_t)value;
}
#endif
