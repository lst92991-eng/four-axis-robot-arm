/** @file pwm_output.h
 * @brief 提供全桥输出与采样时序接口。 */
#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include <stdint.h>
#include "control_timing.h"

//运行期由控制中断调用，前台状态切换由电机模块临界区保护

//---------初始化区---------
/* 两相全桥PWM、采样触发接口；初始化后栅极驱动关闭。 */
void pwm_output_init(void);
//---------输出控制区---------
/** @brief 关闭TMR1功率输出，低侧切回高电平GPIO，并清除调度状态。 */
void pwm_output_disable(void);
/** @brief 将四路功率比较值置零，再开启CH5采样触发和采样调度。 */
void pwm_output_enable_sampling(void);
/** @brief 提交两相有符号比较计数差，由后续PWM更新服务构帧。 */
void pwm_output_set_signed_phase_duty(int16_t phase_a, int16_t phase_b);
//---------采样同步区---------
uint8_t pwm_output_pwm_sample_begin(void);
void pwm_output_pwm_update_service(void);

#endif
