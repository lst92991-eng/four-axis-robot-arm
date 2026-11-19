/** @file current_sense.h
 * @brief 提供母线电压、相电流零偏校准及PWM采样回调。 */
#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

//---------初始化区---------
void current_sense_init(void);
//---------采样控制区---------
/** @brief 在零输出采样状态累积每相偏置；采样期间DMA中断须持续运行。 */
void current_sense_calibrate_offsets(void);
/** @brief 零偏、编码器和控制器就绪后，允许DMA中断进入控制。 */
void current_sense_start_control(void);
//---------反馈读取区---------
uint32_t current_sense_get_bus_mv(void);
#endif
