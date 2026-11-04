/**
 * @file CAN.h
 * @brief 声明CAN-FD硬件收发接口，返回值表示当前硬件操作结果。
 */
#ifndef AT32_CAN_H
#define AT32_CAN_H
#include <stdint.h>

/* CAN1负责CAN-FD帧收发；communication_CAN负责位置命令与状态。 */
//---------初始化区---------
/** @brief 配置500k仲裁段、2M数据段及CAN1引脚，成功返回1。 */
int CAN_init(void);
//---------收发接口区---------
/** @return 1为完整帧，0为暂无帧，-1为已取出的帧格式不支持。 */
int CAN_receive(uint16_t *identifier, uint8_t data[64]);
/** @return 帧写入硬件发送队列返回1；队列满或写入失败返回0。 */
int CAN_send(uint16_t identifier, const uint8_t data[64]);
#endif
