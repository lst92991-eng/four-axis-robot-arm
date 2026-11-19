//CAN协议保存命令序号和通信时间，负责报文处理
#ifndef AT32_COMMUNICATION_CAN_H
#define AT32_COMMUNICATION_CAN_H
#include "common.h"
#include "motion.h"

//---------协议定义区---------
//CAN的精简协议
enum
{
  CMD_ZERO = 2,
  //相对机械零点的绝对目标
  CMD_MOVE,
  CMD_STOP,
  CMD_OFF,
  CMD_CLEAR,
  CMD_PING,
  //相对当前反馈位置的角度增量
  CMD_MOVE_REL = 8,
  CMD_CALIBRATE = 9,
  CMD_TUNE_SPEED = 10,
  CMD_REBOOT = 11,
  CMD_VELOCITY = 12,
  CMD_BRAKE_RATE = 13,
  MSG_STATUS = 0x40
};

//---------报文结构区---------
/* 64字节小端帧；位置和速度单位为毫度、毫度/秒。 */
typedef struct
{
  uint8_t kind;
  uint8_t node;
  //请求填0；应答0表示接受，1表示拒绝
  uint8_t result;
  uint8_t version;
  uint32_t sequence;
  int32_t values[14];
} CanPacket;

//---------初始化区---------
//初始化通信参数和CAN硬件
void CAN_protocol_setup(void);
//---------通信处理区---------
//接收命令并发送应答，定期上传运行状态
void CAN_protocol_process(const motor_status_t *motor, uint32_t now);
//超过命令时限时请求减速保持
void CAN_protocol_watchdog(uint32_t now);
#endif
