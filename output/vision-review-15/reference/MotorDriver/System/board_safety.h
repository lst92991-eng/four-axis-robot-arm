#ifndef BOARD_SAFETY_H
#define BOARD_SAFETY_H
#include <stdint.h>
/* 传感器启动校准完成后启用约100ms独立看门狗。 */
uint8_t Board_WatchdogInit(void);
/* 仅在应用完成一个处理周期后调用。 */
void Board_WatchdogFeed(void);
/* CPU异常、RTOS断言或栈溢出：主动制动，停止喂狗，等待复位。 */
void Board_SafeHalt(void);
#endif
