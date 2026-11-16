#ifndef BOARD_DEBUG_H
#define BOARD_DEBUG_H

#include <stdint.h>

void Board_MotorOff(void);
void Board_DebugInit(void);
int Board_DebugWrite(const char *text);
void Board_DelayMs(uint32_t milliseconds);

#endif
