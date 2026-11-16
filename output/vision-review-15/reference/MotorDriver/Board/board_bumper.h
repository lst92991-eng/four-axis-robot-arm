#ifndef BOARD_BUMPER_H
#define BOARD_BUMPER_H
#include <stdint.h>

/* 先按引脚验证输入，方向名称在实物逐路确认后填写。 */
#define BUMPER_PC15 0x01U
#define BUMPER_PC13 0x02U
#define BUMPER_PC14 0x04U
#define BUMPER_PB9  0x08U

void Board_BumperInit(void);
uint8_t Board_BumperRead(void);
void Board_BumperEnableIRQ(void);
void Board_BumperIRQ(void);
uint8_t Board_BumperUpdate(uint32_t now_ms);
uint8_t Board_BumperClear(uint32_t now_ms);
#endif
