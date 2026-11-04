/** @file mt6816.h
 * @brief 提供MT6816初始化与原始角度读取接口。 */
#ifndef MT6816_H
#define MT6816_H

#include <stdint.h>

//---------初始化区---------
void mt6816_init(void);
//---------角度读取区---------
/** @brief 两次16位SPI读取后检查状态位和偶校验，通过时写入0..16383并返回1。 */
uint8_t mt6816_read_angle(uint16_t *raw_angle);

#endif
