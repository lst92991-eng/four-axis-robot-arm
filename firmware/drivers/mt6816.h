/** @file mt6816.h
 * @brief 提供MT6816初始化与原始角度读取接口。 */
#ifndef MT6816_H
#define MT6816_H

#include <stdint.h>

//---------初始化区---------
void mt6816_init(void);
//---------角度读取区---------
// 返回0表示成功。非零值同时作为FAULT_SENSOR的状态细分，磁场位和奇偶错误可并存。
enum { MT6816_OK=0, MT6816_SPI_HIGH=1, MT6816_SPI_LOW=2, MT6816_FIELD=4, MT6816_PARITY=8 };
/** @brief 两次16位SPI读取后检查状态位和偶校验，通过时写入0..16383。 */
uint8_t mt6816_read_angle_status(uint16_t *raw_angle);

#endif
