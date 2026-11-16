#ifndef IMU_BMI270_H
#define IMU_BMI270_H
#include <stdint.h>

typedef struct {
    float ax_g, ay_g, az_g;
    float gz_raw_dps, bias_dps, gz_dps;
    uint32_t sequence;
    uint8_t valid;
} ImuData;

/* 启动期间保持板子和电机静止，完成SPI配置与64个样本的零偏校准。 */
uint8_t Imu_Init(void);
/* 尝试读取最新数据；通信失败或数据超过50ms未更新时返回0。 */
uint8_t Imu_Update(uint32_t now_ms);
void Imu_Get(ImuData *data);
#endif
