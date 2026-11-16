#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <stdint.h>
typedef struct {
    float x_mm, y_mm, yaw_rad;
    float yaw_deg; /* 对外航向±180°；yaw_rad为内部连续累计角。 */
    float encoder_yaw_rad, gyro_yaw_rad;
    float v_mm_s, w_rad_s;
    uint8_t valid, encoder_rejected;
} Odometry;
/* 只重设里程计原点和差分基准，不清硬件计数，不影响轮控测速。 */
void Odom_Reset(int32_t raw_left, int32_t raw_right);
uint8_t Odom_Update(int32_t raw_left, int32_t raw_right, float gyro_z_dps, float dt);
void Odom_Get(Odometry *data);
#endif
