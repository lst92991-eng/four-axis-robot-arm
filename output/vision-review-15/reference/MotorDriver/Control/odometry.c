#include "odometry.h"
#include "motor_lesson_config.h"
#include <math.h>
#include <string.h>
static Odometry state;
static int32_t last_left, last_right;

static int32_t CountDelta(int32_t current, int32_t previous)
{
    uint32_t delta = (uint32_t)current - (uint32_t)previous;
    return delta <= INT32_MAX ? (int32_t)delta : -(int32_t)(UINT32_MAX - delta) - 1;
}

void Odom_Reset(int32_t raw_left, int32_t raw_right)
{
    memset(&state, 0, sizeof(state));
    last_left = raw_left;
    last_right = raw_right;
}

uint8_t Odom_Update(int32_t raw_left, int32_t raw_right, float gyro_z_dps, float dt)
{
    float dl, dr, distance, encoder_delta, before, midpoint, predicted;
    uint8_t encoder_ok;
    state.valid = 0;
    if (!(dt >= 0.001f && dt <= 0.1f)) return 0;
    dl = (float)CountDelta(raw_left, last_left) * LESSON_LEFT_FEEDBACK_SIGN;
    dr = (float)CountDelta(raw_right, last_right) * LESSON_RIGHT_FEEDBACK_SIGN;
    last_left = raw_left;
    last_right = raw_right;
    dl *= LESSON_WHEEL_CIRCUMFERENCE_MM / LESSON_COUNTS_PER_REV;
    dr *= LESSON_WHEEL_CIRCUMFERENCE_MM / LESSON_COUNTS_PER_REV;
    distance = (dl + dr) * 0.5f;
    encoder_delta = (dr - dl) / LESSON_TRACK_MM;
    before = state.yaw_rad;
    encoder_ok = fabsf(encoder_delta / dt) <= 120.0f * 0.01745329252f;
    if (encoder_ok) state.encoder_yaw_rad += encoder_delta;
    else state.encoder_rejected = 1;
    state.gyro_yaw_rad += gyro_z_dps * 0.01745329252f * dt;
    predicted = before + gyro_z_dps * 0.01745329252f * dt;
    if (LESSON_ODOM_FUSION) {
        /* 原算法：异常轮差不参与纠偏，本次仍用陀螺预测。 */
        state.yaw_rad = encoder_ok ? predicted + dt / (LESSON_ODOM_TAU_S + dt) *
                       (state.encoder_yaw_rad - predicted) : predicted;
    } else {
        if (!encoder_ok) return 0;
        state.yaw_rad = state.encoder_yaw_rad;
    }
    midpoint = before + (state.yaw_rad - before) * 0.5f;
    state.x_mm += distance * cosf(midpoint);
    state.y_mm += distance * sinf(midpoint);
    state.v_mm_s = distance / dt;
    state.w_rad_s = (state.yaw_rad - before) / dt;
    /* 对外保持原工程的±180°表示；累计弧度独立保留供连续转圈观察。 */
    state.yaw_deg = fmodf(state.yaw_rad * 57.2957795f, 360.0f);
    if (state.yaw_deg > 180.0f) state.yaw_deg -= 360.0f;
    if (state.yaw_deg < -180.0f) state.yaw_deg += 360.0f;
    state.valid = 1;
    return 1;
}

void Odom_Get(Odometry *data) { *data = state; }
