#include "wheel_control.h"
#include "pid.h"
#include "motor_driver.h"
#include "motor_lesson_config.h"
#include <math.h>
#include <float.h>
#include <string.h>

static Pid pid[2];
static WheelStatus state;
static float requested[2], stall_ms[2];
static float accel_ramp, decel_ramp;
static uint8_t zero_ticks[2];
static int8_t last_sign[2];
static const float output_sign[2] = {LESSON_LEFT_OUTPUT_SIGN, LESSON_RIGHT_OUTPUT_SIGN};
static const float feedback_sign[2] = {LESSON_LEFT_FEEDBACK_SIGN, LESSON_RIGHT_FEEDBACK_SIGN};

void Wheel_Stop(void)
{
    uint8_t i;
    for (i = 0; i < 2; ++i) {
        requested[i] = state.target[i] = stall_ms[i] = 0.0f;
        state.pwm[i] = 0;
        Pid_Reset(&pid[i]);
    }
    /* 实测速度、历史方向和零速证据保留，停车命令不能伪造已经停稳。 */
    motor_driver_stop_all(MOTOR_STOP_BRAKE);
}

void Wheel_Init(void)
{
    uint8_t i;
    memset(&state, 0, sizeof(state));
    memset(zero_ticks, 0, sizeof(zero_ticks));
    memset(last_sign, 0, sizeof(last_sign));
    accel_ramp = decel_ramp = LESSON_RAMP_RPM_S;
    for (i = 0; i < 2; ++i) {
        pid[i].kp = i == 0 ? LESSON_LEFT_PID_KP : LESSON_RIGHT_PID_KP;
        pid[i].ki = i == 0 ? LESSON_LEFT_PID_KI : LESSON_RIGHT_PID_KI;
        pid[i].kd = i == 0 ? LESSON_LEFT_PID_KD : LESSON_RIGHT_PID_KD;
        pid[i].output_limit = LESSON_PWM_PERMILLE;
        pid[i].integral_limit = LESSON_PID_I_LIMIT;
    }
    Wheel_Stop();
}

uint8_t Wheel_SetTarget(float left, float right)
{
    float peak;
    /* 入口检查同时排除NaN/无穷；PID每次更新不重复检查增益。 */
    if (!(left >= -FLT_MAX && left <= FLT_MAX &&
          right >= -FLT_MAX && right <= FLT_MAX)) { Wheel_Stop(); return 0; }
    peak = fabsf(left) > fabsf(right) ? fabsf(left) : fabsf(right);
    if (peak > LESSON_MAX_RPM) {
        left *= LESSON_MAX_RPM / peak;
        right *= LESSON_MAX_RPM / peak;
    }
    requested[0] = left;
    requested[1] = right;
    accel_ramp = decel_ramp = LESSON_RAMP_RPM_S;
    return 1;
}

uint8_t Wheel_Differential(float v, float omega, float *left, float *right)
{
    float turn;
    if (!left || !right) return 0;
    *left = *right = 0.0f;
    if (!(v >= -300.0f && v <= 300.0f && omega >= -1200.0f && omega <= 1200.0f)) return 0;
    turn = LESSON_TRACK_MM * (omega / 1000.0f) * 0.5f;
    *left = (v - turn) * 60.0f / LESSON_WHEEL_CIRCUMFERENCE_MM;
    *right = (v + turn) * 60.0f / LESSON_WHEEL_CIRCUMFERENCE_MM;
    return 1;
}

uint8_t Wheel_SetMotion(float v, float omega, float accel, float decel)
{
    float left, right;
    if (v == 0.0f && omega == 0.0f && accel == 0.0f && decel == 0.0f) {
        Wheel_Stop();
        return 1;
    }
    if (!(accel >= 10.0f && accel <= 500.0f && decel >= 10.0f && decel <= 1000.0f) ||
        !Wheel_Differential(v, omega, &left, &right)) { Wheel_Stop(); return 0; }
    if (!Wheel_SetTarget(left, right)) return 0;
    accel_ramp = accel * 60.0f / LESSON_WHEEL_CIRCUMFERENCE_MM;
    decel_ramp = decel * 60.0f / LESSON_WHEEL_CIRCUMFERENCE_MM;
    return 1;
}

uint8_t Wheel_Update(float raw_left, float raw_right, float dt, uint8_t run)
{
    float raw[2] = {raw_left, raw_right};
    float goal[2], fraction = 1.0f;
    uint8_t i, reverse = 0;
    if (!(dt > 0.0f && dt <= 0.1f)) { Wheel_Stop(); return 0; }
    for (i = 0; i < 2; ++i) {
        int8_t sign = requested[i] > 0.0f ? 1 : (requested[i] < 0.0f ? -1 : 0);
        if (!(raw[i] >= -LESSON_MAX_RPM && raw[i] <= LESSON_MAX_RPM)) { Wheel_Stop(); return 0; }
        raw[i] *= feedback_sign[i];
        /* 原工程的0.2系数低通；名义20ms周期。 */
        state.measured[i] += LESSON_RPM_FILTER_ALPHA * (raw[i] - state.measured[i]);
        if (fabsf(raw[i]) <= 1.0f) {
            if (zero_ticks[i] < 3U) ++zero_ticks[i];
        } else zero_ticks[i] = 0;
        if (requested[i] * state.target[i] < 0.0f ||
            (sign && last_sign[i] && sign != last_sign[i] && zero_ticks[i] < 3U)) reverse = 1;
    }
    if (!run) { Wheel_Stop(); return 1; }
    /* 任一轮换向，两轮共同降到0，不能只停其中一侧。 */
    for (i = 0; i < 2; ++i) {
        float delta, maximum;
        goal[i] = reverse ? 0.0f : requested[i];
        delta = fabsf(goal[i] - state.target[i]);
        maximum = (goal[i] * state.target[i] >= 0.0f &&
                   fabsf(goal[i]) > fabsf(state.target[i]) ? accel_ramp : decel_ramp) * dt;
        if (delta > maximum && maximum / delta < fraction) fraction = maximum / delta;
    }
    for (i = 0; i < 2; ++i) {
        float output, feedforward, forward;
        state.target[i] += fraction * (goal[i] - state.target[i]);
        if (state.target[i] == 0.0f) {
            Pid_Reset(&pid[i]);
            state.pwm[i] = 0;
            stall_ms[i] = 0;
            continue;
        }
        last_sign[i] = state.target[i] > 0.0f ? 1 : -1;
        forward = state.target[i] > 0.0f ? state.measured[i] : -state.measured[i];
        if (fabsf(state.target[i]) >= 8.0f && forward <= 1.0f) stall_ms[i] += dt * 1000.0f;
        else stall_ms[i] = 0.0f;
        if (stall_ms[i] >= 200.0f) { Wheel_Stop(); return 0; }
        feedforward = state.target[i] * LESSON_FF_PER_RPM +
            (state.target[i] > 0.0f ? LESSON_FF_STATIC : -LESSON_FF_STATIC);
        if (!Pid_Update(&pid[i], state.target[i], state.measured[i], feedforward, dt, &output) ||
            !(output >= -LESSON_PWM_PERMILLE && output <= LESSON_PWM_PERMILLE)) {
            Wheel_Stop(); return 0;
        }
        if (output * state.target[i] < 0.0f) output = 0.0f;
        state.pwm[i] = (int16_t)output;
    }
    for (i = 0; i < 2; ++i) {
        if (state.pwm[i] == 0) motor_driver_stop((motor_id_t)i, MOTOR_STOP_BRAKE);
        else motor_driver_set_pwm((motor_id_t)i, (int16_t)(state.pwm[i] * output_sign[i]));
    }
    return 1;
}

void Wheel_Get(WheelStatus *status) { if (status) *status = state; }
