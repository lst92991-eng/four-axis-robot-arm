#include "pid.h"

static float Clamp(float value, float limit)
{
    return value > limit ? limit : (value < -limit ? -limit : value);
}

void Pid_Reset(Pid *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous = 0;
}

uint8_t Pid_Update(Pid *pid, float target, float measured,
                   float feedforward, float dt, float *output)
{
    float error, derivative, next_i, candidate, result;
    uint8_t pushing_high, pushing_low;
    *output = 0.0f;
    if (!(dt > 0.0f && dt <= 0.1f)) return 0;
    error = target - measured;
    derivative = pid->has_previous ? (error - pid->previous_error) / dt : 0.0f;
    next_i = Clamp(pid->integral + pid->ki * error * dt, pid->integral_limit);
    candidate = pid->kp * error + next_i + pid->kd * derivative;
    /* 与原轮控一致：先限制PID修正量，前馈另加；误差加深修正饱和时冻结积分。 */
    pushing_high = candidate > pid->integral_limit && error > 0.0f;
    pushing_low = candidate < -pid->integral_limit && error < 0.0f;
    if (pushing_high || pushing_low) next_i = pid->integral;
    result = feedforward + Clamp(pid->kp * error + next_i + pid->kd * derivative,
                                 pid->integral_limit);
    *output = Clamp(result, pid->output_limit);
    pid->integral = next_i;
    pid->previous_error = error;
    pid->has_previous = 1;
    return 1;
}
