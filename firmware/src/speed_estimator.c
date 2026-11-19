#include "speed_estimator.h"
#define ENCODER_STATE_COUNTS_PER_REV 16384L
#include <limits.h>

#define SPEED_ESTIMATOR_Q16                 65536LL
#define SPEED_ESTIMATOR_MDEG_PER_REV        360000LL
#define SPEED_ESTIMATOR_MS_PER_SECOND       1000LL
#define SPEED_ESTIMATOR_MAX_UPDATE_GAP_MS   20U

static int32_t speed_estimator_clamp_i64(int64_t value)
{
  if(value > INT32_MAX)
  {
    return INT32_MAX;
  }
  if(value < INT32_MIN)
  {
    return INT32_MIN;
  }
  return (int32_t)value;
}

static int64_t scale_q16(int64_t value,int64_t gain)
{
  /* Preserve truncation toward zero without overflowing gain*position. */
  return (value/65536)*gain+(value%65536)*gain/65536;
}
static int64_t bound_velocity(int64_t value)
{
  const int64_t maximum=(int64_t)INT32_MAX*16384LL*65536LL/(360000LL*1000LL);
  if(value>maximum)return maximum;
  if(value< -maximum)return -maximum;
  return value;
}

static int32_t speed_estimator_convert_speed(int64_t velocity_q16)
{
  int64_t speed;

  speed = velocity_q16 * SPEED_ESTIMATOR_MDEG_PER_REV *
          SPEED_ESTIMATOR_MS_PER_SECOND;
  speed /= (int64_t)ENCODER_STATE_COUNTS_PER_REV *
           SPEED_ESTIMATOR_Q16;
  return speed_estimator_clamp_i64(speed);
}

uint8_t speed_estimator_init(speed_estimator_t *estimator,
                             int32_t alpha_q16,
                             int32_t beta_q16)
{
  if(estimator == 0)
  {
    return 0U;
  }
  estimator->alpha_q16 = alpha_q16;
  estimator->beta_q16 = beta_q16;
  estimator->valid = ((alpha_q16 > 0) &&
                      (alpha_q16 <= 65536L) &&
                      (beta_q16 > 0) &&
                      (beta_q16 <= alpha_q16)) ? 1U : 0U;
  speed_estimator_reset(estimator);
  return estimator->valid;
}

void speed_estimator_reset(speed_estimator_t *estimator)
{
  if(estimator == 0)
  {
    return;
  }
  estimator->position_q16 = 0;
  estimator->velocity_q16_counts_per_ms = 0;
  estimator->previous_tick_ms = 0U;
  estimator->speed_mdeg_s = 0;
  estimator->acceleration_mdeg_s2 = 0;
  estimator->initialized = 0U;
}

uint8_t speed_estimator_update(speed_estimator_t *estimator,
                               int32_t position_counts,
                               uint32_t tick_ms)
{
  uint32_t elapsed_ms;
  int64_t measured_position_q16;
  int64_t predicted_position_q16;
  int64_t residual_q16;
  int64_t position_correction_q16;
  int64_t velocity_correction_q16;

  if((estimator == 0) || (estimator->valid == 0U))
  {
    return 0U;
  }
  measured_position_q16 = (int64_t)position_counts *
                          SPEED_ESTIMATOR_Q16;
  if(estimator->initialized == 0U)
  {
    estimator->position_q16 = measured_position_q16;
    estimator->velocity_q16_counts_per_ms = 0;
    estimator->previous_tick_ms = tick_ms;
    estimator->speed_mdeg_s = 0;
  estimator->acceleration_mdeg_s2 = 0;
    estimator->initialized = 1U;
    return 1U;
  }

  elapsed_ms = tick_ms - estimator->previous_tick_ms;
  if(elapsed_ms == 0U)
  {
    return 0U;
  }
  if(elapsed_ms > SPEED_ESTIMATOR_MAX_UPDATE_GAP_MS)
  {
    estimator->position_q16 = measured_position_q16;
    estimator->velocity_q16_counts_per_ms = 0;
    estimator->previous_tick_ms = tick_ms;
    estimator->speed_mdeg_s = 0;
  estimator->acceleration_mdeg_s2 = 0;
    return 1U;
  }

  predicted_position_q16 = estimator->position_q16 +
    estimator->velocity_q16_counts_per_ms * elapsed_ms;
  residual_q16 = measured_position_q16 - predicted_position_q16;
  position_correction_q16 =
    scale_q16(residual_q16,estimator->alpha_q16);
  velocity_correction_q16 =
    scale_q16(residual_q16,estimator->beta_q16) /
    elapsed_ms;
  estimator->position_q16 = predicted_position_q16 +
                            position_correction_q16;
  estimator->velocity_q16_counts_per_ms=bound_velocity(
    estimator->velocity_q16_counts_per_ms+velocity_correction_q16);
  estimator->previous_tick_ms = tick_ms;
  estimator->speed_mdeg_s = speed_estimator_convert_speed(
    estimator->velocity_q16_counts_per_ms);
  return 1U;
}

int32_t speed_estimator_get_mdeg_s(const speed_estimator_t *estimator)
{
  if((estimator == 0) || (estimator->valid == 0U) ||
     (estimator->initialized == 0U))
  {
    return 0;
  }
  return estimator->speed_mdeg_s;
}

uint8_t speed_estimator_update_interval(speed_estimator_t *e,int32_t counts,uint32_t now)
{
  uint32_t dt;
  int64_t measured,predicted,residual,innovation,p=65536,alpha,beta,old_v,accel;
  if(!e || !e->valid)return 0U;
  if(!e->initialized)return speed_estimator_update(e,counts,now);
  dt=now-e->previous_tick_ms;if(!dt)return 0U;
  if(dt>20U) {speed_estimator_reset(e);return speed_estimator_update(e,counts,now);}
  measured=(int64_t)counts*65536;old_v=e->velocity_q16_counts_per_ms;
  predicted=e->position_q16+old_v*dt;residual=measured-predicted;
  innovation=residual>32768?residual-32768:(residual< -32768?residual+32768:0);
  /* p=exp(-2*pi*100Hz*dt), computed in fixed point outside the PWM ISR. */
  for(uint32_t i=0;i<dt;i++)p=p*34963/65536;
  alpha=65536-p*p/65536;beta=(65536-p)*(65536-p)/65536;
  e->position_q16=predicted+scale_q16(innovation,alpha);
  e->velocity_q16_counts_per_ms=bound_velocity(old_v+scale_q16(innovation,beta)/dt);
  e->previous_tick_ms=now;
  e->speed_mdeg_s=speed_estimator_convert_speed(e->velocity_q16_counts_per_ms);
  accel=((e->velocity_q16_counts_per_ms-old_v)*360000LL/16384LL)*1000000LL/
    (65536LL*dt);
  /* Acceleration is diagnostic/identification data, never a raw D term. */
  e->acceleration_mdeg_s2=speed_estimator_clamp_i64(
    (int64_t)e->acceleration_mdeg_s2+(accel-e->acceleration_mdeg_s2)/4);
  return 1U;
}
