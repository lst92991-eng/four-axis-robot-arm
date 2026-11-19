#ifndef SPEED_ESTIMATOR_H
#define SPEED_ESTIMATOR_H

#include <stdint.h>

typedef struct
{
  int64_t position_q16;
  int64_t velocity_q16_counts_per_ms;
  uint32_t previous_tick_ms;
  int32_t speed_mdeg_s;
  int32_t acceleration_mdeg_s2;
  int32_t alpha_q16;
  int32_t beta_q16;
  uint8_t initialized;
  uint8_t valid;
} speed_estimator_t;

uint8_t speed_estimator_init(speed_estimator_t *estimator,
                             int32_t alpha_q16,
                             int32_t beta_q16);
void speed_estimator_reset(speed_estimator_t *estimator);
uint8_t speed_estimator_update(speed_estimator_t *estimator,
                               int32_t position_counts,
                               uint32_t tick_ms);
/* Quantization-interval observer: a measurement denotes a +/-0.5-count bin.
 * Critically damped 100Hz poles, timestamp-scaled; no invented sub-count data. */
uint8_t speed_estimator_update_interval(speed_estimator_t *estimator,
  int32_t position_counts,uint32_t tick_ms);
int32_t speed_estimator_get_mdeg_s(const speed_estimator_t *estimator);

#endif
