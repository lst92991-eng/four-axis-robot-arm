#include "speed_estimator.h"
#define ENCODER_STATE_COUNTS_PER_REV 16384L
#include <assert.h>
#include <stdio.h>

#define TEST_ALPHA_Q16  4096L
#define TEST_BETA_Q16     64L

static int32_t test_position_for_speed(int32_t speed_mdeg_s,
                                       uint32_t tick_ms)
{
  int64_t numerator = (int64_t)speed_mdeg_s * tick_ms *
                      ENCODER_STATE_COUNTS_PER_REV;
  int64_t denominator = 360000LL * 1000LL;

  if(numerator >= 0)
  {
    numerator += denominator / 2;
  }
  else
  {
    numerator -= denominator / 2;
  }
  return (int32_t)(numerator / denominator);
}

static void test_tracks_low_speed(void)
{
  speed_estimator_t estimator;
  uint32_t tick;
  int32_t speed;

  assert(speed_estimator_init(&estimator,
                              TEST_ALPHA_Q16,
                              TEST_BETA_Q16) == 1U);
  for(tick = 0U; tick <= 3000U; tick++)
  {
    assert(speed_estimator_update(
      &estimator,
      test_position_for_speed(3600, tick),
      tick) == 1U);
  }
  speed = speed_estimator_get_mdeg_s(&estimator);
  assert(speed > 3300);
  assert(speed < 3900);
}

static void test_tracks_two_degrees_per_second(void)
{
  speed_estimator_t estimator;
  uint32_t tick;
  int32_t speed;

  assert(speed_estimator_init(&estimator,
                              TEST_ALPHA_Q16,
                              TEST_BETA_Q16) == 1U);
  for(tick = 0U; tick <= 5000U; tick++)
  {
    assert(speed_estimator_update(
      &estimator,
      test_position_for_speed(2000, tick),
      tick) == 1U);
  }
  speed = speed_estimator_get_mdeg_s(&estimator);
  assert(speed > 1900);
  assert(speed < 2100);
}

static void test_tracks_reverse_and_stop(void)
{
  speed_estimator_t estimator;
  uint32_t tick;
  int32_t position;
  int32_t speed;

  assert(speed_estimator_init(&estimator,
                              TEST_ALPHA_Q16,
                              TEST_BETA_Q16) == 1U);
  for(tick = 0U; tick <= 2000U; tick++)
  {
    position = test_position_for_speed(-5000, tick);
    (void)speed_estimator_update(&estimator, position, tick);
  }
  speed = speed_estimator_get_mdeg_s(&estimator);
  assert(speed < -4500);
  assert(speed > -5500);

  for(; tick <= 3000U; tick++)
  {
    (void)speed_estimator_update(&estimator, position, tick);
  }
  speed = speed_estimator_get_mdeg_s(&estimator);
  assert(speed > -100);
  assert(speed < 100);
}

static void test_rejects_standstill_jitter(void)
{
  speed_estimator_t estimator;
  static const int32_t jitter[4] = {0, 1, 0, -1};
  uint32_t tick;
  int32_t speed;

  assert(speed_estimator_init(&estimator,
                              TEST_ALPHA_Q16,
                              TEST_BETA_Q16) == 1U);
  for(tick = 0U; tick <= 3000U; tick++)
  {
    (void)speed_estimator_update(&estimator,
                                 jitter[tick & 3U],
                                 tick);
  }
  speed = speed_estimator_get_mdeg_s(&estimator);
  assert(speed > -50);
  assert(speed < 50);
}

static void test_rejects_invalid_configuration(void)
{
  speed_estimator_t estimator;

  assert(speed_estimator_init(0, TEST_ALPHA_Q16, TEST_BETA_Q16) == 0U);
  assert(speed_estimator_init(&estimator, 0, TEST_BETA_Q16) == 0U);
  assert(speed_estimator_init(&estimator, TEST_ALPHA_Q16, 0) == 0U);
  assert(speed_estimator_init(&estimator, TEST_ALPHA_Q16,
                              TEST_ALPHA_Q16 + 1) == 0U);
  assert(speed_estimator_update(&estimator, 0, 0U) == 0U);
}

int main(void)
{
  test_tracks_low_speed();
  test_tracks_two_degrees_per_second();
  test_tracks_reverse_and_stop();
  test_rejects_standstill_jitter();
  test_rejects_invalid_configuration();
  puts("speed_estimator_test: PASS");
  return 0;
}
