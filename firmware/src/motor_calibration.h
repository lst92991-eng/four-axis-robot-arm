#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H
#include <stdint.h>
#define MOTOR_CAL_BINS 256U
#define MOTOR_CAL_MAGIC 0x53464341UL
#define MOTOR_CAL_VERSION 1U
typedef enum {
  MOTOR_CAL_IDLE=0, MOTOR_CAL_PRECHECK=1, MOTOR_CAL_ALIGN=2,
  MOTOR_CAL_DIRECTION=3, MOTOR_CAL_RETURN=4, MOTOR_CAL_FORWARD=5,
  MOTOR_CAL_REVERSE=6, MOTOR_CAL_DONE=7, MOTOR_CAL_FAILED=8,
  MOTOR_CAL_ABORTED=9, MOTOR_CAL_STATIC_OK=10
} motor_cal_state_t;
typedef enum {
  MOTOR_CAL_OK=0, MOTOR_CAL_SENSOR=1, MOTOR_CAL_CURRENT=2,
  MOTOR_CAL_TIMEOUT=3, MOTOR_CAL_TRAVEL=4, MOTOR_CAL_UNSTABLE=5,
  MOTOR_CAL_NO_MOTION=6, MOTOR_CAL_HYSTERESIS=7, MOTOR_CAL_MAP=8,
  MOTOR_CAL_LINK=9, MOTOR_CAL_USER=10, MOTOR_CAL_POWER=11
} motor_cal_error_t;
typedef struct {
  uint32_t magic, version, sequence, flags;
  uint16_t pole_pairs, raw_zero;
  int16_t direction;
  uint16_t hysteresis_counts;
  int16_t knots[MOTOR_CAL_BINS+1U];
  uint16_t reserved;
  uint32_t crc;
} motor_cal_record_t;
typedef struct {
  uint32_t now_ms;
  int32_t counts, phase_a, phase_b, id_actual;
  uint32_t bus_mv;
  uint8_t sensors_ready;
  /* Optional prior-calibration phase for a continuous re-alignment entry. */
  uint8_t alignment_hint_valid;
  uint32_t alignment_angle_mdeg;
} motor_cal_input_t;
typedef struct {
  motor_cal_state_t state;
  motor_cal_error_t error;
  motor_cal_record_t candidate;
  uint32_t started, entered, last_sample, speed_tick;
  int32_t start_counts, zero_counts, speed_counts, sample_min, sample_max;
  int64_t sample_sum, current_sum;
  uint16_t samples, index, current, progress;
  int32_t drive_angle, drive_target, drive_reference;
  uint8_t full, drive, releasing, pending_error;
  uint32_t release_started, release_settled;
} motor_cal_t;
void motor_cal_init(motor_cal_t *cal);
uint8_t motor_cal_active(const motor_cal_t *cal);
uint8_t motor_cal_start(motor_cal_t *cal, uint16_t poles, uint16_t current,
                        uint8_t full, const motor_cal_input_t *input);
void motor_cal_abort(motor_cal_t *cal, motor_cal_error_t reason);
void motor_cal_step(motor_cal_t *cal, const motor_cal_input_t *input);
uint32_t motor_cal_crc(const uint8_t *bytes, uint32_t size);
void motor_cal_seal(motor_cal_record_t *record, uint32_t sequence);
uint8_t motor_cal_record_valid(const motor_cal_record_t *record);
uint16_t motor_cal_corrected_raw(const motor_cal_record_t *record, uint16_t raw);
uint32_t motor_cal_angle(const motor_cal_record_t *record, uint16_t raw);
#endif
