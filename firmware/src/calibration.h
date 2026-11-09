#ifndef ARM_CALIBRATION_H
#define ARM_CALIBRATION_H
#include "common.h"
#include "motor_calibration.h"

void Calibration_init(void);
int Calibration_active(void);
int Calibration_ready(void);
int Calibration_stored(void);
int Calibration_direction(void);
float Calibration_angle(uint16_t raw);
const motor_cal_t *Calibration_status(void);
const motor_cal_record_t *Calibration_record(void);
// op: 0 查询；1 基本标定；2 中止；3 保存。电流由主机以 mA 给出。
int Calibration_command(int op, int current_ma, const motor_status_t *feedback, uint32_t now);
void Calibration_process(const motor_status_t *feedback, uint32_t now);
void Calibration_abort(motor_cal_error_t reason);
#endif
