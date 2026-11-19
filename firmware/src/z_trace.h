#include "CAN.h"
#include "motion.h"
#include "motor_control.h"
#include <math.h>
#include <string.h>

//记录电流环给定与估算所需量，不把堵转设成零点
static void z_trace(const motor_status_t *motor, uint32_t now)
{
  static uint32_t last_ms;
  if (now - last_ms < 20)
    return;
  last_ms = now;
  uint32_t frame[16] = {0x3147545au, now};
  memcpy(&frame[2], &motor->encoder_counts, 8);
  memcpy(&frame[4], &motor->velocity_counts_s, 4);
  frame[5] = (uint32_t)(int32_t)(Joint.status.position * 1000);
  frame[6] = (uint32_t)(int32_t)(Joint.status.requested * 1000);
  frame[7] = (uint32_t)motor->trace_iq_ref;
  frame[8] = (uint32_t)motor->trace_iq;
  frame[9] = (uint32_t)motor->trace_vq;
  frame[10] = motor->bus_mv;
  frame[11] = motor->power;
  frame[12] = motor->fault ? motor->fault : Joint.status.fault;
  //低两位保留原标志，位2至15以4毫度为单位记录跟随误差，高半字仍为Vd。
  if (motor->encoder_valid)
  {
    double error_units = motor->power ? (Joint.ref_pos - Joint.status.position) * 250 : 0;
    int32_t following = (int32_t)lround(fmax(-8192, fmin(8191, error_units)));
    frame[13] = 1u | ((uint32_t)motor->trace_voltage_clipped << 1) |
                (((uint32_t)following & 0x3fffu) << 2) |
                ((uint32_t)(uint16_t)motor->trace_vd << 16);
  }
  frame[14] = motor->bus_trip_mv;
  frame[15] = motor->bus_peak_mv;
  CAN_send(0x300u + joint_config->can_node, (const uint8_t *)frame);

  //独立帧保存两相的原始计数和本次使能峰值，保留原诊断帧格式。
  uint32_t phases[16] = {0x31434850u, now,
    (uint32_t)motor->phase_a_counts, (uint32_t)motor->phase_b_counts,
    motor->phase_a_peak_counts, motor->phase_b_peak_counts, motor->power};
  CAN_send(0x300u + joint_config->can_node, (const uint8_t *)phases);

  int32_t evidence[12];
  Motor_readCurrentFault(evidence);
  static uint32_t last_evidence_ms;
  if(!evidence[0] && now-last_evidence_ms<1000)return;
  last_evidence_ms=now;
  memset(frame,0,sizeof(frame));
  frame[0]=0x31464349u;
  memcpy(&frame[1],evidence,sizeof(evidence));
  CAN_send(0x380u + joint_config->can_node,(const uint8_t *)frame);
}

