#include "CAN.h"
#include "motion.h"
#include "motor_control.h"
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
  frame[13] = motor->encoder_valid;
  CAN_send(0x300u + joint_config->can_node, (const uint8_t *)frame);
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

