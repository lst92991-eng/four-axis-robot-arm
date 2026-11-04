#include "CAN.h"
#include "motion.h"
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
  frame[10] = motor->trace_bus;
  frame[11] = motor->power;
  frame[12] = motor->fault;
  frame[13] = motor->encoder_valid;
  CAN_send(0x300u + CONFIG_NODE, (const uint8_t *)frame);
}

