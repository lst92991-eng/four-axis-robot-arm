/**
 * @file CAN.c
 * @brief 配置AT32 CAN1并搬运标准ID的64字节CAN-FD帧。
 */
#include "CAN.h"
#include "at32m412_416_conf.h"
#include <string.h>

//---------初始化区---------
/* 标准 ID、64 字节 CAN-FD 帧 */
int CAN_init(void)
{
  if (crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET)
    return 0;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
  crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_HEXT);
  gpio_init_type gpio;
  gpio_default_para_init(&gpio);
  gpio.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio.gpio_mode = GPIO_MODE_MUX;
  gpio.gpio_pins = GPIO_PINS_2 | GPIO_PINS_3;
  gpio.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_9);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE3, GPIO_MUX_9);
  can_software_reset(CAN1, TRUE);
  can_bittime_type timing;
  can_bittime_default_para_init(&timing);
  timing.bittime_div = 1;
  timing.ac_bts1_size = 18;
  timing.ac_bts2_size = 6;
  timing.ac_rsaw_size = 6;
  timing.fd_bts1_size = 4;
  timing.fd_bts2_size = 2;
  timing.fd_rsaw_size = 2;
  timing.fd_ssp_offset = 5;
  can_bittime_set(CAN1, &timing);
  can_fd_iso_mode_enable(CAN1, TRUE);
  can_stb_transmit_mode_set(CAN1, CAN_STB_TRANSMIT_BY_ID);
  can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_LOSE);
  can_receive_all_enable(CAN1, TRUE);
  can_software_reset(CAN1, FALSE);
  can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_UNLIMIT);
  can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_UNLIMIT);
  can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
  return 1;
}

//---------接收区---------
int CAN_receive(uint16_t *identifier, uint8_t data[64])
{
  /* 立即检查接收缓冲：0表示暂无帧，-1表示该帧格式不支持，1表示读取成功。 */
  can_rxbuf_type frame;
  if (can_rxbuf_read(CAN1, &frame) != SUCCESS)
    return 0;
  if (frame.id_type != CAN_ID_STANDARD || frame.fd_format != CAN_FORMAT_FD ||
      frame.frame_type != CAN_FRAME_DATA || frame.data_length != CAN_DLC_BYTES_64)
    return -1;
  *identifier = (uint16_t)frame.id;
  memcpy(data, frame.data, 64);
  return 1;
}

//---------发送区---------
int CAN_send(uint16_t identifier, const uint8_t data[64])
{
  /* 返回1表示帧已写入硬件发送队列；总线传输随后由CAN控制器执行。 */
  if (can_stb_status_get(CAN1) == CAN_STB_STATUS_FULL)
    return 0;
  can_txbuf_type frame = {0};
  frame.id = identifier;
  frame.id_type = CAN_ID_STANDARD;
  frame.frame_type = CAN_FRAME_DATA;
  frame.data_length = CAN_DLC_BYTES_64;
  frame.fd_format = CAN_FORMAT_FD;
  frame.fd_rate_switch = CAN_BRS_ON;
  memcpy(frame.data, data, 64);
  if (can_txbuf_write(CAN1, CAN_TXBUF_STB, &frame) != SUCCESS)
    return 0;
  (void)can_txbuf_transmit(CAN1, CAN_TRANSMIT_STB_ALL);
  return 1;
}
