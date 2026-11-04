/** @file mt6816.c
 * @brief 通过SPI2读取MT6816角度并检查状态位与偶校验。 */
#include "mt6816.h"
#include "at32m412_416_conf.h"

//---------通信参数区---------
#define MT6816_READ_ANGLE_HIGH ((uint16_t)0x8300U)
#define MT6816_READ_ANGLE_LOW ((uint16_t)0x8400U)
//TMR7每个计数为0.5us；多等1个计数覆盖读取时的计数相位
#define MT6816_TRANSFER_TIMEOUT 41U

//---------初始化区---------
void mt6816_init(void)
{
  gpio_init_type gpio_init_struct;
  spi_init_type spi_init_struct;

  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init(GPIOB, &gpio_init_struct);
  gpio_bits_set(GPIOB, GPIO_PINS_12);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
  gpio_init(GPIOB, &gpio_init_struct);
  for (unsigned pin = 13; pin <= 15; ++pin)
    gpio_pin_mux_config(GPIOB, (gpio_pins_source_type)pin, GPIO_MUX_5);

  spi_default_para_init(&spi_init_struct);
  spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
  /* SPI2由180MHz时钟32分频，传输频率为5.625MHz。 */
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_32;
  spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
  spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
  spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
  spi_init(SPI2, &spi_init_struct);
  spi_software_cs_internal_level_set(SPI2, SPI_SWCS_INTERNAL_LEVEL_HIGHT);
  spi_enable(SPI2, TRUE);
}

//---------SPI通信区---------
static void cs_delay(void)
{
  //等待至少0.5us，16位差值处理定时器回绕
  uint16_t start = (uint16_t)TMR7->cval;
  while ((uint16_t)(TMR7->cval - start) < 2U)
  {
    __NOP();
  }
}

static uint16_t spi_transfer16(uint16_t tx_data, confirm_state *ok)
{
  uint16_t started = (uint16_t)TMR7->cval;

  while (spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET)
  {
    if ((uint16_t)(TMR7->cval - started) >= MT6816_TRANSFER_TIMEOUT)
    {
      *ok = FALSE;
      return 0U;
    }
  }

  spi_i2s_data_transmit(SPI2, tx_data);
  started = (uint16_t)TMR7->cval;
  while (spi_i2s_flag_get(SPI2, SPI_I2S_RDBF_FLAG) == RESET)
  {
    if ((uint16_t)(TMR7->cval - started) >= MT6816_TRANSFER_TIMEOUT)
    {
      *ok = FALSE;
      return 0U;
    }
  }

  return spi_i2s_data_receive(SPI2);
}

//---------数据校验区---------
static confirm_state parity_ok(uint16_t data)
{
  data ^= (uint16_t)(data >> 8);
  data ^= (uint16_t)(data >> 4);
  data ^= (uint16_t)(data >> 2);
  data ^= (uint16_t)(data >> 1);
  return (((~data) & 1U) != 0U) ? TRUE : FALSE;
}

//---------角度读取区---------
uint8_t mt6816_read_angle(uint16_t *raw_angle)
{
  uint16_t angle_data;
  confirm_state ok = TRUE;

  gpio_bits_reset(GPIOB, GPIO_PINS_12);
  cs_delay();
  angle_data = spi_transfer16(MT6816_READ_ANGLE_HIGH, &ok);
  gpio_bits_set(GPIOB, GPIO_PINS_12);
  cs_delay();
  if (ok == FALSE)
  {
    return FALSE;
  }

  gpio_bits_reset(GPIOB, GPIO_PINS_12);
  cs_delay();
  angle_data = (uint16_t)(angle_data << 8);
  angle_data |= (uint16_t)(spi_transfer16(MT6816_READ_ANGLE_LOW, &ok) & 0x00FFU);
  gpio_bits_set(GPIOB, GPIO_PINS_12);
  cs_delay();
  if ((ok == FALSE) || ((angle_data & 0x0002U) != 0U) || (parity_ok(angle_data) == FALSE))
  {
    return FALSE;
  }

  *raw_angle = (uint16_t)(angle_data >> 2);
  return TRUE;
}
