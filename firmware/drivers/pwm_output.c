/** @file pwm_output.c
 * @brief 配置TMR1两相全桥PWM，协调帧锁存和ADC触发。 */
#include "at32m412_416_conf.h"

#include "pwm_output.h"

//---------帧结构与运行数据区---------
typedef struct
{
  uint16_t phase_a_leg_1_duty;
  uint16_t phase_a_leg_2_duty;
  uint16_t phase_b_leg_1_duty;
  uint16_t phase_b_leg_2_duty;
  uint16_t adc_trigger_ticks;
} pwm_frame_t;

#define LOW_A1_PIN GPIO_PINS_7
#define LOW_A2_PIN GPIO_PINS_0
#define LOW_B1_PIN GPIO_PINS_1
#define LOW_B2_PIN GPIO_PINS_4

static uint8_t staged_sample_valid;
enum
{
  PWM_OFF,
  PWM_SAMPLING,
  PWM_DRIVE
};
static volatile uint8_t pwm_mode;
static volatile uint8_t complementary_mode;
static volatile int16_t requested_a_ticks;
static volatile int16_t requested_b_ticks;

//---------内部函数声明区---------
static void init_gpio(void);
static void set_low_side_mode(uint8_t complementary);
static void prepare_zero_frame(void);
static void build_frame(int32_t phase_a_command, int32_t phase_b_command, pwm_frame_t *frame);

//---------初始化区---------
void pwm_output_init(void)
{
  tmr_output_config_type output_config;
  tmr_brkdt_config_type brkdt_config;
  init_gpio();
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
  /* TMR1不分频，180MHz / 7200得到25kHz。 */
  tmr_base_init(TMR1, CONTROL_PWM_TIMER_PERIOD, 0);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
  tmr_period_buffer_enable(TMR1, TRUE);
  tmr_output_default_para_init(&output_config);
  output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  output_config.oc_output_state = TRUE;
  output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  output_config.oc_idle_state = FALSE;
  output_config.occ_output_state = TRUE;
  /* CHxC内部为互补波形；配置低有效极性后输出到LIN，再由XJNG2103反相驱动LO。 */
  output_config.occ_polarity = TMR_OUTPUT_ACTIVE_LOW;
  output_config.occ_idle_state = FALSE;
  for (unsigned channel = TMR_SELECT_CHANNEL_1; channel <= TMR_SELECT_CHANNEL_4; channel += 2)
    tmr_output_channel_config(TMR1, (tmr_channel_select_type)channel, &output_config);
  for (unsigned channel = TMR_SELECT_CHANNEL_1; channel <= TMR_SELECT_CHANNEL_4; channel += 2)
    tmr_output_channel_buffer_enable(TMR1, (tmr_channel_select_type)channel, TRUE);
  tmr_brkdt_default_para_init(&brkdt_config);
  brkdt_config.deadtime = (uint8_t)CONTROL_PWM_DEADTIME_TICKS;
  tmr_brkdt_config(TMR1, &brkdt_config);
  output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
  tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_5, &output_config);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_5, CONTROL_ADC_BLANKING_TICKS);
  tmr_output_channel_buffer_enable(TMR1, TMR_SELECT_CHANNEL_5, TRUE);
  tmr_primary_mode2_select(TMR1, TMR_PRIMARY_SEL_C5ORAW);
  tmr_trgout2_enable(TMR1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
  pwm_output_disable();
}

static void init_gpio(void)
{
  gpio_init_type gpio_init_struct;
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
  gpio_init(GPIOA, &gpio_init_struct);
  for (unsigned pin = 8; pin <= 11; ++pin)
    gpio_pin_mux_config(GPIOA, (gpio_pins_source_type)pin, GPIO_MUX_1);

  /* XJNG2103的LIN为低有效；完整互补帧锁存前，低侧请求引脚保持高电平。 */
  set_low_side_mode(0);
}

//---------输出控制区---------
void pwm_output_disable(void)
{
  tmr_output_enable(TMR1, FALSE);
  pwm_mode = PWM_OFF;
  prepare_zero_frame();
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG);
  staged_sample_valid = 0U;
}

void pwm_output_enable_sampling(void)
{
  tmr_output_enable(TMR1, FALSE);
  prepare_zero_frame();
  //四路功率比较值均为零，CH5保持ADC触发
  staged_sample_valid = 1U;
  //有效零帧覆盖延后到达的DMA回调
  pwm_mode = PWM_SAMPLING;
  tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG);
  tmr_output_enable(TMR1, TRUE);
}

void pwm_output_set_signed_phase_duty(int16_t phase_a, int16_t phase_b)
{
  requested_a_ticks = phase_a;
  requested_b_ticks = phase_b;
  pwm_mode = PWM_DRIVE;
}

//---------采样同步区---------
uint8_t pwm_output_pwm_sample_begin(void)
{
  return staged_sample_valid;
}

void pwm_output_pwm_update_service(void)
{
  int16_t requested_phase_a;
  int16_t requested_phase_b;
  pwm_frame_t frame;

  if (pwm_mode != PWM_DRIVE)
  {
    return;
  }
  requested_phase_a = requested_a_ticks;
  requested_phase_b = requested_b_ticks;
  build_frame(requested_phase_a, requested_phase_b, &frame);
  if (complementary_mode == 0U)
  {
    tmr_output_enable(TMR1, FALSE);
  }
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, frame.phase_a_leg_1_duty);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, frame.phase_a_leg_2_duty);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, frame.phase_b_leg_1_duty);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, frame.phase_b_leg_2_duty);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_5, frame.adc_trigger_ticks);
  if (complementary_mode == 0U)
  {
    tmr_event_sw_trigger(TMR1, TMR_OVERFLOW_SWTRIG);
    set_low_side_mode(1);
  }
  tmr_output_enable(TMR1, TRUE);

  //完整帧交付后允许下一次ADC采样
  staged_sample_valid = 1U;
}

//---------PWM构帧区---------
static int32_t clamp_command(int32_t value, int32_t lower, int32_t upper)
{
  if (value < lower)
  {
    return lower;
  }
  if (value > upper)
  {
    return upper;
  }
  return value;
}

static void split_phase(int32_t command, uint16_t midpoint, uint16_t *leg_1, uint16_t *leg_2)
{
  /* 输入已限幅，取负值不会越过int32范围。 */
  uint32_t magnitude = (uint32_t)((command < 0) ? -command : command);
  uint32_t high_offset = (magnitude + 1U) / 2U;
  uint32_t low_offset = magnitude / 2U;
  uint16_t high_duty = (uint16_t)(midpoint + high_offset);
  uint16_t low_duty = (uint16_t)(midpoint - low_offset);

  if (command >= 0)
  {
    *leg_1 = high_duty;
    *leg_2 = low_duty;
  }
  else
  {
    *leg_1 = low_duty;
    *leg_2 = high_duty;
  }
}

//调用方提供局部帧，限幅与编译期几何约束保证触发点有效
static void build_frame(int32_t phase_a_command, int32_t phase_b_command, pwm_frame_t *frame)
{
  const int32_t maximum_command = (int32_t)CONTROL_PWM_SAMPLE_SAFE_COMMAND;
  const uint16_t midpoint = (uint16_t)((CONTROL_PWM_TIMER_PERIOD + 1U) / 2U);
  uint16_t sampled_leg_peak;
  uint32_t trigger;
  int32_t applied_a;
  int32_t applied_b;

  applied_a = clamp_command(phase_a_command, -maximum_command, maximum_command);
  applied_b = clamp_command(phase_b_command, -maximum_command, maximum_command);
  split_phase(applied_a, midpoint, &frame->phase_a_leg_1_duty, &frame->phase_a_leg_2_duty);
  split_phase(applied_b, midpoint, &frame->phase_b_leg_1_duty, &frame->phase_b_leg_2_duty);

  /* 分流电阻位于A1和B2低侧；相应高侧脉冲结束并经过消隐后采样。
   * 互补同步PWM在该窗口内使两路采样低侧MOS导通。 */
  sampled_leg_peak = frame->phase_a_leg_1_duty;
  if (frame->phase_b_leg_2_duty > sampled_leg_peak)
    sampled_leg_peak = frame->phase_b_leg_2_duty;
  trigger = (uint32_t)sampled_leg_peak + CONTROL_ADC_BLANKING_TICKS;
  // 六轴已验证的运行/标定入口使用固定的周期后段采样。
  // 最大占空比已按该时刻反推限幅，因此不早于任一低侧消隐边界。
  frame->adc_trigger_ticks = (uint16_t)(trigger > CONTROL_RUNTIME_ADC_TRIGGER_TICKS ?
                                      trigger : CONTROL_RUNTIME_ADC_TRIGGER_TICKS);
}

/* 关闭桥臂后准备零输出帧，供失能和仅采样状态共用。 */
static void prepare_zero_frame(void)
{
  set_low_side_mode(0);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 0U);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 0U);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, 0U);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_4, 0U);
  requested_a_ticks = 0;
  requested_b_ticks = 0;
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_5, CONTROL_ADC_BLANKING_TICKS);
}

//---------引脚切换区---------
/* 禁止功率输出时先将低侧请求置高，再切换为 GPIO。 */
static void set_low_side_mode(uint8_t complementary)
{
  gpio_init_type gpio_init_struct;

  if (!complementary)
  {
    gpio_bits_set(GPIOA, LOW_A1_PIN);
    gpio_bits_set(GPIOB, LOW_A2_PIN | LOW_B1_PIN | LOW_B2_PIN);
  }
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = complementary ? GPIO_MODE_MUX : GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init_struct.gpio_pins = LOW_A1_PIN;
  gpio_init(GPIOA, &gpio_init_struct);
  gpio_init_struct.gpio_pins = LOW_A2_PIN | LOW_B1_PIN | LOW_B2_PIN;
  gpio_init(GPIOB, &gpio_init_struct);
  if (complementary)
  {
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_1);
  }
  complementary_mode = complementary;
}
