/** @file current_sense.c
 * @brief 通过双ADC与DMA同步采集相电流和母线电压。 */
#include "current_limits.h"
#include "at32m412_416_conf.h"
#include "control_timing.h"
#include "current_sense.h"

//---------采样数据区---------
#define CURRENT_SENSE_BUS_RATIO 16UL
/* ADC2的第二序位复用PA1；采样时间配置作用于ADC通道。
 * 两个ADC的两个序位统一获取时间，保持双ADC的DMA配对关系。 */

static __IO uint32_t dma_samples[2];
static __IO uint32_t latest_phase_pair;
static __IO uint16_t bus_raw;
static __IO uint32_t sample_sequence;
static uint16_t offset_a_counts;
static uint16_t offset_b_counts;
static volatile uint8_t control_ready;

/* ADC零偏完成后，DMA中断直接进入板级控制。 */
extern void Motor_control_callback(int32_t phase_a, int32_t phase_b);

//---------内部函数声明区---------
static void init_gpio(void);
static void init_dma(void);
static void init_adc(void);

//---------初始化区---------
void current_sense_init(void)
{
  control_ready = 0;
  /* 初始化期间暂停TMR1触发；两路ADC完成使能与校准后，依次启用DMA和定时器。 */
  tmr_counter_enable(TMR1, FALSE);
  init_gpio();
  init_dma();
  init_adc();
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  tmr_counter_enable(TMR1, TRUE);
}

static void init_gpio(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4;
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init(GPIOA, &gpio_init_struct);
}

static void init_dma(void)
{
  dma_init_type dma_init_struct;

  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  dma_reset(DMA1_CHANNEL1);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = 2;
  dma_init_struct.memory_base_addr = (uint32_t)dma_samples;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)&(ADCCOM->codt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  dma_init_struct.loop_mode_enable = TRUE;
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  dmamux_enable(DMA1, TRUE);
  dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_ADC1);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  //DMA和SysTick使用相同逻辑优先级，控制中断顺序执行
  NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void init_adc(void)
{
  adc_common_config_type adc_common_struct;
  adc_base_config_type adc_base_struct;

  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_ADC2_PERIPH_CLOCK, TRUE);
  adc_reset();

  adc_common_default_para_init(&adc_common_struct);
  adc_common_struct.combine_mode = ADC_ORDINARY_SMLT_ONLY_ONESLAVE_MODE;
  adc_common_struct.div = ADC_HCLK_DIV_6;
  adc_common_struct.common_dma_mode = ADC_COMMON_DMAMODE_2;
  adc_common_struct.common_dma_request_repeat_state = TRUE;
  adc_common_config(&adc_common_struct);

  adc_base_default_para_init(&adc_base_struct);
  adc_base_struct.sequence_mode = TRUE;
  adc_base_struct.ordinary_channel_length = 2U;
  adc_base_config(ADC1, &adc_base_struct);
  adc_resolution_set(ADC1, ADC_RESOLUTION_12B);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_1_5);
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 2, ADC_SAMPLETIME_1_5);
  adc_ordinary_conversion_trigger_set(ADC1, ADC_ORDINARY_TRIG_TMR1TRGOUT2,
                                      ADC_ORDINARY_TRIG_EDGE_RISING);
  adc_dma_mode_enable(ADC1, FALSE);
  adc_dma_request_repeat_enable(ADC1, FALSE);

  adc_base_config(ADC2, &adc_base_struct);
  adc_resolution_set(ADC2, ADC_RESOLUTION_12B);
  adc_ordinary_channel_set(ADC2, ADC_CHANNEL_1, 1, ADC_SAMPLETIME_1_5);
  /* 双ADC同步模式使用等长序列；ADC2第二序位重复PA1，配对ADC1的母线采样。 */
  adc_ordinary_channel_set(ADC2, ADC_CHANNEL_1, 2, ADC_SAMPLETIME_1_5);
  adc_ordinary_conversion_trigger_set(ADC2, ADC_ORDINARY_TRIG_TMR1TRGOUT2,
                                      ADC_ORDINARY_TRIG_EDGE_NONE);
  adc_dma_mode_enable(ADC2, FALSE);
  adc_dma_request_repeat_enable(ADC2, FALSE);

  adc_enable(ADC1, TRUE);
  adc_enable(ADC2, TRUE);
  adc_type *adcs[] = {ADC1, ADC2};
  for (unsigned i = 0; i < 2; ++i)
  {
    while (adc_flag_get(adcs[i], ADC_RDY_FLAG) == RESET)
    {
    }
  }

  for (unsigned i = 0; i < 2; ++i)
  {
    adc_calibration_init(adcs[i]);
    while (adc_calibration_init_status_get(adcs[i]))
    {
    }
    adc_calibration_start(adcs[i]);
    while (adc_calibration_status_get(adcs[i]))
    {
    }
  }
}

//---------采样控制区---------
void current_sense_calibrate_offsets(void)
{
  uint32_t a_sum = 0;
  uint32_t b_sum = 0;
  uint16_t sample_index;
  uint32_t sequence_start;

  /* 前置状态为四路功率比较值均为零；临时开启TMR1输出，使CH5持续触发偏置采样。 */
  tmr_output_enable(TMR1, TRUE);
  sequence_start = sample_sequence;
  for (sample_index = 0; sample_index < 128; sample_index++)
  {
    uint32_t phase_pair;
    while (sample_sequence == sequence_start)
    {
    }
    //先取样本再记序号，中断插入时允许跳过样本，避免重复累计
    phase_pair = latest_phase_pair;
    sequence_start = sample_sequence;
    a_sum += (uint16_t)(phase_pair & 0xffffU);
    b_sum += (uint16_t)(phase_pair >> 16);
  }

  tmr_output_enable(TMR1, FALSE);

  offset_a_counts = (uint16_t)(a_sum / 128);
  offset_b_counts = (uint16_t)(b_sum / 128);
}

void current_sense_start_control(void)
{
  /* 单字节标志在 Cortex-M4 上原子写入，置位后中断才进入控制。 */
  control_ready = 1;
}

//---------电压反馈区---------
uint32_t current_sense_get_bus_mv(void)
{
  return ((uint32_t)bus_raw * CURRENT_ADC_VREF_MV * CURRENT_SENSE_BUS_RATIO) /
         CURRENT_ADC_FULL_SCALE;
}

//---------中断回调区---------
void DMA1_Channel1_IRQHandler(void)
{
  uint32_t phase_pair;
  uint32_t bus_pair;
  int32_t a;
  int32_t b;

  if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) == RESET)
  {
    return;
  }
  dma_flag_clear(DMA1_FDT1_FLAG);
  phase_pair = dma_samples[0];
  bus_pair = dma_samples[1];
  latest_phase_pair = phase_pair;
  bus_raw = (uint16_t)(bus_pair & 0xFFFFU);
  /* 发布完成序号，供启动时收集不重复的零偏样本。 */
  sample_sequence++;
  /* A相采样反相，B相采样同相。 */
  a = (int32_t)offset_a_counts - (int32_t)(phase_pair & 0xFFFFU);
  b = (int32_t)(phase_pair >> 16) - (int32_t)offset_b_counts;
  if (control_ready)
  {
    Motor_control_callback(a, b);
  }
}
