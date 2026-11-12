//程序入口：前台组织运动和通信，ADC中断执行三环
#include "common.h"
#include "motion.h"
#include "motor_control.h"
#include "communication_CAN.h"
#include "at32m412_416_conf.h"
#include "current_sense.h"
#include "pwm_output.h"
#include "mt6816.h"
#include "calibration.h"
#include "z_trace.h"

const JointConfig *joint_config;
volatile uint32_t system_ms;
//上一次运动更新时刻，毫秒
static uint32_t last_control_ms;

static void Init_hardware(void);
static void system_clock_config(void);

//---------初始化区---------
static void Init_hardware(void)
{
  //配置系统时钟
  system_clock_config();
  //TMR7以2MHz自由计数，供SPI读取使用0.5us计时单位
  crm_periph_clock_enable(CRM_TMR7_PERIPH_CLOCK, TRUE);
  tmr_base_init(TMR7, 65535, system_core_clock / 2000000U - 1U);
  tmr_counter_enable(TMR7, TRUE);
  //TMR6以1MHz计时，主循环超过5ms未刷新时锁存溢出标志
  crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
  tmr_base_init(TMR6, 5000, system_core_clock / 1000000U - 1U);
  tmr_flag_clear(TMR6, TMR_OVF_FLAG);
  tmr_counter_enable(TMR6, TRUE);
  //配置1ms系统滴答中断
  SysTick_Config(system_core_clock / 1000U);
  //设置系统滴答中断优先级
  NVIC_SetPriority(SysTick_IRQn, 2);
  //初始化电机控制参数
  Motor_initialize();
  //初始化PWM输出和ADC触发
  pwm_output_init();
  //初始化电流与母线电压采样
  current_sense_init();
  //功率输出关闭时校准两相电流零偏
  current_sense_calibrate_offsets();
  //初始化磁编码器通信接口
  mt6816_init();
  //启动持续采样，保持功率输出关闭
  pwm_output_enable_sampling();
  //采样帧准备好后允许中断进入电机控制
  current_sense_start_control();
}

//---------主循环区---------
int main(void)
{
  joint_config = JointConfig_fromImage((const uint32_t *)JOINT_APP_METADATA,
                                      (const uint32_t *)JOINT_APP_ADDRESS);
  if (!joint_config)
  {
    // 引导已关闭功率桥。目标记录无效时不启动控制和 CAN。
    while (1) {}
  }
  //初始化硬件和通信协议
  Init_hardware();
  CAN_protocol_setup();

  while (1)
  {
    //读取同一次位置、速度和故障反馈
    motor_status_t motor;
    uint32_t now = Motor_readFeedback(&motor);
    if (motor.fault)
      Motion_trip(motor.fault);

    //每个系统毫秒更新一次运动，通信在每轮循环处理
    uint32_t elapsed_ms = now - last_control_ms;
    if (elapsed_ms)
    {
      last_control_ms = now;
      Calibration_process(&motor, now);
      //机械参考已建立时检查反馈是否有效
      if (Joint.status.referenced && !motor.encoder_valid)
        Motion_trip(FAULT_SENSOR);
      //将编码器速度换算为输出轴速度
      float velocity = joint_config->ratio > 0 ? motor.velocity_counts_s * 360 * joint_config->joint_direction /
                                             (16384 * joint_config->ratio) : 0;
      //更新轨迹并检查通信时限
      Motion_update(motor.encoder_counts, elapsed_ms, velocity);
      CAN_protocol_watchdog(now);
      //将参考交给中断，交付失败时撤销机械参考
      if (!Motor_applyReference())
        Motion_trip(FAULT_CONTROL);
    }
    //接收命令并回复，命令从下一轮运动更新开始生效
    CAN_protocol_process(&motor, now);
    if (joint_config->can_node == 1)
      z_trace(&motor, now);
  }
}

//---------时钟配置区---------
/* 系统时钟配置移自at32m412_416_clock.c，保留其厂商许可。 */
/**
  **************************************************************************
  * @file     at32m412_416_clock.c
  * @brief    system clock config program
  **************************************************************************
  *
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* HEXT 12MHz经PLL 60/1/4产生180MHz；AHB/APB1/APB2为180MHz，APB3为45MHz。 */
static void system_clock_config(void)
{
  //将时钟控制寄存器恢复为默认配置
  crm_reset();
  //设置Flash访问等待周期为5，适应高速运行
  flash_psr_set(FLASH_WAIT_CYCLE_5);
  //开启电源控制模块时钟
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
  //设置内部LDO输出电压为1.3V
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
  //开启外部高速晶振
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  //等待外部高速晶振稳定
  while (crm_hext_stable_wait() == ERROR)
  {
  }
  //配置PLL：12MHz × 60 ÷ 1 ÷ 4 = 180MHz
  crm_pll_config(CRM_PLL_SOURCE_HEXT, 60, 1, CRM_PLL_FR_4);
  //开启PLL
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  //等待PLL锁定
  while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }
  //配置AHB不分频，时钟切换后为180MHz
  crm_ahb_div_set(CRM_AHB_DIV_1);
  //配置APB3四分频，时钟切换后为45MHz
  crm_apb3_div_set(CRM_APB3_DIV_4);
  //配置APB2不分频，时钟切换后为180MHz
  crm_apb2_div_set(CRM_APB2_DIV_1);
  //配置APB1不分频，时钟切换后为180MHz
  crm_apb1_div_set(CRM_APB1_DIV_1);
  //开启时钟切换自动步进模式
  crm_auto_step_mode_enable(TRUE);
  //选择PLL作为系统时钟源
  crm_sysclk_switch(CRM_SCLK_PLL);
  //等待系统时钟切换到PLL
  while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }
  //关闭时钟切换自动步进模式
  crm_auto_step_mode_enable(FALSE);
  //更新system_core_clock，记录当前内核时钟频率
  system_core_clock_update();
}
