/**
  **************************************************************************
  * @file     at32f45x_int.c
  * @brief    main interrupt service routines.
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

/* includes ------------------------------------------------------------------*/
#include "at32f45x_int.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board_bumper.h"
#include "motor_encoder.h"
#include "adc_dma.h"
#include "board_safety.h"

/** @addtogroup AT32F456_periph_template
  * @{
  */

/** @addtogroup 456_LED_toggle
  * @{
  */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
    Board_SafeHalt();
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
    Board_SafeHalt();
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
    Board_SafeHalt();
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
    Board_SafeHalt();
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
    Board_SafeHalt();
}



/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
}



/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */

extern void xPortSysTickHandler(void);
volatile uint32_t board_ms = 0;

void SysTick_Handler(void)
{
    /* 保留原来的板级毫秒计数。 */
    board_ms++;

    /* 调度器启动后，再调用 FreeRTOS 的 tick 处理。 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    adc_dma_irq();
}

extern void canfd_driver_rx_irq(void);
void CAN3_RX_IRQHandler(void)
{
    canfd_driver_rx_irq();
}

void EXINT4_IRQHandler(void)
{
    if (exint_interrupt_flag_get(EXINT_LINE_4) != RESET) {
        exint_flag_clear(EXINT_LINE_4);
        motor_encoder_right_edge_handler();
    }
}

void EXINT9_5_IRQHandler(void)
{
    uint8_t left_changed = 0;
    if (exint_interrupt_flag_get(EXINT_LINE_5) != RESET) {
        exint_flag_clear(EXINT_LINE_5);
        motor_encoder_right_edge_handler();
    }
    if (exint_interrupt_flag_get(EXINT_LINE_6) != RESET) {
        exint_flag_clear(EXINT_LINE_6);
        left_changed = 1;
    }
    if (exint_interrupt_flag_get(EXINT_LINE_7) != RESET) {
        exint_flag_clear(EXINT_LINE_7);
        left_changed = 1;
    }
    if (left_changed) motor_encoder_left_edge_handler();
    Board_BumperIRQ();
}

void EXINT15_10_IRQHandler(void)
{
    Board_BumperIRQ();
}

/**
  * @}
  */

/**
  * @}
  */
