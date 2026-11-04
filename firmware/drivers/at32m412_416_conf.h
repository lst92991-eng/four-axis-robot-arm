/**
  **************************************************************************
  * @file     at32m412_416_conf.h
  * @brief    at32m412_416 config header file
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

/* 头文件重复包含保护。 */
#ifndef __AT32M412_416_CONF_H
#define __AT32M412_416_CONF_H


//---------时钟参数区---------
/**
  * @brief 设置应用板的外部高速晶振频率，单位Hz。
  * @note 工具链预定义的HEXT_VALUE优先于此处默认值。
  */
#if !defined  HEXT_VALUE
#define HEXT_VALUE                       ((uint32_t)12000000) /*!< 外部高速晶振频率，Hz。 */
#endif

/**
  * @brief 外部高速晶振启动等待上限，供厂商稳定等待函数使用。
  */
#define HEXT_STARTUP_TIMEOUT             ((uint16_t)0x3000)  /*!< HEXT启动等待计数上限。 */
#define HICK_VALUE                       ((uint32_t)8000000) /*!< 内部高速时钟频率，Hz。 */
#define LEXT_VALUE                       ((uint32_t)32768)   /*!< 外部低速时钟频率，Hz。 */

//---------外设选择区---------
/* 启用的外设模块。 */
#define ADC_MODULE_ENABLED
#define CAN_MODULE_ENABLED
#define CRM_MODULE_ENABLED
#define DMA_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define MISC_MODULE_ENABLED
#define PWC_MODULE_ENABLED
#define SPI_MODULE_ENABLED
#define TMR_MODULE_ENABLED

//---------库头文件区---------
/* 本板固定使用下列厂商外设接口。 */
#include "at32m412_416_adc.h"
#include "at32m412_416_can.h"
#include "at32m412_416_crm.h"
#include "at32m412_416_dma.h"
#include "at32m412_416_flash.h"
#include "at32m412_416_gpio.h"
#include "at32m412_416_misc.h"
#include "at32m412_416_pwc.h"
#include "at32m412_416_spi.h"
#include "at32m412_416_tmr.h"


#endif
