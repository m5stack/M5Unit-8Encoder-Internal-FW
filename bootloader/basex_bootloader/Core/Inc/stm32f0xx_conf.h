/**
  ******************************************************************************
  * @file    FLASH/FLASH_Program/stm32f0xx_conf.h 
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    13-October-2021
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F0XX_CONF_H
#define __STM32F0XX_CONF_H

#include "main.h"
/* Includes ------------------------------------------------------------------*/
/* Comment the line below to disable peripheral header file inclusion */
//#include "stm32f0xx_adc.h"
//#include "stm32f0xx_can.h"
//#include "stm32f0xx_cec.h"
//#include "stm32f0xx_comp.h"
//#include "stm32f0xx_crc.h"
//#include "stm32f0xx_crs.h"
//#include "stm32f0xx_dac.h"
//#include "stm32f0xx_dbgmcu.h"
//#include "stm32f0xx_dma.h"
//#include "stm32f0xx_exti.h"
#include "stm32f0xx_flash.h"
//#include "stm32f0xx_gpio.h"
//#include "stm32f0xx_i2c.h"
//#include "stm32f0xx_iwdg.h"
//#include "stm32f0xx_pwr.h"
//#include "stm32f0xx_rcc.h"
//#include "stm32f0xx_rtc.h"
//#include "stm32f0xx_spi.h"
//#include "stm32f0xx_syscfg.h"
//#include "stm32f0xx_tim.h"
//#include "stm32f0xx_usart.h"
//#include "stm32f0xx_wwdg.h"
//#include "stm32f0xx_misc.h"  /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */
#define USE_FULL_ASSERT    1
/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F0XX_CONF_H */

