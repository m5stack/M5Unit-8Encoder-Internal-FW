/**
 ******************************************************************************
 * File Name          : gpio.h
 * Description        : This file contains all the functions prototypes for
 *                      the gpio
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ws2812_H__
#define __ws2812_H__
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define PIXEL_MAX 9
/* TIM3 runs at 48 MHz with ARR = 59, so one PWM tick is 20.833 ns
 * and one
 * WS2812 bit slot is 1.25 us (60 ticks).
 *
 * BIT_0: 15 ticks high = 312.5 ns,
 * 45 ticks low = 937.5 ns
 * BIT_1: 30 ticks high = 625.0 ns, 30 ticks low =
 * 625.0 ns
 * These values maximize the minimum timing margin for a fixed PWM
 * period.
 */
#define BIT_1 30U
#define BIT_0 15U

/* 256 zero slots target 320 us; DMA completion precedes the final slot.
 * At
 * least 255 full low slots (318.75 us) precede the stop callback.
 */
#define WS2812_RESET_SLOTS 256U

/* USER CODE END Private defines */
void sk6812_init(uint8_t num);
void neopixel_set_color(uint8_t num, uint32_t color);
void ws2812_show(void);

/* USER CODE BEGIN Prototypes */
typedef struct {
    const uint16_t head[3];                   //先发送3个0等待DMA稳定
    uint16_t data[24 * PIXEL_MAX];            //真正的数据
    const uint16_t tail[WS2812_RESET_SLOTS];  //保持复位低电平至少280us
} frame_buf;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
