/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim3;
extern uint32_t *color_buf;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_S8_A_Pin GPIO_PIN_0
#define EN_S8_A_GPIO_Port GPIOA
#define EN_S8_A_EXTI_IRQn EXTI0_1_IRQn
#define EN_S8_B_Pin GPIO_PIN_1
#define EN_S8_B_GPIO_Port GPIOA
#define EN_S8_B_EXTI_IRQn EXTI0_1_IRQn
#define EN_S7_A_Pin GPIO_PIN_2
#define EN_S7_A_GPIO_Port GPIOA
#define EN_S7_A_EXTI_IRQn EXTI2_3_IRQn
#define EN_S7_B_Pin GPIO_PIN_3
#define EN_S7_B_GPIO_Port GPIOA
#define EN_S7_B_EXTI_IRQn EXTI2_3_IRQn
#define EN_S6_A_Pin GPIO_PIN_4
#define EN_S6_A_GPIO_Port GPIOA
#define EN_S6_A_EXTI_IRQn EXTI4_15_IRQn
#define EN_S6_B_Pin GPIO_PIN_5
#define EN_S6_B_GPIO_Port GPIOA
#define EN_S6_B_EXTI_IRQn EXTI4_15_IRQn
#define EN_S5_A_Pin GPIO_PIN_6
#define EN_S5_A_GPIO_Port GPIOA
#define EN_S5_A_EXTI_IRQn EXTI4_15_IRQn
#define EN_S5_B_Pin GPIO_PIN_7
#define EN_S5_B_GPIO_Port GPIOA
#define EN_S5_B_EXTI_IRQn EXTI4_15_IRQn
#define SW_K9_Pin GPIO_PIN_1
#define SW_K9_GPIO_Port GPIOB
#define SW_K1_Pin GPIO_PIN_2
#define SW_K1_GPIO_Port GPIOB
#define EN_S4_A_Pin GPIO_PIN_12
#define EN_S4_A_GPIO_Port GPIOB
#define EN_S4_A_EXTI_IRQn EXTI4_15_IRQn
#define EN_S4_B_Pin GPIO_PIN_13
#define EN_S4_B_GPIO_Port GPIOB
#define EN_S4_B_EXTI_IRQn EXTI4_15_IRQn
#define EN_S3_A_Pin GPIO_PIN_14
#define EN_S3_A_GPIO_Port GPIOB
#define EN_S3_A_EXTI_IRQn EXTI4_15_IRQn
#define EN_S3_B_Pin GPIO_PIN_15
#define EN_S3_B_GPIO_Port GPIOB
#define EN_S3_B_EXTI_IRQn EXTI4_15_IRQn
#define EN_S2_A_Pin GPIO_PIN_8
#define EN_S2_A_GPIO_Port GPIOA
#define EN_S2_A_EXTI_IRQn EXTI4_15_IRQn
#define EN_S2_B_Pin GPIO_PIN_9
#define EN_S2_B_GPIO_Port GPIOA
#define EN_S2_B_EXTI_IRQn EXTI4_15_IRQn
#define EN_S1_A_Pin GPIO_PIN_10
#define EN_S1_A_GPIO_Port GPIOA
#define EN_S1_A_EXTI_IRQn EXTI4_15_IRQn
#define EN_S1_B_Pin GPIO_PIN_11
#define EN_S1_B_GPIO_Port GPIOA
#define EN_S1_B_EXTI_IRQn EXTI4_15_IRQn
#define SW_K2_Pin GPIO_PIN_3
#define SW_K2_GPIO_Port GPIOB
#define SW_K3_Pin GPIO_PIN_4
#define SW_K3_GPIO_Port GPIOB
#define SW_K4_Pin GPIO_PIN_5
#define SW_K4_GPIO_Port GPIOB
#define SW_K5_Pin GPIO_PIN_6
#define SW_K5_GPIO_Port GPIOB
#define SW_K6_Pin GPIO_PIN_7
#define SW_K6_GPIO_Port GPIOB
#define SW_K7_Pin GPIO_PIN_8
#define SW_K7_GPIO_Port GPIOB
#define SW_K8_Pin GPIO_PIN_9
#define SW_K8_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
