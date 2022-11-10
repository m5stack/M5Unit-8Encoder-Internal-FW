/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_ex.h"
#include "ws2812.h"
#include "flash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION 1
#define SWITCH_FILTER 10
#define SWITCH_FILTER_TIMEROUT SWITCH_FILTER*15
#define BUTTON_FILTER 500
#define BUTTON_FILTER_TIMEROUT BUTTON_FILTER*3
#define I2C_ADDRESS 0x41
#define APPLICATION_ADDRESS     ((uint32_t)0x08001000) 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch3;

/* USER CODE BEGIN PV */
uint16_t phase_a_pin_map[8] = {
    EN_S1_A_Pin, EN_S2_A_Pin, EN_S3_A_Pin, EN_S4_A_Pin, EN_S5_A_Pin, EN_S6_A_Pin, EN_S7_A_Pin, EN_S8_A_Pin,
};
uint16_t phase_b_pin_map[8] = {
    EN_S1_B_Pin, EN_S2_B_Pin, EN_S3_B_Pin, EN_S4_B_Pin, EN_S5_B_Pin, EN_S6_B_Pin, EN_S7_B_Pin, EN_S8_B_Pin,
};
uint8_t i2c_buff[4] = {0};
uint32_t g_counter[8] = {0};
uint32_t g_inc_counter[8] = {0};
uint8_t g_phase_a_last_encoder_status[8] = {0};
uint8_t g_phase_b_last_encoder_status[8] = {0};
uint8_t i2c_address[1] = {0};
volatile uint8_t fm_version = FIRMWARE_VERSION;
volatile uint8_t g_mode_sel = 0;
volatile uint8_t g_button_status = 0;
volatile uint8_t f_encoder_read_fail = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void switch_encoder_gpio_input(void);
void switch_encoder_gpio_interrupt(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//复制中断向量表到SRAM首地址
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //重映射 SRAM 地址到 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

void user_i2c_init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = i2c_address[0]<<1;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

void i2c_address_write_to_flash(void) 
{   
  writeMessageToFlash(i2c_address , 1);   
}

void i2c_address_read_from_flash(void) 
{   
  if (!(readPackedMessageFromFlash(i2c_address, 1))) {
    i2c_address[0] = I2C_ADDRESS;
    i2c_address_write_to_flash();
  }
}

void disable_exti(void)
{
  HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
  HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn); 
}

void enable_exti(void)
{
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);   
}

void ec11_init(void)
{
  uint16_t counter;
  uint16_t pin_status[8] = {0};
  uint8_t last_pin_status[8] = {0};  

  switch_encoder_gpio_input();

  for (int i = 0; i < 8; i++) {
    counter = 0;
    if (i == 2 || i == 3) {
      last_pin_status[i] = HAL_GPIO_ReadPin(GPIOB, phase_a_pin_map[i]);
    }
    else {
      last_pin_status[i] = HAL_GPIO_ReadPin(GPIOA, phase_a_pin_map[i]);
    }    
    for (uint16_t j = 0; j < SWITCH_FILTER_TIMEROUT; j++) {  
      if (i == 2 || i == 3) {
        pin_status[i] = HAL_GPIO_ReadPin(GPIOB, phase_a_pin_map[i]);
      }
      else {
        pin_status[i] = HAL_GPIO_ReadPin(GPIOA, phase_a_pin_map[i]);
      }
      if (pin_status[i] == last_pin_status[i]) {
        counter++;
      }
      else {
        last_pin_status[i] = pin_status[i];
        counter = 0;
      }
      if (counter >= SWITCH_FILTER) {
        g_phase_a_last_encoder_status[i] = pin_status[i];
        break;
      }
    }    
  }

  for (int i = 0; i < 8; i++) {
    counter = 0;
    if (i == 2 || i == 3) {
      last_pin_status[i] = HAL_GPIO_ReadPin(GPIOB, phase_b_pin_map[i]);
    }
    else {
      last_pin_status[i] = HAL_GPIO_ReadPin(GPIOA, phase_b_pin_map[i]);
    }    
    for (uint16_t j = 0; j < SWITCH_FILTER_TIMEROUT; j++) {  
      if (i == 2 || i == 3) {
        pin_status[i] = HAL_GPIO_ReadPin(GPIOB, phase_b_pin_map[i]);
      }
      else {
        pin_status[i] = HAL_GPIO_ReadPin(GPIOA, phase_b_pin_map[i]);
      }
      if (pin_status[i] == last_pin_status[i]) {
        counter++;
      }
      else {
        last_pin_status[i] = pin_status[i];
        counter = 0;
      }
      if (counter >= SWITCH_FILTER) {
        g_phase_b_last_encoder_status[i] = pin_status[i];
        break;
      }
    }    
  }  

  switch_encoder_gpio_interrupt();  
}

void switch_pulse_mode(void)
{
  disable_exti();  

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pins : EN_S8_A_Pin EN_S8_B_Pin EN_S7_A_Pin EN_S7_B_Pin
                           EN_S6_A_Pin EN_S6_B_Pin EN_S5_A_Pin EN_S5_B_Pin
                           EN_S2_A_Pin EN_S2_B_Pin EN_S1_A_Pin EN_S1_B_Pin */
  GPIO_InitStruct.Pin = EN_S8_B_Pin|EN_S7_B_Pin|EN_S6_B_Pin
                        |EN_S5_B_Pin|EN_S2_B_Pin|EN_S1_B_Pin;
  GPIO_InitStruct.Mode = MODE_OUTPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = EN_S8_A_Pin|EN_S7_A_Pin|EN_S6_A_Pin
                        |EN_S5_A_Pin|EN_S2_A_Pin|EN_S1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  

  /*Configure GPIO pins : EN_S4_A_Pin EN_S4_B_Pin EN_S3_A_Pin EN_S3_B_Pin */
  GPIO_InitStruct.Pin = EN_S4_B_Pin|EN_S3_B_Pin;
  GPIO_InitStruct.Mode = MODE_OUTPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = EN_S4_A_Pin|EN_S3_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  

  enable_exti();      
}

void switch_ab_mode(void)
{
  disable_exti(); 
  ec11_init();  
  enable_exti();    
}

void switch_encoder_gpio_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pins : EN_S8_A_Pin EN_S8_B_Pin EN_S7_A_Pin EN_S7_B_Pin
                           EN_S6_A_Pin EN_S6_B_Pin EN_S5_A_Pin EN_S5_B_Pin
                           EN_S2_A_Pin EN_S2_B_Pin EN_S1_A_Pin EN_S1_B_Pin */
  GPIO_InitStruct.Pin = EN_S8_A_Pin|EN_S8_B_Pin|EN_S7_A_Pin|EN_S7_B_Pin
                          |EN_S6_A_Pin|EN_S6_B_Pin|EN_S5_A_Pin|EN_S5_B_Pin
                          |EN_S2_A_Pin|EN_S2_B_Pin|EN_S1_A_Pin|EN_S1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_S4_A_Pin EN_S4_B_Pin EN_S3_A_Pin EN_S3_B_Pin */
  GPIO_InitStruct.Pin = EN_S4_A_Pin|EN_S4_B_Pin|EN_S3_A_Pin|EN_S3_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
}

void switch_encoder_gpio_interrupt(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0}; 

  /*Configure GPIO pins : EN_S4_A_Pin EN_S4_B_Pin EN_S3_A_Pin EN_S3_B_Pin */
  GPIO_InitStruct.Pin = EN_S4_A_Pin|EN_S4_B_Pin|EN_S3_A_Pin|EN_S3_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  

  /*Configure GPIO pins : EN_S8_A_Pin EN_S8_B_Pin EN_S7_A_Pin EN_S7_B_Pin
                           EN_S6_A_Pin EN_S6_B_Pin EN_S5_A_Pin EN_S5_B_Pin
                           EN_S2_A_Pin EN_S2_B_Pin EN_S1_A_Pin EN_S1_B_Pin */
  GPIO_InitStruct.Pin = EN_S8_A_Pin|EN_S8_B_Pin|EN_S7_A_Pin|EN_S7_B_Pin
                          |EN_S6_A_Pin|EN_S6_B_Pin|EN_S5_A_Pin|EN_S5_B_Pin
                          |EN_S2_A_Pin|EN_S2_B_Pin|EN_S1_A_Pin|EN_S1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void switch_phase_a_gpio_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = EN_S8_A_Pin|EN_S7_A_Pin|EN_S6_A_Pin
                        |EN_S5_A_Pin|EN_S2_A_Pin|EN_S1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

  GPIO_InitStruct.Pin = EN_S4_A_Pin|EN_S3_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
}

void switch_phase_a_gpio_interrupt(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = EN_S8_A_Pin|EN_S7_A_Pin|EN_S6_A_Pin
                        |EN_S5_A_Pin|EN_S2_A_Pin|EN_S1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

  GPIO_InitStruct.Pin = EN_S4_A_Pin|EN_S3_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
}

uint8_t read_button_switch_status(uint8_t offset)
{
  uint16_t GPIO_Pin = 0;
  uint16_t counter = 0;
  uint8_t button_status = 0; 
  uint8_t last_button_status = 0;  

  switch (offset)
  {
  case 0:
    GPIO_Pin = SW_K1_Pin;
    break;
  case 1:
    GPIO_Pin = SW_K2_Pin;
    break;
  case 2:
    GPIO_Pin = SW_K3_Pin;
    break;        
  case 3:
    GPIO_Pin = SW_K4_Pin;
    break;        
  case 4:
    GPIO_Pin = SW_K5_Pin;
    break;        
  case 5:
    GPIO_Pin = SW_K6_Pin;
    break;        
  case 6:
    GPIO_Pin = SW_K7_Pin;
    break;        
  case 7:
    GPIO_Pin = SW_K8_Pin;
    break;        
  case 0x60:
    GPIO_Pin = SW_K9_Pin;
    break;        
  
  default:
    break;
  }

  last_button_status = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin);
  for (uint16_t i = 0; i < BUTTON_FILTER_TIMEROUT; i++) {  
    button_status = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin);
    if (button_status == last_button_status) {
      counter++;
    }
    else {
      last_button_status = button_status;
      counter = 0;
    }
    if (counter >= BUTTON_FILTER) {
      return button_status;
    }
  }
  //TODO: Just for debug
  return 1;
}

void i2c2_receive_callback(uint8_t *rx_data, uint16_t len) 
{
  // if(len > 1 && (rx_data[0] == 0x00)) 
  // {
  //   g_mode_sel = (rx_data[1]==1)?1:0;
  //   if (g_mode_sel)
  //     switch_pulse_mode();
  //   else
  //     switch_ab_mode();
	// }
  // else if(len == 1 && (rx_data[0] == 0x00)) 
  // {
  //   i2c2_set_send_data((uint8_t *)&g_mode_sel, 1);
	// }
  if (len > 1 && (rx_data[0] <= 0x1F))
  {
    if ((len - 1) % 4 == 0) {
      for(int i = 0; i < (len - 1) / 4; i++) {
        g_counter[(rx_data[0]+i*4) / 4] = (rx_data[4+i*4] << 24) | (rx_data[3+i*4] << 16) | (rx_data[2+i*4] << 8) | (rx_data[1+i*4]);  
      }
    }
  }
  else if (len == 1 && (rx_data[0] <= 0x1F))
  {
    i2c2_set_send_data((uint8_t *)&g_counter[(rx_data[0])/4], 4);
  }
  else if (len == 1 && ((rx_data[0] >= 0x20) & (rx_data[0] <= 0x3F)))
  {
    i2c2_set_send_data((uint8_t *)&g_inc_counter[(rx_data[0]-0x20)/4], 4);
    g_inc_counter[(rx_data[0]-0x20)/4] = 0;
  }
  else if (len > 1 && ((rx_data[0] >= 0x40) & (rx_data[0] <= 0x47)))
  {
    for(int i = 0; i < len - 1; i++) {
      if (rx_data[1+i])
        g_counter[(rx_data[0]+i - 0x40)] = 0;  
    }
  }
  else if (len == 1 && ((rx_data[0] >= 0x50) & (rx_data[0] <= 0x57)))
  {
    g_button_status = read_button_switch_status(rx_data[0] - 0x50);
    i2c2_set_send_data((uint8_t *)&g_button_status, 1);
  }
  else if (len == 1 && (rx_data[0] == 0x60))
  {
    g_button_status = read_button_switch_status(rx_data[0]);
    i2c2_set_send_data((uint8_t *)&g_button_status, 1);
  }
	else if (len > 1 && ((rx_data[0] >= 0x70) & (rx_data[0] <= 0x8A))) 
	{
    if ((len-1) % 3 == 0) {
      for (int i = 0; i < (len - 1) / 3; i++) {
        neopixel_set_color(((rx_data[0]+i*3 - 0x70)/3), ((rx_data[1+i*3] << 16) | (rx_data[2+i*3] << 8) | (rx_data[3+i*3])));   
      }
      ws2812_show();
    }
  }
	else if (len == 1 && ((rx_data[0] >= 0x70) & (rx_data[0] <= 0x8A))) 
	{
    i2c_buff[0] = (color_buf[((rx_data[0] - 0x70)/3)] >> 16) & 0xff;
    i2c_buff[1] = (color_buf[((rx_data[0] - 0x70)/3)] >> 8) & 0xff;
    i2c_buff[2] = (color_buf[((rx_data[0] - 0x70)/3)]) & 0xff;
    i2c2_set_send_data(i2c_buff, 3);
  }                  
  else if (len == 1 && (rx_data[0] == 0xFE))
  {
    i2c2_set_send_data((uint8_t *)&fm_version, 1);
  }
  else if (len > 1 && (rx_data[0] == 0xFF))
  {
    if (len == 2) {
      if ((rx_data[1] >= 0) && (rx_data[1] < 128)) {
        i2c_address[0] = rx_data[1];
        i2c_address_write_to_flash();
        user_i2c_init();
      }
    }    
  }
  else if (len == 1 && (rx_data[0] == 0xFF))
  {
    i2c2_set_send_data(i2c_address, 1);    
  }     
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t flag = 0;
  uint16_t counter = 0;
  uint8_t phase_status = 0;
  uint8_t index = 0;

  disable_exti();
  if (g_mode_sel) {
    switch_phase_a_gpio_input();

    for (uint16_t i = 0; i < SWITCH_FILTER_TIMEROUT; i++) {
      if (GPIO_Pin == EN_S4_A_Pin || GPIO_Pin == EN_S3_A_Pin) {
        phase_status = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin);
      }
      else {
        phase_status = HAL_GPIO_ReadPin(GPIOA, GPIO_Pin);
      }

      if (!phase_status) {
        counter++;
      } else {
        counter = 0;
      }
      if (counter >= SWITCH_FILTER) {
        flag = 1;
        break;
      }        
    }

    if (!flag) {
      switch_phase_a_gpio_interrupt(); 
      enable_exti();
      return;
    }          

    switch (GPIO_Pin)
    {
    case EN_S1_A_Pin:
      g_counter[0]++;
      g_inc_counter[0]++;
      break;
    case EN_S2_A_Pin:
      g_counter[1]++;
      g_inc_counter[1]++;
      break;    
    case EN_S3_A_Pin:
      g_counter[2]++;
      g_inc_counter[2]++;
      break;    
    case EN_S4_A_Pin:
      g_counter[3]++;
      g_inc_counter[3]++;
      break;    
    case EN_S5_A_Pin:
      g_counter[4]++;
      g_inc_counter[4]++;
      break;    
    case EN_S6_A_Pin:
      g_counter[5]++;
      g_inc_counter[5]++;
      break;    
    case EN_S7_A_Pin:
      g_counter[6]++;
      g_inc_counter[6]++;
      break;    
    case EN_S8_A_Pin:
      g_counter[7]++;
      g_inc_counter[7]++;
      break;    
    
    default:
      break;
    }

    switch_phase_a_gpio_interrupt();
    enable_exti();     
    return;    
  }

  switch_encoder_gpio_input();    

  switch (GPIO_Pin)
  {
  case EN_S1_A_Pin:
    index = 0;
    break;
  case EN_S2_A_Pin:
    index = 1;
    break;    
  case EN_S3_A_Pin:
    index = 2;
    break;    
  case EN_S4_A_Pin:
    index = 3;
    break;    
  case EN_S5_A_Pin:
    index = 4;
    break;    
  case EN_S6_A_Pin:
    index = 5;
    break;    
  case EN_S7_A_Pin:
    index = 6;
    break;    
  case EN_S8_A_Pin:
    index = 7;
    break;
  case EN_S1_B_Pin:
    index = 0;
    break;
  case EN_S2_B_Pin:
    index = 1;
    break;    
  case EN_S3_B_Pin:
    index = 2;
    break;    
  case EN_S4_B_Pin:
    index = 3;
    break;    
  case EN_S5_B_Pin:
    index = 4;
    break;    
  case EN_S6_B_Pin:
    index = 5;
    break;    
  case EN_S7_B_Pin:
    index = 6;
    break;    
  case EN_S8_B_Pin:
    index = 7;
    break;        
  
  default:
    break;
  }  

  if (GPIO_Pin == EN_S1_A_Pin || GPIO_Pin == EN_S2_A_Pin || GPIO_Pin == EN_S3_A_Pin || GPIO_Pin == EN_S4_A_Pin \
|| GPIO_Pin == EN_S5_A_Pin || GPIO_Pin == EN_S6_A_Pin || GPIO_Pin == EN_S7_A_Pin || GPIO_Pin == EN_S8_A_Pin) {

    for (uint16_t i = 0; i < SWITCH_FILTER_TIMEROUT; i++) {
      if (GPIO_Pin == EN_S4_A_Pin || GPIO_Pin == EN_S3_A_Pin) {
        phase_status = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin);
      }
      else {
        phase_status = HAL_GPIO_ReadPin(GPIOA, GPIO_Pin);
      }
      
      if (g_phase_a_last_encoder_status[index]) {
        if (!phase_status) {
          counter++;
        } else {
          counter = 0;
        }
        if (counter >= SWITCH_FILTER) {
          flag = 1;
          break;
        }
      } else {
        if (phase_status) {
          counter++;
        } else {
          counter = 0;
        }
        if (counter >= SWITCH_FILTER) {
          flag = 1;
          break;
        }      
      }
    }

    if (!flag) {
      switch_encoder_gpio_interrupt();
      enable_exti(); 
      f_encoder_read_fail = 1;
      return;
    }
    
    switch (GPIO_Pin)
    {
    case EN_S1_A_Pin:
      if (g_phase_a_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S1_B_Pin))) {
          // if (g_counter[0] > -32768)
          g_counter[0]--;
          g_inc_counter[0]--;
        }
        else {
          // if (g_counter[0] < 32767)
          g_counter[0]++;
          g_inc_counter[0]++;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S1_B_Pin)) {
          // if (g_counter[0] > -32768)
          g_counter[0]--;
          g_inc_counter[0]--;
        }
        else {
          // if (g_counter[0] < 32767)
          g_counter[0]++;
          g_inc_counter[0]++;
        }       
      }
      break;
    case EN_S2_A_Pin:
      if (g_phase_a_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S2_B_Pin))) {
          // if (g_counter[1] > -32768)
          g_counter[1]--;
          g_inc_counter[1]--;
        }
        else {
          // if (g_counter[1] < 32767)
          g_counter[1]++;
          g_inc_counter[1]++;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S2_B_Pin)) {
          // if (g_counter[1] > -32768)
          g_counter[1]--;
          g_inc_counter[1]--;
        }
        else {
          // if (g_counter[1] < 32767)
          g_counter[1]++;
          g_inc_counter[1]++;
        }       
      }
      break;
    case EN_S3_A_Pin:
      if (g_phase_a_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOB, EN_S3_B_Pin))) {
          // if (g_counter[2] > -32768)
          g_counter[2]--;
          g_inc_counter[2]--;
        }
        else {
          // if (g_counter[2] < 32767)
          g_counter[2]++;
          g_inc_counter[2]++;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOB, EN_S3_B_Pin)) {
          // if (g_counter[2] > -32768)
          g_counter[2]--;
          g_inc_counter[2]--;
        }
        else {
          // if (g_counter[2] < 32767)
          g_counter[2]++;
          g_inc_counter[2]++;
        }       
      }
      break;
    case EN_S4_A_Pin:
      if (g_phase_a_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOB, EN_S4_B_Pin))) {
          // if (g_counter[3] > -32768)
          g_counter[3]--;
          g_inc_counter[3]--;
        }
        else {
          // if (g_counter[3] < 32767)
          g_counter[3]++;
          g_inc_counter[3]++;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOB, EN_S4_B_Pin)) {
          // if (g_counter[3] > -32768)
          g_counter[3]--;
          g_inc_counter[3]--;
        }
        else {
          // if (g_counter[3] < 32767)
          g_counter[3]++;
          g_inc_counter[3]++;
        }       
      }
      break;
    case EN_S5_A_Pin:
      if (g_phase_a_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S5_B_Pin))) {
          // if (g_counter[4] > -32768)
          g_counter[4]--;
          g_inc_counter[4]--;
        }
        else {
          // if (g_counter[4] < 32767)
          g_counter[4]++;
          g_inc_counter[4]++;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S5_B_Pin)) {
          // if (g_counter[4] > -32768)
          g_counter[4]--;
          g_inc_counter[4]--;
        }
        else {
          // if (g_counter[4] < 32767)
          g_counter[4]++;
          g_inc_counter[4]++;
        }       
      }
      break;
    case EN_S6_A_Pin:
      if (g_phase_a_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S6_B_Pin))) {
          // if (g_counter[5] > -32768)
          g_counter[5]--;
          g_inc_counter[5]--;
        }
        else {
          // if (g_counter[5] < 32767)
          g_counter[5]++;
          g_inc_counter[5]++;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S6_B_Pin)) {
          // if (g_counter[5] > -32768)
          g_counter[5]--;
          g_inc_counter[5]--;
        }
        else {
          // if (g_counter[5] < 32767)
          g_counter[5]++;
          g_inc_counter[5]++;
        }       
      }
      break;
    case EN_S7_A_Pin:
      if (g_phase_a_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S7_B_Pin))) {
          // if (g_counter[6] > -32768)
          g_counter[6]--;
          g_inc_counter[6]--;
        }
        else {
          // if (g_counter[6] < 32767)
          g_counter[6]++;
          g_inc_counter[6]++;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S7_B_Pin)) {
          // if (g_counter[6] > -32768)
          g_counter[6]--;
          g_inc_counter[6]--;
        }
        else {
          // if (g_counter[6] < 32767)
          g_counter[6]++;
          g_inc_counter[6]++;
        }       
      }
      break;
    case EN_S8_A_Pin:
      if (g_phase_a_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S8_B_Pin))) {
          // if (g_counter[7] > -32768)
          g_counter[7]--;
          g_inc_counter[7]--;
        }
        else {
          // if (g_counter[7] < 32767)
          g_counter[7]++;
          g_inc_counter[7]++;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S8_B_Pin)) {
          // if (g_counter[7] > -32768)
          g_counter[7]--;
          g_inc_counter[7]--;
        }
        else {
          // if (g_counter[7] < 32767)
          g_counter[7]++;
          g_inc_counter[7]++;
        }       
      }
      break;
    
    default:
      break;
    }
    g_phase_a_last_encoder_status[index] = !g_phase_a_last_encoder_status[index];
    switch_encoder_gpio_interrupt();
    enable_exti();            
  } else {
    for (uint16_t i = 0; i < SWITCH_FILTER_TIMEROUT; i++) {
      if (GPIO_Pin == EN_S4_B_Pin || GPIO_Pin == EN_S3_B_Pin) {
        phase_status = HAL_GPIO_ReadPin(GPIOB, GPIO_Pin);
      }
      else {
        phase_status = HAL_GPIO_ReadPin(GPIOA, GPIO_Pin);
      }
      
      if (g_phase_b_last_encoder_status[index]) {
        if (!phase_status) {
          counter++;
        } else {
          counter = 0;
        }
        if (counter >= SWITCH_FILTER) {
          flag = 1;
          break;
        }
      } else {
        if (phase_status) {
          counter++;
        } else {
          counter = 0;
        }
        if (counter >= SWITCH_FILTER) {
          flag = 1;
          break;
        }      
      }
    }

    if (!flag) {
      switch_encoder_gpio_interrupt();
      enable_exti(); 
      f_encoder_read_fail = 1;
      return;
    }
    
    switch (GPIO_Pin)
    {
    case EN_S1_B_Pin:
      if (g_phase_b_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S1_A_Pin))) {
          // if (g_counter[0] < 32767)
          g_counter[0]++;
          g_inc_counter[0]++;
        }
        else {
          // if (g_counter[0] > -32768)
          g_counter[0]--;
          g_inc_counter[0]--;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S1_A_Pin)) {
          // if (g_counter[0] < 32767)
          g_counter[0]++;
          g_inc_counter[0]++;
        }
        else {
          // if (g_counter[0] > -32768)
          g_counter[0]--;
          g_inc_counter[0]--;
        }       
      }
      break;
    case EN_S2_B_Pin:
      if (g_phase_b_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S2_A_Pin))) {
          // if (g_counter[1] < 32767)
          g_counter[1]++;
          g_inc_counter[1]++;
        }
        else {
          // if (g_counter[1] > -32768)
          g_counter[1]--;
          g_inc_counter[1]--;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S2_A_Pin)) {
          // if (g_counter[1] < 32767)
          g_counter[1]++;
          g_inc_counter[1]++;
        }
        else {
          // if (g_counter[1] > -32768)
          g_counter[1]--;
          g_inc_counter[1]--;
        }       
      }
      break;
    case EN_S3_B_Pin:
      if (g_phase_b_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOB, EN_S3_A_Pin))) {
          // if (g_counter[2] < 32767)
          g_counter[2]++;
          g_inc_counter[2]++;
        }
        else {
          // if (g_counter[2] > -32768)
          g_counter[2]--;
          g_inc_counter[2]--;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOB, EN_S3_A_Pin)) {
          // if (g_counter[2] < 32767)
          g_counter[2]++;
          g_inc_counter[2]++;
        }
        else {
          // if (g_counter[2] > -32768)
          g_counter[2]--;
          g_inc_counter[2]--;
        }       
      }
      break;
    case EN_S4_B_Pin:
      if (g_phase_b_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOB, EN_S4_A_Pin))) {
          // if (g_counter[3] < 32767)
          g_counter[3]++;
          g_inc_counter[3]++;
        }
        else {
          // if (g_counter[3] > -32768)
          g_counter[3]--;
          g_inc_counter[3]--;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOB, EN_S4_A_Pin)) {
          // if (g_counter[3] < 32767)
          g_counter[3]++;
          g_inc_counter[3]++;
        }
        else {
          // if (g_counter[3] > -32768)
          g_counter[3]--;
          g_inc_counter[3]--;
        }       
      }
      break;
    case EN_S5_B_Pin:
      if (g_phase_b_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S5_A_Pin))) {
          // if (g_counter[4] < 32767)
          g_counter[4]++;
          g_inc_counter[4]++;
        }
        else {
          // if (g_counter[4] > -32768)
          g_counter[4]--;
          g_inc_counter[4]--;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S5_A_Pin)) {
          // if (g_counter[4] < 32767)
          g_counter[4]++;
          g_inc_counter[4]++;
        }
        else {
          // if (g_counter[4] > -32768)
          g_counter[4]--;
          g_inc_counter[4]--;
        }       
      }
      break;
    case EN_S6_B_Pin:
      if (g_phase_b_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S6_A_Pin))) {
          // if (g_counter[5] < 32767)
          g_counter[5]++;
          g_inc_counter[5]++;
        }
        else {
          // if (g_counter[5] > -32768)
          g_counter[5]--;
          g_inc_counter[5]--;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S6_A_Pin)) {
          // if (g_counter[5] < 32767)
          g_counter[5]++;
          g_inc_counter[5]++;
        }
        else {
          // if (g_counter[5] > -32768)
          g_counter[5]--;
          g_inc_counter[5]--;
        }       
      }
      break;
    case EN_S7_B_Pin:
      if (g_phase_b_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S7_A_Pin))) {
          // if (g_counter[6] < 32767)
          g_counter[6]++;
          g_inc_counter[6]++;
        }
        else {
          // if (g_counter[6] > -32768)
          g_counter[6]--;
          g_inc_counter[6]--;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S7_A_Pin)) {
          // if (g_counter[6] < 32767)
          g_counter[6]++;
          g_inc_counter[6]++;
        }
        else {
          // if (g_counter[6] > -32768)
          g_counter[6]--;
          g_inc_counter[6]--;
        }       
      }
      break;
    case EN_S8_B_Pin:
      if (g_phase_b_last_encoder_status[index]) {
        if (!(HAL_GPIO_ReadPin(GPIOA, EN_S8_A_Pin))) {
          // if (g_counter[7] < 32767)
          g_counter[7]++;
          g_inc_counter[7]++;
        }
        else {
          // if (g_counter[7] > -32768)
          g_counter[7]--;
          g_inc_counter[7]--;
        }    
      } else {
        if (HAL_GPIO_ReadPin(GPIOA, EN_S8_A_Pin)) {
          // if (g_counter[7] < 32767)
          g_counter[7]++;
          g_inc_counter[7]++;
        }
        else {
          // if (g_counter[7] > -32768)
          g_counter[7]--;
          g_inc_counter[7]--;
        }       
      }
      break;
    
    default:
      break;
    }
    g_phase_b_last_encoder_status[index] = !g_phase_b_last_encoder_status[index];
    switch_encoder_gpio_interrupt();
    enable_exti();            
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // MX_I2C2_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */ 
  disable_exti();
  ec11_init();
  enable_exti();
  i2c_address_read_from_flash();
  user_i2c_init();
  HAL_I2C_EnableListen_IT(&hi2c2);   
  sk6812_init(PIXEL_MAX);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (f_encoder_read_fail) {
      f_encoder_read_fail = 0;
      disable_exti();
      ec11_init();
      enable_exti();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0x41<<1;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : EN_S8_A_Pin EN_S8_B_Pin EN_S7_A_Pin EN_S7_B_Pin
                           EN_S6_A_Pin EN_S6_B_Pin EN_S5_A_Pin EN_S5_B_Pin
                           EN_S2_A_Pin EN_S2_B_Pin EN_S1_A_Pin EN_S1_B_Pin */
  GPIO_InitStruct.Pin = EN_S8_A_Pin|EN_S8_B_Pin|EN_S7_A_Pin|EN_S7_B_Pin
                          |EN_S6_A_Pin|EN_S6_B_Pin|EN_S5_A_Pin|EN_S5_B_Pin
                          |EN_S2_A_Pin|EN_S2_B_Pin|EN_S1_A_Pin|EN_S1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_K9_Pin SW_K1_Pin SW_K2_Pin SW_K3_Pin
                           SW_K4_Pin SW_K5_Pin SW_K6_Pin SW_K7_Pin
                           SW_K8_Pin */
  GPIO_InitStruct.Pin = SW_K9_Pin|SW_K1_Pin|SW_K2_Pin|SW_K3_Pin
                          |SW_K4_Pin|SW_K5_Pin|SW_K6_Pin|SW_K7_Pin
                          |SW_K8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_S4_A_Pin EN_S4_B_Pin EN_S3_A_Pin EN_S3_B_Pin */
  GPIO_InitStruct.Pin = EN_S4_A_Pin|EN_S4_B_Pin|EN_S3_A_Pin|EN_S3_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

