/* Includes ------------------------------------------------------------------*/
#include "i2c_ex.h"
#include <string.h>
#include <stdlib.h>

#define I2C_WRITE_OPERATION		0
#define I2C_READ_OPERATION		1
#define	I2C_RECEIVE_BUFFER_LEN	50

__IO uint8_t rx_buffer[I2C_RECEIVE_BUFFER_LEN];
__IO uint8_t tx_buffer[I2C_RECEIVE_BUFFER_LEN];
__IO uint16_t tx_len = 0;
__IO uint8_t tx_state = 0;
__IO uint8_t trans_state = 0;
__IO uint32_t i2c_timeout_delay = 0;
__IO uint32_t trans_timeout_state = 0;

__weak void i2c2_receive_callback(uint8_t *rx_data, uint16_t len) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(rx_data);
	UNUSED(len);

	/* NOTE : This function should not be modified, when the callback is needed,
			  the i2c2_receive_callback could be implemented in the user file
	 */
}

__weak void i2c2_addr_req_callback(uint8_t TransferDirection) {
  UNUSED(TransferDirection);
}

void i2c2_set_send_data(uint8_t *tx_ptr, uint16_t len) {
  if (len > I2C_RECEIVE_BUFFER_LEN) {
    len = I2C_RECEIVE_BUFFER_LEN;
	}
	tx_len = len;

  if (len == 0 || tx_ptr == NULL) {
    return;
  }
  memcpy((void *)tx_buffer, tx_ptr, len);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
	if (hi2c->Instance == hi2c2.Instance) {
		trans_state = 1;
		hi2c->State = HAL_I2C_STATE_READY;
		i2c2_addr_req_callback(TransferDirection);
		if (TransferDirection == I2C_WRITE_OPERATION) {
			HAL_I2C_Slave_Receive_IT(hi2c, (uint8_t *)rx_buffer, I2C_RECEIVE_BUFFER_LEN);
		}
		else {
			HAL_I2C_Slave_Transmit_IT(hi2c, (uint8_t *)tx_buffer, tx_len);
      tx_state = 1;
		}
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
		HAL_I2C_EnableListen_IT(&hi2c2);
	}
}

// read finish will callback
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == hi2c2.Instance) {

    if (tx_state != 1) {
      i2c2_receive_callback((uint8_t *)&rx_buffer[0], I2C_RECEIVE_BUFFER_LEN - hi2c->XferSize);
    }
    tx_state = 0;
	trans_state = 0;
	trans_timeout_state = 0;	
		HAL_I2C_EnableListen_IT(&hi2c2);
	}
}

// write finish will callback
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == hi2c2.Instance) {
		i2c2_receive_callback((uint8_t *)&rx_buffer[0], I2C_RECEIVE_BUFFER_LEN);
		trans_state = 0;
		trans_timeout_state = 0;		
		HAL_I2C_EnableListen_IT(&hi2c2);
	}
}

// write finish will callback
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == hi2c2.Instance) {
    tx_state = 0;
		trans_state = 0;
		trans_timeout_state = 0;	
		HAL_I2C_EnableListen_IT(&hi2c2);
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {

  if (hi2c->Instance == hi2c2.Instance) {
		trans_state = 0;
		trans_timeout_state = 0;	  
		HAL_I2C_EnableListen_IT(&hi2c2);
		__HAL_I2C_GENERATE_NACK(&hi2c2);
	}
}

