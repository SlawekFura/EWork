/*
 * I2C_SPI.c
 *
 *  Created on: 26 sie 2016
 *      Author: Mariusz
 */
#include "I2C_SPI.h"

HAL_StatusTypeDef initAccI2C(I2C_HandleTypeDef * hi2c){
	  return HAL_I2C_Mem_Write(hi2c, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &AccSettings, 1, 100);
}
HAL_StatusTypeDef initGyroSPI(){
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,RESET);
	if((status = HAL_SPI_Transmit(&hspi1,pSendSPI,1,100))!= HAL_OK){
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,SET);
		return status;
	}

	if((status = HAL_SPI_Transmit(&hspi1,pSendSPI + 1,1,100))!= HAL_OK){
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,SET);
		return status;
	}
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,SET);
	return HAL_ERROR;
}

void getDataACC(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pDataGetXAxis,/
				uint8_t *pDataGetYAxis,  uint8_t *pDataGetZAxis, uint32_t Timeout){


}

void getDataSPI(SPI_HandleTypeDef *hspi,  uint8_t * RegAddress,	 uint8_t *pDataGetXAxis,/
			 	 uint8_t *pDataGetYAxis,  uint8_t *pDataGetZAxis,uint32_t Timeout);



