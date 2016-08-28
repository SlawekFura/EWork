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
HAL_StatusTypeDef initGyroSPI(SPI_HandleTypeDef *hspi){
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,RESET);
	if((status = HAL_SPI_Transmit(hspi,*pSendSPI,1,100))!= HAL_OK){
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,SET);
		return status;
	}

	if((status = HAL_SPI_Transmit(hspi,*pSendSPI + 1,1,100))!= HAL_OK){
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,SET);
		return status;
	}
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,SET);
	return HAL_ERROR;
}

void getPositionDataACC(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t *pDataGetXAxis,
				uint16_t *pDataGetYAxis,  uint16_t *pDataGetZAxis, uint32_t Timeout){


}

void getPositionDataSPI(SPI_HandleTypeDef *hspi,  uint16_t * pDataGetXAxis,
			 	 uint16_t *pDataGetYAxis,  uint16_t *pDataGetZAxis,uint32_t Timeout){
	uint8_t DataGetAxisTemp;
	uint8_t *pDataGetAxisTemp = &DataGetAxisTemp;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,RESET);
	HAL_SPI_Transmit(hspi,*pSendSPI + 2,1,100);

	HAL_SPI_Receive(hspi,pDataGetAxisTemp,1,100);
	*pDataGetXAxis = *pDataGetAxisTemp;
	HAL_SPI_Receive(hspi,pDataGetAxisTemp,1,100);
	*pDataGetXAxis = (*pDataGetAxisTemp) << 8;

	HAL_SPI_Receive(hspi,pDataGetAxisTemp,1,100);
	*pDataGetYAxis = *pDataGetAxisTemp;
	HAL_SPI_Receive(hspi,pDataGetAxisTemp,1,100);
	*pDataGetYAxis = (*pDataGetAxisTemp)<<8;

	HAL_SPI_Receive(hspi,pDataGetAxisTemp,1,100);
	*pDataGetZAxis = *pDataGetAxisTemp;
	HAL_SPI_Receive(hspi,pDataGetAxisTemp,1,100);
	*pDataGetZAxis = (*pDataGetAxisTemp)<<8;

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,SET);
}



