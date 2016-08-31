/*
 * I2C_SPI.c
 *
 *  Created on: 26 sie 2016
 *      Author: Mariusz
 */
#include "I2C_SPI.h"
#include <limits.h>


uint8_t AccSettings = LSM303_ACC_Z_ENABLE | LSM303_ACC_100HZ;

enum {GyroSettings = L3GD20_GYRO_190HZ_25BW | L3GD20_GYRO_ENABLE | L3GD20_GYRO_ZXY_ENABLE};
uint8_t sendToGyro[9]= {L3GD20_GYRO_CTRL_REG1 , GyroSettings,
		L3GD20_GYRO_X_L | 128 | L3GD20_GYRO_MS_BIT,L3GD20_GYRO_X_H | 128, //multi reading
		L3GD20_GYRO_Y_L | 128,L3GD20_GYRO_Y_H | 128,
		L3GD20_GYRO_Z_L | 128,L3GD20_GYRO_Z_H | 128,
		L3GD20_GYRO_WHO_AM_I | 128};
uint8_t * pSendSPI[9] = {sendToGyro, &sendToGyro[1], &sendToGyro[2],
		&sendToGyro[3], &sendToGyro[4], &sendToGyro[5],
		&sendToGyro[6], &sendToGyro[7], &sendToGyro[8]};

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

void getPositionDataACC(I2C_HandleTypeDef *hi2c, int16_t DevAddress, int16_t *pDataGetXAxis,
				int16_t *pDataGetYAxis,  int16_t *pDataGetZAxis, uint32_t Timeout){
}

void getPositionDataSPI(SPI_HandleTypeDef *hspi,  float * pDataGetXAxis,
		float *pDataGetYAxis,  float *pDataGetZAxis,uint32_t Timeout){
	uint8_t DataGetAxisTemp1;
	int16_t DataGetAxisTemp2;
	uint8_t *pDataGetAxisTemp1 = &DataGetAxisTemp1;
	int16_t *pDataGetAxisTemp2 = &DataGetAxisTemp2;

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,RESET);
	HAL_SPI_Transmit(hspi,*pSendSPI + 2,1,100);

	HAL_SPI_Receive(hspi,pDataGetAxisTemp1,1,100);
	HAL_SPI_Receive(hspi,pDataGetAxisTemp2,1,100);
	*pDataGetAxisTemp2 = ((*pDataGetAxisTemp2 << 8) | (*pDataGetAxisTemp1));
	*pDataGetXAxis = (float)*pDataGetAxisTemp2/INT16_MAX*2*3.1415*250/360;;

	HAL_SPI_Receive(hspi,pDataGetAxisTemp1,1,100);
	HAL_SPI_Receive(hspi,pDataGetAxisTemp2,1,100);
	*pDataGetAxisTemp2 = ((*pDataGetAxisTemp2 << 8) | (*pDataGetAxisTemp1));
	*pDataGetYAxis = (float)*pDataGetAxisTemp2/INT16_MAX*2*3.1415*250/360;;

	HAL_SPI_Receive(hspi,pDataGetAxisTemp1,1,100);
	HAL_SPI_Receive(hspi,pDataGetAxisTemp2,1,100);
	*pDataGetAxisTemp2 = ((*pDataGetAxisTemp2 << 8) | (*pDataGetAxisTemp1));
	*pDataGetZAxis = (float)*pDataGetAxisTemp2/INT16_MAX*2*3.1415*250/360;

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,SET);
}



