/*
 * I2C_SPI.h
 *
 *  Created on: 26 sie 2016
 *      Author: Mariusz
 */

#ifndef I2C_SPI_H_
#include "stm32f4xx_hal.h"

#define I2C_SPI_H_


#define LSM303_ACC_ADDRESS (0x19 << 1) // adres akcelerometru: 0011 001x
#define LSM303_ACC_CTRL_REG1_A 0x20 // rejestr ustawien 1
#define LSM303_ACC_Z_ENABLE 0x04 // 0000 0100
#define LSM303_ACC_100HZ 0x50 // 0101 0000
#define LSM303_ACC_Z_H_A 0x2D // wyzszy bajt danych osi Z
#define LSM303_ACC_RESOLUTION 2
uint8_t AccSettings = LSM303_ACC_Z_ENABLE | LSM303_ACC_100HZ;


#define L3GD20_GYRO_CTRL_REG1 0x20 // rejestr ustawien 1
#define L3GD20_GYRO_190HZ_25BW 0x50 // 0101 0000
#define L3GD20_GYRO_ENABLE 0x08 // 0000 1000
#define L3GD20_GYRO_ZXY_ENABLE 0x07 // 0000 0100
#define L3GD20_GYRO_Z_H 0x2D // wyzszy bajt danych osi Z
#define L3GD20_GYRO_Z_L 0x2C // wyzszy bajt danych osi Z
#define L3GD20_GYRO_WHO_AM_I 0x0F

enum {GyroSettings = L3GD20_GYRO_190HZ_25BW | L3GD20_GYRO_ENABLE | L3GD20_GYRO_ZXY_ENABLE};
uint8_t sendToGyro[4] = {L3GD20_GYRO_CTRL_REG1, GyroSettings, L3GD20_GYRO_Z_H | 128, L3GD20_GYRO_WHO_AM_I | 128};
uint8_t * pSendSPI[4] = {sendToGyro,&sendToGyro[1],&sendToGyro[2],&sendToGyro[3]};

HAL_StatusTypeDef initAccI2C(I2C_HandleTypeDef * hi2c);
HAL_StatusTypeDef initGyroSPI();

void getDataACC(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pDataGetXAxis,/
				uint8_t *pDataGetYAxis,  uint8_t *pDataGetZAxis, uint32_t Timeout);

void getDataSPI(SPI_HandleTypeDef *hspi,  uint8_t * RegAddress,	 uint8_t *pDataGetXAxis,/
			 	 uint8_t *pDataGetYAxis,  uint8_t *pDataGetZAxis,uint32_t Timeout);





#endif /* I2C_SPI_H_ */
