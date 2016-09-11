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
#define LSM303_ACC_XYZ_ENABLE 0x07 // 0000 0100
#define LSM303_ACC_100HZ 0x50 // 0101 0000
#define LSM303_ACC_X_H 0x29 // wyzszy bajt danych osi X
#define LSM303_ACC_X_L 0x28 // nizszy bajt danych osi X
#define LSM303_ACC_Y_H 0x2B // wyzszy bajt danych osi Y
#define LSM303_ACC_Y_L 0x2A // nizszy bajt danych osi Y
#define LSM303_ACC_Z_H 0x2D // wyzszy bajt danych osi Z
#define LSM303_ACC_Z_L 0x2C // nizszy bajt danych osi Z
#define LSM303_ACC_RESOLUTION 2
uint8_t AccSettings;

#define L3GD20_GYRO_CTRL_REG1 0x20 // rejestr ustawien 1
#define L3GD20_GYRO_190HZ_25BW 0x50 // 0101 0000
#define L3GD20_GYRO_ENABLE 0x08 // 0000 1000
#define L3GD20_GYRO_ZXY_ENABLE 0x07 // 0000 0100
#define L3GD20_GYRO_MS_BIT 0x40
#define L3GD20_GYRO_X_H 0x29 // wyzszy bajt danych osi Z
#define L3GD20_GYRO_X_L 0x28 // wyzszy bajt danych osi Z
#define L3GD20_GYRO_Y_H 0x2B // wyzszy bajt danych osi Z
#define L3GD20_GYRO_Y_L 0x2A // wyzszy bajt danych osi Z
#define L3GD20_GYRO_Z_H 0x2D // wyzszy bajt danych osi Z
#define L3GD20_GYRO_Z_L 0x2C // wyzszy bajt danych osi Z
#define L3GD20_GYRO_WHO_AM_I 0x0F


uint8_t sendToGyro[9];
uint8_t * pSendSPI[9];

HAL_StatusTypeDef initAccI2C(I2C_HandleTypeDef * hi2c);
HAL_StatusTypeDef initGyroSPI(SPI_HandleTypeDef *hspi);

void getPositionDataACC(I2C_HandleTypeDef *hi2c, double *pDataGetXAxis,
		double *pDataGetYAxis,  double *pDataGetZAxis, uint32_t Timeout);

void getPositionDataSPI(SPI_HandleTypeDef *hspi, double *pDataGetXAxis,
		double *pDataGetYAxis,  double *pDataGetZAxis, uint32_t Timeout);





#endif /* I2C_SPI_H_ */
