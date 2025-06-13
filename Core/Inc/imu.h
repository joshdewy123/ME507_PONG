/*
 * imu.h
 *
 *  Created on: Jun 9, 2025
 *      Author: jdeweese
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

#define BNO055_ADDR (0x28 << 1)  // 7-bit address, shifted for HAL

void BNO055_Init(I2C_HandleTypeDef *hi2c);
void BNO055_StartCalibration(void);
void BNO055_ReadCalibStatus(void);
void BNO055_GetCalibStatus(uint8_t* sys, uint8_t* gyr, uint8_t* acc, uint8_t* mag);
void BNO055_Debug_ReadGyro(void);


float BNO055_ReadHeading(void);
void BNO055_ReadEuler(float* heading, float* roll, float* pitch);


#endif /* INC_IMU_H_ */
