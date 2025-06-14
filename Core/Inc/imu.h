/**
 * @file imu.h
 * @brief Interface for BNO055 IMU sensor over I2C
 *
 * This module provides initialization, calibration, and data retrieval functions for the
 * BNO055 inertial measurement unit (IMU), configured in NDOF mode for Euler angle output.
 *
 * Although IMU functionality was verified and outputs correct heading values, it was not
 * integrated into the main FSM due to time constraints. Future work may involve using the
 * IMU heading to actively stabilize turret orientation or detect drift during autonomous aiming.
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

#define BNO055_ADDR (0x28 << 1)  // 7-bit address, shifted for HAL

/**
 * @brief Initializes the BNO055 IMU in NDOF mode
 * @param hi2c Pointer to I2C handle
 */
void BNO055_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Sends serial instructions to the user to calibrate the IMU
 *
 * This function prints out instructions via UART to help the user perform
 * system, gyroscope, accelerometer, and magnetometer calibration.
 */
void BNO055_StartCalibration(void);

/**
 * @brief Reads and prints the current IMU calibration status over UART
 *
 * SYS/GYR/ACC/MAG status values are printed via UART for debug purposes.
 */
void BNO055_ReadCalibStatus(void);

/**
 * @brief Reads the current calibration status into provided pointers
 * @param sys Pointer to store system calibration level
 * @param gyr Pointer to store gyroscope calibration level
 * @param acc Pointer to store accelerometer calibration level
 * @param mag Pointer to store magnetometer calibration level
 */
void BNO055_GetCalibStatus(uint8_t* sys, uint8_t* gyr, uint8_t* acc, uint8_t* mag);

/**
 * @brief Debug function that reads and prints raw gyroscope data over UART
 */
void BNO055_Debug_ReadGyro(void);

/**
 * @brief Reads the yaw heading angle from the IMU
 * @return Heading in degrees (0–360°), or -1 on error
 */
float BNO055_ReadHeading(void);

/**
 * @brief Reads Euler angles (heading, roll, pitch) from the IMU
 * @param heading Pointer to store heading angle (deg)
 * @param roll Pointer to store roll angle (deg)
 * @param pitch Pointer to store pitch angle (deg)
 */
void BNO055_ReadEuler(float* heading, float* roll, float* pitch);

/**
 * @brief Debug function to print 22-byte calibration offset register
 *
 * This can be used to save offsets for reinitialization later.
 */
void BNO055_SaveCalibrationOffsets(void);

#endif /* INC_IMU_H_ */
