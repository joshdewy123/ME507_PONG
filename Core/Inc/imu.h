#ifndef IMU_H
#define IMU_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BNO08X_I2C_ADDR 0x4A << 1  // Shifted for STM32 HAL

typedef struct {
    I2C_HandleTypeDef* hi2c;
} IMU_Handle_t;

typedef struct {
    float yaw;
    float pitch;
    float roll;
} IMU_Euler_t;

typedef struct {
    float gx;
    float gy;
    float gz;
} IMU_Gyro_t;

// Initialization
bool IMU_Init(IMU_Handle_t* imu, I2C_HandleTypeDef* hi2c);

// Orientation and Gyro
bool IMU_ReadEuler(IMU_Handle_t* imu, IMU_Euler_t* euler);
bool IMU_ReadGyro(IMU_Handle_t* imu, IMU_Gyro_t* gyro);

// Calibration
bool IMU_GetCalibrationStatus(IMU_Handle_t* imu, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

#endif
