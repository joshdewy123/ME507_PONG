#include "imu.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>


// Replace with actual registers for BNO085 if you're using sensor fusion reports
#define BNO08X_EULER_HEADING_LSB 0x1A
#define BNO08X_EULER_LEN         6
#define BNO08X_GYRO_DATA         0x14
#define BNO08X_GYRO_LEN          6
#define BNO08X_CALIB_STAT        0x35

bool IMU_Init(IMU_Handle_t* imu, I2C_HandleTypeDef* hi2c) {
    imu->hi2c = hi2c;

    // Optional: reset, check chip ID, etc.
    HAL_Delay(50);
    return true;
}

bool IMU_ReadEuler(IMU_Handle_t* imu, IMU_Euler_t* euler) {
    uint8_t buf[BNO08X_EULER_LEN];
    if (HAL_I2C_Mem_Read(imu->hi2c, BNO08X_I2C_ADDR, BNO08X_EULER_HEADING_LSB,
                         I2C_MEMADD_SIZE_8BIT, buf, BNO08X_EULER_LEN, HAL_MAX_DELAY) != HAL_OK)
        return false;

    int16_t heading = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t roll    = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t pitch   = (int16_t)((buf[5] << 8) | buf[4]);

    euler->yaw   = heading / 16.0f;
    euler->roll  = roll / 16.0f;
    euler->pitch = pitch / 16.0f;
    return true;
}

bool IMU_ReadGyro(IMU_Handle_t* imu, IMU_Gyro_t* gyro) {
    uint8_t buf[BNO08X_GYRO_LEN];
    if (HAL_I2C_Mem_Read(imu->hi2c, BNO08X_I2C_ADDR, BNO08X_GYRO_DATA,
                         I2C_MEMADD_SIZE_8BIT, buf, BNO08X_GYRO_LEN, HAL_MAX_DELAY) != HAL_OK)
        return false;

    int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);

    gyro->gx = gx / 16.0f;
    gyro->gy = gy / 16.0f;
    gyro->gz = gz / 16.0f;
    return true;
}

bool IMU_GetCalibrationStatus(IMU_Handle_t* imu, uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
    uint8_t status;
    if (HAL_I2C_Mem_Read(imu->hi2c, BNO08X_I2C_ADDR, BNO08X_CALIB_STAT,
                         I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;

    *sys   = (status >> 6) & 0x03;
    *gyro  = (status >> 4) & 0x03;
    *accel = (status >> 2) & 0x03;
    *mag   = status & 0x03;
    return true;
}
