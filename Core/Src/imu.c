/**
 * @file imu.c
 * @brief Source file for BNO055 IMU functions (I2C interface)
 *
 * Implements configuration and data reading functions for BNO055 sensor.
 * Output includes heading angle and full Euler orientation. The IMU was tested
 * and confirmed functional but not yet integrated into control FSM.
 */

#include "imu.h"
#include "main.h"       // To access huart1
#include "stdio.h"      // For sprintf
#include "string.h"     // For strlen

static I2C_HandleTypeDef *imu_i2c = NULL;

void BNO055_Init(I2C_HandleTypeDef *hi2c) {
    imu_i2c = hi2c;

    // Soft reset
    uint8_t reset = 0x20;
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x3F, I2C_MEMADD_SIZE_8BIT, &reset, 1, 100);
    HAL_Delay(750);  // Per datasheet

    // Confirm chip ID (optional but robust)
    uint8_t chip_id = 0x00;
    HAL_I2C_Mem_Read(imu_i2c, BNO055_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &chip_id, 1, 10);
    if (chip_id != 0xA0) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"BNO055 ID Mismatch!\r\n", 22, HAL_MAX_DELAY);
        return;
    }

    // Set to config mode
    uint8_t mode = 0x00;
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x3D, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);
    HAL_Delay(20);

    // Set power mode to NORMAL
    uint8_t power = 0x00;
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x3E, I2C_MEMADD_SIZE_8BIT, &power, 1, 10);
    HAL_Delay(10);

    // Set page 0 (required before setting units)
    uint8_t page = 0x00;
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, &page, 1, 10);

    // Set units (optional — can skip this)
    uint8_t units = 0x00;  // Degrees, C, m/s²
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, &units, 1, 10);

    // Set operation mode (IMU or NDOF)
    mode = 0x0C;  // 0x08 = IMU, 0x0C = NDOF
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x3D, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);
    HAL_Delay(20);
}

/*
void BNO055_Init(I2C_HandleTypeDef *hi2c) {
    imu_i2c = hi2c;

    // Reset IMU
    uint8_t reset = 0x20;
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x3F, I2C_MEMADD_SIZE_8BIT, &reset, 1, 100);
    HAL_Delay(650);  // Reset time per datasheet

    // Set to config mode
    uint8_t mode = 0x00;
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x3D, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);
    HAL_Delay(20);

    // Set power mode to normal
    uint8_t power = 0x00;
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x3E, I2C_MEMADD_SIZE_8BIT, &power, 1, 10);
    HAL_Delay(10);

    // Set operation mode to NDOF
    mode = 0x0C;
    HAL_I2C_Mem_Write(imu_i2c, BNO055_ADDR, 0x3D, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);
    HAL_Delay(20);
}*/

void BNO055_StartCalibration(void) {
    const char* instructions =
        "Calibrate IMU:\r\n"
        "- Move in a figure 8\r\n"
        "- Tilt forward/backward\r\n"
        "- Roll side to side\r\n"
        "- Rotate in place\r\n";

    HAL_UART_Transmit(&huart1, (uint8_t*)instructions, strlen(instructions), HAL_MAX_DELAY);
}


void BNO055_ReadCalibStatus(void) {
    if (!imu_i2c) return;

    uint8_t cal;
    HAL_I2C_Mem_Read(imu_i2c, BNO055_ADDR, 0x35, I2C_MEMADD_SIZE_8BIT, &cal, 1, 10);

    uint8_t sys = (cal >> 6) & 0x03;
    uint8_t gyr = (cal >> 4) & 0x03;
    uint8_t acc = (cal >> 2) & 0x03;
    uint8_t mag =  cal       & 0x03;

    char msg[64];
    int len = sprintf(msg, "Calib SYS:%d GYR:%d ACC:%d MAG:%d\r\n", sys, gyr, acc, mag);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
}


void BNO055_GetCalibStatus(uint8_t* sys, uint8_t* gyr, uint8_t* acc, uint8_t* mag) {
    if (!imu_i2c) return;

    uint8_t cal;
    HAL_I2C_Mem_Read(imu_i2c, BNO055_ADDR, 0x35, I2C_MEMADD_SIZE_8BIT, &cal, 1, 10);

    *sys = (cal >> 6) & 0x03;
    *gyr = (cal >> 4) & 0x03;
    *acc = (cal >> 2) & 0x03;
    *mag =  cal       & 0x03;
}


void BNO055_Debug_ReadGyro(void) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(imu_i2c, BNO055_ADDR, 0x14, I2C_MEMADD_SIZE_8BIT, data, 6, 10);

    int16_t gx = (int16_t)(data[0] | (data[1] << 8));
    int16_t gy = (int16_t)(data[2] | (data[3] << 8));
    int16_t gz = (int16_t)(data[4] | (data[5] << 8));

    char msg[64];
    int len = sprintf(msg, "GYRO: X:%d Y:%d Z:%d\r\n", gx, gy, gz);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
}


float BNO055_ReadHeading(void) {
    if (!imu_i2c) return -1;

    uint8_t data[2];
    HAL_I2C_Mem_Read(imu_i2c, BNO055_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, data, 2, 10);

    uint16_t raw = data[0] | (data[1] << 8);
    return raw / 16.0f;
}


void BNO055_ReadEuler(float* heading, float* roll, float* pitch) {
    if (!imu_i2c) return;

    uint8_t buf[6];
    HAL_I2C_Mem_Read(imu_i2c, BNO055_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, buf, 6, 10);

    int16_t h_raw = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t r_raw = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t p_raw = (int16_t)(buf[4] | (buf[5] << 8));

    *heading = h_raw / 16.0f;
    *roll    = r_raw / 16.0f;
    *pitch   = p_raw / 16.0f;
}

void BNO055_SaveCalibrationOffsets(void)
{
    uint8_t offset_data[22];
    HAL_I2C_Mem_Read(imu_i2c, BNO055_ADDR, 0x55, I2C_MEMADD_SIZE_8BIT, offset_data, 22, 100);

    char msg[64];
    for (int i = 0; i < 22; ++i) {
        int len = sprintf(msg, "0x%02X ", offset_data[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}



