/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file main.h
  * @brief Main header for ping pong launcher project
  *
  * This file contains global includes, definitions, and peripheral handles used throughout
  * the autonomous ping pong ball launcher project.
  *
  * Core modules include:
  * - FSM for control logic
  * - IMU (BNO055) for orientation (demo only)
  * - Motor control for turret and flywheel
  * - Servo control for ball loading
  *
  * The IMU is functional and reports heading data, but is not used in closed-loop control.
  * This was left as future work.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/**
 * @brief UART interface used for IMU debug and user command output
 */
extern UART_HandleTypeDef huart1; // Added by Josh when incorporating IMU code
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief General error handler; enters infinite loop on fault
 */
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
