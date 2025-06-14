/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file main.c
  * @brief Application entry point for ping pong launcher project
  *
  * This file initializes the STM32 peripherals, sets up the finite state machine (FSM),
  * and starts the main control loop. The IMU is initialized and verified, but not actively
  * used in the FSM during runtime.
  *
  * Major functions:
  * - Setup clocks, timers, I2C, and UART
  * - Initialize motors and servo
  * - Run FSM in infinite loop
  * - Provide IMU debug output for calibration and heading
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "imu.h"
//#include "fsm.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern I2C_HandleTypeDef hi2c2;

volatile uint8_t imu_mode = 0;
volatile uint8_t calib_mode = 0;

volatile uint8_t lidar_mode = 0;
uint8_t lidar_buf[4];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CPR_TURRET 3200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int32_t duty_flywheel1 = 0;
int32_t duty_flywheel2 = 0;
int32_t duty_turret1 = 0;
int32_t duty_turret2 = 0;

int32_t turret1_enc_count = 0;
int32_t turret1_enc_deg = 0;
int32_t turret1_target = 0;

int32_t turret2_enc_pos = 0;
int32_t turret2_enc_deg = 0;
int32_t turret2_target = 0;

motor_t flywheel1 = {
    .htim_forward = &htim1, .channel_forward = TIM_CHANNEL_1,
    .htim_reverse = &htim1, .channel_reverse = TIM_CHANNEL_4,
    .duty = 0, .target = 0, .actual = 0
};

motor_t flywheel2 = {
    .htim_forward = &htim9, .channel_forward = TIM_CHANNEL_1,
    .htim_reverse = &htim9, .channel_reverse = TIM_CHANNEL_2,
    .duty = 0, .target = 0, .actual = 0
};

motor_t turret1 = {
    .htim_forward = &htim1, .channel_forward = TIM_CHANNEL_2,
    .htim_reverse = &htim1, .channel_reverse = TIM_CHANNEL_3,
    .duty = 0, .target = 0, .actual = 0
};

motor_t turret2 = {
    .htim_forward = &htim10, .channel_forward = TIM_CHANNEL_1,
    .htim_reverse = &htim11, .channel_reverse = TIM_CHANNEL_1,
    .duty = 0, .target = 0, .actual = 0
};

// Define FSM states
typedef enum {
    STATE_0,
    STATE_1,
	STATE_2,
	STATE_3,
	STATE_4,
	STATE_5,
	STATE_6
} State_t;
State_t current_state = STATE_0;

uint8_t auto_mode = 0;
uint8_t auto_mot = 0;
uint8_t auto_lidar = 0;
uint16_t scan_index = 0;

uint32_t turret_move_start = 0;
uint16_t min_scan_dist = 0xFFFF;
uint16_t min_scan_index = 0;
uint16_t scan_res = 20;
int scan_dist[20];
int scan_amp[20];

#define RX_BUFFER_SIZE 7
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
char msg_int[64];
char msg_main[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// === Servo Software PWM ===
void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint16_t servo_angle = 140;  // global control variable
uint16_t servo_step = 0;
uint32_t servo_step_time = 0;

void servo_pwm_update(void) {
    static enum { IDLE, HIGH, LOW } state	 = IDLE;
    static uint32_t pulse_start_us = 0;
    static uint32_t period_start_us = 0;
    uint32_t now_us = DWT->CYCCNT / (SystemCoreClock / 1000000);

    uint32_t pulse_width_us = 1000 + ((uint32_t)servo_angle * 1000) / 180;

    switch (state) {
        case IDLE:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
            pulse_start_us = now_us;
            period_start_us = now_us;
            state = HIGH;
            break;
        case HIGH:
            if ((now_us - pulse_start_us) >= pulse_width_us) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
                state = LOW;
            }
            break;
        case LOW:
            if ((now_us - period_start_us) >= 20000) {
                state = IDLE;
            }
            break;
    }
}
/* USER CODE END 0 */

/**
  * @brief Main entry point — configures peripherals and enters control loop
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Force TF_Luna into I2C mode
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  // Drive IMU RESET PIN high
  HAL_Delay(100);  // Optional but useful
  dwt_init();            // For accurate microsecond delay
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // Initialize servo pin low
  HAL_Delay(100);

  // LUNA LIDAR I2C Check
  if (HAL_I2C_IsDeviceReady(&hi2c2, 0x10 << 1, 3, 100) == HAL_OK) {
      HAL_UART_Transmit(&huart1, (uint8_t*)"TF-Luna ready!\r\n", 17, HAL_MAX_DELAY);
  } else {
      HAL_UART_Transmit(&huart1, (uint8_t*)"TF-Luna NOT responding\r\n", 25, HAL_MAX_DELAY);
  }

  // BNO055 IMU I2C Check
  // #define BNO055_ADDR (0x29 << 1)  // 7-bit address shifted for STM32 HAL

  if (HAL_I2C_IsDeviceReady(&hi2c1, BNO055_ADDR, 3, 100) == HAL_OK) {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"BNO055 ready!\r\n", 16, HAL_MAX_DELAY);
  } else {
   	  HAL_UART_Transmit(&huart1, (uint8_t*)"BNO055 NOT responding\r\n", 25, HAL_MAX_DELAY);
  }

  // IMU Init
  BNO055_Init(&hi2c1);
  //BNO055_StartCalibration();

  //uint8_t sys, gyr, acc, mag;
  //BNO055_GetCalibStatus(&sys, &gyr, &acc, &mag);
  //int len = sprintf(msg_main, "Calib SYS:%d GYR:%d ACC:%d MAG:%d\r\n", sys, gyr, acc, mag);
  //HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
  HAL_Delay(100);
  int len = sprintf(msg_main, "Ping Pong Bot Starting...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
  HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int32_t turret1_enc_last = -1;
  int16_t turret2_enc_last = -1;
  uint8_t printed = 0;
  float heading, roll, pitch;
  int32_t imu_head_deg10 = 0;
  int32_t imu_roll_deg10 = 0;
  HAL_Delay(1000);
  //FSM_Init();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//uint8_t mode = 0x00;
		//HAL_I2C_Mem_Write(&hi2c1, (0x29 << 1), 0x3D, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);
		//BNO055_ReadEuler(&heading,&roll,&pitch);
		//imu_head_deg10 = (int32_t)(heading*10.0f);
		//imu_roll_deg10 = (int32_t)(roll*10.0f);
		//HAL_Delay(20);
	    if (imu_mode) {
	        //float heading, roll, pitch;
	        //BNO055_ReadEuler(&heading, &roll, &pitch);

	        //int len = sprintf(msg_main, "H: %.1f R: %.1f P: %.1f\r\n", heading / 16.0f, roll / 16.0f, pitch / 16.0f);
	        int len = sprintf(msg_main, "H: %.1f R: %.1f P: %.1f\r\n", heading, roll, pitch);
	    	HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
	        len = sprintf(msg_main, "H: %d R: %d\r\n",imu_head_deg10,imu_roll_deg10);
	    	HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);

	        HAL_Delay(500);  // Limit update rate
	    }

	  // ENCODERS
	  turret1_enc_count = __HAL_TIM_GET_COUNTER(&htim2);
	  if (turret1_enc_count != turret1_enc_last) {
	      turret1_enc_deg = (7200 * turret1_enc_count) / (3200*10); // 7200 = 360.0 deg × 20 (tenths)
 	      turret1_enc_last = turret1_enc_count;
	  }
	  int16_t turret2_enc_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	  int16_t delta = turret2_enc_now - turret2_enc_last;
      turret2_enc_last = turret2_enc_now;
      turret2_enc_pos += delta;
      turret2_enc_deg = (7200 * turret2_enc_pos) / (3200*10); // degrees × 10
      //turret1_enc_deg = imu_roll_deg10;
      // SOFTWARE PWM
      servo_pwm_update();
      // FINITE-STATE-MACHINE
      switch(current_state)
      {
      	  //int len;
      	  case STATE_0: // Initialize
      		  turret1_target = -150;
      		  turret2_target = 30;
				if (turret_move_start == 0) {
					turret_move_start = HAL_GetTick();
				}
      		  //turret1_target = 0;
      		  //turret2_target = 0;
      		  move_to(&turret1, turret1_target, turret1_enc_deg);
      		  move_to(&turret2, turret2_target, turret2_enc_deg);
      		  //if (HAL_GetTick() - turret_move_start >= 2000)
      		  if (abs(turret1_target - turret1_enc_deg) < 5 && abs(turret2_target - turret2_enc_deg) < 5)
      		  {
      		      turret_move_start = 0;
      		      //enable(&turret2);
      			  current_state = STATE_1;
      		      int len = sprintf(msg_main, "Motors Initialized\r\n");
    	          HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
      		  }
      		  break;
      	  case STATE_1: // Hub
      		  if (!printed) {
      			  int len = sprintf(msg_main, "Waiting Command...\r\n");
    	          HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
      			  printed = 1;
      		  }
      		  break;
			case STATE_2: // Move turret
			{
				// Start timing on first entry into this state
				if (turret_move_start == 0) {
					turret_move_start = HAL_GetTick();
				}
					move_to(&turret1, turret1_target, turret1_enc_deg);
					move_to(&turret2, turret2_target, turret2_enc_deg);

					// Wait 1 second before considering the move "complete"
					if (abs(turret1_target - turret1_enc_deg) < 5 && abs(turret2_target - turret2_enc_deg) < 5){
					//if (HAL_GetTick() - turret_move_start >= 1000) {
						turret_move_start = 0;  // Reset for next use
						enable(&turret1);
						enable(&turret2);
						//int len = sprintf(msg_main, "Turret Move Timed Out\r\n");
						//HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
						if (auto_mode == 0){
							current_state = STATE_1;
							printed = 0;
						}
						else {
							auto_mot = 1;
							current_state = STATE_6;
						}
					}
			}
			break;
      	  case STATE_3: // Lidar
      	  {
      		  uint8_t reg = 0x00;
      	      uint16_t dist = 0;
      	      uint16_t amp = 0;
      	      if (HAL_I2C_Master_Transmit(&hi2c2, 0x10 << 1, &reg, 1, 10) == HAL_OK &&
      	      	  HAL_I2C_Master_Receive(&hi2c2, 0x10 << 1, lidar_buf, 4, 10) == HAL_OK) {
      	      	  dist = lidar_buf[0] | (lidar_buf[1] << 8);
      	      	  amp  = lidar_buf[2] | (lidar_buf[3] << 8);
      	      } else {

      	      }
	          if (auto_mode == 0){
	      	      int len = sprintf(msg_main, "Dist: %d cm, Amp: %d\r\n", dist, amp);
		          HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
		          current_state = STATE_1;
		          printed = 0;
	          }
	          else {
	      	      int len = sprintf(msg_main, "Auto Dist: %d cm, Amp: %d\r\n", dist, amp);
		          HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
		          scan_dist[scan_index] = dist;
		          scan_amp[scan_index] = amp;
		          if (dist < min_scan_dist && amp > 100) {
		              min_scan_dist = dist;
		              min_scan_index = scan_index;
		          }
		          current_state = STATE_6;
		          auto_lidar = 1;
	          }
      	  }
	      break;
      	  case STATE_4: // Flywheels
      	  {
      		  set_duty(&flywheel1, duty_flywheel1);
	          int len = sprintf(msg_main, "Flywheel Speed Set\r\n");
	          HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
      		  if (auto_mode == 0){
      			  current_state = STATE_1;
      			  printed = 0;
      		  }
      		  else {
      			  current_state = STATE_6;
      		  }
      	  }
          break;
      	  case STATE_5: // launch
      	  {
      	    //int len = sprintf(msg_main, "Servo state: %d\r\n", servo_step);
      	    //HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
      	    switch (servo_step) {
      	        case 0:
      	            servo_angle = 320;  // move back
      	            servo_step_time = HAL_GetTick();
      	            servo_step = 1;
      	            break;
      	        case 1:
      	            if (HAL_GetTick() - servo_step_time >= 300) {
      	                servo_angle = 140;  // move forward
      	                servo_step_time = HAL_GetTick();
      	                servo_step = 2;
      	            }
      	            break;
      	        case 2:
      	            if (HAL_GetTick() - servo_step_time >= 300) {
      	                servo_angle = 140;  // return to center
      	                if (auto_mode == 0){
      	                	current_state = STATE_1;
      	                	printed = 0;
      	                	servo_step = 0;
      	                }
      	                else {
      	                	current_state = STATE_6;
      	                	servo_step = 0;  // reset step
      	                }
      	            }
      	            break;
      	    }
      	}
      	break;
      	case STATE_6: // Auto scan
      	{
      		HAL_Delay(50); // Avoid console clogging
      		if (scan_index < 20){
      			if (auto_mot == 0){
      				turret2_target = (10*scan_index);
      				current_state = STATE_2;
            	    //int len = sprintf(msg_main, "Motor Movement %d \r\n", scan_index);
      	            //HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
      			}
      			else if (auto_lidar == 0){
      				current_state = STATE_3;
            	    //int len = sprintf(msg_main, "LIDAR Scan %d \r\n", scan_index);
      	            //HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
      			}
      			else if (auto_mot == 1 && auto_lidar == 1){
      				scan_index += 1;
      				auto_mot = 0;
      				auto_lidar = 0;
      			}
      		}
      		else if (scan_index == 20){
      		    // Reset scan for next time
      		    if (min_scan_dist > 450){
      		    	int len = sprintf(msg_main,"No object in range");
      		    	HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
      		    	scan_index = 24;
      		    }
      		    else {//scan_index = 0;
          		    len = sprintf(msg_main, "Scan complete. Closest at index %d, dist %d cm\r\n",
          		                      min_scan_index, min_scan_dist);
          		    HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
      		    	//min_scan_index = 0;
      		    	turret2_target = (10*min_scan_index);
      		    	//turret1_target = -180;
      		    	scan_index = 21;
      		    	current_state = STATE_2;
      		    }
      		}
      		else if (scan_index == 21){
      			if (min_scan_dist < 100){
      				duty_flywheel1 = 85;
      			}
      			else if (min_scan_dist < 250){
      				duty_flywheel1 = 90;
      			}
      			else {
      				duty_flywheel1 = 100;
      			}
      			current_state = STATE_4;
      			scan_index = 22;
      		}
      		else if (scan_index == 22){
      			current_state = STATE_5;
      			scan_index = 23;
      		}
      		else if (scan_index == 23){
      			duty_flywheel1 = 0;
      			current_state = 4;
      			scan_index = 24;
      		}
      		else if (scan_index == 24){
      			auto_mode = 0;
      			printed = 0;
      			scan_index = 0;
      			min_scan_dist = 0xFFFF;
      			min_scan_index = 0;
      			current_state = STATE_1;
      		}
      		else {
        	    int len = sprintf(msg_main, "BROKEN CODE");
        	   	HAL_UART_Transmit(&huart1, (uint8_t*)msg_main, len, HAL_MAX_DELAY);
        	   	scan_index = 0;
        	   	current_state = STATE_1;
      		}
        }
      	break;
      }
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2399;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 2399;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 2399;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint8_t received = rx_buffer[rx_index];

        if (received == '\r')  // End of command
        {
            // Enforce valid format: exactly 4 printable characters
            if (rx_index != 4 ||
                !isprint(rx_buffer[0]) ||
                !isprint(rx_buffer[1]) ||
                !isprint(rx_buffer[2]) ||
                !isprint(rx_buffer[3]))
            {
                int len = sprintf(msg_int, "Invalid command format\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
            }
            else if (current_state == STATE_1)
            {
                rx_buffer[rx_index] = '\0';  // Null-terminate

                // Echo for debugging
                int len = sprintf(msg_int, "RX: %s\r\n", rx_buffer);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);

                // ---------------- Motor Control ----------------
                if ((rx_buffer[0] == 'M' || rx_buffer[0] == 'm') &&
                    (rx_buffer[1] >= '1' && rx_buffer[1] <= '2') &&
                    isxdigit(rx_buffer[2]) && isxdigit(rx_buffer[3]))
                {
                    uint8_t motor_id = rx_buffer[1] - '0';
                    char hex_str[3] = { rx_buffer[2], rx_buffer[3], '\0' };
                    uint8_t raw = (uint8_t)strtol(hex_str, NULL, 16);
                    int8_t signed_val = *(int8_t*)&raw;
                    int32_t duty = signed_val;

                    if (duty > 100) duty = 100;
                    if (duty < -100) duty = -100;

                    if (motor_id == 1) duty_flywheel1 = duty;
                    else if (motor_id == 2) duty_flywheel2 = duty;

                    len = sprintf(msg_int, "Motor %d set to %ld%%\r\n", motor_id, duty);
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);

                    current_state = STATE_4;
                }

                // ---------------- Turret 1 Read ----------------
                else if (strncmp((char*)rx_buffer, "T1re", 4) == 0)
                {
                    int len = sprintf(msg_int, "Turret 1 angle: %ld.%01ld deg\r\n", turret1_enc_deg / 10, labs(turret1_enc_deg % 10));
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
                }

                // ---------------- Turret 1 Set ----------------
                else if (rx_buffer[0] == 'T' && rx_buffer[1] == '1' &&
                         isxdigit(rx_buffer[2]) && isxdigit(rx_buffer[3]))
                {
                    char hex_str[3] = { rx_buffer[2], rx_buffer[3], '\0' };
                    uint8_t raw = (uint8_t)strtol(hex_str, NULL, 16);
                    int8_t signed_val = *(int8_t*)&raw;
                    turret1_target = (int32_t)signed_val * 10;

                    int len = sprintf(msg_int, "Turret 1 target set to: %ld.%01ld deg\r\n", turret1_target / 10, labs(turret1_target % 10));
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);

                    current_state = STATE_2;
                }

                // ---------------- Turret 2 Read ----------------
                else if (strncmp((char*)rx_buffer, "T2re", 4) == 0)
                {
                    int len = sprintf(msg_int, "Turret 2 angle: %ld.%01ld deg\r\n", turret2_enc_deg / 10, labs(turret2_enc_deg % 10));
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
                }

                // ---------------- Turret 2 Set ----------------
                else if (rx_buffer[0] == 'T' && rx_buffer[1] == '2' &&
                         isxdigit(rx_buffer[2]) && isxdigit(rx_buffer[3]))
                {
                    char hex_str[3] = { rx_buffer[2], rx_buffer[3], '\0' };
                    uint8_t raw = (uint8_t)strtol(hex_str, NULL, 16);
                    int8_t signed_val = *(int8_t*)&raw;
                    turret2_target = (int32_t)signed_val * 10;

                    int len = sprintf(msg_int, "Turret 2 target set to: %ld.%01ld deg\r\n", turret2_target / 10, labs(turret2_target % 10));
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);

                    current_state = STATE_2;
                }

                // ---------------- LIDAR Start ----------------
                else if (strncmp((char*)rx_buffer, "LIDA", 4) == 0)
                {
                    lidar_mode = 1;
                    int len = sprintf(msg_int, "LIDAR streaming started.\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
                    current_state = STATE_3;
                }
                // SERVO LAUNCH
                else if (strncmp((char*)rx_buffer, "SRUN", 4) == 0)
                {
                	current_state = STATE_5;
                    int len = sprintf(msg_int, "Servo routine starting\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
                }
                else if (strncmp((char*)rx_buffer, "SCAN", 4) == 0)
                {
                	auto_mode = 1;
                	current_state = STATE_6;
                    int len = sprintf(msg_int, "Starting scan\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
                }
                else if (strncmp((char*)rx_buffer, "HEAD", 4) == 0) {
                    imu_mode = 1;
                    int len = sprintf(msg_int, "IMU heading stream started\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
                }
                else if (strncmp((char*)rx_buffer, "STOP", 4) == 0) {
                    imu_mode = 0;
                    int len = sprintf(msg_int, "Streaming stopped\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
                }
                // ---------------- LIDAR Stop ----------------
                /*else if (strncmp((char*)rx_buffer, "STOP", 4) == 0)
                {
                    lidar_mode = 0;
                    int len = sprintf(msg_int, "LIDAR streaming stopped.\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
                    current_state = STATE_1;
                }*/

                // ---------------- Unknown Command ----------------
                else
                {
                    int len = sprintf(msg_int, "Invalid command\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)msg_int, len, HAL_MAX_DELAY);
                }
            }

            // Always reset buffer after processing a command
            memset(rx_buffer, 0, RX_BUFFER_SIZE);
            rx_index = 0;
        }
        else
        {
            rx_index++;
            if (rx_index >= RX_BUFFER_SIZE)
                rx_index = 0;
        }

        // Re-arm UART reception
        HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
