/**
 * @file motor.h
 * @brief Motor control driver for bidirectional PWM-based DC motors
 *
 * This module defines the motor_t struct and function prototypes for controlling
 * brushed DC motors (e.g., turret and flywheel motors) via dual PWM channels.
 * 
 * Used for:
 * - Turret aiming motors (Pololu 50:1 DC motors with encoders)
 * - Flywheel launcher motors (Maxon DC motors)
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
 * @brief Motor object structure for bidirectional PWM control
 */
typedef struct {
    TIM_HandleTypeDef* htim_forward;
    TIM_HandleTypeDef* htim_reverse;
    uint32_t channel_forward;
    uint32_t channel_reverse;
    int32_t duty;
    int32_t target;
    int32_t actual;
} motor_t;

/**
 * @brief Set motor duty cycle and direction
 * @param p_mot Pointer to motor object
 * @param duty Signed duty cycle (-100 to +100)
 */
void set_duty(motor_t* p_mot, int32_t duty);

/**
 * @brief Move motor toward target position using proportional control
 * @param p_mot Pointer to motor object
 * @param target Desired position in user units (e.g., degrees × 10)
 * @param actual Current position in user units
 */
void move_to(motor_t* p_mot, int32_t target, int32_t actual);

/**
 * @brief Disable motor output (coast both sides)
 * @param p_mot Pointer to motor object
 */
void disable(motor_t* p_mot);

/**
 * @brief Enable motor output with full PWM to both channels
 * @param p_mot Pointer to motor object
 */
void enable(motor_t* p_mot);

#endif /* INC_MOTOR_H_ */
