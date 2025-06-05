#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

// Motor object data structure
typedef struct {
    TIM_HandleTypeDef* htim;   // Pointer to timer instance
    uint32_t duty;             // Actual PWM compare value (0 to PWM_MAX)
    uint32_t channel_forward;  // Timer channel for forward PWM
    uint32_t channel_reverse;  // Timer channel for reverse PWM
} motor_t;

// Sets motor speed and direction using duty percentage (-100 to +100)
void set_duty(motor_t* p_mot, int32_t duty);
void disable(motor_t* p_mot);
void enable(motor_t* p_mot);

#endif /* INC_MOTOR_H_ */
