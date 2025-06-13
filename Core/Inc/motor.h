#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

// Motor object data structure
typedef struct {
    TIM_HandleTypeDef* htim_forward;
    TIM_HandleTypeDef* htim_reverse;
    uint32_t channel_forward;
    uint32_t channel_reverse;
    int32_t duty;
    int32_t target;
    int32_t actual;
} motor_t;

// Sets motor speed and direction using duty percentage (-100 to +100)
void set_duty(motor_t* p_mot, int32_t duty);
void move_to(motor_t* p_mot, int32_t target, int32_t actual);
void disable(motor_t* p_mot);
void enable(motor_t* p_mot);

#endif /* INC_MOTOR_H_ */
