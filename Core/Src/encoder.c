#include "motor.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

#define PWM_MAX 2399

void set_duty(motor_t* p_mot, int32_t duty)
{
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;

    uint32_t pwm_val = PWM_MAX * abs(duty) / 100;
    p_mot->duty = pwm_val;

    if (duty > 0)
    {
        // Forward: PWM on IN1, LOW on IN2
        HAL_TIM_PWM_Start(p_mot->htim, p_mot->channel_forward);
        HAL_TIM_PWM_Stop(p_mot->htim, p_mot->channel_reverse);

        __HAL_TIM_SET_COMPARE(p_mot->htim, p_mot->channel_forward, pwm_val);
        __HAL_TIM_SET_COMPARE(p_mot->htim, p_mot->channel_reverse, 0); // Always low
    }
    else if (duty < 0)
    {
        // Reverse: PWM on IN2, LOW on IN1
        HAL_TIM_PWM_Stop(p_mot->htim, p_mot->channel_forward);
        HAL_TIM_PWM_Start(p_mot->htim, p_mot->channel_reverse);

        __HAL_TIM_SET_COMPARE(p_mot->htim, p_mot->channel_forward, 0); // Always low
        __HAL_TIM_SET_COMPARE(p_mot->htim, p_mot->channel_reverse, pwm_val);
    }
    else
    {
        // Coast: both low
        HAL_TIM_PWM_Stop(p_mot->htim, p_mot->channel_forward);
        HAL_TIM_PWM_Stop(p_mot->htim, p_mot->channel_reverse);

        __HAL_TIM_SET_COMPARE(p_mot->htim, p_mot->channel_forward, 0);
        __HAL_TIM_SET_COMPARE(p_mot->htim, p_mot->channel_reverse, 0);
    }
}

void disable(motor_t* p_mot)
{
    HAL_TIM_PWM_Stop(p_mot->htim, p_mot->channel_forward);
    HAL_TIM_PWM_Stop(p_mot->htim, p_mot->channel_reverse);
}

void enable(motor_t* p_mot)
{
    HAL_TIM_PWM_Start(p_mot->htim, p_mot->channel_forward);
    HAL_TIM_PWM_Start(p_mot->htim, p_mot->channel_reverse);
}
