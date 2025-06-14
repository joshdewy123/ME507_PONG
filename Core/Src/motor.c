/**
 * @file motor.c
 * @brief Implementation of bidirectional PWM motor driver functions
 *
 * Provides PWM control for motors using two timer channels (forward and reverse).
 * Includes proportional control logic to move toward target angles for turret motors.
 */

#include "motor.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

#define PWM_MAX 2399


/**
 * @brief Set the motor's PWM output based on signed duty cycle
 *
 * Adjusts direction and magnitude using complementary PWM logic.
 * For turret1, logic is inverted to account for wiring or mechanical setup.
 */
void set_duty(motor_t* motor, int32_t duty)
{
    // Invert logic for turret1 using complementary outputs
    extern motor_t turret1;
    if (motor == &turret1)
    {
        if (duty > 0) duty = 100 - duty;
        else if (duty < 0) duty = -(100 + duty); // e.g. -30 → -70
    }

    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;

    uint32_t pwm_val_fwd = motor->htim_forward->Init.Period * abs(duty) / 100;
    uint32_t pwm_val_rev = motor->htim_reverse->Init.Period * abs(duty) / 100;
    motor->duty = duty;

    if (duty > 0)
    {
        // Forward: PWM on IN1, LOW on IN2
        HAL_TIM_PWM_Start(motor->htim_forward, motor->channel_forward);
        HAL_TIM_PWM_Stop(motor->htim_reverse, motor->channel_reverse);

        __HAL_TIM_SET_COMPARE(motor->htim_forward, motor->channel_forward, pwm_val_fwd);
        __HAL_TIM_SET_COMPARE(motor->htim_reverse, motor->channel_reverse, 0);
    }
    else if (duty < 0)
    {
        // Reverse: PWM on IN2, LOW on IN1
        HAL_TIM_PWM_Stop(motor->htim_forward, motor->channel_forward);
        HAL_TIM_PWM_Start(motor->htim_reverse, motor->channel_reverse);

        __HAL_TIM_SET_COMPARE(motor->htim_forward, motor->channel_forward, 0);
        __HAL_TIM_SET_COMPARE(motor->htim_reverse, motor->channel_reverse, pwm_val_rev);
    }
    else
    {
        // Coast: both low
        HAL_TIM_PWM_Stop(motor->htim_forward, motor->channel_forward);
        HAL_TIM_PWM_Stop(motor->htim_reverse, motor->channel_reverse);

        __HAL_TIM_SET_COMPARE(motor->htim_forward, motor->channel_forward, 0);
        __HAL_TIM_SET_COMPARE(motor->htim_reverse, motor->channel_reverse, 0);
    }
}

/**
 * @brief Move the motor toward a target using proportional control
 *
 * Uses a simple P-controller with anti-windup and deadband handling.
 */
void move_to(motor_t* p_mot, int32_t target, int32_t actual)
{
    int32_t error = target - actual;
    int32_t duty = 0;

    // Deadband threshold
    /*if (abs(error) < 2) {
        set_duty(p_mot, 0); // Stop the motor
        return;
    }*/

    // Proportional gain (tune as needed)
    float Kp = 0.9f;

    // Proportional control calculation
    duty = (int32_t)(Kp * error);

    // Apply minimum duty to overcome static friction
    if (duty > 0 && duty < 50)
        duty = 50;
    else if (duty < 0 && duty > -50)
        duty = -50;

    // Clamp to safe duty limits
    if (duty > 80) duty = 80;
    if (duty < -80) duty = -80;

    set_duty(p_mot, duty);
}

/**
 * @brief Disable PWM output (stop both channels)
 */
void disable(motor_t* p_mot)
{
    HAL_TIM_PWM_Stop(p_mot->htim_forward, p_mot->channel_forward);
    HAL_TIM_PWM_Stop(p_mot->htim_reverse, p_mot->channel_reverse);
}

/**
 * @brief Enable both channels to full PWM output
 */
void enable(motor_t* p_mot)
{
    HAL_TIM_PWM_Start(p_mot->htim_forward, p_mot->channel_forward);
    HAL_TIM_PWM_Start(p_mot->htim_reverse, p_mot->channel_reverse);
    __HAL_TIM_SET_COMPARE(p_mot->htim_forward, p_mot->channel_forward, PWM_MAX);
    __HAL_TIM_SET_COMPARE(p_mot->htim_reverse, p_mot->channel_reverse, PWM_MAX);
}
