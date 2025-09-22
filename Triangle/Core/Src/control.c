#include "control.h"
#include <math.h>

#define PWM_PIN GPIO_PIN_7
#define PWM_PORT GPIOA
#define DIR_PIN GPIO_PIN_9
#define DIR_PORT GPIOB
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
extern TIM_HandleTypeDef htim3; // Timer for PWM

void motor_init(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Start PWM
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET); // Default direction
}

void set_motor_pwm(float control_signal) {
    uint16_t pwm_value = (uint16_t)(fabs(control_signal));
    if (pwm_value > 1000) pwm_value = 1000; // limit PWM
    
    if (control_signal >= 0)
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
}

void control(float theta, float theta_dot, float phi, float phi_dot , float ay, float az){

    float M = 0.366 ;
    float g = sqrt(pow(ay,2)+pow(az,2));
    if (theta < M_PI/180.0f * 5.0f){

    }

}