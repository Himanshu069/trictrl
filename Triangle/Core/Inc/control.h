#ifndef CONTROL_H
#define CONtROL_H

#include "stm32f1xx_hal.h"

void motor_init(void);
void set_motor_pwm(float control_signal);

#endif
