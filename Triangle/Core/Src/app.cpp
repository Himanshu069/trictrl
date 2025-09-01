#include "app.h"

#include "stm32f1xx_hal.h"

void setup()
{
    // executes once
}

void loop()
{
    // executes continuesly
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(100);
}