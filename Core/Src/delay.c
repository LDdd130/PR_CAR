#include "delay.h"
#include "tim.h"

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim11, 0);
    while ((__HAL_TIM_GET_COUNTER(&htim11)) < us);
}