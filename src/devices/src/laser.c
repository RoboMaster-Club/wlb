#include "laser.h"
#include "tim.h"

void Laser_On()
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
}

void Laser_Off()
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}
