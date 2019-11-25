#ifndef __APP_MODE_H
#define __APP_MODE_H
#include "stm32f4xx_hal.h"

#define encoder_to_angle  360/8192
#define Claw_Open HAL_GPIO_WritePin(GPIOC, Air_Cylinder1_Pin, GPIO_PIN_RESET);
#define Claw_Close HAL_GPIO_WritePin(GPIOC, Air_Cylinder1_Pin, GPIO_PIN_SET);
void Mode_Choose(int s1,int s2);


#endif

