#ifndef __APP_MODE_H
#define __APP_MODE_H
#include "stm32f4xx_hal.h"

#define encoder_to_angle  360/8192
#define Claw_Open1 HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port, Air_Cylinder1_Pin, GPIO_PIN_RESET);  //�е�ҩ���צ��
#define Claw_Close1 HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port, Air_Cylinder1_Pin, GPIO_PIN_SET);
#define Claw_Open2 HAL_GPIO_WritePin(Air_Cylinder_GPIO_Port, Air_Cylinder_Pin, GPIO_PIN_RESET);  //����ȥ��ȡ��צ��
#define Claw_Close2 HAL_GPIO_WritePin(Air_Cylinder_GPIO_Port, Air_Cylinder_Pin, GPIO_PIN_SET);

void Mode_Choose(int s1,int s2);

void Air_Cylinder_Ctr(uint8_t flag1,uint8_t flag2);
extern uint8_t Air_Flag1; //����1flag,0�أ�1��
extern uint8_t Air_Flag2; //����2flag,0�գ�1��

#endif

