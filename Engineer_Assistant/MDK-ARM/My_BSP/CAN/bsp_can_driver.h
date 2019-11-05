/** 
* @file         Can_Driver.hpp 
* @author      WMD
* @version  1.0
* @par Copyright (c):  
*       WMD 
* @par ÈÕÖ¾¼ûcpp
*/  
#ifndef __BSP_CAN_DRIVER_H
#define __BSP_CAN_DRIVER_H
#include "stm32f4xx_hal.h"


void CAN_Init_All(void);
HAL_StatusTypeDef CAN_SEND_PROTECT(CAN_HandleTypeDef *_hcan,int id);
HAL_StatusTypeDef can_send_msg(CAN_HandleTypeDef* _hcan, int id, int16_t* s16buff);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

typedef struct 
{
  float V;
	float Encoder;
	float Itrue;
	float Temper;
	int16_t id;
}Motor;

extern Motor motor_data[6];
extern int8_t send_buf[5];
//extern int16_t V,Encoder,Itrue,Temper;
#define USE_CAN1
#endif
