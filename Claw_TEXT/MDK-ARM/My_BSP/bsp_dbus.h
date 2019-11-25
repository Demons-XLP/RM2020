#ifndef __BSP_DBUS_H
#define __BSP_DBUS_H
#include "stm32f4xx_hal.h"

#define HDBUS huart3  


typedef struct
{
	struct
	{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;                
		int16_t ch3;
		uint8_t s1;
		uint8_t s2;
		int16_t thumbwheel;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z; 
		uint8_t press_l;
		uint8_t press_r;		
	}mouse;
	
	struct
	{
		uint16_t v;
		uint8_t W;
		uint8_t S;
		uint8_t A;
		uint8_t D;
		uint8_t Shift;
		uint8_t Ctrl;
		uint8_t Q;
		uint8_t E;
		uint8_t R;
		uint8_t F;
		uint8_t G;
		uint8_t Z;
		uint8_t X;
		uint8_t C;
		uint8_t V;
		uint8_t B;
	}key;
	
  uint8_t IsDbusOnline;  
	
}RC_Ctl_t;

extern RC_Ctl_t RC_Ctl;//处理后的遥控器控制量

void DBUS_Init(void);
void DBUS_IT(void);

#endif


