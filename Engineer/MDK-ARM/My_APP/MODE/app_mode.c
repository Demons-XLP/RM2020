/**
    ****************************(C) COPYRIGHT LittleDargon****************************
    * @file       app_mode.c
    * @brief      设置遥控器模式
    * @note
    * @history
    *  Version    Date            Author          Modification
    *  V1.0.0     2018-11-15      LittleDargon    1. 按照命名规范要求，实现了命名的规范性
    *  V2.0.0     2019-02-15      LittleDargon    2. 加入了云台电机专用PID算法
    *  v4.4.2	  2019-10-07      Kison He		  13.Clion Reformat. Code improved.
  	*
  	**************************(C) COPYRIGHT LittleDargon****************************
  	*/

#include "app_mode.h"
#include "bsp_can.h"
#include "my_bsp.h"
#include <string.h>
#include "can.h"
#include "my_app.h"

int16_t send_Claws[4];  //通过CAN向副控发送的数组
int16_t send_buf[4];  //通过CAN向底盘电机发送的数组
/*
* @brief  遥控器模式函数
* @details  放在DBUS中断中使用
* @param  NULL
* @retval  NULL
*  2019.10.22 添加
*
*/
void Mode_Choose(int s1,int s2)
{
	memset(send_Claws,0,4*sizeof(int16_t));
  if (s1 ==2 && s2 == 2)   //安全模式
	{
		memset(send_buf,0,4*sizeof(int16_t));
		bsp_can_Sendmessage(&hcan1,0x200,send_buf);
		
	}
	else if(s1 ==3 && s2 == 2)
 {
			send_buf[0] =  RC_Ctl.rc.ch3*20 + RC_Ctl.rc.ch2*20 + RC_Ctl.rc.ch0*10;
			send_buf[1] = -RC_Ctl.rc.ch3*20 + RC_Ctl.rc.ch2*20 + RC_Ctl.rc.ch0*10;
			send_buf[2] =  RC_Ctl.rc.ch3*20 - RC_Ctl.rc.ch2*20 + RC_Ctl.rc.ch0*10;
			send_buf[3] = -RC_Ctl.rc.ch3*20 - RC_Ctl.rc.ch2*20 + RC_Ctl.rc.ch0*10;

//	 if(RC_Ctl.rc.thumbwheel < 0) //上推
//	 {
//	   //取弹函数开
//     send_Claws[0] = 1;
//	 }
//	 if(RC_Ctl.rc.thumbwheel >= 0)  //下推
//	 {
//	   //取弹函数关
//		 
//     send_Claws[0] = 0;
//	 }
	 int i = 0;
	 for (i=0;i<4;i++)
	 {
			PID_Control(&motor_V[i],send_buf[i],bsp_can_motor_data[i].V);
			send_buf[i] = motor_V[i].OUT;
		 	bsp_can_Sendmessage(&hcan1,0x200,send_buf);
	 }
	 
 }
// can_send_msg1(&hcan1,0x20B,Claws_send);
   bsp_can_Sendmessage(&hcan1,0x20B,send_Claws);
	
}

/*
* @brief  气缸
* @details  
* @param  NULL
* @retval  NULL
*  2019.10.24 添加
*
*/


void Mode_Air_Choose(int s1,int s2)
{
  if (RC_Ctl.rc.s1 ==3 && RC_Ctl.rc.s2 == 3)
	{
		HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
	  HAL_GPIO_WritePin(GPIOB,Air__Cylinder1_Pin,GPIO_PIN_SET);  //气缸1开启(全中)前轮抬升
	}
	else if (RC_Ctl.rc.s1 ==3 && RC_Ctl.rc.s2 == 1)
	{
	  HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
	  HAL_GPIO_WritePin(GPIOB,Air_Cylinder2_Pin,GPIO_PIN_SET);  //气缸2开启（左中右上）爪子
	}
		else if (RC_Ctl.rc.s1 ==1 && RC_Ctl.rc.s2 == 1)
	{
		HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
	  HAL_GPIO_WritePin(GPIOB,Air_Cylinder3_Pin,GPIO_PIN_SET);  //气缸3开启（全上）弹仓开启
	}
	 if (RC_Ctl.rc.s1 ==1 && RC_Ctl.rc.s2 == 3)
	{
		HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  //气缸全关（全下）
	  HAL_GPIO_WritePin(GPIOB,Air_Cylinder4_Pin,GPIO_PIN_SET);  //气缸2开启（左上右中）未知作用
	}
		else if (RC_Ctl.rc.s1 ==2 && RC_Ctl.rc.s2 == 2)
	{
	  HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
	}
}




	

