/**
    ****************************(C) COPYRIGHT LittleDargon****************************
    * @file       app_mode.c
    * @brief      设置遥控器模式
    * @note
    * @history
    *  Version    Date            Author          Modification
    *  V1.0.0     2019-11.28   TaoU Monkey    测试所用
  	*
  	**************************(C) COPYRIGHT LittleDargon****************************
  	*/

#include "app_mode.h"
#include "bsp_can.hpp"
#include <string.h>
#include "can.h"
#include "app_chassis.h"
#include "bsp_dbus.h"


int16_t	Vx;   //前后
int16_t Vy;  //左右
int16_t	omegaYaw;	 //旋转
//底盘四个3508电机
Motor_t DJI_Motor_3508(8192, 19);  //电机类型
pid PID_Chassis_Speed(5,0.1f,0,5000,5000,0,80);  //底盘电机PID
pid PID_Chassis_Follow(1,0,0,0,0,0,0);  //底盘跟随PID
chassis Chassis_Engineer(1,0x201,&DJI_Motor_3508,&PID_Chassis_Speed,NULL);  //创建底盘类对象


/*
* @brief  遥控器模式函数
* @details 放在循环中使用
* @param  NULL
* @retval  NULL
*  2019.10.22 添加
*
*/
void Mode_Choose(int s1,int s2)
{

  if (s1 ==2 && s2 == 2)   //安全模式
	{ 
		Chassis_Engineer.Safe();  //底盘
	}
	else if(s1 ==3 && s2 == 2)  //底盘跟随，机械角
 {
	 	Vx = (bsp_dbus_Data.CH_3) * 14; 
    Vy = (bsp_dbus_Data.CH_2) * 14;
		omegaYaw = (bsp_dbus_Data.CH_0)*4;
		Chassis_Engineer.Run(Vx,Vy,omegaYaw);
 }
 else if(s1 ==3 && s2 == 3)   //底盘独立
 {
    Vx = (bsp_dbus_Data.CH_3) * 14; 
    Vy = (bsp_dbus_Data.CH_2) * (-14);
		omegaYaw = (bsp_dbus_Data.CH_0)*8;
		Chassis_Engineer.Run(Vx,Vy,omegaYaw);
 }
}


	

