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


int16_t	Vx;   
int16_t Vy;
int16_t	omegaYaw;	
//底盘四个3508电机
Motor_t DJI_Motor_3508(8192, 19);  //电机类型
pid PID_Chassis_Speed(20,0.1f,130,1000,16000,0,100);  //底盘电机PID
pid PID_Chassis_Follow();  //底盘跟随PID
chassis Chassis_Engineer(1,0x201,&DJI_Motor_3508,&PID_Chassis_Speed);  //创建底盘类对象

//云台两个GM3510电机,编码器模式下无内环
Motor_t DJI_Motor_3510(8192, 1);  //电机类型
//pid PID_Cloud_Pitch_G_In(1,0,0,0,0,0,0);  //陀螺仪模式内环
//pid PID_Cloud_Pitch_G_Out(1,0,0,0,0,0,0);  //陀螺仪模式外环
pid PID_Cloud_Pitch_E_Out(15,0.5,0,1000,20000,0,15);  //编码器模式外环
//pid PID_Cloud_Yaw_G_In(0,0,0,0,0,0,0);  //陀螺仪模式内环
//pid PID_Cloud_Yaw_G_Out(0,0,0,0,0,0,0);  //陀螺仪模式外环
pid PID_Cloud_Yaw_E_Out(25,0.5,20,3000,29000,7,120);  //编码器模式外环
//float Pitch_G_Angle,Pitch_G_Angle_Rate,Yaw_G_Angle,Yaw_G_Angle_Rate;  //陀螺仪模式下的陀螺仪角度与角速度，类似于路程与速度
softmotor Cloud_Engineer_Pitch(2,0x207,&DJI_Motor_3510,NULL,&PID_Cloud_Pitch_E_Out);
softmotor Cloud_Engineer_Yaw(2,0x206,&DJI_Motor_3510,NULL,&PID_Cloud_Yaw_E_Out);






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
		Cloud_Engineer_Pitch.Safe_Set();  //云台Pitch
		Cloud_Engineer_Yaw.Safe_Set(); //云台Yaw
		
	}
	else if(s1 ==3 && s2 == 2)  //底盘跟随，机械角
 {
	 static float Cloud_Pitch_Zero = 24.35 ,Cloud_Yaw_Zero = 158.73f;  //定义云台零点位置
	 Cloud_Yaw_Zero += bsp_dbus_Data.CH_2*0.002;
	 Cloud_Pitch_Zero += bsp_dbus_Data.CH_1*0.0004;
	 Cloud_Engineer_Pitch.Angle_Set(Cloud_Pitch_Zero);
	 Cloud_Engineer_Yaw.Angle_Set(Cloud_Yaw_Zero);
	 	Vx = (bsp_dbus_Data.CH_3) * 14; 
    Vy = (bsp_dbus_Data.CH_2) * 14;
		omegaYaw = (bsp_dbus_Data.CH_0)*4;
		Chassis_Engineer.Run(Vx,Vy,omegaYaw);
	 
 }

	
}






	

