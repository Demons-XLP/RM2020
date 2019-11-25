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
pid PID_Chassis_Speed(20,0.1f,0,1000,16000,0,100);  //底盘电机PID
chassis Chassis_Engineer(1,0x201,&DJI_Motor_3508,&PID_Chassis_Speed);  //创建底盘类对象

//云台两个GM3510电机,编码器模式下无内环
Motor_t DJI_Motor_3510(8192, 1);  //电机类型
//pid PID_Cloud_Pitch_G_In(1,0,0,0,0,0,0);  //陀螺仪模式内环
//pid PID_Cloud_Pitch_G_Out(1,0,0,0,0,0,0);  //陀螺仪模式外环
////pid PID_Cloud_Pitch_E_In(0,0,0,0,0,0,0);  //编码器模式内环
//pid PID_Cloud_Pitch_E_Out(3,0,0,0,0,0,0);  //编码器模式外环
//pid PID_Cloud_Yaw_G_In(0,0,0,0,0,0,0);  //陀螺仪模式内环
//pid PID_Cloud_Yaw_G_Out(0,0,0,0,0,0,0);  //陀螺仪模式外环
////pid PID_Cloud_Yaw_E_In(10,0,0,0,0,0,0);  //编码器模式内环
//pid PID_Cloud_Yaw_E_Out(3,0,0,0,0,0,0);  //编码器模式外环
//float Pitch_G_Angle,Pitch_G_Angle_Rate,Yaw_G_Angle,Yaw_G_Angle_Rate;  //陀螺仪模式下的陀螺仪角度与角速度，类似于路程与速度
//softcloud Cloud_Engineer_Pitch(2,0x207,22,&DJI_Motor_3510,NULL,&PID_Cloud_Pitch_E_Out,&PID_Cloud_Pitch_G_In,&PID_Cloud_Pitch_G_Out,&Pitch_G_Angle_Rate,&Pitch_G_Angle);  //云台Pitch类
//softcloud Cloud_Engineer_Yaw(2,0x206,5,&DJI_Motor_3510,NULL,&PID_Cloud_Yaw_E_Out,&PID_Cloud_Yaw_G_In,&PID_Cloud_Yaw_G_Out,&Yaw_G_Angle_Rate,&Yaw_G_Angle);  //云台Yaw类




///** 
//	* @brief   无参数的位置环运算
//	* @retval  电流值
//	* @note    重新定义的云台类位置环运行函数
//	*          该函数重写父类函数，在cansend循环调用不可改名
//	* @par 日志
//*/

//void softcloud::Position_Run(void)
//{
//	if(PID_Out==NULL)while(1);//暴露错误，你没有外环还想用位置PID?
//	int32_t err=0;
//	err = (TargetAngle-RealAngle)*MotorType->max_mechanical_position/360;     //得到误差
//	//err=TargetPosition-RealPosition;     //得到单圈误差
//	//err+=MotorType->max_mechanical_position*(Soft_TargetPosition-Soft_RealPosition);//加上圈数位置误差
//	if(PID_Out->ap==0 && PID_Out->bp==0)               //外环pid为传统pid
//		TargetCurrent=PID_Out->pid_run(err);               //传统pid计算得到目标速度
//	else TargetCurrent = PID_Out->nonlinear_pid_run(err);//非线性pid计算后得到目标速度
////	if(PID_In->ap==0 && PID_In->bp==0)                 //内环pid为传统pid
////		TargetCurrent=PID_In->pid_run(TargetSpeed-RealSpeed);                //传统pid计算得到目标电流
////	else TargetCurrent = PID_In->nonlinear_pid_run(TargetSpeed-RealSpeed); //非线性pid计算后得到目标电流
//	InsertCurrent();
//}

/** 
	* @brief  更新电机值 包括了软路程的解算
	* @retval  为了能够使用单环PID，云台重定义的软路程解算
	* @par 日志 
	*
*/




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

  if (s1 ==2 && s2 == 2)   //安全模式
	{ 
		Chassis_Engineer.Safe();  //底盘
//		Cloud_Engineer_Pitch.Safe_Set();  //云台Pitch
//		Cloud_Engineer_Yaw.Safe_Set(); //云台Yaw
		
	}
	else if(s1 ==3 && s2 == 2)  //底盘跟随，机械角
 {
//	 Cloud_Engineer_Pitch.Pid_Select(NULL,&PID_Cloud_Pitch_E_Out);  //机械角调节Pitch
//	 Cloud_Engineer_Yaw.Pid_Select(NULL,&PID_Cloud_Yaw_E_Out);  //机械角调节Yaw
//	 Cloud_Engineer_Pitch.Angle_Set(24.35f);
//	 Cloud_Engineer_Yaw.Angle_Set(158.73f);
//	 Cloud_Engineer_Pitch
//	 	Vx = (bsp_dbus_Data.CH_3) * 14; 
//    Vy = (bsp_dbus_Data.CH_2) * 14;
//		omegaYaw = (bsp_dbus_Data.CH_0)*4;
//		Chassis_Engineer.Run(Vx,Vy,omegaYaw);
 }

	
}

/*
* @brief  气缸
* @details  
* @param  NULL
* @retval  NULL
*  2019.10.24 添加
*
*/


//void Mode_Air_Choose(int s1,int s2)
//{
//  if (RC_Ctl.rc.s1 ==3 && RC_Ctl.rc.s2 == 3)
//	{
//		HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
//	  HAL_GPIO_WritePin(GPIOB,Air__Cylinder1_Pin,GPIO_PIN_SET);  //气缸1开启(全中)前轮抬升
//	}
//	else if (RC_Ctl.rc.s1 ==3 && RC_Ctl.rc.s2 == 1)
//	{
//	  HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
//	  HAL_GPIO_WritePin(GPIOB,Air_Cylinder2_Pin,GPIO_PIN_SET);  //气缸2开启（左中右上）爪子
//	}
//		else if (RC_Ctl.rc.s1 ==1 && RC_Ctl.rc.s2 == 1)
//	{
//		HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
//	  HAL_GPIO_WritePin(GPIOB,Air_Cylinder3_Pin,GPIO_PIN_SET);  //气缸3开启（全上）弹仓开启
//	}
//	 if (RC_Ctl.rc.s1 ==1 && RC_Ctl.rc.s2 == 3)
//	{
//		HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  //气缸全关（全下）
//	  HAL_GPIO_WritePin(GPIOB,Air_Cylinder4_Pin,GPIO_PIN_SET);  //气缸2开启（左上右中）未知作用
//	}
//		else if (RC_Ctl.rc.s1 ==2 && RC_Ctl.rc.s2 == 2)
//	{
//	  HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
//	}
//}




	

