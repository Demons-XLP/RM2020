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





/** 
	* @brief   重定义的位置环运算
	* @retval  电流值
	* @note   用于19工程3510电机无速度环的重定义函数
	* @par 日志 
	*       2018年10月31日17:13:46 改名，该运算由motor::CANSend()托管执行
	*       2018年11月3日11:32:35  加入了路程环校验环节
	*       2019.11.26   20：02 修改为无速度环
*/
void softmotor::Position_Run(void)
{
  if(PID_Out==NULL)while(1);//暴露错误，你没有外环还想用位置PID?
	int32_t err=0;
	err=TargetPosition-RealPosition;     //得到单圈误差
	err+=MotorType->max_mechanical_position*(Soft_TargetPosition-Soft_RealPosition);//加上圈数位置误差
	TargetCurrent = PID_Out->pid_run(err);//位置环得到目标电流
//	TargetCurrent = PID_In->pid_run(TargetSpeed-RealSpeed);
	InsertCurrent();
}

/** 
	* @brief  softmotor重定义的update
	* @retval  为了适应3510电机不同的反馈报文
	* @par 日志 
	*
*/
void softmotor::update(uint8_t Data[])
{
	LastSpeed = RealSpeed;
	LastPosition = RealPosition;
	LastAngle = RealAngle;
	LastCurrent = Current;//保存转矩电流
	RealPosition = Data[0]<<8 | Data[1];
	Current = Data[2]<<8 | Data[3];  //3510电机反馈电流
	if(Current != LastCurrent)//前后转矩电流不同才算作有效数据
		LastUpdateTime = HAL_GetTick();//更新本次电机数据最后更新的时间
	RealAngle = RealPosition*360.f/MotorType->max_mechanical_position;//根据机械角计算出的真实角度
	if(running_flag==0)
	{
		LastPosition=RealPosition;
		running_flag=1;
	}
	if(RealPosition	-	LastPosition	>	4096)//圈数累计
		Soft_RealPosition--;
	else if(RealPosition	-	LastPosition	<-4096)//圈数累计
		Soft_RealPosition++;
	RealAngle = /*圈数对应角度*/(Soft_RealPosition)/(1.0*MotorType->Reduction_ratio)*360 \
							+ /*单圈内角度*/1.0f*RealPosition / (8192 * MotorType->Reduction_ratio )*360;//转换为角度
}


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






	

