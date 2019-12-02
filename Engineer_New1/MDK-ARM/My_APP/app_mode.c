/**
    ****************************(C) COPYRIGHT LittleDargon****************************
    * @file       app_mode.c
    * @brief      ����ң����ģʽ
    * @note
    * @history
    *  Version    Date            Author          Modification
    *  V1.0.0     2019-11.28   TaoU Monkey    ��������
  	*
  	**************************(C) COPYRIGHT LittleDargon****************************
  	*/

#include "app_mode.h"
#include "bsp_can.hpp"
#include <string.h>
#include "can.h"
#include "app_chassis.h"
#include "bsp_dbus.h"


int16_t	Vx;   //ǰ��
int16_t Vy;  //����
int16_t	omegaYaw;	 //��ת
//�����ĸ�3508���
Motor_t DJI_Motor_3508(8192, 19);  //�������
pid PID_Chassis_Speed(5,0.1f,0,5000,5000,0,80);  //���̵��PID
pid PID_Chassis_Follow(1,0,0,0,0,0,0);  //���̸���PID
chassis Chassis_Engineer(1,0x201,&DJI_Motor_3508,&PID_Chassis_Speed,NULL);  //�������������


/*
* @brief  ң����ģʽ����
* @details ����ѭ����ʹ��
* @param  NULL
* @retval  NULL
*  2019.10.22 ���
*
*/
void Mode_Choose(int s1,int s2)
{

  if (s1 ==2 && s2 == 2)   //��ȫģʽ
	{ 
		Chassis_Engineer.Safe();  //����
	}
	else if(s1 ==3 && s2 == 2)  //���̸��棬��е��
 {
	 	Vx = (bsp_dbus_Data.CH_3) * 14; 
    Vy = (bsp_dbus_Data.CH_2) * 14;
		omegaYaw = (bsp_dbus_Data.CH_0)*4;
		Chassis_Engineer.Run(Vx,Vy,omegaYaw);
 }
 else if(s1 ==3 && s2 == 3)   //���̶���
 {
    Vx = (bsp_dbus_Data.CH_3) * 14; 
    Vy = (bsp_dbus_Data.CH_2) * (-14);
		omegaYaw = (bsp_dbus_Data.CH_0)*8;
		Chassis_Engineer.Run(Vx,Vy,omegaYaw);
 }
}


	

