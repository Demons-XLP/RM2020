/**
    ****************************(C) COPYRIGHT LittleDargon****************************
    * @file       app_mode.c
    * @brief      ����ң����ģʽ
    * @note
    * @history
    *  Version    Date            Author          Modification
    *  V1.0.0     2018-11-15      LittleDargon    1. ���������淶Ҫ��ʵ���������Ĺ淶��
    *  V2.0.0     2019-02-15      LittleDargon    2. ��������̨���ר��PID�㷨
    *  v4.4.2	  2019-10-07      Kison He		  13.Clion Reformat. Code improved.
  	*
  	**************************(C) COPYRIGHT LittleDargon****************************
  	*/

/*���ļ���Ϊ����ץȡ�������ԣ���ʱ��ǵ��޸��븱�صĹ�����Ӧ*/

#include "app_mode.h"
#include "bsp_can.hpp"
#include <string.h>
#include "can.h"
#include "car_driver.hpp"
#include "bsp_dbus.h"
#include "cmsis_os.h"

float EXP_L,EXP_R;
float EXP_L_OUT,EXP_R_OUT;
float StartAngle_L;
float StartAngle_R;
uint8_t flag = 0;  //��ʼ��һ���õ�
uint8_t Air_Flag = 0; //����flag,0����1��
pid pid_Claw_IN3(15,1,0,1000,16000,0,50);
pid pid_Claw_OUT3(0.12,0,0,0,10000,0,0);
//pid pid_Claw_IN4(0,0,0,0,0,0,0);
//pid pid_Claw_OUT4(0,0,0,0,0,0,0);
Motor_t DJI_Motor_3508(8192, 19);
softmotor left_motor(1,0x203,&DJI_Motor_3508,&pid_Claw_IN3,&pid_Claw_OUT3);
softmotor right_motor(1,0x204,&DJI_Motor_3508,&pid_Claw_IN3,&pid_Claw_OUT3);

/*
* @brief  ���׿���flag
* @details  
* @param  NULL
* @retval  NULL
*  2019.10.24 ���
*
*/
//static void Air_Cylinder_Ctr(uint8_t flag)
//{
// if(flag == 0)
// {	 
//	 Claw_Open;  
// }
// else if(flag == 1)
// {
//	 Claw_Close;
// }
//}


/*
* @brief  �ض����������
* @details 
* @param  NULL
* @retval  NULL
*  2019.11.20 ���
*
*/
//void MainTask(void const * argument)
//{
//	 
// 	manager::CANSelect(&hcan1,NULL);
//  bsp_can_Init();
//	DBUS_Init();
//	for(;;)
//	{
//		static TickType_t xLastWakeTime;
//	  Mode_Choose(RC_Ctl.rc.s1,RC_Ctl.rc.s2);
//		Air_Cylinder_Ctr(Air_Flag);
//		manager::CANSend();
//		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS);
// 	}
//}




/*
* @brief  ң����ģʽ����
* @details  ����DBUS�ж���ʹ��
* @param  NULL
* @retval  NULL
*  2019.10.22 ���
*
*/

void Mode_Choose(int s1,int s2)
{
  
	if(flag == 0)
	{
		osDelay(1);
	  StartAngle_L = left_motor.RealAngle;
		StartAngle_R = right_motor.RealAngle;
		flag = 1;
	}
  if (s1 ==3 && s2 == 2)   //��ȫģʽ
	{
		Air_Flag = 0;  //�ſ�
		left_motor.Safe_Set();
		right_motor.Safe_Set();
		
	}
 if(s1 ==3 && s2 == 3)  //��
 {
		right_motor.Angle_Set(StartAngle_R + 24.42f);

 }
	 if(s1 ==3 && s2 == 1)  //�� 
	 {
		 
		 right_motor.Angle_Set(StartAngle_R + 177.37f);
		 if(ABS(right_motor.RealAngle - (StartAngle_R + 177.37f)) <= 120)
		 {
		   Air_Flag = 0;  //�ſ�
			 if(ABS(right_motor.RealAngle - (StartAngle_R + 177.37f)) <= 5)	Air_Flag = 1;  //����
			 
		 }
			 
	 }
 
 
	
}


/** 
* @brief  CAN�����ϵ��Զ������ݴ�����
* @par ��־ 
*       ����PID����ִ���괦��
*       2019��3��8��16:12:26 wmd �ú���������
*/
void manager::UserProcess(void)
{
	CAN1CurrentList[2]=-CAN1CurrentList[3];  //��idΪ3�ĵ����ֵ����idΪ4�ĵ��
}










	

