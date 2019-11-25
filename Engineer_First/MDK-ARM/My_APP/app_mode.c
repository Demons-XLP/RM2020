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

#include "app_mode.h"
#include "bsp_can.hpp"
#include <string.h>
#include "can.h"
#include "app_chassis.h"
#include "bsp_dbus.h"


int16_t	Vx;   
int16_t Vy;
int16_t	omegaYaw;	
//�����ĸ�3508���
Motor_t DJI_Motor_3508(8192, 19);  //�������
pid PID_Chassis_Speed(20,0.1f,0,1000,16000,0,100);  //���̵��PID
chassis Chassis_Engineer(1,0x201,&DJI_Motor_3508,&PID_Chassis_Speed);  //�������������

//��̨����GM3510���,������ģʽ�����ڻ�
Motor_t DJI_Motor_3510(8192, 1);  //�������
//pid PID_Cloud_Pitch_G_In(1,0,0,0,0,0,0);  //������ģʽ�ڻ�
//pid PID_Cloud_Pitch_G_Out(1,0,0,0,0,0,0);  //������ģʽ�⻷
////pid PID_Cloud_Pitch_E_In(0,0,0,0,0,0,0);  //������ģʽ�ڻ�
//pid PID_Cloud_Pitch_E_Out(3,0,0,0,0,0,0);  //������ģʽ�⻷
//pid PID_Cloud_Yaw_G_In(0,0,0,0,0,0,0);  //������ģʽ�ڻ�
//pid PID_Cloud_Yaw_G_Out(0,0,0,0,0,0,0);  //������ģʽ�⻷
////pid PID_Cloud_Yaw_E_In(10,0,0,0,0,0,0);  //������ģʽ�ڻ�
//pid PID_Cloud_Yaw_E_Out(3,0,0,0,0,0,0);  //������ģʽ�⻷
//float Pitch_G_Angle,Pitch_G_Angle_Rate,Yaw_G_Angle,Yaw_G_Angle_Rate;  //������ģʽ�µ������ǽǶ�����ٶȣ�������·�����ٶ�
//softcloud Cloud_Engineer_Pitch(2,0x207,22,&DJI_Motor_3510,NULL,&PID_Cloud_Pitch_E_Out,&PID_Cloud_Pitch_G_In,&PID_Cloud_Pitch_G_Out,&Pitch_G_Angle_Rate,&Pitch_G_Angle);  //��̨Pitch��
//softcloud Cloud_Engineer_Yaw(2,0x206,5,&DJI_Motor_3510,NULL,&PID_Cloud_Yaw_E_Out,&PID_Cloud_Yaw_G_In,&PID_Cloud_Yaw_G_Out,&Yaw_G_Angle_Rate,&Yaw_G_Angle);  //��̨Yaw��




///** 
//	* @brief   �޲�����λ�û�����
//	* @retval  ����ֵ
//	* @note    ���¶������̨��λ�û����к���
//	*          �ú�����д���ຯ������cansendѭ�����ò��ɸ���
//	* @par ��־
//*/

//void softcloud::Position_Run(void)
//{
//	if(PID_Out==NULL)while(1);//��¶������û���⻷������λ��PID?
//	int32_t err=0;
//	err = (TargetAngle-RealAngle)*MotorType->max_mechanical_position/360;     //�õ����
//	//err=TargetPosition-RealPosition;     //�õ���Ȧ���
//	//err+=MotorType->max_mechanical_position*(Soft_TargetPosition-Soft_RealPosition);//����Ȧ��λ�����
//	if(PID_Out->ap==0 && PID_Out->bp==0)               //�⻷pidΪ��ͳpid
//		TargetCurrent=PID_Out->pid_run(err);               //��ͳpid����õ�Ŀ���ٶ�
//	else TargetCurrent = PID_Out->nonlinear_pid_run(err);//������pid�����õ�Ŀ���ٶ�
////	if(PID_In->ap==0 && PID_In->bp==0)                 //�ڻ�pidΪ��ͳpid
////		TargetCurrent=PID_In->pid_run(TargetSpeed-RealSpeed);                //��ͳpid����õ�Ŀ�����
////	else TargetCurrent = PID_In->nonlinear_pid_run(TargetSpeed-RealSpeed); //������pid�����õ�Ŀ�����
//	InsertCurrent();
//}

/** 
	* @brief  ���µ��ֵ ��������·�̵Ľ���
	* @retval  Ϊ���ܹ�ʹ�õ���PID����̨�ض������·�̽���
	* @par ��־ 
	*
*/




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

  if (s1 ==2 && s2 == 2)   //��ȫģʽ
	{ 
		Chassis_Engineer.Safe();  //����
//		Cloud_Engineer_Pitch.Safe_Set();  //��̨Pitch
//		Cloud_Engineer_Yaw.Safe_Set(); //��̨Yaw
		
	}
	else if(s1 ==3 && s2 == 2)  //���̸��棬��е��
 {
//	 Cloud_Engineer_Pitch.Pid_Select(NULL,&PID_Cloud_Pitch_E_Out);  //��е�ǵ���Pitch
//	 Cloud_Engineer_Yaw.Pid_Select(NULL,&PID_Cloud_Yaw_E_Out);  //��е�ǵ���Yaw
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
* @brief  ����
* @details  
* @param  NULL
* @retval  NULL
*  2019.10.24 ���
*
*/


//void Mode_Air_Choose(int s1,int s2)
//{
//  if (RC_Ctl.rc.s1 ==3 && RC_Ctl.rc.s2 == 3)
//	{
//		HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
//	  HAL_GPIO_WritePin(GPIOB,Air__Cylinder1_Pin,GPIO_PIN_SET);  //����1����(ȫ��)ǰ��̧��
//	}
//	else if (RC_Ctl.rc.s1 ==3 && RC_Ctl.rc.s2 == 1)
//	{
//	  HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
//	  HAL_GPIO_WritePin(GPIOB,Air_Cylinder2_Pin,GPIO_PIN_SET);  //����2�������������ϣ�צ��
//	}
//		else if (RC_Ctl.rc.s1 ==1 && RC_Ctl.rc.s2 == 1)
//	{
//		HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
//	  HAL_GPIO_WritePin(GPIOB,Air_Cylinder3_Pin,GPIO_PIN_SET);  //����3������ȫ�ϣ����ֿ���
//	}
//	 if (RC_Ctl.rc.s1 ==1 && RC_Ctl.rc.s2 == 3)
//	{
//		HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  //����ȫ�أ�ȫ�£�
//	  HAL_GPIO_WritePin(GPIOB,Air_Cylinder4_Pin,GPIO_PIN_SET);  //����2�������������У�δ֪����
//	}
//		else if (RC_Ctl.rc.s1 ==2 && RC_Ctl.rc.s2 == 2)
//	{
//	  HAL_GPIO_WritePin(GPIOB, Air_Cylinder4_Pin|Air_Cylinder3_Pin|Air_Cylinder2_Pin|Air__Cylinder1_Pin, GPIO_PIN_RESET);  
//	}
//}




	

