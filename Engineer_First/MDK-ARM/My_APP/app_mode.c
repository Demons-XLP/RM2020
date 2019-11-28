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


int16_t	Vx;   
int16_t Vy;
int16_t	omegaYaw;	
//�����ĸ�3508���
Motor_t DJI_Motor_3508(8192, 19);  //�������
pid PID_Chassis_Speed(20,0.1f,130,1000,16000,0,100);  //���̵��PID
pid PID_Chassis_Follow();  //���̸���PID
chassis Chassis_Engineer(1,0x201,&DJI_Motor_3508,&PID_Chassis_Speed);  //�������������

//��̨����GM3510���,������ģʽ�����ڻ�
Motor_t DJI_Motor_3510(8192, 1);  //�������
//pid PID_Cloud_Pitch_G_In(1,0,0,0,0,0,0);  //������ģʽ�ڻ�
//pid PID_Cloud_Pitch_G_Out(1,0,0,0,0,0,0);  //������ģʽ�⻷
pid PID_Cloud_Pitch_E_Out(15,0.5,0,1000,20000,0,15);  //������ģʽ�⻷
//pid PID_Cloud_Yaw_G_In(0,0,0,0,0,0,0);  //������ģʽ�ڻ�
//pid PID_Cloud_Yaw_G_Out(0,0,0,0,0,0,0);  //������ģʽ�⻷
pid PID_Cloud_Yaw_E_Out(25,0.5,20,3000,29000,7,120);  //������ģʽ�⻷
//float Pitch_G_Angle,Pitch_G_Angle_Rate,Yaw_G_Angle,Yaw_G_Angle_Rate;  //������ģʽ�µ������ǽǶ�����ٶȣ�������·�����ٶ�
softmotor Cloud_Engineer_Pitch(2,0x207,&DJI_Motor_3510,NULL,&PID_Cloud_Pitch_E_Out);
softmotor Cloud_Engineer_Yaw(2,0x206,&DJI_Motor_3510,NULL,&PID_Cloud_Yaw_E_Out);





/** 
	* @brief   �ض����λ�û�����
	* @retval  ����ֵ
	* @note   ����19����3510������ٶȻ����ض��庯��
	* @par ��־ 
	*       2018��10��31��17:13:46 ��������������motor::CANSend()�й�ִ��
	*       2018��11��3��11:32:35  ������·�̻�У�黷��
	*       2019.11.26   20��02 �޸�Ϊ���ٶȻ�
*/
void softmotor::Position_Run(void)
{
  if(PID_Out==NULL)while(1);//��¶������û���⻷������λ��PID?
	int32_t err=0;
	err=TargetPosition-RealPosition;     //�õ���Ȧ���
	err+=MotorType->max_mechanical_position*(Soft_TargetPosition-Soft_RealPosition);//����Ȧ��λ�����
	TargetCurrent = PID_Out->pid_run(err);//λ�û��õ�Ŀ�����
//	TargetCurrent = PID_In->pid_run(TargetSpeed-RealSpeed);
	InsertCurrent();
}

/** 
	* @brief  softmotor�ض����update
	* @retval  Ϊ����Ӧ3510�����ͬ�ķ�������
	* @par ��־ 
	*
*/
void softmotor::update(uint8_t Data[])
{
	LastSpeed = RealSpeed;
	LastPosition = RealPosition;
	LastAngle = RealAngle;
	LastCurrent = Current;//����ת�ص���
	RealPosition = Data[0]<<8 | Data[1];
	Current = Data[2]<<8 | Data[3];  //3510�����������
	if(Current != LastCurrent)//ǰ��ת�ص�����ͬ��������Ч����
		LastUpdateTime = HAL_GetTick();//���±��ε�����������µ�ʱ��
	RealAngle = RealPosition*360.f/MotorType->max_mechanical_position;//���ݻ�е�Ǽ��������ʵ�Ƕ�
	if(running_flag==0)
	{
		LastPosition=RealPosition;
		running_flag=1;
	}
	if(RealPosition	-	LastPosition	>	4096)//Ȧ���ۼ�
		Soft_RealPosition--;
	else if(RealPosition	-	LastPosition	<-4096)//Ȧ���ۼ�
		Soft_RealPosition++;
	RealAngle = /*Ȧ����Ӧ�Ƕ�*/(Soft_RealPosition)/(1.0*MotorType->Reduction_ratio)*360 \
							+ /*��Ȧ�ڽǶ�*/1.0f*RealPosition / (8192 * MotorType->Reduction_ratio )*360;//ת��Ϊ�Ƕ�
}


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
		Cloud_Engineer_Pitch.Safe_Set();  //��̨Pitch
		Cloud_Engineer_Yaw.Safe_Set(); //��̨Yaw
		
	}
	else if(s1 ==3 && s2 == 2)  //���̸��棬��е��
 {
	 static float Cloud_Pitch_Zero = 24.35 ,Cloud_Yaw_Zero = 158.73f;  //������̨���λ��
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






	

