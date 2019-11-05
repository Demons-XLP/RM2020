/**
    ****************************(C) COPYRIGHT LittleDargon****************************
    * @file       app_pid.c
    * @brief      ��������6�������pid
    * @note
    * @history
    *  Version    Date            Author          Modification
    *  V1.0.0     2019-10-23     Demons    1. ��������6�������pid
    *  
    *  
  	*
  	**************************(C) COPYRIGHT LittleDargon****************************
  	*/
#include "app_pid.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include "bsp_can.h"

//CAN_RxHeaderTypeDef Bsp_CAN1_Rx;		/*Can���߽���*/
//uint8_t CAN1_RxData[8];       /*CAN1���ջ�������*/
//int16_t V,Encoder,Itrue,Temper;    /*����ʵ�ʵ��ת��ת�٣�������������ֵ���¶�*/
PID motor_V[6];         //�����ṹ�����飬������6����Ա���ֱ����6�����



/**
* @brief  PID��ʼ������
* @details  PID��ʼ������
* @param  �޷�����PID��ʼ��
* @retval  NULL
*/
void PID_Init (void)
{
  int i = 0;
	for (i=0;i<4;i++)
	{
	  PID_Set(&motor_V[i],0,0,0,10000,16000);
	}
}
void PID_Set(PID*app_pid_init,float P,float I,float D,float imax,float outmax)  
{
  app_pid_init->ERROR = 0;
	app_pid_init->LAST_ERROR = 0;
	app_pid_init->ALL_ERROR = 0;
	app_pid_init->OUT = 0;
	app_pid_init->imax = imax;
	app_pid_init->kp = P;
	app_pid_init->ki = I;
	app_pid_init->kd = D;
	app_pid_init->pid_outmax = outmax;
	app_pid_init->OUT = 0;
}







/**
* @brief  ͨ����pid���ں���
* @details  ����PID��ʽ����
* @param  NULL
* @retval  NULL
*/
void PID_Control(PID*app_pid,float EXP,float Real)
{
		app_pid->ERROR = EXP - Real;  //���
		app_pid->ALL_ERROR += app_pid->ki*app_pid->ERROR;  //�ۻ����
		if (app_pid->ALL_ERROR > app_pid->imax)  //��������޷�
		{
			app_pid->ALL_ERROR = app_pid->imax;
		}
		else if(-app_pid->ALL_ERROR > app_pid->imax)
		{
			app_pid->ALL_ERROR = -app_pid->imax;
		}
		if(app_pid->ERROR>app_pid->i_sec || app_pid->ERROR<-app_pid->i_sec)
		app_pid->iout =  app_pid->ALL_ERROR;
		app_pid->iout = 0;
		app_pid->pout =  app_pid->kp*app_pid->ERROR ;
		app_pid->OUT = app_pid->pout + app_pid->iout + app_pid->kd*(app_pid->ERROR - app_pid->LAST_ERROR);  //ʵ�ʿ������
		if (app_pid->OUT > app_pid->pid_outmax)  //����޷�
		{
			app_pid->OUT = app_pid->pid_outmax;
		}	
		else if (-app_pid->OUT > app_pid->pid_outmax)  //����޷�
		{
			app_pid->OUT = -app_pid->pid_outmax;
		}
		
		app_pid->LAST_ERROR = app_pid->ERROR;  //������һ�δ���ֵ
	
}










