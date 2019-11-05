/**
    ****************************(C) COPYRIGHT LittleDargon****************************
    * @file       app_pid.c
    * @brief      用来调节6个电机的pid
    * @note
    * @history
    *  Version    Date            Author          Modification
    *  V1.0.0     2019-10-23     Demons    1. 用来调节6个电机的pid
    *  
    *  
  	*
  	**************************(C) COPYRIGHT LittleDargon****************************
  	*/
#include "app_pid.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include "bsp_can.h"

//CAN_RxHeaderTypeDef Bsp_CAN1_Rx;		/*Can总线接收*/
//uint8_t CAN1_RxData[8];       /*CAN1接收缓存数组*/
//int16_t V,Encoder,Itrue,Temper;    /*定义实际电机转子转速，编码器，电流值，温度*/
PID motor_V[6];         //声明结构体数组，数组有6个成员，分别代表6个电机



/**
* @brief  PID初始化函数
* @details  PID初始化函数
* @param  限幅，误差，PID初始化
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
* @brief  通用型pid调节函数
* @details  利用PID公式调节
* @param  NULL
* @retval  NULL
*/
void PID_Control(PID*app_pid,float EXP,float Real)
{
		app_pid->ERROR = EXP - Real;  //误差
		app_pid->ALL_ERROR += app_pid->ki*app_pid->ERROR;  //累积误差
		if (app_pid->ALL_ERROR > app_pid->imax)  //积分误差限幅
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
		app_pid->OUT = app_pid->pout + app_pid->iout + app_pid->kd*(app_pid->ERROR - app_pid->LAST_ERROR);  //实际控制输出
		if (app_pid->OUT > app_pid->pid_outmax)  //输出限幅
		{
			app_pid->OUT = app_pid->pid_outmax;
		}	
		else if (-app_pid->OUT > app_pid->pid_outmax)  //输出限幅
		{
			app_pid->OUT = -app_pid->pid_outmax;
		}
		
		app_pid->LAST_ERROR = app_pid->ERROR;  //更新上一次错误值
	
}










