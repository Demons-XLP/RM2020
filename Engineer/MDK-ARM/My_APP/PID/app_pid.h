#ifndef __pid_h
#define __pid_h
#include "bsp_can.h"
#include "stm32f4xx_hal.h"
/*PID结构体*/
typedef struct
{
	float ERROR;  /*转速实时误差*/
	float LAST_ERROR;  /*上一个时刻的误差*/
	float ALL_ERROR;  /*累计的误差*/
	float OUT;  /*要发送调节电机状态的值*/
	float kp;  /*编码器P的系数*/
	float ki;  /*编码器I的系数*/
	float kd;  /*编码器D的系数*/
	float pid_outmax;  //PID输出限幅
	float imax;  //积分限幅
	float iout;  //i输出量，便于观察调节PID
	float pout;  //p输出量，便于观察调节PID
	float i_sec;  //积分误差累加的区域范围，只有在这个范围内的误差才会被累加上去
}PID;

extern PID motor_V[6];  /*让结构体能进行外部调用*/

void PID_Set(PID*app_pid_init,float P,float I,float D,float imax,float outmax);
void PID_Init(void);  /*PID初始化*/
void PID_Control(PID*app_pid_V,float EXP_V,float Real);  /*PID控制函数函数*/





#endif


