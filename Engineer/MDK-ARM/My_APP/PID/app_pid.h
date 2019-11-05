#ifndef __pid_h
#define __pid_h
#include "bsp_can.h"
#include "stm32f4xx_hal.h"
/*PID�ṹ��*/
typedef struct
{
	float ERROR;  /*ת��ʵʱ���*/
	float LAST_ERROR;  /*��һ��ʱ�̵����*/
	float ALL_ERROR;  /*�ۼƵ����*/
	float OUT;  /*Ҫ���͵��ڵ��״̬��ֵ*/
	float kp;  /*������P��ϵ��*/
	float ki;  /*������I��ϵ��*/
	float kd;  /*������D��ϵ��*/
	float pid_outmax;  //PID����޷�
	float imax;  //�����޷�
	float iout;  //i����������ڹ۲����PID
	float pout;  //p����������ڹ۲����PID
	float i_sec;  //��������ۼӵ�����Χ��ֻ���������Χ�ڵ����Żᱻ�ۼ���ȥ
}PID;

extern PID motor_V[6];  /*�ýṹ���ܽ����ⲿ����*/

void PID_Set(PID*app_pid_init,float P,float I,float D,float imax,float outmax);
void PID_Init(void);  /*PID��ʼ��*/
void PID_Control(PID*app_pid_V,float EXP_V,float Real);  /*PID���ƺ�������*/





#endif


