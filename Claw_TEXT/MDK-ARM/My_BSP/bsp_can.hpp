#ifndef __BSP_CAN_H
#define __BSP_CAN_H
#include "stm32f4xx.h"
//#include "app_motor.h"
#include "can.h"

//�������뿪�غ궨�壬����Ҫ�����ľ�ע�͵���غ궨��
//#define BSP_CAN_USE_SIGNAL
//������غ궨��,��ֲʱ����޸����������������޸�
#define BSP_CAN_USE_CAN1					hcan1
//#define BSP_CAN_USE_CAN2					hcan2


typedef struct 
{
  float V;
	float Encoder;
	float Itrue;
	float Temper;
	int16_t id;
}Motor;

extern Motor bsp_can_motor_data[6]; //������ݽ����ṹ������
extern Motor bsp_can_cloud_data[2]; //������ݽ����ṹ������


void bsp_can_Init(void);	//CAN���߳�ʼ������
HAL_StatusTypeDef bsp_can_Sendmessage(CAN_HandleTypeDef* hcan,int16_t StdId,int16_t* Can_Send_Data);	//CAN�������ݷ��ͺ���
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);	//�ض����CAN�жϻص�����

#endif