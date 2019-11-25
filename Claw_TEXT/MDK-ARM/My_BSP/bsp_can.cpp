/** 
* @brief    CAN�弶֧�ְ�
* @details  CAN����������ã����ݽ��ս�������
* @author   Evan-GH
* @date      2019.10
* @version  1.6
* @par Copyright (c):  RM2020���
* @par ��־
*				����ʹ�÷�����Readme.md
*				�汾���:
*				1.0		|		��ͨCAN��������жϴ�����
*				1.1		|		������������,����ѡ����ĳ��CAN��
*				1.2		|		������������,��Ϊʹ���ź�������ͨ�ж�����
*				1.3		|		�޸ķ�����غ�����ʹ�ø�����ʹ��
*				1.4		|		���ݴ���淶�޸���һ���ֺ�������
*				1.5		|		�ο�RM19���룬�޸Ķ���ӿں�������װ������
*				1.6		|		�޸Ľ����жϺ�CAN�߷��ͺ����߼�
*/
#include "bsp_can.hpp"
#include "cmsis_os.h"
#include "Car_Driver.hpp"

Motor bsp_can_motor_data[6]; //������ݽ����ṹ������
Motor bsp_can_cloud_data[2]; //��̨������ݽ����ṹ������

/**
* @brief  CAN�������ó�ʼ��
* @details  ��ʼ���˲��������ݺ궨��Ŀ�������ʼ��CAN����
* @param  NULL
* @retval  NULL
*/
void bsp_can_Init(void)
{
	//CAN�˲������ã��˲��ֲ���Ҫ�޸ģ�ֱ���þ���
	CAN_FilterTypeDef CAN_FilterConfig;
	
	CAN_FilterConfig.SlaveStartFilterBank=0;
	CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfig.FilterIdHigh = 0X0000;
	CAN_FilterConfig.FilterIdLow = 0X0000;
	CAN_FilterConfig.FilterMaskIdHigh = 0X0000;
	CAN_FilterConfig.FilterMaskIdLow = 0X0000;
	CAN_FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_FilterConfig.FilterActivation = ENABLE;
	#ifdef BSP_CAN_USE_CAN1
		CAN_FilterConfig.FilterBank = 0;
		HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterConfig);
		HAL_CAN_Start(&hcan1);
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); /*�����ж�*/
	#endif
	
	#ifdef BSP_CAN_USE_CAN2
		CAN_FilterConfig.FilterBank = 14;
		HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterConfig);
		HAL_CAN_Start(&hcan2);
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); /*�����ж�*/
	#endif
}

/**
* @brief  CAN��������
* @details  ͨ��CAN���߷��Ϳ�������
* @param  CAN_HandleTypeDef* hcan ����ʹ�õ�CAN����,int16_t StdId CAN�߷���ID,int16_t* Can_Send_Data ��Ҫ���͵�����
* @retval  HAL_RESULT ���ͽ�� HAL_OK �ɹ���HAL_ERROR ʧ��
*/
HAL_StatusTypeDef bsp_can_Sendmessage(CAN_HandleTypeDef* hcan,int16_t StdId,int16_t* Can_Send_Data)
{
	uint32_t MailBox;
	CAN_TxHeaderTypeDef bsp_can_Tx;
	HAL_StatusTypeDef HAL_RESULT;
	
	//�����������ת��Ϊ��׼CAN֡����
	uint8_t Data[8];
	Data[0] = (uint8_t)((*(Can_Send_Data+0)>>8));
	Data[1] = (uint8_t)(*(Can_Send_Data+0)) & 0XFF;
	Data[2] = (uint8_t)((*(Can_Send_Data+1)>>8));
	Data[3] = (uint8_t)(*(Can_Send_Data+1)) & 0XFF;
	Data[4] = (uint8_t)((*(Can_Send_Data+2)>>8));
	Data[5] = (uint8_t)(*(Can_Send_Data+2)) & 0XFF;
	Data[6] = (uint8_t)((*(Can_Send_Data+3)>>8));
	Data[7] = (uint8_t)(*(Can_Send_Data+3)) & 0XFF;
	
	//����CAN֡����
	bsp_can_Tx.StdId=StdId;
	bsp_can_Tx.RTR = CAN_RTR_DATA;
	bsp_can_Tx.IDE = CAN_ID_STD;
	bsp_can_Tx.DLC = 8;
	HAL_RESULT = HAL_CAN_AddTxMessage(hcan, &bsp_can_Tx, Data, &MailBox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan)!=3);//�ȴ��������
	
	return HAL_RESULT;
}

//Motor_t DJI_M3508(8192,19);  //����������
//motor left_motor(1,0x204,&DJI_Motor_3508,&pid_Claw_IN,&pid_Claw_OUT);  //���岢��ʼ��motor�ຯ��
//motor right_motor(1,0x203,&DJI_Motor_3508,&pid_Claw_IN,&pid_Claw_OUT);

/**
* @brief  CAN�����ж�
* @details  ���¶�������жϣ����Զ���CAN�ж��е��ã�����Ҫ�ֶ����,ʹ�õ�ʱ�������ڴ˺������滻��������
* @param  NULL
* @retval  NULL
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	static CAN_RxHeaderTypeDef bsp_can_Rx;
	uint8_t CAN_RxData[8];
	
	#ifdef BSP_CAN_USE_SIGNAL
		static BaseType_t CAN_xHigherPriorityTaskWoken;
	#endif
	if(hcan == &hcan1)
	{
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)!=0) //�ж��жϲ���
	{
		HAL_CAN_GetRxMessage(hcan, 0, &bsp_can_Rx, CAN_RxData);	//��ȡCAN����
		#ifdef BSP_CAN_USE_SIGNAL
			xSemaphoreGiveFromISR(CAN_Update_Handle, &CAN_xHigherPriorityTaskWoken); //�ͷ�CAN���ݸ��¶�ֵ�ź���
			xSemaphoreGiveFromISR(CAN_Check_Handle, &CAN_xHigherPriorityTaskWoken); //�ͷ�CAN���ݸ��¶�ֵ�ź���
		#else
		motor::CANUpdate(hcan,&bsp_can_Rx,(uint8_t *)CAN_RxData);

		#endif
	}
}
	
}

