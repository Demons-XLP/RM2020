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
#include "bsp_can.h"

Motor bsp_can_motor_data[6]; //������ݽ����ṹ������


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
	
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)!=0) //�ж��жϲ���
	{
		HAL_CAN_GetRxMessage(hcan, 0, &bsp_can_Rx, CAN_RxData);	//��ȡCAN����
		#ifdef BSP_CAN_USE_SINGNAL
			xSemaphoreGiveFromISR(CAN_Update_Handle, &CAN_xHigherPriorityTaskWoken); //�ͷ�CAN���ݸ��¶�ֵ�ź���
			xSemaphoreGiveFromISR(CAN_Check_Handle, &CAN_xHigherPriorityTaskWoken); //�ͷ�CAN���ݸ��¶�ֵ�ź���
		#endif
		//app_motor_Data_update(CAN_RxData,CAN_RxData); //�˴��滻�Լ��Ľ���������ʹ�ã�ע�������CAN_RxData�Ǿֲ�����������Ҫ�Ŀ�����������������ȥ��Ϊȫ��
		switch (bsp_can_Rx.StdId) 
			{				//��������ע�ⲻͬ���Ҫ�ò�ͬID
		case(0x201):
			  bsp_can_motor_data[0].Encoder = (int16_t) (CAN_RxData[0]<<8) | CAN_RxData[1];   //��ȡ��ת�����ʵʱλ�ã���������
				bsp_can_motor_data[0].V = (int16_t) (CAN_RxData[2]<<8) | CAN_RxData[3]; /*��ȡ��ת�����ʵʱת��*/		
			  bsp_can_motor_data[0].Itrue = (int16_t) (CAN_RxData[4]<<8) | CAN_RxData[5];   //��ȡ��ת�����ʵʱ����
			  bsp_can_motor_data[0].Temper = (int16_t) (CAN_RxData[6]<<8) | CAN_RxData[7];   //��ȡ��ת�����ʵʱ�¶�
		    break;
			
		case(0x202):
			  bsp_can_motor_data[1].Encoder = (int16_t) (CAN_RxData[0]<<8) | CAN_RxData[1];   
				bsp_can_motor_data[1].V = (int16_t) (CAN_RxData[2]<<8) | CAN_RxData[3]; 	
			  bsp_can_motor_data[1].Itrue = (int16_t) (CAN_RxData[4]<<8) | CAN_RxData[5];   
			  bsp_can_motor_data[1].Temper = (int16_t) (CAN_RxData[6]<<8) | CAN_RxData[7];   
			  break;
				
		case(0x203):
			  bsp_can_motor_data[2].Encoder = (int16_t) (CAN_RxData[0]<<8) | CAN_RxData[1];   
				bsp_can_motor_data[2].V = (int16_t) (CAN_RxData[2]<<8) | CAN_RxData[3]; 
			  bsp_can_motor_data[2].Itrue = (int16_t) (CAN_RxData[4]<<8) | CAN_RxData[5];   
			  bsp_can_motor_data[2].Temper = (int16_t) (CAN_RxData[6]<<8) | CAN_RxData[7];   
			  break;
				
		case(0x204):
			  bsp_can_motor_data[3].id = 4;  //���IDΪ4
		
		
			  bsp_can_motor_data[3].Encoder = (int16_t) (CAN_RxData[0]<<8) | CAN_RxData[1];   
				bsp_can_motor_data[3].V = (int16_t) (CAN_RxData[2]<<8) | CAN_RxData[3]; 
			  bsp_can_motor_data[3].Itrue = (int16_t) (CAN_RxData[4]<<8) | CAN_RxData[5];   
			  bsp_can_motor_data[3].Temper = (int16_t) (CAN_RxData[6]<<8) | CAN_RxData[7];   
		    break;
			
		case(0x205):
			  bsp_can_motor_data[4].id = 5;  //���IDΪ5
			  bsp_can_motor_data[4].Encoder = (int16_t) (CAN_RxData[0]<<8) | CAN_RxData[1];   
				bsp_can_motor_data[4].V = (int16_t) (CAN_RxData[2]<<8) | CAN_RxData[3]; 
			  bsp_can_motor_data[4].Itrue = (int16_t) (CAN_RxData[4]<<8) | CAN_RxData[5];   
			  bsp_can_motor_data[4].Temper = (int16_t) (CAN_RxData[6]<<8) | CAN_RxData[7];   
			  break;
				
		case(0x206):
			  bsp_can_motor_data[5].id = 6;  //���IDΪ6
			  bsp_can_motor_data[5].Encoder = (int16_t) (CAN_RxData[0]<<8) | CAN_RxData[1];   
				bsp_can_motor_data[5].V = (int16_t) (CAN_RxData[2]<<8) | CAN_RxData[3]; 
			  bsp_can_motor_data[5].Itrue = (int16_t) (CAN_RxData[4]<<8) | CAN_RxData[5];   
			  bsp_can_motor_data[5].Temper = (int16_t) (CAN_RxData[6]<<8) | CAN_RxData[7];   
				break;
		    
		default:
			  break;
		}
	}
}
