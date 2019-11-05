/** 
* @file         Can_Driver.cpp 
* @brief        CAN_Driver����ִ������
* @details      robomaster���õ�ִ������
* @author      WMD
* @date     2018��10��10��17:37:37
* @version  1.0
* @par Copyright (c):  
*       WMD 
* @par ��־
*      2018��10��10��17:37:39 ����µ�cube�̼�������һ�μ����Ը���
*       2018��11��7��20:10:11 ���ٽ��궨����ڴ˴������ô��������֮ ����ʹ��ʲôCAN����can.h
*/  
#include "bsp_can_driver.h"
#include "can.h"

//CAN�ĳ�ʼ������
#define ABS(x) ((x)>0?(x):-(x))
#define ERROR_STACK_SIZE 20

#define USE_CAN1
#define USE_CAN2

CAN_RxHeaderTypeDef Bsp_CAN1_Rx;		/*Can���߽���*/
int8_t CAN1_RxData[8];       /*CAN1���ջ�������*/
//int16_t V,Encoder,Itrue,Temper;    /*����ʵ�ʵ��ת��ת�٣�������������ֵ���¶�*/
Motor motor_data[6]; //������ݽ����ṹ������


/** 
    * @brief �����������궨��ȷ���Ƿ�ѡ����CAN
*/
#ifndef USE_CAN1 
#ifndef USE_CAN2
#error Not selected any CAN to init!Please define USE_CAN1 or USE_CAN2 to use it.
#endif
#endif
int8_t send_buf[5] = {0,0,0,0,0};
#define BLOCK_ERROR //�������� �ڱ���ʱ��Ҫ��
/**
*�˲������ã�������HAL��1.7.3+���̼�1.21.0+�汾
***/
static void My_CAN_FilterConfig(CAN_HandleTypeDef* _hcan)
{

	CAN_FilterTypeDef		CAN_FilterConfigStructure;
	//�˲�������
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x1111;
	CAN_FilterConfigStructure.FilterIdLow = 0x1000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;
#ifdef USE_CAN1
	if(_hcan == &hcan1){
    CAN_FilterConfigStructure.FilterBank = 0;
		if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
			while(1);//�ȴ�
	}
#endif
//#ifdef USE_CAN2
//	if(_hcan == &hcan2){
//    CAN_FilterConfigStructure.FilterBank = 14;
//		if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
//			while(1);//�ȴ�
//	}
//#endif
}

//�ǵ� �����Ҫ�ı�CAN������ ����һ��Ҫ�ٸ�
void CAN_Init_All(void)
{
#ifdef USE_CAN1
	My_CAN_FilterConfig(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan1);
#endif
//#ifdef USE_CAN2
//	My_CAN_FilterConfig(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
//    HAL_CAN_Start(&hcan2);
//#endif
}
/****
*@func  ͨ����can���ͺ���
*@brief ȷ��id �ͷ������� �����ݷ��ͳ�ȥ ע��˴���ڲ�����4��int16_t������ ��8��uint8_t��һ��
*@Para
*@Retal
*@data
*******/
HAL_StatusTypeDef can_send_msg(CAN_HandleTypeDef* _hcan, int id, int16_t* s16buff)
{
    static CAN_TxHeaderTypeDef	TxHead;//!�涨���͵�֡��ʽ
		TxHead.RTR=CAN_RTR_DATA;//��׼RTR
    TxHead.IDE=CAN_ID_STD;//��׼ID����չ
		TxHead.DLC = 8;
		TxHead.StdId =id;
    static uint8_t Data[8];//�м����飬���ڸߵ�λ����
		Data[0] = (uint8_t)((*(s16buff+0)>>8));
		Data[1] = (uint8_t)(*(s16buff+0));
		Data[2] = (uint8_t)((*(s16buff+1)>>8));
		Data[3] = (uint8_t)(*(s16buff+1));
		Data[4] = (uint8_t)((*(s16buff+2)>>8));
		Data[5] = (uint8_t)(*(s16buff+2));
		Data[6] = (uint8_t)((*(s16buff+3)>>8));
		Data[7] = (uint8_t)(*(s16buff+3));
        uint32_t FifoLevel;//��ǰ���Ͷ��еĳ���
        HAL_StatusTypeDef result=(HAL_CAN_AddTxMessage(_hcan,&TxHead,Data,&FifoLevel));
	return result;
	
}

/*
* @brief  CAN�����ж�
* @details  ���¶�������жϣ���CAN�ж��е���
* @param  NULL
* @retval  NULL
*  2019.10.22  �ں�������Ӹ�������������ĵ����ݽ���
*
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
  {
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)!=0) /*�жϴ���*/
		{
			HAL_CAN_GetRxMessage(&hcan1, 0, &Bsp_CAN1_Rx, CAN1_RxData);	/*��ȡCAN����*/
			switch (Bsp_CAN1_Rx.StdId) 
			{				//��������ע�ⲻͬ���Ҫ�ò�ͬID
		case(0x201):
				motor_data[0].id = 1;  //���IDΪ1
			  motor_data[0].Encoder = (CAN1_RxData[0]<<8)+CAN1_RxData[1];   //��ȡ��ת�����ʵʱλ�ã���������
				motor_data[0].V = (CAN1_RxData[2]<<8)+CAN1_RxData[3]; /*��ȡ��ת�����ʵʱת��*/		
			  motor_data[0].Itrue = (CAN1_RxData[4]<<8)+CAN1_RxData[5];   //��ȡ��ת�����ʵʱ����
			  motor_data[0].Temper = (CAN1_RxData[6]<<8)+CAN1_RxData[7];   //��ȡ��ת�����ʵʱ�¶�
		    break;
			
		case(0x202):
			  motor_data[1].id = 2;  //���IDΪ2
			  motor_data[1].Encoder = (CAN1_RxData[0]<<8)+CAN1_RxData[1];   //��ȡ��ת�����ʵʱλ�ã���������
				motor_data[1].V = (CAN1_RxData[2]<<8)+CAN1_RxData[3]; /*��ȡ��ת�����ʵʱת��*/		
			  motor_data[1].Itrue = (CAN1_RxData[4]<<8)+CAN1_RxData[5];   //��ȡ��ת�����ʵʱ����
			  motor_data[1].Temper = (CAN1_RxData[6]<<8)+CAN1_RxData[7];   //��ȡ��ת�����ʵʱ�¶�
			  break;
				
		case(0x203):
			  motor_data[2].id = 3;  //���IDΪ3
			  motor_data[2].Encoder = (CAN1_RxData[0]<<8)+CAN1_RxData[1];   //��ȡ��ת�����ʵʱλ�ã���������
				motor_data[2].V = (CAN1_RxData[2]<<8)+CAN1_RxData[3]; /*��ȡ��ת�����ʵʱת��*/		
			  motor_data[2].Itrue = (CAN1_RxData[4]<<8)+CAN1_RxData[5];   //��ȡ��ת�����ʵʱ����
			  motor_data[2].Temper = (CAN1_RxData[6]<<8)+CAN1_RxData[7];   //��ȡ��ת�����ʵʱ�¶�
			  break;
				
		case(0x20B):
			  send_buf[0] = CAN1_RxData[0];  //��������צ��
			  send_buf[1] = CAN1_RxData[1];   
				send_buf[2] = CAN1_RxData[2]; 	
			  send_buf[3] = CAN1_RxData[3];   
			  send_buf[4] = CAN1_RxData[4];   
		    break;
			
		default:
			  break;
		}
		}
		
}





