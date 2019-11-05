/** 
* @file         Can_Driver.cpp 
* @brief        CAN_Driver常用执行驱动
* @details      robomaster常用的执行驱动
* @author      WMD
* @date     2018年10月10日17:37:37
* @version  1.0
* @par Copyright (c):  
*       WMD 
* @par 日志
*      2018年10月10日17:37:39 针对新的cube固件库做了一次兼容性更新
*       2018年11月7日20:10:11 不再将宏定义放在此处，而用错误检测替代之 建议使用什么CAN放在can.h
*/  
#include "bsp_can_driver.h"
#include "can.h"

//CAN的初始化函数
#define ABS(x) ((x)>0?(x):-(x))
#define ERROR_STACK_SIZE 20

#define USE_CAN1
#define USE_CAN2

CAN_RxHeaderTypeDef Bsp_CAN1_Rx;		/*Can总线接收*/
int8_t CAN1_RxData[8];       /*CAN1接收缓存数组*/
//int16_t V,Encoder,Itrue,Temper;    /*定义实际电机转子转速，编码器，电流值，温度*/
Motor motor_data[6]; //电机数据解析结构体数组


/** 
    * @brief 在下面两个宏定义确认是否选用了CAN
*/
#ifndef USE_CAN1 
#ifndef USE_CAN2
#error Not selected any CAN to init!Please define USE_CAN1 or USE_CAN2 to use it.
#endif
#endif
int8_t send_buf[5] = {0,0,0,0,0};
#define BLOCK_ERROR //错误场屏蔽 在比赛时不要用
/**
*滤波器设置，适用于HAL库1.7.3+，固件1.21.0+版本
***/
static void My_CAN_FilterConfig(CAN_HandleTypeDef* _hcan)
{

	CAN_FilterTypeDef		CAN_FilterConfigStructure;
	//滤波器设置
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
			while(1);//等待
	}
#endif
//#ifdef USE_CAN2
//	if(_hcan == &hcan2){
//    CAN_FilterConfigStructure.FilterBank = 14;
//		if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
//			while(1);//等待
//	}
//#endif
}

//记得 如果需要改变CAN的数量 这里一定要再改
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
*@func  通用型can发送函数
*@brief 确定id 和发送数据 把数据发送出去 注意此处入口参数是4个int16_t型数据 和8个uint8_t不一样
*@Para
*@Retal
*@data
*******/
HAL_StatusTypeDef can_send_msg(CAN_HandleTypeDef* _hcan, int id, int16_t* s16buff)
{
    static CAN_TxHeaderTypeDef	TxHead;//!规定发送的帧格式
		TxHead.RTR=CAN_RTR_DATA;//标准RTR
    TxHead.IDE=CAN_ID_STD;//标准ID不扩展
		TxHead.DLC = 8;
		TxHead.StdId =id;
    static uint8_t Data[8];//中间数组，用于高低位更换
		Data[0] = (uint8_t)((*(s16buff+0)>>8));
		Data[1] = (uint8_t)(*(s16buff+0));
		Data[2] = (uint8_t)((*(s16buff+1)>>8));
		Data[3] = (uint8_t)(*(s16buff+1));
		Data[4] = (uint8_t)((*(s16buff+2)>>8));
		Data[5] = (uint8_t)(*(s16buff+2));
		Data[6] = (uint8_t)((*(s16buff+3)>>8));
		Data[7] = (uint8_t)(*(s16buff+3));
        uint32_t FifoLevel;//当前发送队列的长度
        HAL_StatusTypeDef result=(HAL_CAN_AddTxMessage(_hcan,&TxHead,Data,&FifoLevel));
	return result;
	
}

/*
* @brief  CAN接收中断
* @details  重新定义接收中断，在CAN中断中调用
* @param  NULL
* @retval  NULL
*  2019.10.22  在函数中添加各个电机反馈报文的数据解析
*
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
  {
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)!=0) /*中断处理*/
		{
			HAL_CAN_GetRxMessage(&hcan1, 0, &Bsp_CAN1_Rx, CAN1_RxData);	/*获取CAN报文*/
			switch (Bsp_CAN1_Rx.StdId) 
			{				//！！！！注意不同电机要用不同ID
		case(0x201):
				motor_data[0].id = 1;  //电机ID为1
			  motor_data[0].Encoder = (CAN1_RxData[0]<<8)+CAN1_RxData[1];   //获取并转换电机实时位置（编码器）
				motor_data[0].V = (CAN1_RxData[2]<<8)+CAN1_RxData[3]; /*获取并转换电机实时转速*/		
			  motor_data[0].Itrue = (CAN1_RxData[4]<<8)+CAN1_RxData[5];   //获取并转换电机实时电流
			  motor_data[0].Temper = (CAN1_RxData[6]<<8)+CAN1_RxData[7];   //获取并转换电机实时温度
		    break;
			
		case(0x202):
			  motor_data[1].id = 2;  //电机ID为2
			  motor_data[1].Encoder = (CAN1_RxData[0]<<8)+CAN1_RxData[1];   //获取并转换电机实时位置（编码器）
				motor_data[1].V = (CAN1_RxData[2]<<8)+CAN1_RxData[3]; /*获取并转换电机实时转速*/		
			  motor_data[1].Itrue = (CAN1_RxData[4]<<8)+CAN1_RxData[5];   //获取并转换电机实时电流
			  motor_data[1].Temper = (CAN1_RxData[6]<<8)+CAN1_RxData[7];   //获取并转换电机实时温度
			  break;
				
		case(0x203):
			  motor_data[2].id = 3;  //电机ID为3
			  motor_data[2].Encoder = (CAN1_RxData[0]<<8)+CAN1_RxData[1];   //获取并转换电机实时位置（编码器）
				motor_data[2].V = (CAN1_RxData[2]<<8)+CAN1_RxData[3]; /*获取并转换电机实时转速*/		
			  motor_data[2].Itrue = (CAN1_RxData[4]<<8)+CAN1_RxData[5];   //获取并转换电机实时电流
			  motor_data[2].Temper = (CAN1_RxData[6]<<8)+CAN1_RxData[7];   //获取并转换电机实时温度
			  break;
				
		case(0x20B):
			  send_buf[0] = CAN1_RxData[0];  //用来控制爪子
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





