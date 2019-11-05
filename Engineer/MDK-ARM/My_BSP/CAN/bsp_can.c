/** 
* @brief    CAN板级支持包
* @details  CAN总线相关设置，数据接收解析函数
* @author   Evan-GH
* @date      2019.10
* @version  1.6
* @par Copyright (c):  RM2020电控
* @par 日志
*				具体使用方法见Readme.md
*				版本变更:
*				1.0		|		调通CAN总线相关中断处理函数
*				1.1		|		加入条件编译,自主选择开启某个CAN线
*				1.2		|		加入条件编译,分为使用信号量和普通中断两种
*				1.3		|		修改发送相关函数，使得更方便使用
*				1.4		|		根据代码规范修改了一部分函数名称
*				1.5		|		参考RM19代码，修改对外接口函数，封装更彻底
*				1.6		|		修改接收中断和CAN线发送函数逻辑
*/
#include "bsp_can.h"

Motor bsp_can_motor_data[6]; //电机数据解析结构体数组


/**
* @brief  CAN总线配置初始化
* @details  初始化滤波器，根据宏定义的开启来初始化CAN总线
* @param  NULL
* @retval  NULL
*/
void bsp_can_Init(void)
{
	//CAN滤波器设置，此部分不需要修改，直接用就行
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
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); /*开启中断*/
	#endif
	
	#ifdef BSP_CAN_USE_CAN2
		CAN_FilterConfig.FilterBank = 14;
		HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterConfig);
		HAL_CAN_Start(&hcan2);
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); /*开启中断*/
	#endif
}

/**
* @brief  CAN发送数据
* @details  通过CAN总线发送控制数据
* @param  CAN_HandleTypeDef* hcan 发送使用的CAN总线,int16_t StdId CAN线发送ID,int16_t* Can_Send_Data 需要发送的数据
* @retval  HAL_RESULT 发送结果 HAL_OK 成功，HAL_ERROR 失败
*/
HAL_StatusTypeDef bsp_can_Sendmessage(CAN_HandleTypeDef* hcan,int16_t StdId,int16_t* Can_Send_Data)
{
	uint32_t MailBox;
	CAN_TxHeaderTypeDef bsp_can_Tx;
	HAL_StatusTypeDef HAL_RESULT;
	
	//将传入的数据转换为标准CAN帧数据
	uint8_t Data[8];
	Data[0] = (uint8_t)((*(Can_Send_Data+0)>>8));
	Data[1] = (uint8_t)(*(Can_Send_Data+0)) & 0XFF;
	Data[2] = (uint8_t)((*(Can_Send_Data+1)>>8));
	Data[3] = (uint8_t)(*(Can_Send_Data+1)) & 0XFF;
	Data[4] = (uint8_t)((*(Can_Send_Data+2)>>8));
	Data[5] = (uint8_t)(*(Can_Send_Data+2)) & 0XFF;
	Data[6] = (uint8_t)((*(Can_Send_Data+3)>>8));
	Data[7] = (uint8_t)(*(Can_Send_Data+3)) & 0XFF;
	
	//设置CAN帧配置
	bsp_can_Tx.StdId=StdId;
	bsp_can_Tx.RTR = CAN_RTR_DATA;
	bsp_can_Tx.IDE = CAN_ID_STD;
	bsp_can_Tx.DLC = 8;
	HAL_RESULT = HAL_CAN_AddTxMessage(hcan, &bsp_can_Tx, Data, &MailBox);
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan)!=3);//等待发送完成
	
	return HAL_RESULT;
}

/**
* @brief  CAN接收中断
* @details  重新定义接收中断，会自动在CAN中断中调用，不需要手动添加,使用的时候自行在此函数中替换解析函数
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
	
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)!=0) //判断中断产生
	{
		HAL_CAN_GetRxMessage(hcan, 0, &bsp_can_Rx, CAN_RxData);	//获取CAN报文
		#ifdef BSP_CAN_USE_SINGNAL
			xSemaphoreGiveFromISR(CAN_Update_Handle, &CAN_xHigherPriorityTaskWoken); //释放CAN数据更新二值信号量
			xSemaphoreGiveFromISR(CAN_Check_Handle, &CAN_xHigherPriorityTaskWoken); //释放CAN数据更新二值信号量
		#endif
		//app_motor_Data_update(CAN_RxData,CAN_RxData); //此处替换自己的解析函数来使用，注意这里的CAN_RxData是局部变量，有需要的可以自行声明到外面去作为全局
		switch (bsp_can_Rx.StdId) 
			{				//！！！！注意不同电机要用不同ID
		case(0x201):
			  bsp_can_motor_data[0].Encoder = (int16_t) (CAN_RxData[0]<<8) | CAN_RxData[1];   //获取并转换电机实时位置（编码器）
				bsp_can_motor_data[0].V = (int16_t) (CAN_RxData[2]<<8) | CAN_RxData[3]; /*获取并转换电机实时转速*/		
			  bsp_can_motor_data[0].Itrue = (int16_t) (CAN_RxData[4]<<8) | CAN_RxData[5];   //获取并转换电机实时电流
			  bsp_can_motor_data[0].Temper = (int16_t) (CAN_RxData[6]<<8) | CAN_RxData[7];   //获取并转换电机实时温度
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
			  bsp_can_motor_data[3].id = 4;  //电机ID为4
		
		
			  bsp_can_motor_data[3].Encoder = (int16_t) (CAN_RxData[0]<<8) | CAN_RxData[1];   
				bsp_can_motor_data[3].V = (int16_t) (CAN_RxData[2]<<8) | CAN_RxData[3]; 
			  bsp_can_motor_data[3].Itrue = (int16_t) (CAN_RxData[4]<<8) | CAN_RxData[5];   
			  bsp_can_motor_data[3].Temper = (int16_t) (CAN_RxData[6]<<8) | CAN_RxData[7];   
		    break;
			
		case(0x205):
			  bsp_can_motor_data[4].id = 5;  //电机ID为5
			  bsp_can_motor_data[4].Encoder = (int16_t) (CAN_RxData[0]<<8) | CAN_RxData[1];   
				bsp_can_motor_data[4].V = (int16_t) (CAN_RxData[2]<<8) | CAN_RxData[3]; 
			  bsp_can_motor_data[4].Itrue = (int16_t) (CAN_RxData[4]<<8) | CAN_RxData[5];   
			  bsp_can_motor_data[4].Temper = (int16_t) (CAN_RxData[6]<<8) | CAN_RxData[7];   
			  break;
				
		case(0x206):
			  bsp_can_motor_data[5].id = 6;  //电机ID为6
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
