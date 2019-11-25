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
#include "bsp_can.hpp"
#include "cmsis_os.h"
#include "Car_Driver.hpp"

Motor bsp_can_motor_data[6]; //电机数据解析结构体数组
Motor bsp_can_cloud_data[2]; //云台电机数据解析结构体数组

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

//Motor_t DJI_M3508(8192,19);  //定义电机类型
//motor left_motor(1,0x204,&DJI_Motor_3508,&pid_Claw_IN,&pid_Claw_OUT);  //定义并初始化motor类函数
//motor right_motor(1,0x203,&DJI_Motor_3508,&pid_Claw_IN,&pid_Claw_OUT);

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
	if(hcan == &hcan1)
	{
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)!=0) //判断中断产生
	{
		HAL_CAN_GetRxMessage(hcan, 0, &bsp_can_Rx, CAN_RxData);	//获取CAN报文
		#ifdef BSP_CAN_USE_SIGNAL
			xSemaphoreGiveFromISR(CAN_Update_Handle, &CAN_xHigherPriorityTaskWoken); //释放CAN数据更新二值信号量
			xSemaphoreGiveFromISR(CAN_Check_Handle, &CAN_xHigherPriorityTaskWoken); //释放CAN数据更新二值信号量
		#else
		motor::CANUpdate(hcan,&bsp_can_Rx,(uint8_t *)CAN_RxData);

		#endif
	}
}
	
}

