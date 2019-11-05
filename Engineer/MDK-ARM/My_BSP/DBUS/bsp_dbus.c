/** 
* @brief        BSP——DBUS
* @details  参考WMD模块化散装库——Dbus库
* @author      mijures
* @date     2018.10.11
* @version  v0.1
* @par Copyright (c):      mijures
* @par 0级驱动
* @par 使用方法：1.在头文件中更改对应uart的宏定义 2.将DBUS_IT放到对应串口中断函数中 3.使用DBUS_Init初始化DBUS 
* @par 更换串口时除以上外还要注意波特率等设置
* @par v0.1 调整了函数位置和头文件的部分内容，使调用更加方便，更新了标准化注释
*/  

#include "bsp_dbus.h"
#include "usart.h"
#include "app_mode.h"
unsigned char DBUS_rx_buffer[25]={0};//DBUS数据缓存
RC_Ctl_t RC_Ctl; //处理后的遥控器控制量

/** 
* @brief  DBUS数据存储
* @details  将接受到的数据处理存储到RC_Ctl中
* @param  NULL
* @retval  HAL_OK 成功 HAL_ERROR 失败
* @par log
*/
static HAL_StatusTypeDef DBUS_remember(void)
{
//	uint8_t i;
//	for(i=0;i<18;i++)
//	{
//		if(DBUS_rx_buffer[i]==0)continue;
//		break;
//	}
//	if(i==18)//无数据 返回错误
//	{
//		return HAL_ERROR;
//	}
//	for(uint8_t i=18;i<25;i++)//0缓冲区检测
//	{
//		if(DBUS_rx_buffer[i]!=0)		//0缓冲区（18-24）出现数据 接收/发射错误
//		{
//			return HAL_ERROR;
//		}
//	}
	
	//数据存储
	RC_Ctl.rc.ch0 = ((DBUS_rx_buffer[0]| (DBUS_rx_buffer[1] << 8)) & 0x07ff) - 1024; //!< Channel 0
	RC_Ctl.rc.ch1 = (((DBUS_rx_buffer[1] >> 3) | (DBUS_rx_buffer[2] << 5)) & 0x07ff) - 1024; //!< Channel 1
	RC_Ctl.rc.ch2 = (((DBUS_rx_buffer[2] >> 6) | (DBUS_rx_buffer[3] << 2) | (DBUS_rx_buffer[4] << 10)) & 0x07ff) - 1024;//!< Channel 2
	RC_Ctl.rc.ch3 = (((DBUS_rx_buffer[4] >> 1) | (DBUS_rx_buffer[5] << 7)) & 0x07ff) - 1024; //!< Channel 3
	RC_Ctl.rc.s1 = ((DBUS_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
	RC_Ctl.rc.s2 = ((DBUS_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
  RC_Ctl.rc.thumbwheel = ((int16_t)DBUS_rx_buffer[16] | ((int16_t)DBUS_rx_buffer[17] << 8)) & 0x07FF;
  RC_Ctl.rc.thumbwheel-=1024;
	RC_Ctl.mouse.x = DBUS_rx_buffer[6] | (DBUS_rx_buffer[7] << 8); //!< Mouse X axis  代表鼠标移动的速度
	RC_Ctl.mouse.y = DBUS_rx_buffer[8] | (DBUS_rx_buffer[9] << 8); //!< Mouse Y axis
	RC_Ctl.mouse.z = DBUS_rx_buffer[10] | (DBUS_rx_buffer[11] << 8); //!< Mouse Z axis
	RC_Ctl.mouse.press_l = DBUS_rx_buffer[12]; //!< Mouse Left Is Press ?
	RC_Ctl.mouse.press_r = DBUS_rx_buffer[13]; //!< Mouse Right Is Press ?
	RC_Ctl.key.v = DBUS_rx_buffer[14] | (DBUS_rx_buffer[15] << 8); //!< KeyBoard value
	RC_Ctl.key.W = DBUS_rx_buffer[14] & 0x01;
	RC_Ctl.key.S = (DBUS_rx_buffer[14]>>1) & 0x01;
	RC_Ctl.key.A = (DBUS_rx_buffer[14]>>2)& 0x01;
	RC_Ctl.key.D = (DBUS_rx_buffer[14]>>3) & 0x01;
	RC_Ctl.key.Shift = (DBUS_rx_buffer[14]>>4) & 0x01;
	RC_Ctl.key.Ctrl =  (DBUS_rx_buffer[14]>>5) & 0x01;
	RC_Ctl.key.Q =  (DBUS_rx_buffer[14]>>6) & 0x01; 
	RC_Ctl.key.E =  (DBUS_rx_buffer[14]>>7) & 0x01;	
	
	RC_Ctl.key.R = (DBUS_rx_buffer[15]) & 0x01;
	RC_Ctl.key.F = (DBUS_rx_buffer[15]>>1) & 0x01;
	RC_Ctl.key.G = (DBUS_rx_buffer[15]>>2) & 0x01;
	RC_Ctl.key.Z = (DBUS_rx_buffer[15]>>3) & 0x01;
	RC_Ctl.key.X = (DBUS_rx_buffer[15]>>4) & 0x01;
	RC_Ctl.key.C = (DBUS_rx_buffer[15]>>5) & 0x01;
	RC_Ctl.key.V = (DBUS_rx_buffer[15]>>6) & 0x01;
	RC_Ctl.key.B = (DBUS_rx_buffer[15]>>7) & 0x01;	
  
	RC_Ctl.IsDbusOnline = 1;
	
	return HAL_OK;
}

/** 
* @brief  DBUS初始化
* @details  This is the detail description. 
* @par log
*/
void DBUS_Init(void)
{
	RC_Ctl.IsDbusOnline = 1;
	__HAL_UART_ENABLE_IT(&HDBUS,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&HDBUS,(uint8_t *)DBUS_rx_buffer,sizeof(DBUS_rx_buffer));
}
/** 
* @brief  DBUS中断处理
* @details  收到数据会进中断，将数据存储到Ctl结构体中,要将此函数放进HDBUS_IRQHandler中
* @par log
*/
void DBUS_IT(void) 
{
	
	if(__HAL_UART_GET_FLAG(&HDBUS,UART_FLAG_IDLE))
	{
		HAL_UART_DMAStop(&HDBUS);
		DBUS_remember();
		Mode_Air_Choose(RC_Ctl.rc.s1,RC_Ctl.rc.s2);  //临时加来气缸用的，可以删掉
		__HAL_UART_CLEAR_IDLEFLAG(&HDBUS);
		HAL_UART_Receive_DMA(&HDBUS,DBUS_rx_buffer,25);
	}
}

