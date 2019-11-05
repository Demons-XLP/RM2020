/** 
* @brief        BSP����DBUS
* @details  �ο�WMDģ�黯ɢװ�⡪��Dbus��
* @author      mijures
* @date     2018.10.11
* @version  v0.1
* @par Copyright (c):      mijures
* @par 0������
* @par ʹ�÷�����1.��ͷ�ļ��и��Ķ�Ӧuart�ĺ궨�� 2.��DBUS_IT�ŵ���Ӧ�����жϺ����� 3.ʹ��DBUS_Init��ʼ��DBUS 
* @par ��������ʱ�������⻹Ҫע�Ⲩ���ʵ�����
* @par v0.1 �����˺���λ�ú�ͷ�ļ��Ĳ������ݣ�ʹ���ø��ӷ��㣬�����˱�׼��ע��
*/  

#include "bsp_dbus.h"
#include "usart.h"
#include "app_mode.h"
unsigned char DBUS_rx_buffer[25]={0};//DBUS���ݻ���
RC_Ctl_t RC_Ctl; //������ң����������

/** 
* @brief  DBUS���ݴ洢
* @details  �����ܵ������ݴ���洢��RC_Ctl��
* @param  NULL
* @retval  HAL_OK �ɹ� HAL_ERROR ʧ��
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
//	if(i==18)//������ ���ش���
//	{
//		return HAL_ERROR;
//	}
//	for(uint8_t i=18;i<25;i++)//0���������
//	{
//		if(DBUS_rx_buffer[i]!=0)		//0��������18-24���������� ����/�������
//		{
//			return HAL_ERROR;
//		}
//	}
	
	//���ݴ洢
	RC_Ctl.rc.ch0 = ((DBUS_rx_buffer[0]| (DBUS_rx_buffer[1] << 8)) & 0x07ff) - 1024; //!< Channel 0
	RC_Ctl.rc.ch1 = (((DBUS_rx_buffer[1] >> 3) | (DBUS_rx_buffer[2] << 5)) & 0x07ff) - 1024; //!< Channel 1
	RC_Ctl.rc.ch2 = (((DBUS_rx_buffer[2] >> 6) | (DBUS_rx_buffer[3] << 2) | (DBUS_rx_buffer[4] << 10)) & 0x07ff) - 1024;//!< Channel 2
	RC_Ctl.rc.ch3 = (((DBUS_rx_buffer[4] >> 1) | (DBUS_rx_buffer[5] << 7)) & 0x07ff) - 1024; //!< Channel 3
	RC_Ctl.rc.s1 = ((DBUS_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
	RC_Ctl.rc.s2 = ((DBUS_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
  RC_Ctl.rc.thumbwheel = ((int16_t)DBUS_rx_buffer[16] | ((int16_t)DBUS_rx_buffer[17] << 8)) & 0x07FF;
  RC_Ctl.rc.thumbwheel-=1024;
	RC_Ctl.mouse.x = DBUS_rx_buffer[6] | (DBUS_rx_buffer[7] << 8); //!< Mouse X axis  ��������ƶ����ٶ�
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
* @brief  DBUS��ʼ��
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
* @brief  DBUS�жϴ���
* @details  �յ����ݻ���жϣ������ݴ洢��Ctl�ṹ����,Ҫ���˺����Ž�HDBUS_IRQHandler��
* @par log
*/
void DBUS_IT(void) 
{
	
	if(__HAL_UART_GET_FLAG(&HDBUS,UART_FLAG_IDLE))
	{
		HAL_UART_DMAStop(&HDBUS);
		DBUS_remember();
		Mode_Air_Choose(RC_Ctl.rc.s1,RC_Ctl.rc.s2);  //��ʱ���������õģ�����ɾ��
		__HAL_UART_CLEAR_IDLEFLAG(&HDBUS);
		HAL_UART_Receive_DMA(&HDBUS,DBUS_rx_buffer,25);
	}
}

