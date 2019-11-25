#include "task_main.h"
#include "Car_Driver.hpp"
#include "bsp_can.hpp"
#include "can.h"
#include "cmsis_os.h"
#include "app_mode.h" 

static uint8_t Air_Flag = 0; //����flag,0����1��
/*
* @brief  ���׿���flag
* @details  
* @param  NULL
* @retval  NULL
*  2019.11.20 ���
*
*/
static void Air_Cylinder_Ctr(uint8_t flag)
{
 if(flag == 0)
 {	 
//	 Claw_Open;    //�õ�����ʱһ���ǵ����º궨�壡������
 }
 else if(flag == 1)
 {
//	 Claw_Close;
 }
}


/*
* @brief  ���̺߳���
* @details 
* @param  NULL
* @retval  NULL
*  2019.11.20 ���
*
*/
void MainTask(void const * argument)
{
	 
 	manager::CANSelect(&hcan1,NULL);
  bsp_can_Init();
//	DBUS_Init();
	for(;;)
	{
		static TickType_t xLastWakeTime;
//	  Mode_Choose(RC_Ctl.rc.s1,RC_Ctl.rc.s2);
		Air_Cylinder_Ctr(Air_Flag);
		manager::CANSend();  //�й�
		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS);
 	}
}
