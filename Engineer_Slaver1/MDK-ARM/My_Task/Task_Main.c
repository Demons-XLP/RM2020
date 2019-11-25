#include "task_main.h"
#include "Car_Driver.hpp"
#include "bsp_can.hpp"
#include "can.h"
#include "cmsis_os.h"
#include "app_mode.h" 

static uint8_t Air_Flag = 0; //气缸flag,0开，1关
/*
* @brief  气缸控制flag
* @details  
* @param  NULL
* @retval  NULL
*  2019.11.20 添加
*
*/
static void Air_Cylinder_Ctr(uint8_t flag)
{
 if(flag == 0)
 {	 
//	 Claw_Open;    //用到副控时一定记得重新宏定义！！！！
 }
 else if(flag == 1)
 {
//	 Claw_Close;
 }
}


/*
* @brief  主线程函数
* @details 
* @param  NULL
* @retval  NULL
*  2019.11.20 添加
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
		manager::CANSend();  //托管
		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS);
 	}
}
