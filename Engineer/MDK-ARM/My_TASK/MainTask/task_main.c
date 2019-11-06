#include "task_main.h"
#include "my_bsp.h"
#include "my_app.h"
#include "cmsis_os.h"

void Task_Main(void const *argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();  //获取当前的系统时间
	
	taskENTER_CRITICAL(); //进入临界区域不被中断打扰，有利于陀螺仪成功初始化
  MPU9250_Init();  //陀螺仪初始化
	IMU_Init();  //姿态解算融合初始化函数
	taskEXIT_CRITICAL();  //退出临界区域
  bsp_can_Init();  
	
  DBUS_Init();  //DBUS初始化
	PID_Init();  //PID初始化
	
	
	for (;;)
	{
    taskENTER_CRITICAL();  //进入临界防打扰
    IMUSO3Thread();  //9250姿态解算
		taskEXIT_CRITICAL(); 
		
		Mode_Choose(RC_Ctl.rc.s1,RC_Ctl.rc.s2);  //模式函数，目前所有的底盘调试包括云台都在里面
		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS);

	}
}
