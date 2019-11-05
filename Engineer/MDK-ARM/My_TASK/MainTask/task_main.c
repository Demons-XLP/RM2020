#include "task_main.h"
#include "my_bsp.h"
#include "my_app.h"
#include "cmsis_os.h"

void Task_Main(void const *argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();  //��ȡ��ǰ��ϵͳʱ��
  MPU9250_Init();  //�����ǳ�ʼ��
	IMU_Init();  //��̬�����ںϳ�ʼ������
  bsp_can_Init();  
  DBUS_Init();  //DBUS��ʼ��
	PID_Init();  //PID��ʼ��
	
	
	for (;;)
	{
		Mode_Choose(RC_Ctl.rc.s1,RC_Ctl.rc.s2);  //ģʽ������Ŀǰ���еĵ��̵��԰�����̨��������
//	  IMUSO3Thread();
		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS);
	}
}
