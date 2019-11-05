#include "task_monitor.h"

/** 
* @brief  DBUS����
* @details  ����DBUS�ж�ʱ��RC_Ctl.IsDbusOnline�ᱻ��ֵΪ1��������Ӻ�����ᱻ��ֵΪ0������30ms��һֱΪ0�����ж�Ϊ���ߣ����밲ȫģʽ
* @par 2019.10.23���
*/


void Task_Monitor(void const *argument)
{
  DBUS_Monitor();
}

void DBUS_Monitor(void)
{
  RC_Ctl.IsDbusOnline = 0;  //���¸�ֵ
	HAL_Delay(10);
	if (RC_Ctl.IsDbusOnline == 0)
	{
	  RC_Ctl.rc.s1 = 2;  //��ȫģʽ������
		RC_Ctl.rc.s2 = 2;
	}
}
