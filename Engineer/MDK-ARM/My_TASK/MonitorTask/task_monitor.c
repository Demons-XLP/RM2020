#include "task_monitor.h"

/** 
* @brief  DBUS监视
* @details  进入DBUS中段时，RC_Ctl.IsDbusOnline会被赋值为1，进入监视函数则会被赋值为0，若隔30ms还一直为0，则判定为离线，进入安全模式
* @par 2019.10.23添加
*/


void Task_Monitor(void const *argument)
{
  DBUS_Monitor();
}

void DBUS_Monitor(void)
{
  RC_Ctl.IsDbusOnline = 0;  //重新赋值
	HAL_Delay(10);
	if (RC_Ctl.IsDbusOnline == 0)
	{
	  RC_Ctl.rc.s1 = 2;  //安全模式的条件
		RC_Ctl.rc.s2 = 2;
	}
}
