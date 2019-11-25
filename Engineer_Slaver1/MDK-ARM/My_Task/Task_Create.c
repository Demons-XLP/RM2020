#include "task_create.h"
#include "cmsis_os.h"
#include "task.h"
#include  "task_main.h"


//osThreadId MonitorTaskHandle;
osThreadId MainTaskHandle;
//osThreadId AutoTaskHandle;
void CreateTask(void const * argument)
{
  osThreadDef(MainTask, MainTask, osPriorityNormal, 0, 128);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);
}
