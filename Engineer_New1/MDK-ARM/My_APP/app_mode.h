#ifndef __APP_MODE_H
#define __APP_MODE_H
#include "stm32f4xx_hal.h"
#include "bsp_dbus.h"
#include "Car_Driver.hpp"
void Mode_Choose(int s1,int s2);
void Mode_Air_Choose(int s1,int s2);

extern chassis Chassis_Engineer;  //底盘类
extern softmotor Cloud_Engineer_Pitch;  //Pitch云台类
extern softmotor Cloud_Engineer_Yaw; //Yaw云台类
extern float Pitch_G_Angle,Pitch_G_Angle_Rate,Yaw_G_Angle,Yaw_G_Angle_Rate;  //用陀螺仪模式的实际角度与速度

#endif

