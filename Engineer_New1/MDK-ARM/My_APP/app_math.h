#ifndef __APP_MATH_H
#define __APP_MATH_H
#include "stm32f4xx.h"
#include "math.h"

#define APP_MATH_ABS(x)   ((x)>0?(x):-(x))
#define 	PI 		3.1415926535f
#define SECTOR_11								FLASH_SECTOR_11
#define SECTOR_11_STR_ADDR			(uint32_t)(0x080E0000)	
#define USER_FLASH_END_ADDRESS	(uint32_t)(0x080FFFFF)

typedef struct  //< butterworth lowpass filter 
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float G;   //输出增益
    
    float current_input;		     
    float last_input;			       
    float pre_input;
    
    float current_output;		            //y[n]
    float last_output;			            //y[n-1]
    float pre_output;	                  //y[n-2]
}IIR;

typedef struct 
{
    float P_last;				/*上次预测过程协方差矩阵P(k|k-1)*/
    float X_last;				/*系统状态预测矩阵X(k|k-1)，列矩阵*/
    
    float Q;						/*过程白噪声协方差系数*/
    float R;						/*观测白噪声协方差系数*/
    
    float K;						/*卡尔曼增益K(k)，列矩阵*/
    float X;						/*最优估计状态变量矩阵X(k|k)，列矩阵*/
    float P;						/*最优估计协方差矩阵P(k|k)*/
                                                        
    float input;				    /*系统输出值，即Z(k)*/
    uint8_t flag;
}kalman_filter;	

typedef struct 
{
	float _cutoff_freq;
	float _a1;
	float _a2;
	float _b0;
	float _b1;
	float _b2;
	float _delay_element_1;        // buffered sample -1
	float _delay_element_2;        // buffered sample -2	
}LPF2;

float app_math_Limit(float Data,float Max,float Min);//数据限幅
void app_math_LPF2pSetCutoffFreq(LPF2* LPF,float sample_freq, float cutoff_freq);
float app_math_LPF2pApply(LPF2* LPF,float sample);
float app_math_Kalman(kalman_filter* kalman,float input);
float app_math_invSqrt(float number);

#endif
