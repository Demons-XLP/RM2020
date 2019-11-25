#ifndef  __ADC_DEAL_H
#define  __ADC_DEAL_H
#include "adc.h"
#define ADC_USE_ADC1 hadc1
#define ADC_USE_ADC2 hadc2

extern uint16_t final_ADC1_volage[4];
extern uint16_t ADC1_Value[400];
extern float    ADC1_Sharp_Distance[4];

extern uint16_t final_ADC2_volage[4];
extern uint16_t ADC2_Value[400];
extern float    ADC2_Sharp_Distance[4];
void ADC_Sensor_Init();



#endif  /*__ADC_DEAL_H*/
