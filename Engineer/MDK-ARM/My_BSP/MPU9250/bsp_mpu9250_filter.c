/*****************************************************************************
*                                                                            *
*  @file     filter.c                                                        *
*  @brief    flash���� + �˲�����                                            *
*  @details  ��Ҫ�������������ֵ�洢��ԭʼֵ���˲�                          *
*  @see      quad_math.c                                                     *
*                                                                            *
*  @author   Nankel  Li                                                      *
*  @email    2512126660@qq.com                                               *                                             
*  @warning                                                                  *
*----------------------------------------------------------------------------*
*  @remark                                                                   *
*----------------------------------------------------------------------------*
*  @version                                                                  *
*           <Date>      |<Version>| <Author>   | <Description>               *
*----------------------------------------------------------------------------*
* 2019��4��12��10:00:11 |   1.0   |  Nankel    | Creat the file              *
*****************************************************************************/
#include "string.h"
#include "bsp_mpu9250_filter.h"

/**
* @Brief��kalman�˲�
* @Param: ���ڵݹ�����˲��㷨���ǹ��ƾ��������С������Ҫ����������
*�˺���һ��ֻ��Ϊһ�������˲������۲����H=1��A=1
*����ϵͳ���ܿأ���B=0
*�������˲�������ʽ(Ԥ��(ʱ�����)��У��(��������))��
1��״̬Ԥ�ⷽ�̣�  X(k|k-1) = A*X(k-1|k-1)+B*U(k)    //kʱ��ϵͳԤ��ֵ=A*(k-1ʱ��ϵͳ״̬����������ֵ)+B*ϵͳ����
2��Э����Ԥ�ⷽ�̣�P(k|k-1)=A*P(k-1|k-1)A��ת��+Q    //kʱ��ϵͳЭ�������Ԥ��ֵP(k|k-1)��k-1ʱ��ϵͳЭ�������P(k-1|k-1)
3��������������㷽�̣�K(k)=P(k|k-1)H��ת��/[H*P(k|k-1)*H��ת��+R]
4������ֵ���·��̣� X(k|k)=X(k|k-1)+K(k)*(Z(k)-H*X(k|k-1))   //kʱ��״̬��������ֵX(k|k)��ϵͳ�������Z(k)
5��Э������·��̣� P(k|k)=(1-K(k)*H)*P(k|k-1)
6��ϵ��QԽС���˳���������Խǿ��ϵ��RԽС���˲���Ӧ������ԽѸ�١�
 **/
float Kalman(kalman_filter* kalman,float input)
{
    if(kalman->flag== 0)
    {
       kalman->P_last = 1.0f; 
       kalman->Q = 0.01; //0.0001
       kalman->R = 4.57;   //4.57
       kalman->flag = 1;
    }
    kalman->input = input;
	/*������£�3�鷽��*/
	kalman->K = (kalman->P_last)/(kalman->P_last + kalman->R);
	kalman->X  = kalman->X_last + kalman->K * (kalman->input - kalman->X_last);
	kalman->P =  (1-kalman->K)*(kalman->P_last);
	/*ʱ����£�2�鷽��*/
	kalman->X_last = kalman->X;										
	kalman->P_last = kalman->P + kalman->Q;			
	
	return kalman->X;
}

/**
  * @brief   IIR�˲��������޳����Ӧ�˲�������Butterworth IIR Lowpass
  * @details ���Ϊ��ͨIIR�˲���
  * @par     ����ָ�꣺����Ƶ��f0,����Ƶ��fs,����dB,Ʒ������   
  *��ͨ�˲�����Ƶ��=��ֹƵ��
  *��Ƶ��omega=2��*f0/sampleRate
  *��sin=sin(omega),cos=cos(omega),alpha=sin/(2*Q)
  *�����IIR��ͨ�˲�����ϵ��Ϊ��b0=(1-cos)/2, b1=1-cos, b2=(1-cos)/2, a0=1+alpha, a1=-2cos, a2=1-alpha;
  *����matlab����fdatool������ֹƵ��30Hz�����Ƶ��800Hz,����b0=1, b1=2, b2=1, a0=1, a1=-1.6692031429311931, a2=0.71663387350415764, Scale Values:0.011857682643241156
	*1KHz����Ƶ�ʣ�20Hz��ֹƵ��  1,2,1,1��-1.8226949251963083,0.83718165125602262��0.0036216815149286421
  **/  
float IIR_LPF(IIR* IIR,float input)
{
	IIR->current_input = input;
  IIR->current_output = IIR->b0*IIR->current_input+IIR->b1*IIR->last_input+IIR->b2*IIR->pre_input-(IIR->a1*IIR->last_output+IIR->a2*IIR->pre_output);
	IIR->pre_input = IIR->last_input;
	IIR->last_input = IIR->current_input;
	IIR->pre_output = IIR->last_output;
	IIR->last_output = IIR->current_output;	
	return IIR->G*IIR->current_output;
}

/** 
* @brief    ���׵�ͨ�˲���
* @remarks 
* @par ��־
*/
void LPF2pSetCutoffFreq(LPF2* LPF,float sample_freq, float cutoff_freq)   //��ϵ���ģ���ʡ��Ҳ�У�ϵ���͵��Լ�����
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(PI/fr);
		c=1.0f+2.0f*cosf(PI/4.0f)*ohm + ohm*ohm;
	
    LPF->_cutoff_freq = cutoff_freq;
    if (LPF->_cutoff_freq > 0.0f) 
		{
				LPF->_b0 = ohm*ohm/c;
				LPF->_b1 = 2.0f*LPF->_b0;
				LPF->_b2 = LPF->_b0;
				LPF->_a1 = 2.0f*(ohm*ohm-1.0f)/c;
				LPF->_a2 = (1.0f-2.0f*cosf(PI/4.0f)*ohm+ohm*ohm)/c;
		}
}

float LPF2pApply(LPF2* LPF,float sample)
{
		float delay_element_0 = 0, output=0;	
    if (LPF->_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }
		else
		{
				delay_element_0 = sample - LPF->_delay_element_1 * LPF->_a1 - LPF->_delay_element_2 * LPF->_a2;
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) 
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;

				output = delay_element_0 * LPF->_b0 + LPF->_delay_element_1 * LPF->_b1 + LPF->_delay_element_2 * LPF->_b2;
				
				LPF->_delay_element_2 = LPF->_delay_element_1;
				LPF->_delay_element_1 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
		}
}
/******************************************Flash����**************************************************/
/** 
* @brief    flashд�����
* @remarks  д����Ҫ�����Ͳ���,���ڴ浹����ǰд
* @pra    
*/
uint8_t Flash_WriteIn(uint8_t destination,int16_t* pData,uint8_t len)
{
	uint32_t Address = SECTOR_11_STR_ADDR + destination*2;  //��ʼ����ʼλ��
	HAL_FLASH_Unlock();  //����
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR | FLASH_FLAG_OPERR);
	FLASH_Erase_Sector(FLASH_SECTOR_11,FLASH_VOLTAGE_RANGE_3);  //����
	for(uint8_t i = 0; (i < len) && (Address < USER_FLASH_END_ADDRESS - 4); i++) //д��
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address + i*2,*(pData+i)) != HAL_OK)
			return 0;    //д��ʧ��
	}
	HAL_FLASH_Lock(); //����
	return 1;   //д��ɹ������ش������ݵ����һ����ַ	
}

/** 
* @brief    flash��ȡ����
* @remarks  �������
* @pra    
*/
int16_t Flash_ReadOut(uint8_t destination)
{
	int16_t pData;
	uint32_t Address = SECTOR_11_STR_ADDR + destination*2;  //��ʼ����ʼλ��
	memcpy(&pData,(uint32_t*)Address,2);
	return pData;
}

