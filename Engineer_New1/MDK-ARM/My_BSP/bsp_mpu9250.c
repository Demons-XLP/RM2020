/** 
* @brief    MPU9250��ز���
* @details  ͨ��SPI��ȡ��д��MPU9250�ڲ��Ĵ�������ʼ��
* @author   Evan-GH
* @date      2019.10
* @version  1.1
* @par Copyright (c):  RM2020���
* @par ��־
* @par 	����ʹ�÷�����Readme.md
*				�汾���:
*				1.0		|		���ݴ���淶�޸���RM19�����ǿ����غ����������ļ������
*				1.1		|		ע�͵�һЩû�����ϵĺ��������Ժ����������û��ɾ�����ȷ���
*				1.2		|		��װ��SPI�ӿں궨��
*/

#include "bsp_mpu9250.h"
/**
* @brief  MPU9250Ƭѡ
* @details  ͨ��SPI����Ƭѡ9250оƬ
* @param  NULL
* @retval  NULL
*/
static void bsp_mpu9250_ChipSelect(void)
{
	HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port,SPI1_nCS_Pin,GPIO_PIN_RESET);	//ƬѡMPU9250
}

/**
* @brief  MPU9250����Ƭѡ
* @details  ͨ��SPI���߷���Ƭѡ9250оƬ
* @param  NULL
* @retval  NULL
*/
static void bsp_mpu9250_NoChipSelect(void)
{
	HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port,SPI1_nCS_Pin,GPIO_PIN_SET);	//����ƬѡMPU9250
}

/**
* @brief  MPU9250д�Ĵ���
* @details  ͨ��SPI������MPU9250�Ĵ���д������
* @param  Regaddr �Ĵ�����ַ Dataд�������
* @retval  NULL
*/
static void bsp_mpu9250_Writereg(uint8_t Regaddr, uint8_t Data)
{
	bsp_mpu9250_ChipSelect();
	HAL_SPI_Transmit(&BSP_MPU9250_USE_SPI, &Regaddr,1,0x05);
	HAL_SPI_Transmit(&BSP_MPU9250_USE_SPI, &Data,1,0x05);
	bsp_mpu9250_NoChipSelect();
	HAL_Delay(5);
}

/**
* @brief  ��ȡMPU9250�Ĵ���ֵ
* @details  ͨ��SPI���߶�ȡMPU9250�мĴ�����ֵ
* @param  uint8_t Regadrr �Ĵ�����ַ
* @retval  uint8_t Spi_Rx ��ȡ�õ�������
*/
uint8_t bsp_mpu9250_Readreg(uint8_t Regaddr)
{
	uint8_t temp;
	bsp_mpu9250_Readregs(Regaddr,&temp,1);
	return temp;	
}

/**
* @brief  ��ȡMPU9250����Ĵ���ֵ
* @details  ͨ��SPI���߶�ȡMPU9250����������Ĵ�����ֵ
* @param  Regadrr �Ĵ�����ַ Read_Buffer �������ݵ����� Length�Ĵ�������
* @retval  NULL
*/
void bsp_mpu9250_Readregs(uint8_t Regaddr, uint8_t* Read_Buffer, uint8_t Length)
{
	static uint8_t Spi_Tx,Spi_Rx;
	Spi_Tx = Regaddr | 0x80;
  bsp_mpu9250_ChipSelect();
	HAL_SPI_TransmitReceive(&BSP_MPU9250_USE_SPI, &Spi_Tx, &Spi_Rx, 1, 0x05);
	HAL_SPI_Receive(&hspi1, Read_Buffer, Length, 0x05);
	bsp_mpu9250_NoChipSelect();
}

/**
* @brief  �����ƼĴ���д��
* @details  ͨ��SPI���߶�MPU9250�ڲ������ƼĴ���д������
* @param  uint8_t Regaddrд��ļĴ�����ַ, uint8_t dataд�������
* @retval  NULL
*/
void bsp_SPI_AK8963_Writereg(uint8_t Regaddr, uint8_t data)
{
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_ADDR, 0x0C); //д
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_REG,Regaddr);
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_CTRL,0x81);	
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_DO,data);
}

/**
* @brief  �����ƼĴ�����ȡ
* @details  ͨ��SPI���߶�ȡMPU9250�ڲ��Ĵ����ƼĴ�������
* @param  Readaddr �Ĵ�����ַ
* @retval  ��ȡ��������
*/
uint8_t bsp_SPI_AK8963_Readreg(uint8_t Regaddr)
{
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_ADDR, 0x8C); //��
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_REG, Regaddr);  // ��ַ
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_CTRL,0x81);  // ����MPU6500_EXT_SENS_DATA_00	
	return  bsp_mpu9250_Readreg(MPU6500_EXT_SENS_DATA_00);
}

/**
* @brief  �����ƼĴ���������ȡ
* @details  ͨ��SPI��ȡMPU9250�ڲ���������Ĵ�������
* @param  Regadrr �Ĵ�����ַ Read_Buff�������ݵ����� Length��ȡ����
* @retval  NULL
*/
void bsp_SPI_AK8963_Readregs(uint8_t Regaddr,uint8_t *Read_Buff,uint8_t Length)
{
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_ADDR, 0x8C); //��
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_REG, Regaddr);  // ��ַ
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_CTRL,0x80+Length);  // ����MPU6500_EXT_SENS_DATA_00
	bsp_mpu9250_Readregs(MPU6500_EXT_SENS_DATA_00,Read_Buff,Length);
}

uint8_t bsp_mpu9250_AK8963_ASA[3];
/**
* @brief  MPU9250��ʼ��
* @details  ͨ��SPI���߶�MPU9250�ڲ��Ĵ���д����������ʼ������ID�ż�ⲻͨ��ʱ����������г�ʼ�����ǽ�����ѭ��
* @param  NULL
* @retval  NULL
*/
void bsp_mpu9250_Init(void)
{
	//while(bsp_mpu9250_Readreg(0X75) == 0X71);			//MPU9250��ID��⣬��������ط����ܻῨ��ʼ����������ע���ˣ�����������ԭ��
	
	bsp_mpu9250_Writereg(MPU6500_PWR_MGMT_1,0x80);   // ����ڲ��Ĵ���ΪĬ������ 
	bsp_mpu9250_Writereg(MPU6500_PWR_MGMT_1,0x01);   // Clock Source
	bsp_mpu9250_Writereg(MPU6500_CONFIG,0x04);       // ��ͨ�˲�Ƶ��5  250HZ (00) 184 92 41 20 10 5 3600 
	bsp_mpu9250_Writereg(MPU6500_USER_CTRL,0x20);    // SPI + I2C master Enable AUX
//*************************** ��ʼ�������Ǻͼ��ٶȼ� **************************
	bsp_mpu9250_Writereg(MPU6500_SMPLRT_DIV,0x00);      // sample rate,this is the updata rate of sensor register 1000Hz
	bsp_mpu9250_Writereg(MPU6500_GYRO_CONFIG,0x18);     // +-2000dps ���Լ�
	bsp_mpu9250_Writereg(MPU6500_ACCEL_CONFIG,0x00);    // +-2G
	bsp_mpu9250_Writereg(MPU6500_ACCEL_CONFIG_2,0x06);  // ���ټƵ�ͨ�˲������� ��5Hz
	bsp_mpu9250_Writereg(MPU6500_I2C_MST_CTRL,0x5D);    // I2C Speed 400 kHz there is a stop between reads.	
//*************************** ��ʼ�������� **************************
	bsp_SPI_AK8963_Writereg(AK8963_CNTL2,0x01);         // reset
	bsp_SPI_AK8963_Writereg(AK8963_CNTL1,0x10);         // Power-down mode
	bsp_SPI_AK8963_Writereg(AK8963_CNTL1,0x1F);   			 // Fuse ROM access mode
	bsp_SPI_AK8963_Readregs(AK8963_ASAX,bsp_mpu9250_AK8963_ASA,3);  // AK8963 get calibration data
	bsp_SPI_AK8963_Writereg(AK8963_CNTL1,0x10);         // Power-down mode
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_ADDR, 0x8C);       // ��
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_REG, AK8963_ST1);  // ��ַ
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV0_CTRL,0x88);	
  bsp_SPI_AK8963_Writereg(AK8963_CNTL1,0x16);         // Continuous measurement mode 2
	bsp_mpu9250_Writereg(MPU6500_I2C_SLV4_CTRL,0x09);  //When enabled, slave 0 will only be accessed 1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG
	bsp_mpu9250_Writereg(MPU6500_I2C_MST_DELAY_CTRL,0x81); //[7]Delays shadowing of external sensor data until all data is received 	
}
