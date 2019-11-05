/**
*   MPU6500具有16位加速度AD输出，三个16位陀螺仪AD输出，三个16位磁力计AD输出
*   陀螺仪可选量程250,500,1000,2000(dps)，加速度计2,4,6,8,16(g)，磁力计4800uT
*   通信采用400KHz的I2C和1MHz的SPI，若需更快速度，可用SPI在20MHz的模式下直接读取传感器和中断寄存器
*   9250片选引脚必须改为SPI1_nCS
**/
#include "bsp_mpu9250.h"

static uint8_t tx, rx;
/** 
* @brief   往寄存器中写数据
* @remarks 第一个8位发送RW,MS,AD5`AD0，第二个八位发送D17`D10，
* @retval  Null     
*/
static void SPI_MPU_Write_Reg(uint8_t regAddr, uint8_t data){
	Slaver_Enable();   
	HAL_SPI_Transmit(&hspi1, &regAddr,1,0x05);
	HAL_SPI_Transmit(&hspi1, &data,1,0x05);
	Slaver_Disable();
	HAL_Delay(5);
}	
/** 
* @brief   单次的读取
* @remarks 一次只能读取一个寄存器的值，即8位
*/
uint8_t SPI_MPU_Read(uint8_t regAddr){
	uint8_t temp;
	SPI_MPU_Read_Regs(regAddr,&temp,1);
	return temp;	
}

	/** 
* @brief   从寄存器中读数据
* @remarks 与上面不一样，RW需要为1
*          同时，需要先发送数据（瞎几把发8位就行了）
* @retval  读取到的8位数据    
* @bug     磁力计要按照标准的0xC0才能连续读取，其他要0x80
*/
void SPI_MPU_Read_Regs(uint8_t regAddr,uint8_t *pBuff,uint8_t length)
{
  tx = regAddr | 0x80;
  Slaver_Enable();
	HAL_SPI_TransmitReceive(&MPU_SPI, &tx, &rx, 1, 0x05);
	HAL_SPI_Receive(&MPU_SPI, pBuff, length, 0x05);
	Slaver_Disable();	
}
/** 
* @brief   磁力计写数据
* @remarks 通过SPI给磁力计写数据
* @pra    
*/
void AK8963_Write_Reg(uint8_t addr, uint8_t data)
{
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x0C); //写
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG,addr);
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x81);	
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_DO,data);
}
/** 
* @brief   磁力计读数据
* @remarks 通过SPI读取磁力计一个寄存器数据
* @pra    
*/
uint8_t AK8963_Read_Reg(uint8_t addr)
{
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x8C); //读
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG, addr);  // 地址
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x81);  // 读到MPU6500_EXT_SENS_DATA_00	
	return  SPI_MPU_Read(MPU6500_EXT_SENS_DATA_00);
}
/** 
* @brief   磁力计读数据
* @remarks 通过SPI读取磁力计多个寄存器数据
* @pra    
*/
void AK8963_Read_Regs(uint8_t addr,uint8_t *pBuff,uint8_t len)
{
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x8C); //读
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG, addr);  // 地址
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x80+len);  // 读到MPU6500_EXT_SENS_DATA_00
	SPI_MPU_Read_Regs(MPU6500_EXT_SENS_DATA_00,pBuff,len);
}

void Mag_Read(uint8_t *pBuff)
{
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x8C); //读
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG, AK8963_HXL);  // 地址
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x86);  // 读到MPU6500_EXT_SENS_DATA_00
	SPI_MPU_Read_Regs(MPU6500_EXT_SENS_DATA_00,pBuff,1);
	AK8963_Write_Reg(AK8963_CNTL1,0x16);
}

/** 
* @brief   对MPU6500进行配置
* @remarks 需要在工程中初始化
*/
uint8_t AK8963_ASA[3];
void MPU9250_Init(void)
{
	SPI_MPU_Write_Reg(MPU6500_PWR_MGMT_1,0x80);   // 清除内部寄存器为默认设置 
	SPI_MPU_Write_Reg(MPU6500_PWR_MGMT_1,0x01);   // Clock Source
	SPI_MPU_Write_Reg(MPU6500_CONFIG,0x04);       // 低通滤波频率5  250HZ (00) 184 92 41 20 10 5 3600 
	SPI_MPU_Write_Reg(MPU6500_USER_CTRL,0x20);    // SPI + I2C master Enable AUX
/*************************** 初始化陀螺仪和加速度计 **************************/	
	SPI_MPU_Write_Reg(MPU6500_SMPLRT_DIV,0x00);      // sample rate,this is the updata rate of sensor register 1000Hz
	SPI_MPU_Write_Reg(MPU6500_GYRO_CONFIG,0x18);     // +-2000dps 不自检
	SPI_MPU_Write_Reg(MPU6500_ACCEL_CONFIG,0x00);    // +-2G
	SPI_MPU_Write_Reg(MPU6500_ACCEL_CONFIG_2,0x06);  // 加速计低通滤波器设置 ，5Hz
	SPI_MPU_Write_Reg(MPU6500_I2C_MST_CTRL,0x5D);    // I2C Speed 400 kHz there is a stop between reads.	
/*************************** 初始化磁力计 **************************/		
	AK8963_Write_Reg(AK8963_CNTL2,0x01);         // reset
	AK8963_Write_Reg(AK8963_CNTL1,0x10);         // Power-down mode
	AK8963_Write_Reg(AK8963_CNTL1,0x1F);   			 // Fuse ROM access mode
	AK8963_Read_Regs(AK8963_ASAX,AK8963_ASA,3);  // AK8963 get calibration data
	AK8963_Write_Reg(AK8963_CNTL1,0x10);         // Power-down mode
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x8C);       // 读
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_REG, AK8963_ST1);  // 地址
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV0_CTRL,0x88);	
  AK8963_Write_Reg(AK8963_CNTL1,0x16);         // Continuous measurement mode 2
	SPI_MPU_Write_Reg(MPU6500_I2C_SLV4_CTRL,0x09);  //When enabled, slave 0 will only be accessed 1+I2C_MST_DLY) samples as determined by SMPLRT_DIV and DLPF_CFG
	SPI_MPU_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL,0x81); //[7]Delays shadowing of external sensor data until all data is received 	
}
