/*****************************************************************************
*                                                                            *
*  @file     quad_math.c                                                     *
*  @brief    任务列表                                                        *
*  @details                                                                  *
*  @see      mpu9250.c（注意引脚定义，和初始化函数使用）                                                      *
*                                                                            *
*  @author   Nankel  Li                                                      *
*  @email    2512126660@qq.com                                               *                                             
*  @warning  1、如果要使用角速度，注意是滤波和校正后的16位数据               *
             2、使用：把IMU_Init()初始化，把IMUSO3Thread()循环执行           *
						 3、如果使用stm32f1或者内存小的单片机，可能会报错空间不足，调小Zero_Sample_Num即可，一般1000就可以了*
*！！！！！！！！！！！！！总的来说，要使用，必须初始化IMU_Init()和MPU9250_Init(),周期执行IMUSO3Thread(),且片选引脚名字为SPI1_nCS！！！！！！！！！！！！！！！！！！！！！！！！
！！！！！！！！！！！！！！另外，自己看用不用校正，各有利弊，当然如果没有温飘的话，校正才是正道，只有好处，然而。。。。。。。！！！！！！！！！！！！！！！！！！！！！！！！！
！！！！！！！！！！！！！！其实9250的YAW，90%的可能性是能用在云台上的，但是都懒得搞，也持怀疑态度。反正工程车不调云台，只用底盘，谁用谁享受！！！！！！！！！！！！！！！！！！！
*----------------------------------------------------------------------------*
*  @remark   1、使用低通滤波会更好一些，卡尔曼虽然滤波效果不错，但最后几度过于平滑，收敛速度跟不上（可能我用的不到位）*
             2、已经尝试过，单独使用磁力计解算出来的YAW是没法用的，只能指大概方向*
						 3、建议使用soft.Yaw,然而没有动静态校正的话没有卵用，又然而动静态校正的话，融合后的结果会有最多1度左右的变化*
（不校正是0.5度左右），个人猜测是陀螺仪量程缺少一个系数来校正，但是这个系数需要测试，而且会变化，和温度关系挺大，反正我是不介意
介意的话注释掉USE_OFFSET和DYNAMIC_OFFSET就行
    实在想参考YAW，就只给YAW进行校正呗
		         4、Offset_Coeff系数选取方法，开启USE_OFFSET和Simple_Soft，转一圈令soft为360左右即可。只用soft角度的话可以不改，只是转一圈不是360而已，增量式角度单环控制没有校正的必要
						 5、四元数校正PI参数也可以调整。截止频率也可以调整*
*----------------------------------------------------------------------------*
*  @version                                                                  *
*           <Date>      |<Version>| <Author> | <Description>               *
*----------------------------------------------------------------------------*
* 2019年1月24日08:53:30 |   1.0   |  Nankel  | DONE                        *
* 2019年1月30日10:28:12 |   1.1   |  Nankel  | 更改成不使用USE_OFFSET时，只对角速度进行校正，融合输入数据不校正*
* 2019年1月31日13:01:38 |   1.2   |  Nankel  | 新增soft.Roll和pitch，经测试可用在底盘，@remark中第3条不再适用,如果实测云台能用，也建议pitch用soft*
* 2019年1月31日13:12:19 |   1.3   |  Nankel  | 因为9250不是独立板子，故取消初始化环境检测，保证9250不影响程序运行*
* 2019年1月31日17:18:37 |   1.4   |  Nankel  | 增加角速度系数校正，每个板子都不一样，校正越准，融合越快，简单解算的也越准*
* 2019年2月12日23:54:39 |   1.5   |  Nankel  | 修改零点校正环境检测，修改采样和截止频率*
* 2019年2月13日21:43:22 |   1.6   |  Nankel  | 解决9250离线，imu.ready不为0的问题;更改环境检测判断，5s初始化不成功则没有校正*
* 2019年4月4日09:27:02  |   1.7   |  Nankel  | 解决9250离线，imu.ready不为0的问题（更改判断条件，原先的不对）
* 2019年4月5日16:17:18  |   2.0   |  Nankel  | 1、添加Is_Yaw_Invalid参数，如果soft_Yaw不能用，则该参数为1，建议可利用此参数设置提醒
*                                              2、更改自检时间，改为零点采样总时长不允许超过5s(原先的5s内成功只要成功一次就重新计时)，超过5s则Is_Yaw_Invalid为1
*                                              3、鉴于自检存在最高时间限制，建议更换9250时参考其原始数据，设置合适的Zero_Threshold（代码第(”需调整的全局变量“最后一个)行，可以Dbug读取original）
*                                              4、重新启用unstable_num参数，缩小动态零点校正阈值Dynamic_Zero_Thre（4.0f->2.0f）
* 2019年4月12日09:11:50 |   2.1   |  Nankel  | 1、调整了卡尔曼滤波系数，应该能用了，但是感觉不需要。
*                                              2、截止频率可能给的有些高，在速度低，频率低的时候，有些高频噪声，不过据调过云台的人反应，都能调稳(后发现另外一个人的没有问题，应该是个体差异)。
*                                              3、重新调回Dynamic_Zero_Thre（2.0f->4.0f）
* 2019年4月12日19:17:23 |   2.2   |  Nankel  | 1、添加Flash存储每次的校正值，综合计算历史零点值，驱动在filter里面，不想重新创文件了
*                                              2、更改Is_Yaw_Invalid为isThisTimeInvalid，因为flash存在了,所以基本不会因为零点采样漂,此参数用作本次采样失败，调整动态校正作用时间
*                                              3、利用全局宏决定是否使用Flash，若非F405，则取消Flash(但是没测，应该不会出问题)
* 2019年5月31日08:30:38 |   2.3   |  Nankel  | 1、优化部分代码，改变数据类型，节约空间，主要是滤波数据类型和采样数据类型
*                                              2、删除外接磁力计数据，替换为读取9250磁力计数据(可以读出来，但必须保证EDA和ECL引脚悬空，虽然电路组说悬空了，可实测还是剪断才能读出来)
*                                              3、限制动态更新，静止50个周期才开始更新零点（原来容易造成错误采样）
*                                              4、取消成功采样的时候参考Flash内存储的值
*                                              5、放宽动态校正阈值到5.0f
*                                              6、取消注释NonlinearSO3AHRSinit函数，可以使最开始融合的更快
* 2019年6月3日20:17:03  |   2.4   |  Nankel  | 1、增加校正值限幅，也增加了限幅函数，想用这个函数的可自行加extern!!!!!!
*                                              2、增加：满足静止条件50个周期，才开始更新零点
*                                              3、增加：对比动态采样值，进一步筛选数据的可靠性
*                                              4、增加手动补充校正值，原因是原始数据类型是整型造成的，追求完美的可以单独给某个陀螺仪校准!!!!
* 2019年6月7日14:19:55  |   2.5   |  Nankel  | 1、将手动校正值移动到最后积分处，不校正角速度
* 2019年6月8日21:55:35  |   2.6   |  Nankel  | 1、增加陀螺仪分辨率(2000dps->500dps),加速度计分辨率(8g->4g),但是我把Angle_Rate乘了0.25，所以兼容原来的，使用这个参数的不需要再改PID
*                                              2、修改滤波数据类型，原来的是错误的，int16_t->float
*                                              3、删除版本2.5内容
* 2019年6月9日20:50:23  |   2.7   |  Nankel  | 1、修改NonlinearSO3AHRSinit的BUG，融合解算零延时，不再需要等待初始化
* 2019年6月11日10:23:57 |   2.8   |  Nankel  | 1、还原到原来的量程
*                                              2、减小加速度计的低通滤波频率，解决抖的问题
* 2019年6月14日21:26:27 |   2.9   |  Nankel  | 1、修改加速计软硬件低通滤波频率，调整加速计的作用（调整PI），解决融合数据飘的问题
* 2019年7月10日17:26:55 |   3.0   |  Nankel  | 1、将Soft中的PITCH和ROLL改为绝对的，不过范围是+-无穷
*                                              2、将加速度计的量程改为2g，减小动态加速度对重力加速度的影响（好像没用，还是得改滤波）
* 2019年7月12日23:51:24 |   3.1   |  Nankel  | 1、增加积分结构体，取消简单积分宏定义，注意积分角与融合角正方向不同
*****************************************************************************/
#include "bsp_mpu9250.h"
#include "bsp_mpu9250_quad_math.h"
#include "bsp_mpu9250_filter.h"
#include "math.h"

float Limit(float data,float max,float min);
///** 
//* @brief   限幅函数 
//* @remarks min<data<max
//*/
float Limit(float data,float max,float min){
	float Temp = data;
	if(data >= max) Temp = max;
	if(data <= min) Temp = min;
	return Temp;
}


/* 功能性函数宏定义 */
#define micros() 1000*HAL_GetTick()  //计时，单位us,除1000000是秒
#define ABS(x)   ((x)>0?(x):-(x))
//#define USE_MAG         // 一打开就会振荡几次，跟滤波过头的感觉一样，暂时未解决，应该是内部I2C降低了采样频率
#define USE_OFFSET      // 初始化使用零点校正   只要有动静态校正，陀螺仪本身还是不太漂的，这个用不用看效果吧   ！！！！！！这个注释掉的话，三个轴的效果是之前的，但是soft不受影响！！！！！！！！！！！！！！
#define DYNAMIC_OFFSET    // 使用动态校正零点，这个可以使用，imu还是主要受上面的影响    
//#define MAG_OFFSET      // 磁力计校正，需要围绕空间内3个轴旋转
//#define USE_KALMAN      // 卡尔曼滤波和低通滤波只能存在一个，同时开启的话默认为低通滤波
#define USE_LPF           // 使用低通滤波器 
/* 变量宏定义 */
#define Sample_Frequency 1000     //采样频率
#define Gyro_Cut_Off_Frequency 80    //截止频率，这里要根据情况和需求更改  !!!!!!!!!!!!!!往届有把截止频率改的特别小的（10左右），底盘实测收敛很慢，会造成超调，故改大。过大的话过滤高频噪声的能力差
#define Acce_Cut_Off_Frequency 5
#define SELF_TEST_T  5           //自检时间，单位秒
#define g 9.80665f                           //< 重力加速度
#define TORADIAN   0.0174533f                //< 转换为弧度制，四元数的姿态解算需要使用 π/180
#define TOANGLE    57.2957795f               //< 最后解算出来的弧度制转换为角度
#define ACC_RESOLUTION  (2.0f*g/32768.0f)    //< 加速度计分辨率 m/s^2/LSb
#define GYRO_RESOLUTION (2000.0f/32768.0f)   //< 陀螺仪分辨率   dps/LSb  
/* 需调整的全局变量 */
float so3_comp_params_Kp = 2.0f ;            //< 四元数校正PI参数
float so3_comp_params_Ki = 0.03f; 
uint16_t Zero_Threshold[3] = {100,100,100};  //< 用于零点校正，判断数据是否为静止数据------------------------> 100足够大了，最好看一下原始数据，看看够不够（其实有点大）<--------------------建议更改，提升自检要求
float  Dynamic_Zero_Thre = 4.0f;             //< 动态校正的零点阈值
float Offset_Coeff[3] = {1.0f,1.0f,1.0f};    //< 对量程进行校正，角速度校正系数           ！！！！！！在此标准化量程，即转一个直角，显示90度的效果！！！！！！！！！！！！！！
float manualOffsetGyro[3] = {0,0,0};         //< 手动添加校正值，解决零点误差在+-1之间的问题，因为原始数据是int16_t类型，这个参数能解决有规律的漂移,追求完美的可以试试
/* 无需调整的全局变量 */
int16_t Flash_Val[4];                        //< [0]记录Flash写入的次数，[1-3]为陀螺仪零点值
uint8_t Flash_Limit_t = 10;                  //< 零点值保存的次数（其实不准确，但主要目的还是想让最新的零点值占比高一点）
float startOffsetVal[3];
/* 结构体 */
MPU_HMC imu;
kalman_filter AccFilter[3];
kalman_filter GyroFilter[3];
LPF2 Acc_LPF[3];
LPF2 Gyro_LPF[3];
#ifdef USE_MAG
#define MAG_RESOLUTION  0.1464129f           //< 磁力计分辨率   mG/LSb 
LPF2 Mag_LPF[3];
kalman_filter MagFilter[3];
#endif

/** 
* @brief   求平方根的倒数
* @remarks 使用经典的Carmack算法，效率高，输入根号下的内容
*/
static float invSqrt(float number){
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5f;

    x = number * 0.5f;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

/** 
* @brief   零点值计算
* @remarks 用于零点校正，采样均值,只对陀螺仪进行
*/
uint8_t IMU_Init(void){
	static uint16_t unstable_num;
#ifdef  STM32F405xx	
	for(uint8_t i=0;i<4;i++)  // 读取flash存储的值
	  Flash_Val[i] = (uint16_t)Flash_ReadOut(i);
	if(Flash_Val[0] < 0 || Flash_Val[0] >10000)  Flash_Val[0] = 0;  //第一次烧程序的值不是想要的
	else if(Flash_Val[0] >= Flash_Limit_t)  Flash_Val[0] = Flash_Limit_t;    //限制存储次数为20次，太多担心最新的零点值占比太小
#endif	
#ifdef USE_LPF	
	for(uint8_t i=0;i<3;i++){
		LPF2pSetCutoffFreq(&Acc_LPF[i],Sample_Frequency,Acce_Cut_Off_Frequency);
		LPF2pSetCutoffFreq(&Gyro_LPF[i],Sample_Frequency,Gyro_Cut_Off_Frequency);
	#ifdef USE_MAG		
		LPF2pSetCutoffFreq(&Mag_LPF[i],Sample_Frequency,Cut_Off_Frequency);
	#endif		
	}
#endif
	static uint8_t flag = 1;  ///<采样标志位
	static float tick;        ///<用于超时计数
	while(flag){
		tick = micros();
		for(uint16_t num=0;num<Zero_Sample_Num;num++){   /*零点采样*/
			imu.original.Gyro[0] = SPI_MPU_Read(MPU6500_GYRO_XOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_XOUT_L);
			imu.original.Gyro[1] = SPI_MPU_Read(MPU6500_GYRO_YOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_YOUT_L);
			imu.original.Gyro[2] = SPI_MPU_Read(MPU6500_GYRO_ZOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_ZOUT_L);		
			while(ABS(imu.original.Gyro[0])>Zero_Threshold[0] || ABS(imu.original.Gyro[1])>Zero_Threshold[1] || ABS(imu.original.Gyro[2])>Zero_Threshold[2])  {
				imu.original.Gyro[0] = SPI_MPU_Read(MPU6500_GYRO_XOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_XOUT_L);
				imu.original.Gyro[1] = SPI_MPU_Read(MPU6500_GYRO_YOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_YOUT_L);
				imu.original.Gyro[2] = SPI_MPU_Read(MPU6500_GYRO_ZOUT_H)<<8|SPI_MPU_Read(MPU6500_GYRO_ZOUT_L);
				unstable_num++;
	      if((micros() - tick)/1000000.0f > SELF_TEST_T) {
					for(uint8_t j=0;j<3;j++){
					  imu.offset.Gyro[j] = Flash_Val[j+1]/300.0f;
					  imu.isThisTimeInvalid[j] = 1;
						imu.offset.Cnt[j] = 0;      // 清除计数
					}
					return 0;   //5s内初始化不成功就使用Flash内存的历史零点值
				}
			}
			for(uint8_t k=0;k<3;k++){
				imu.offset.Data[k][imu.offset.Cnt[k]] = imu.original.Gyro[k]; //<零点采样值
				imu.offset.Sum[k] += imu.offset.Data[k][imu.offset.Cnt[k]];   //<零点采样和
				imu.offset.Cnt[k]++;                                          //<采样计数
			}	
		}	
		if (unstable_num > 300){   /*采样数据无效*/  
			unstable_num = 0;
			for(uint8_t i=0;i<3;i++){
				imu.offset.Sum[i] = 0;
				imu.offset.Cnt[i] = 0;
			}					 
		}
		else{
			for(uint8_t i=0;i<3;i++){  /*采样数据有效*/
				startOffsetVal[i] = imu.offset.Gyro[i] = imu.offset.Sum[i]/Zero_Sample_Num;
				Flash_Val[i+1] = (int16_t)imu.offset.Gyro[i]*300;  //更新flsh的值，×300的目的是让数值更准一点，毕竟将float转成了int16_t存起来的			
				imu.offset.Cnt[i] = 0;
			}
			Flash_Val[0]++;
#ifdef  STM32F405xx			
			Flash_WriteIn(0,Flash_Val,4);
#endif			
			flag = 0;
		}
	} 
  return 1;	
}

/** 
* @brief   读取原始数据和单位换算
* @remarks 
*/
#ifdef USE_MAG
int16_t Mag_max[2];  //平面校准法，磁力计只校正x,和y
int16_t Mag_min[2];
#endif
static void MPU_Read_Raw(void)
{
	static uint8_t dynamicFlag[3];
#ifdef USE_MAG	
	static uint8_t akm_data[6];	
#endif	
	static uint8_t mpu_data_buf[14];
	
	/* 读取加速度计&陀螺仪 */
	SPI_MPU_Read_Regs(MPU6500_ACCEL_XOUT_H,mpu_data_buf,14);      
	imu.original.Accel[0] = (mpu_data_buf[0]<<8 | mpu_data_buf[1]); 
	imu.original.Accel[1] = (mpu_data_buf[2]<<8 | mpu_data_buf[3]);
	imu.original.Accel[2] = (mpu_data_buf[4]<<8 | mpu_data_buf[5]);
	imu.original.MPU_Temp = (mpu_data_buf[6]<<8 | mpu_data_buf[7]);
	imu.unitized.MPU_Temp = imu.original.MPU_Temp/333.87f + 21;
	imu.original.Gyro[0] = (mpu_data_buf[8]<<8  | mpu_data_buf[9]);  
	imu.original.Gyro[1] = (mpu_data_buf[10]<<8 | mpu_data_buf[11]);
	imu.original.Gyro[2] = (mpu_data_buf[12]<<8 | mpu_data_buf[13]);
#ifdef USE_MAG	
	/* 读取磁力计 */
	Mag_Read(akm_data);
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;	调节校准的公式
	for(uint8_t i=0;i<3;i++)
		imu.original.Mag[i] = (akm_data[i*2+1]<<8 | akm_data[i*2]);	
#ifdef MAG_OFFSET  
	for(uint8_t i=0;i<2;i++){  //水平校正磁力计
		Mag_max[i] = imu.original.Mag[i]>Mag_max[i]?imu.original.Mag[i]:Mag_max[i];
		Mag_min[i] = imu.original.Mag[i]<Mag_min[i]?imu.original.Mag[i]:Mag_min[i];
		imu.offset.Mag[i] = (float)(Mag_max[i] + Mag_min[i])/2;
	}
#endif
#endif	
	for(uint8_t i=0;i<3;i++){	
		/* 进行卡尔曼滤波 */
#ifdef USE_KALMAN		
		imu.kalman.Accel[i] = Kalman(&AccFilter[i],(float)imu.original.Accel[i]);
    imu.kalman.Gyro[i] = Kalman(&GyroFilter[i],(float)(imu.original.Gyro[i]));	
    		/* 取角速度 */
		imu.Angle_Rate[i] = (float)(imu.kalman.Gyro[i]  - imu.offset.Gyro[i])*Offset_Coeff[i];  //dps		
		/* 单位化 */
#ifdef USE_OFFSET
		imu.unitized.Gyro[i] = imu.Angle_Rate[i]*GYRO_RESOLUTION*TORADIAN;              //rad/s 		
#else
		imu.unitized.Gyro[i] = imu.kalman.Gyro[i]*Offset_Coeff[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#endif		
		imu.unitized.Accel[i] = imu.kalman.Accel[i]*ACC_RESOLUTION;     //m/s^2  

#ifdef USE_MAG		
		imu.kalman.Mag[i] = Kalman(&MagFilter[i],imu.original.Mag[i]);		
		imu.unitized.Mag[i] = (float)imu.kalman.Mag[i] * ((AK8963_ASA[i]-128)/256.0f+1.0f); //uT	
#endif		
#endif

#ifdef USE_LPF
		imu.LPF.Accel[i] = LPF2pApply(&Acc_LPF[i],(float)imu.original.Accel[i]);
    imu.LPF.Gyro[i] = LPF2pApply(&Gyro_LPF[i],(float)imu.original.Gyro[i]);
		/* 取角速度 */
    imu.Angle_Rate[i] = (float)(imu.LPF.Gyro[i] - imu.offset.Gyro[i] + manualOffsetGyro[i])*Offset_Coeff[i];  //16位量程，只对原始数据进行补偿和滤波处理,需要根据9250改		
    /* 单位化 */		
#ifdef 	USE_OFFSET
    imu.unitized.Gyro[i] = imu.Angle_Rate[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#else
    imu.unitized.Gyro[i] = (float)imu.LPF.Gyro[i]*Offset_Coeff[i]*GYRO_RESOLUTION*TORADIAN;           //rad/s 
#endif	
		imu.unitized.Accel[i] = (float)imu.LPF.Accel[i]*ACC_RESOLUTION;     //m/s^2  
#ifdef USE_MAG	
		imu.LPF.Mag[i] = LPF2pApply(&Mag_LPF[i],imu.original.Mag[i]);
		imu.unitized.Mag[i] = (float)imu.LPF.Mag[i] * ((AK8963_ASA[i]-128)/256.0f+1.0f); //uT
#endif		
#endif	
#ifdef DYNAMIC_OFFSET		
		/*静止时，更新零点*/
		if (ABS(imu.Angle_Rate[i]) < Dynamic_Zero_Thre && ABS(imu.offset.Gyro[i] - imu.original.lastGyro[i]) < 3){
			dynamicFlag[i]++;
			if(dynamicFlag[i] >= 200)  dynamicFlag[i] = 200; //限位
	  }
		else
			dynamicFlag[i] = 0;
    if(dynamicFlag[i] >= 50){  //大于50个周期开始更新零点
			imu.offset.Sum[i] -= imu.offset.Data[i][imu.offset.Cnt[i]];    //< 清除旧数据
			imu.offset.Data[i][imu.offset.Cnt[i]] = imu.original.Gyro[i];  //< 更新数据
			imu.offset.Sum[i] += imu.offset.Data[i][imu.offset.Cnt[i]];    //< 加入新数据
			if(imu.isThisTimeInvalid[i] == 0) /* 启动时采样成功 */
			  imu.offset.Gyro[i] = Limit(imu.offset.Sum[i] / Zero_Sample_Num,startOffsetVal[i]+0.5f,startOffsetVal[i]-0.5f);    //< 更新零点
			imu.offset.Cnt[i]++;
			if(imu.offset.Cnt[i] == Zero_Sample_Num){
				imu.offset.Cnt[i] = 0;
				imu.isThisTimeInvalid[i] = 0;  /* 若启动时没有初始化成功，等待动态采样成功，启用动态校正值 */
			}
      imu.Angle_Rate[i] = 0;	
		}			
#endif 
    imu.original.lastGyro[i] = imu.original.Gyro[i];			
	}
}

/***************************四元数解算部分*********************************/
/* 无需调整的全局变量 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f,q3 = 0.0f;  /** quaternion of sensor frame relative to auxiliary frame */
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f,dq3 = 0.0f;  /** quaternion of sensor frame relative to auxiliary frame */
static float q0q0, q0q1, q0q2, q0q3,q1q1, q1q2, q1q3,q2q2, q2q3,q3q3;
static float gyro_bias[3];  
static uint8_t bFilterInit;
//! Using accelerometer, sense the gravity vector.
//! Using magnetometer, sense yaw.
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx,
                                 float my, float mz)        //其实这个函数是没什么用的，他本身就是利用加速度计和磁力计算出偏航角，再算出四元数，但是一般不用磁力计，懒得注释进宏定义了
{
	float initialRoll, initialPitch;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float magX, magY;
	float initialHdg, cosHeading, sinHeading;	
	
	initialRoll = 3.1415f + atan2(-ay, -az);
	initialPitch = - atan2(ax, -az);

	cosRoll = cosf(initialRoll);
	sinRoll = sinf(initialRoll);
	cosPitch = cosf(initialPitch);
	sinPitch = sinf(initialPitch);
	if(mx == 0 || mz == 0){
		mx=1;  
    mz=1;
	}
	magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
	magY = my * cosRoll - mz * sinRoll;

	initialHdg = atan2f(-magY, magX);

	cosRoll = cosf(initialRoll * 0.5f);
	sinRoll = sinf(initialRoll * 0.5f);

	cosPitch = cosf(initialPitch * 0.5f);
	sinPitch = sinf(initialPitch * 0.5f);

	cosHeading = cosf(initialHdg * 0.5f);
	sinHeading = sinf(initialHdg * 0.5f);

	q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
	q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
	q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

	// auxillary variables to reduce number of repeated operations, for 1st pass
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
}
/** 
* @brief   Mahony算法
* @remarks 
*/
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax,
                                   float ay, float az, float mx, float my, float mz, float twoKp, float twoKi,
                                   float dt)
{
    float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
    float recipNorm;
    // Make filter converge to initial solution faster
    if (bFilterInit == 0){
			NonlinearSO3AHRSinit(ax, ay, az, mx, my, mz);
			bFilterInit = 1;
    }
    if (!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)))
    {
			float hx, hy, hz, bx, bz;
			float halfwx, halfwy, halfwz;
			
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;
			// Reference direction of Earth's magnetic field
			hx = 2.0f*(mx*(0.5f-q2q2-q3q3)+my*(q1q2-q0q3)+mz*(q1q3+q0q2));
			hy = 2.0f*(mx*(q1q2+q0q3)+my*(0.5f-q1q1-q3q3)+mz*(q2q3 - q0q1));
			hz = 2.0f*mx*(q1q3-q0q2)+2.0f*my*(q2q3+q0q1)+2.0f*mz*(0.5f-q1q1-q2q2);
			bx = sqrt(hx*hx+hy*hy);
			bz = hz;
			// Estimated direction of magnetic field
			halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
			halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
			halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex += (my * halfwz - mz * halfwy);
			halfey += (mz * halfwx - mx * halfwz);
			halfez += (mx * halfwy - my * halfwx);
    }
	
    //增加一个条件：  加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){
			float halfvx, halfvy, halfvz;
			
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			if((1/(0.75f*9.8f))>recipNorm>(1/(1.25f*9.8f))){
				ax *= recipNorm;
				ay *= recipNorm;
				az *= recipNorm;
				// Estimated direction of gravity and magnetic field
				halfvx = q1q3 - q0q2;
				halfvy = q0q1 + q2q3;
				halfvz = q0q0 - 0.5f + q3q3;
				// Error is sum of cross product between estimated direction and measured direction of field vectors
				halfex += ay * halfvz - az * halfvy;
				halfey += az * halfvx - ax * halfvz;
				halfez += ax * halfvy - ay * halfvx;
			}
    }
    // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
    if (halfex != 0.0f && halfey != 0.0f && halfez != 0.0f){
        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f){
            gyro_bias[0] += twoKi * halfex * dt;    // integral error scaled by Ki
            gyro_bias[1] += twoKi * halfey * dt;
            gyro_bias[2] += twoKi * halfez * dt;
            // apply integral feedback
            gx += gyro_bias[0];
            gy += gyro_bias[1];
            gz += gyro_bias[2];
        }
        else{
            gyro_bias[0] = 0.0f;    // prevent integral windup
            gyro_bias[1] = 0.0f;
            gyro_bias[2] = 0.0f;
        }
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    // Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
    //! q_k = q_{k-1} + dt*\dot{q}
    //! \dot{q} = 0.5*q \otimes P(\omega)
    dq0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    dq1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    dq2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    dq3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    q0 += dt * dq0;
    q1 += dt * dq1;
    q2 += dt * dq2;
    q3 += dt * dq3;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}

/** 
* @brief    SoftYaw
* @remarks  没有边界的Yaw值
* @par 日志
*/
static float Soft_Angle(float angle,uint8_t whichAngle)
{
	static int16_t angleCircle[3];	
	static float LastAngle[3];
	if((angle - LastAngle[whichAngle]) > 180) 
		angleCircle[whichAngle]--;
	else if((angle - LastAngle[whichAngle]) < -180)
		angleCircle[whichAngle]++;
	LastAngle[whichAngle] = angle;
	return  angleCircle[whichAngle]*360 + angle;
}
/** 
* @brief   姿态解算
* @remarks 
*/
uint32_t tPrev,tNow; 
void IMUSO3Thread(void)
{   
    float euler[3] = {0,0,0};            //rad  
    float Rot_matrix[9] = {1.0f,  0.0f,  0.0f, 0.0f,  1.0f,  0.0f, 0.0f,  0.0f,  1.0f };       /**< init: identity matrix */
    /* 计算两次解算时间间隔 */
    tNow = micros();
    float dt = (tPrev > 0) ? (tNow - tPrev) / 1000000.0f : 0;
    tPrev = tNow;
//    if(dt == 0)  return;    // 第一次是0也没关系，反正是积分
    /* 读取数据(已经滤波、校正、单位化) */
    MPU_Read_Raw();
    /* 四元数姿态融合 */  
    #ifdef USE_MAG		
    NonlinearSO3AHRSupdate(imu.unitized.Gyro[0],imu.unitized.Gyro[1],imu.unitized.Gyro[2],
													 imu.unitized.Accel[0],imu.unitized.Accel[1],imu.unitized.Accel[2],
													 imu.unitized.Mag[0],imu.unitized.Mag[1],imu.unitized.Mag[2],so3_comp_params_Kp,so3_comp_params_Ki,dt);
    #else
    NonlinearSO3AHRSupdate(imu.unitized.Gyro[0],imu.unitized.Gyro[1],imu.unitized.Gyro[2],
													 imu.unitized.Accel[0],imu.unitized.Accel[1],imu.unitized.Accel[2],
													 0,0,0,so3_comp_params_Kp,so3_comp_params_Ki,dt);		
		#endif

    /* 转换成方向余弦矩阵 */
    // Convert q->R, This R converts inertial frame to body frame.
    Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
    Rot_matrix[1] = 2.f * (q1 * q2 + q0 * q3); // 12
    Rot_matrix[2] = 2.f * (q1 * q3 - q0 * q2); // 13
//    Rot_matrix[3] = 2.f * (q1 * q2 - q0 * q3); // 21
//    Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
    Rot_matrix[5] = 2.f * (q2 * q3 + q0 * q1); // 23
//    Rot_matrix[6] = 2.f * (q1 * q3 + q0 * q2); // 31
//    Rot_matrix[7] = 2.f * (q2 * q3 - q0 * q1); // 32
    Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33 
    /* 转换成欧拉角 */
    euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);    //! Roll
    euler[1] = -asinf(Rot_matrix[2]);                   //! Pitch
    euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);    //！Yaw
    /* 得出姿态角 */
    imu.Roll = euler[0] * TOANGLE;            // 绝对角
    imu.Pitch = euler[1] * TOANGLE;	
    imu.Yaw = -euler[2] * TOANGLE;
    imu.soft.Roll = Soft_Angle(imu.Roll,0);   // 绝对路程角
    imu.soft.Pitch = Soft_Angle(imu.Pitch,1);
    imu.soft.Yaw = Soft_Angle(imu.Yaw,2);
		imu.integral.Roll += imu.Angle_Rate[0]*GYRO_RESOLUTION*dt;  // 相对积分角
		imu.integral.Pitch += imu.Angle_Rate[1]*GYRO_RESOLUTION*dt;
		imu.integral.Yaw += imu.Angle_Rate[2]*GYRO_RESOLUTION*dt;
    if(ABS(imu.original.Gyro[0]) <= 1 && ABS(imu.original.Gyro[1]) <= 1 && ABS(imu.original.Gyro[2]) <= 1 &&
			 ABS(imu.original.Accel[0] <= 1) && ABS(imu.original.Accel[1] <= 1) && ABS(imu.original.Accel[2] <= 1))
			imu.ready = 0;
		else  imu.ready = 1;
}

