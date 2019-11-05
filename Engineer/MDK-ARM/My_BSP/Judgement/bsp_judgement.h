#ifndef __BSP_JUDGEMENT_H_
#define __BSP_JUDGEMENT_H_
#include "stm32f4xx.h"
#define JUDGE_HUART huart6
/* 通信协议格式：frame_header(5-byte)，cmd_id(2-byte)，data(n-byte)，frame_tail(2-byte，CRC16，整包校验) */

/************************* frame_header 详细定义 ******************************/
typedef __packed struct 
{
	uint8_t SOF;					//< 帧起始字节，固定值为0xA5(一个字节)
	uint16_t DataLength;	//< 数据段DATA长度(2个字节)
	uint8_t Seq;					//< 包序号(1个字节)
	uint8_t CRC8;					//< 帧头CRC校验(一个字节)
}ext_FrameHeader;
/************************* cmd_id 命令码 ID  ******************************/
 /* 比赛状态：0x0001 1Hz */
typedef __packed struct   
{   
	uint8_t game_type : 4;				// [0-3]比赛类型： 1：RoboMaster 机甲大师赛；2：RoboMaster 机甲大师单项赛；3：ICRA RoboMaster 人工智能挑战赛 
	uint8_t game_progress : 4;		// [4-7]当前比赛阶段： 0：未开始比赛；1：准备阶段；2：自检阶段；3：5s 倒计时；4：对战中；5：比赛结算中 
	uint16_t stage_remain_time; 	// 当前阶段剩余时间，单位 s 
} ext_game_state_t; 
/* 比赛结果数据：0x0002。发送频率：比赛结束后发送 */
typedef __packed struct
{    
	uint8_t winner;               //0 平局 1 红方胜利 2 蓝方胜利 
} ext_game_result_t; 

/**
*   机器人存活数据：0x0003。发送频率：1Hz 
*   bit 0：红方英雄机器人； 
*		bit 1：红方工程机器人； 
*		bit 2：红方步兵机器人 1； 
*		bit 3：红方步兵机器人 2； 
*		bit 4：红方步兵机器人 3； 
*		bit 5：红方空中机器人； 
*		bit 6：红方哨兵机器人； 
*		bit 7：保留 
*		bit 8：蓝方英雄机器人； 
*		bit 9：蓝方工程机器人； 
*		bit 10：蓝方步兵机器人 1； 
*		bit 11：蓝方步兵机器人 2； 
*		bit 12：蓝方步兵机器人 3； 
*		bit 13：蓝方空中机器人； 
*		bit 14：蓝方哨兵机器人； 
*		bit 15：保留 
*		对应的 bit 数值置 1 代表机器人存活，数值置 0 代表机器人死亡或者未上场。
*/
typedef __packed struct 
{  
	uint16_t robot_legion; 
} ext_game_robot_survivors_t; 
/**
*		场地事件数据：0x0101。发送频率：事件改变后发送 
*		bit 0-1：己方停机坪占领状态  
*				0为无机器人占领； 
*				1为空中机器人已占领但未停桨； 
*				2为空中机器人已占领并停桨 
*		bit 2：己方补给站 1号补血点占领状态 1为已占领； 
*		bit 3：己方补给站 2号补血点占领状态 1为已占领； 
*		bit 4：己方补给站 3号补血点占领状态 1为已占领； 
*		bit 5-6：己方大能量机关状态： 
*				0为打击点未占领且大能量机关未激活， 
*				1为打击点占领且大能量机关未激活， 
*				2为大能量机关已激活， 
*				3为大能量机关已激活且打击点被占领； 
*		bit 7：己方关口占领状态 1为已占领； 
*		bit 8：己方碉堡占领状态 1为已占领； 
*		bit 9：己方资源岛占领状态 1为已占领； 
*		bit 10-11：己方基地防御状态  
*				2 为基地百分之百防御 ， 
*				1 为基地有哨兵防御， 
*				0 为基地无防御，
*		bit 12-13：ICRA 红方防御加成 
*				0：防御加成未激活； 
*				1：防御加成 5s触发激活中； 
*				2：防御加成已激活 
*		bit 14-15：ICRA 蓝方防御加成 
*				0：防御加成未激活； 
*				1：防御加成 5s触发激活中； 
*				2：防御加成已激活 
//其余保留 
**/
typedef __packed struct 
{ 
  uint32_t event_type; 
} ext_event_data_t; 
/* 补给站动作标识：0x0102。发送频率：动作改变后发送 */
typedef __packed struct 
{   
	uint8_t supply_projectile_id;    // 补给站口 ID：1：1号补给口；2号补给口 
	uint8_t supply_robot_id;    		 // 补弹机器人ID：0为当前无机器人补弹，1为红方英雄机器人补弹，2为红方工程机器人补弹，3/4/5 为红方步兵机器人补弹，11为蓝方英雄机器人补弹，12为蓝方工程机器人补弹，13/14/15为蓝方步兵机器人补弹
	uint8_t supply_projectile_step;  // 子弹口开闭状态：0 为关闭，1为子弹准备中，2为子弹下落 
	uint8_t supply_projectile_num;   // 补弹数量
} ext_supply_projectile_action_t; 
/**
*		补给站预约子弹：0x0103。发送频率：上限 10Hz。RM 对抗赛尚未开放 
*   预约补给站口 ID： 
*				0：空闲补给口，依照 1，2 顺序查询补给空闲情况； 
*				1：1 号补给口； 
*				2：2 号补给口
*		补弹机器人ID：
*				1为红方英雄机器人补弹，2为红方工程机器人补弹， 3/4/5 为红方步兵机器人补弹，
*				11 为蓝方英雄机器人补弹，12 为蓝方 工程机器人补弹，13/14/15为蓝方步兵机器人补弹 
*		预约子弹数目：  
*				0-50 为预约 50 颗子弹，  
*				51-100 为预约 100 颗子弹，101-150 为预约150 颗子弹， 
*				151-255 为预约 200 颗子弹。（上限 200 颗子弹
**/
typedef __packed struct 
{   
	uint8_t supply_projectile_id;   
	uint8_t supply_robot_id;
	uint8_t supply_num;  
} ext_supply_projectile_booking_t;

/* 比赛机器人状态：0x0201。发送频率：10Hz */
typedef __packed struct 
{   
	uint8_t robot_id;      // 机器人 ID：1:红方英雄机器人 2:红方工程机器人 3/4/5:红方步兵机器人 6:红方空中机器人 7:红方哨兵机器人 11:蓝方英雄机器人 12:蓝方工程机器人 13/14/15:蓝方步兵机器人 16:蓝方空中机器人 17:蓝方哨兵机器人
	uint8_t robot_level;   // 机器人等级
	uint16_t remain_HP;    // 机器人剩余血量 
	uint16_t max_HP;       // 机器人上限血量 
	uint16_t shooter_heat0_cooling_rate;    //机器人 17mm 子弹热量冷却速度 单位 /s 
	uint16_t shooter_heat0_cooling_limit;   //机器人 17mm 子弹热量上限 
	uint16_t shooter_heat1_cooling_rate;    //机器人 42mm 子弹热量冷却速度 单位 /s 
	uint16_t shooter_heat1_cooling_limit;   //机器人 42mm 子弹热量上限 
	uint8_t mains_power_gimbal_output : 1;  //主控电源输出情况， gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出； 
	uint8_t mains_power_chassis_output : 1; //chassis 口输出：1 为有 24V 输出，0 为无 24v 输出； 
	uint8_t mains_power_shooter_output : 1; //shooter 口输出：1 为有 24V 输出，0 为无 24v 输出； 
} ext_game_robot_state_t; 


//实时功率热量数据：0x0202。发送频率：50Hz 
typedef __packed struct
{
  uint16_t chassis_volt;    //底盘输出电压 单位 毫伏 
	uint16_t chassis_current;    //底盘输出电流 单位 毫安 
	float chassis_power;    //底盘输出功率 单位 W 瓦 
	uint16_t chassis_power_buffer;    //底盘功率缓冲 单位 J 焦耳 
	uint16_t shooter_heat0;    //17mm 枪口热量 
	uint16_t shooter_heat1; 	 //42mm 枪口热量 
} ext_power_heat_data_t;

//机器人位置：0x0203。发送频率：10Hz
typedef __packed struct 
{   
	float x;   //位置 x 坐标，单位 m 
	float y;   //位置 y 坐标，单位 m 
	float z;   //位置 z 坐标，单位 m 
	float yaw; //位置枪口，单位度 
} ext_game_robot_pos_t;

// 机器人增益：0x0204。发送频率：状态改变后发送 
//bit 0：机器人血量补血状态 
//bit 1：枪口热量冷却加速 
//bit 2：机器人防御加成 
//bit 3：机器人攻击加成 
//其他 bit 保留
typedef __packed struct 
{   
	uint8_t power_rune_buff; 
}ext_buff_musk_t;

//空中机器人能量状态：0x0205。发送频率：10Hz 
typedef __packed struct 
{   
	uint8_t energy_point;   //积累的能量点 
	uint8_t attack_time; 		//可攻击时间 单位 s。50s 递减至 0 
} aerial_robot_energy_t; 

// 伤害状态：0x0206。发送频率：伤害发生后发送 
typedef __packed struct 
{   
uint8_t armor_id : 4;   //当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。 
uint8_t hurt_type : 4; //0x0 装甲伤害扣血； 0x1 模块掉线扣血； 0x2 超枪口热量扣血； 0x3 超底盘功率扣血。
} ext_robot_hurt_t; 

//实时射击信息：0x0207。发送频率：射击后发送 
typedef __packed struct 
{   
	uint8_t bullet_type;    //子弹类型: 1：17mm 弹丸 2：42mm 弹丸 
	uint8_t bullet_freq;    //子弹射频 单位 Hz 
	float bullet_speed;     //子弹射速 单位 m/s 
} ext_shoot_data_t; 

//交互数据接收信息：0x0301。发送频率：上限 10Hz 
typedef __packed struct 
{   
	uint16_t data_cmd_id;   //数据段的内容 ID  
	uint16_t send_ID;    //发送者的 ID 需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1 
	uint16_t receiver_ID; //接收者的 ID 需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的ID 
												//若发送的是客户端自定义数据 则此处ID为对应客户端ID
												//客户端 ID：英雄(红)0x0101 工程(红)0x102 步兵（红）0x0103/0x0104/0x0105 飞机(红)0x106
												//					 英雄(蓝)0x0111 工程(蓝)0x112 步兵（蓝）0x0113/0x0114/0x0115 飞机(蓝)0x116
}ext_student_interactive_header_data_t; 

//客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。 发送频率：上限 10Hz 
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; //bit 0-5：分别控制客户端自定义数据显示面板上的六个指示灯，值为1时显示绿色，值为0是显示红色。 Bit 6-7：保留 
} client_custom_data_t;
	
//交互数据 机器人间通信：0x0301。内容 ID:0x0200~0x02FF  发送频率：上限 10Hz 
typedef __packed struct 
{ 
	uint8_t data[20];//自定义长度 不超过113 
} robot_interactive_data_t;
				
typedef __packed struct
{
		ext_FrameHeader Header_Tx;  	//5bytes
		uint16_t CmdID_Tx;				    //2bytes
		ext_student_interactive_header_data_t student_interactive_header_data;	//6bytes
		client_custom_data_t client_custom_data;//13bytes
		uint16_t CRC16_Tx;				//2bytes
}client_custom_dataPack_t;			//28bytes

typedef __packed struct
{
		ext_FrameHeader Header_Tx;  	//5bytes
		uint16_t CmdID_Tx;				//2bytes
		ext_student_interactive_header_data_t student_interactive_header_data;		//6bytes
		robot_interactive_data_t robot_interactive_data;//自定义bytes
		uint16_t CRC16_Tx;				//2bytes
}robot_interactive_dataPack_t;			//bytes

typedef enum
{
game_state_id=0x0001, //3 比赛状态数据，1Hz 周期发送 
game_result_id = 0x0002, //1 比赛结果数据，比赛结束后发送 
game_robot_survivors_id = 0x0003, //2 比赛机器人存活数据，1Hz 发送 
event_data_id = 0x0101, //4 场地事件数据，事件改变后发送 
supply_projectile_action_id = 0x0102, //3 场地补给站动作标识数据，动作改变后发送 
supply_projectile_booking_id = 0x0103, //2 场地补给站预约子弹数据，由参赛队发送，上限 10Hz。（ RM 对抗赛尚未开放）
game_robot_state_id = 0x0201, //15 机器人状态数据，10Hz 周期发送 
power_heat_data_id = 0x0202, //14 实时功率热量数据，50Hz 周期发送 
game_robot_pos_id = 0x0203, //16 机器人位置数据，10Hz 发送 
buff_musk_id = 0x0204, //1 机器人增益数据,增益状态改变后发送 
aerial_robot_energy_id = 0x0205, //3 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送 
robot_hurt_id = 0x0206, //1 伤害状态数据，伤害发生后发送 
shoot_data_id = 0x0207, //6 实时射击数据，子弹发射后发送 
student_interactive_header_data_id = 0x0301, //n 机器人间交互数据，发送方触发发送，上限 10Hz 
}CmdID_t;

extern ext_game_state_t game_state;
extern ext_game_result_t game_result;
extern ext_game_robot_survivors_t game_robot_survivors;
extern ext_event_data_t event_data;
extern ext_supply_projectile_action_t supply_projectile_action;
extern ext_supply_projectile_booking_t supply_projectile_booking;
extern ext_game_robot_state_t game_robot_state;
extern ext_power_heat_data_t power_heat_data;
extern ext_game_robot_pos_t game_robot_pos;
extern ext_buff_musk_t buff_musk;
extern aerial_robot_energy_t aerial_robot_energy;
extern ext_robot_hurt_t robot_hurt;
extern ext_shoot_data_t shoot_data;
extern client_custom_dataPack_t client_custom_dataPack;
extern robot_interactive_dataPack_t robot_interactive_dataPack_rx;
extern robot_interactive_dataPack_t robot_interactive_dataPack_tx;

extern uint8_t  Judgement_Online;

void Judgement_Init(void);			
void Judgement_Start(void);	//串口中断函数，记得丢到对应的串口中断里面
void Judgement_calculate(void);			
void Judgement_SendCustomData(void);
void Judgement_SendInteractiveData(uint16_t _data_cmd_id,uint16_t _receiver_ID);

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
static void JudgeMent_Uart_ResetIT(UART_HandleTypeDef *huart);		
int Judge_UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void uart_reset_idle_rx_callback(UART_HandleTypeDef *huart);
void uart_reset_uartIT(UART_HandleTypeDef *huart); 

#endif
