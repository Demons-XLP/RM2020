#ifndef __BSP_JUDGEMENT_H_
#define __BSP_JUDGEMENT_H_
#include "stm32f4xx.h"
#define JUDGE_HUART huart6
/* ͨ��Э���ʽ��frame_header(5-byte)��cmd_id(2-byte)��data(n-byte)��frame_tail(2-byte��CRC16������У��) */

/************************* frame_header ��ϸ���� ******************************/
typedef __packed struct 
{
	uint8_t SOF;					//< ֡��ʼ�ֽڣ��̶�ֵΪ0xA5(һ���ֽ�)
	uint16_t DataLength;	//< ���ݶ�DATA����(2���ֽ�)
	uint8_t Seq;					//< �����(1���ֽ�)
	uint8_t CRC8;					//< ֡ͷCRCУ��(һ���ֽ�)
}ext_FrameHeader;
/************************* cmd_id ������ ID  ******************************/
 /* ����״̬��0x0001 1Hz */
typedef __packed struct   
{   
	uint8_t game_type : 4;				// [0-3]�������ͣ� 1��RoboMaster ���״�ʦ����2��RoboMaster ���״�ʦ��������3��ICRA RoboMaster �˹�������ս�� 
	uint8_t game_progress : 4;		// [4-7]��ǰ�����׶Σ� 0��δ��ʼ������1��׼���׶Σ�2���Լ�׶Σ�3��5s ����ʱ��4����ս�У�5������������ 
	uint16_t stage_remain_time; 	// ��ǰ�׶�ʣ��ʱ�䣬��λ s 
} ext_game_state_t; 
/* ����������ݣ�0x0002������Ƶ�ʣ������������� */
typedef __packed struct
{    
	uint8_t winner;               //0 ƽ�� 1 �췽ʤ�� 2 ����ʤ�� 
} ext_game_result_t; 

/**
*   �����˴�����ݣ�0x0003������Ƶ�ʣ�1Hz 
*   bit 0���췽Ӣ�ۻ����ˣ� 
*		bit 1���췽���̻����ˣ� 
*		bit 2���췽���������� 1�� 
*		bit 3���췽���������� 2�� 
*		bit 4���췽���������� 3�� 
*		bit 5���췽���л����ˣ� 
*		bit 6���췽�ڱ������ˣ� 
*		bit 7������ 
*		bit 8������Ӣ�ۻ����ˣ� 
*		bit 9���������̻����ˣ� 
*		bit 10���������������� 1�� 
*		bit 11���������������� 2�� 
*		bit 12���������������� 3�� 
*		bit 13���������л����ˣ� 
*		bit 14�������ڱ������ˣ� 
*		bit 15������ 
*		��Ӧ�� bit ��ֵ�� 1 ��������˴���ֵ�� 0 �����������������δ�ϳ���
*/
typedef __packed struct 
{  
	uint16_t robot_legion; 
} ext_game_robot_survivors_t; 
/**
*		�����¼����ݣ�0x0101������Ƶ�ʣ��¼��ı���� 
*		bit 0-1������ͣ��ƺռ��״̬  
*				0Ϊ�޻�����ռ�죻 
*				1Ϊ���л�������ռ�쵫δͣ���� 
*				2Ϊ���л�������ռ�첢ͣ�� 
*		bit 2����������վ 1�Ų�Ѫ��ռ��״̬ 1Ϊ��ռ�죻 
*		bit 3����������վ 2�Ų�Ѫ��ռ��״̬ 1Ϊ��ռ�죻 
*		bit 4����������վ 3�Ų�Ѫ��ռ��״̬ 1Ϊ��ռ�죻 
*		bit 5-6����������������״̬�� 
*				0Ϊ�����δռ���Ҵ���������δ��� 
*				1Ϊ�����ռ���Ҵ���������δ��� 
*				2Ϊ�����������Ѽ�� 
*				3Ϊ�����������Ѽ����Ҵ���㱻ռ�죻 
*		bit 7�������ؿ�ռ��״̬ 1Ϊ��ռ�죻 
*		bit 8�������ﱤռ��״̬ 1Ϊ��ռ�죻 
*		bit 9��������Դ��ռ��״̬ 1Ϊ��ռ�죻 
*		bit 10-11���������ط���״̬  
*				2 Ϊ���ذٷ�֮�ٷ��� �� 
*				1 Ϊ�������ڱ������� 
*				0 Ϊ�����޷�����
*		bit 12-13��ICRA �췽�����ӳ� 
*				0�������ӳ�δ��� 
*				1�������ӳ� 5s���������У� 
*				2�������ӳ��Ѽ��� 
*		bit 14-15��ICRA ���������ӳ� 
*				0�������ӳ�δ��� 
*				1�������ӳ� 5s���������У� 
*				2�������ӳ��Ѽ��� 
//���ౣ�� 
**/
typedef __packed struct 
{ 
  uint32_t event_type; 
} ext_event_data_t; 
/* ����վ������ʶ��0x0102������Ƶ�ʣ������ı���� */
typedef __packed struct 
{   
	uint8_t supply_projectile_id;    // ����վ�� ID��1��1�Ų����ڣ�2�Ų����� 
	uint8_t supply_robot_id;    		 // ����������ID��0Ϊ��ǰ�޻����˲�����1Ϊ�췽Ӣ�ۻ����˲�����2Ϊ�췽���̻����˲�����3/4/5 Ϊ�췽���������˲�����11Ϊ����Ӣ�ۻ����˲�����12Ϊ�������̻����˲�����13/14/15Ϊ�������������˲���
	uint8_t supply_projectile_step;  // �ӵ��ڿ���״̬��0 Ϊ�رգ�1Ϊ�ӵ�׼���У�2Ϊ�ӵ����� 
	uint8_t supply_projectile_num;   // ��������
} ext_supply_projectile_action_t; 
/**
*		����վԤԼ�ӵ���0x0103������Ƶ�ʣ����� 10Hz��RM �Կ�����δ���� 
*   ԤԼ����վ�� ID�� 
*				0�����в����ڣ����� 1��2 ˳���ѯ������������� 
*				1��1 �Ų����ڣ� 
*				2��2 �Ų�����
*		����������ID��
*				1Ϊ�췽Ӣ�ۻ����˲�����2Ϊ�췽���̻����˲����� 3/4/5 Ϊ�췽���������˲�����
*				11 Ϊ����Ӣ�ۻ����˲�����12 Ϊ���� ���̻����˲�����13/14/15Ϊ�������������˲��� 
*		ԤԼ�ӵ���Ŀ��  
*				0-50 ΪԤԼ 50 ���ӵ���  
*				51-100 ΪԤԼ 100 ���ӵ���101-150 ΪԤԼ150 ���ӵ��� 
*				151-255 ΪԤԼ 200 ���ӵ��������� 200 ���ӵ�
**/
typedef __packed struct 
{   
	uint8_t supply_projectile_id;   
	uint8_t supply_robot_id;
	uint8_t supply_num;  
} ext_supply_projectile_booking_t;

/* ����������״̬��0x0201������Ƶ�ʣ�10Hz */
typedef __packed struct 
{   
	uint8_t robot_id;      // ������ ID��1:�췽Ӣ�ۻ����� 2:�췽���̻����� 3/4/5:�췽���������� 6:�췽���л����� 7:�췽�ڱ������� 11:����Ӣ�ۻ����� 12:�������̻����� 13/14/15:�������������� 16:�������л����� 17:�����ڱ�������
	uint8_t robot_level;   // �����˵ȼ�
	uint16_t remain_HP;    // ������ʣ��Ѫ�� 
	uint16_t max_HP;       // ����������Ѫ�� 
	uint16_t shooter_heat0_cooling_rate;    //������ 17mm �ӵ�������ȴ�ٶ� ��λ /s 
	uint16_t shooter_heat0_cooling_limit;   //������ 17mm �ӵ��������� 
	uint16_t shooter_heat1_cooling_rate;    //������ 42mm �ӵ�������ȴ�ٶ� ��λ /s 
	uint16_t shooter_heat1_cooling_limit;   //������ 42mm �ӵ��������� 
	uint8_t mains_power_gimbal_output : 1;  //���ص�Դ�������� gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v ����� 
	uint8_t mains_power_chassis_output : 1; //chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v ����� 
	uint8_t mains_power_shooter_output : 1; //shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v ����� 
} ext_game_robot_state_t; 


//ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz 
typedef __packed struct
{
  uint16_t chassis_volt;    //���������ѹ ��λ ���� 
	uint16_t chassis_current;    //����������� ��λ ���� 
	float chassis_power;    //����������� ��λ W �� 
	uint16_t chassis_power_buffer;    //���̹��ʻ��� ��λ J ���� 
	uint16_t shooter_heat0;    //17mm ǹ������ 
	uint16_t shooter_heat1; 	 //42mm ǹ������ 
} ext_power_heat_data_t;

//������λ�ã�0x0203������Ƶ�ʣ�10Hz
typedef __packed struct 
{   
	float x;   //λ�� x ���꣬��λ m 
	float y;   //λ�� y ���꣬��λ m 
	float z;   //λ�� z ���꣬��λ m 
	float yaw; //λ��ǹ�ڣ���λ�� 
} ext_game_robot_pos_t;

// ���������棺0x0204������Ƶ�ʣ�״̬�ı���� 
//bit 0��������Ѫ����Ѫ״̬ 
//bit 1��ǹ��������ȴ���� 
//bit 2�������˷����ӳ� 
//bit 3�������˹����ӳ� 
//���� bit ����
typedef __packed struct 
{   
	uint8_t power_rune_buff; 
}ext_buff_musk_t;

//���л���������״̬��0x0205������Ƶ�ʣ�10Hz 
typedef __packed struct 
{   
	uint8_t energy_point;   //���۵������� 
	uint8_t attack_time; 		//�ɹ���ʱ�� ��λ s��50s �ݼ��� 0 
} aerial_robot_energy_t; 

// �˺�״̬��0x0206������Ƶ�ʣ��˺��������� 
typedef __packed struct 
{   
uint8_t armor_id : 4;   //��Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ�������˵����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0�� 
uint8_t hurt_type : 4; //0x0 װ���˺���Ѫ�� 0x1 ģ����߿�Ѫ�� 0x2 ��ǹ��������Ѫ�� 0x3 �����̹��ʿ�Ѫ��
} ext_robot_hurt_t; 

//ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������� 
typedef __packed struct 
{   
	uint8_t bullet_type;    //�ӵ�����: 1��17mm ���� 2��42mm ���� 
	uint8_t bullet_freq;    //�ӵ���Ƶ ��λ Hz 
	float bullet_speed;     //�ӵ����� ��λ m/s 
} ext_shoot_data_t; 

//�������ݽ�����Ϣ��0x0301������Ƶ�ʣ����� 10Hz 
typedef __packed struct 
{   
	uint16_t data_cmd_id;   //���ݶε����� ID  
	uint16_t send_ID;    //�����ߵ� ID ��ҪУ�鷢���ߵ� ID ��ȷ�ԣ������ 1 ���͸��� 5��������ҪУ��� 1 
	uint16_t receiver_ID; //�����ߵ� ID ��ҪУ������ߵ� ID ��ȷ�ԣ����粻�ܷ��͵��жԻ����˵�ID 
												//�����͵��ǿͻ����Զ������� ��˴�IDΪ��Ӧ�ͻ���ID
												//�ͻ��� ID��Ӣ��(��)0x0101 ����(��)0x102 �������죩0x0103/0x0104/0x0105 �ɻ�(��)0x106
												//					 Ӣ��(��)0x0111 ����(��)0x112 ����������0x0113/0x0114/0x0115 �ɻ�(��)0x116
}ext_student_interactive_header_data_t; 

//�ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180�� ����Ƶ�ʣ����� 10Hz 
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; //bit 0-5���ֱ���ƿͻ����Զ���������ʾ����ϵ�����ָʾ�ƣ�ֵΪ1ʱ��ʾ��ɫ��ֵΪ0����ʾ��ɫ�� Bit 6-7������ 
} client_custom_data_t;
	
//�������� �����˼�ͨ�ţ�0x0301������ ID:0x0200~0x02FF  ����Ƶ�ʣ����� 10Hz 
typedef __packed struct 
{ 
	uint8_t data[20];//�Զ��峤�� ������113 
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
		robot_interactive_data_t robot_interactive_data;//�Զ���bytes
		uint16_t CRC16_Tx;				//2bytes
}robot_interactive_dataPack_t;			//bytes

typedef enum
{
game_state_id=0x0001, //3 ����״̬���ݣ�1Hz ���ڷ��� 
game_result_id = 0x0002, //1 ����������ݣ������������� 
game_robot_survivors_id = 0x0003, //2 ���������˴�����ݣ�1Hz ���� 
event_data_id = 0x0101, //4 �����¼����ݣ��¼��ı���� 
supply_projectile_action_id = 0x0102, //3 ���ز���վ������ʶ���ݣ������ı���� 
supply_projectile_booking_id = 0x0103, //2 ���ز���վԤԼ�ӵ����ݣ��ɲ����ӷ��ͣ����� 10Hz���� RM �Կ�����δ���ţ�
game_robot_state_id = 0x0201, //15 ������״̬���ݣ�10Hz ���ڷ��� 
power_heat_data_id = 0x0202, //14 ʵʱ�����������ݣ�50Hz ���ڷ��� 
game_robot_pos_id = 0x0203, //16 ������λ�����ݣ�10Hz ���� 
buff_musk_id = 0x0204, //1 ��������������,����״̬�ı���� 
aerial_robot_energy_id = 0x0205, //3 ���л���������״̬���ݣ�10Hz ���ڷ��ͣ�ֻ�п��л��������ط��� 
robot_hurt_id = 0x0206, //1 �˺�״̬���ݣ��˺��������� 
shoot_data_id = 0x0207, //6 ʵʱ������ݣ��ӵ�������� 
student_interactive_header_data_id = 0x0301, //n �����˼佻�����ݣ����ͷ��������ͣ����� 10Hz 
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
void Judgement_Start(void);	//�����жϺ������ǵö�����Ӧ�Ĵ����ж�����
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
