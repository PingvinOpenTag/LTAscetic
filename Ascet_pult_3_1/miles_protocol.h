#define IR_START_BIT_DURATION 2400	// ������������ �����-���� (� �������������)
#define IR_ONE_BIT_DURATION 1200	// ������������ ����, ���������������� �������� (� �������������)
#define IR_ZERO_BIT_DURATION 600	// ������������ ����, ���������������� ���� (� �������������)
#define IR_SPACE_DURATION 600		// ������������ ����, ���������������� ��������� ����� ������ (� �������������)
//#define IR_F0 56000					// ������� ������� ��-��������� (f0)
#define IR_F0 36000

#define IR_ZERO (IR_ZERO_BIT_DURATION*2*IR_F0)/1000000 //������������ �������, ���������������� ���� = 0
													  //���������� � "�����" ������� 


#define RX_IR_ZERO (IR_ZERO_BIT_DURATION*8)/1000 //������������ �������, ���������������� ���� = 0
													  //���������� � "�����" ������� 



#define IR_ONE (IR_ONE_BIT_DURATION*2*IR_F0)/1000000 //������������ �������, ���������������� ���� = 1
													  //���������� � "�����" ������� 

#define RX_IR_ONE (IR_ONE_BIT_DURATION*8)/1000 //������������ �������, ���������������� ���� = 1
													  //���������� � "�����" ������� 



#define IR_START (IR_START_BIT_DURATION*2*IR_F0)/1000000 //������������ ��������� 
														//���������� � "�����" ������� 


#define RX_IR_START (IR_START_BIT_DURATION*8)/1000 //������������ ��������� 
														//���������� � "�����" ������� 


#define IR_SPACE  (IR_SPACE_DURATION*2*IR_F0)/1000000 //������������ ��������� ����� ������������� ������
														//���������� � "�����" ������� 


#define RX_IR_SPACE  (IR_SPACE_DURATION*8)/1000 //������������ ��������� ����� ������������� ������
														//���������� � "�����" ������� 





//#define ERROR_TOLERANCE 22//14 //���������� ����������� ������������ ����������� ��� (� "�����" �������)

#define ERROR_TOLERANCE 14

//#define ERROR_TOLERANCE 22


//---------------------------------------------------------------------//

enum Team_Color {	//��������� ������������� ��� ��� ������ � ������ �������
	Red, 	//00 = Red
	Blue, 	//01 = Blue		
	Yellow, //10 = Yellow
	Green}; //11 = Green

typedef enum Team_Color tteam_color;			

//---------------------------------------------------------------------//





//��������� ������������� ���
//��� ������ � "������"
enum GunDamage {	
	
	Damage_1,  //0000 = 1
	Damage_2,  //0001 = 2
	Damage_4,  //0010 = 4
	Damage_5,  //0011 = 5
	Damage_7,  //0100 = 7
	Damage_10, //0101 = 10
	Damage_15, //0110 = 15
	Damage_17, //0111 = 17
	Damage_20, //1000 = 20
	Damage_25, //1001 = 25
	Damage_30, //1010 = 30
	Damage_35, //1011 = 35
	Damage_40, //1100 = 40
	Damage_50, //1101 = 50
	Damage_75, //1110 = 75
	Damage_100 //1111 = 100
	};
	
	    								
typedef enum GunDamage tgun_damage; 	




//---------------------------------------------------------------------//


/*
union player_id_union {
tplayer_id bits;
uint8_t data [8];
};
*/

typedef struct message { //��������� ��� ������������� ���������
uint8_t DATA;
uint8_t ID;
} tmessage;


typedef union command_union {
uint16_t raw;
tmessage message;

} tcommand_union;

//miles �������� �������������� ���������
#define ID_Add_Health 			0x80 //��������� �������� �����
#define ID_Add_Rounds 			0x81 //��������� �������� ��������
#define ID_Command 				0x83 //���������-�������
#define ID_Clips_Pickup 		0x8A //��������� 
#define ID_Health_Pickup 		0x8B 
#define ID_Flag_Pickup			0x8C

//LW �������������� �������������� ���������
#define ID_Radiation 			0xA0
#define ID_Anomaly 				0xA1
#define ID_Presetting 			0xA8
#define ID_Change_Color			0xA9
#define ID_Play_Sound			0xAA


//miles ������� 
#define Command_ImmediateNewGame 	0x05 //������� "����� ����"
#define Command_Admin_Kill 			0x00 //������� "����������" ������
#define Command_Full_Health			0x0D //����� - 100%
#define Command_Full_Ammo			0x06 //������ �������

//LW �������������� �������

#define Command_IR_Power_Change		0x0E
#define Command_Health_Double 		0x10

//����������� ����
#define Control_Byte 				0xE8 //���������� �������� ������������ ����� ���������

