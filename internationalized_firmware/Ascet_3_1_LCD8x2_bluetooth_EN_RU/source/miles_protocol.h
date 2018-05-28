#define IR_START_BIT_DURATION 2400	// ������������ �����-���� (� �������������)
#define IR_ONE_BIT_DURATION 1200	// ������������ ����, ���������������� �������� (� �������������)
#define IR_ZERO_BIT_DURATION 600	// ������������ ����, ���������������� ���� (� �������������)
#define IR_SPACE_DURATION 600		// ������������ ����, ���������������� ��������� ����� ������ (� �������������)
//#define IR_F0 miles_protocol.carrier_frequency /*56000*/					// ������� ������� ��-��������� (f0)
#define IR_F0 36000
#define ERROR_TOLERANCE miles_protocol.err_tolerance //14//22 //���������� ����������� ������������ ����������� ��� (� "�����" �������)


#define ERROR_TOLERANCE_FOR_36KHZ 12//14
#define ERROR_TOLERANCE_FOR_56KHZ 19//22


#define DEFAULT_IR_F0 36000					// ������� ������� ��-��������� (f0)
#define DEFAULT_ERROR_TOLERANCE 12//14 //���������� ����������� ������������ ����������� ��� (� "�����" �������)


#define IR_ZERO miles_protocol.ir_zero//(IR_ZERO_BIT_DURATION*2*IR_F0)/1000000 //������������ �������, ���������������� ���� = 0
													  //���������� � "�����" ������� 


#define RX_IR_ZERO (IR_ZERO_BIT_DURATION*8)/1000 //������������ �������, ���������������� ���� = 0
													  //���������� � "�����" ������� 



#define IR_ONE miles_protocol.ir_one//(IR_ONE_BIT_DURATION*2*IR_F0)/1000000 //������������ �������, ���������������� ���� = 1
													  //���������� � "�����" ������� 

#define RX_IR_ONE (IR_ONE_BIT_DURATION*8)/1000 //������������ �������, ���������������� ���� = 1
													  //���������� � "�����" ������� 



#define IR_START miles_protocol.ir_start//(IR_START_BIT_DURATION*2*IR_F0)/1000000 //������������ ��������� 
														//���������� � "�����" ������� 


#define RX_IR_START (IR_START_BIT_DURATION*8)/1000 //������������ ��������� 
														//���������� � "�����" ������� 


#define IR_SPACE  miles_protocol.ir_space//(IR_SPACE_DURATION*2*IR_F0)/1000000 //������������ ��������� ����� ������������� ������
														//���������� � "�����" ������� 


#define RX_IR_SPACE  (IR_SPACE_DURATION*8)/1000 //������������ ��������� ����� ������������� ������
														//���������� � "�����" ������� 


#define  SAFE_DURATION eeprom_read_byte(&eeprom_safe_duration)//5      //����� (� ������� �������) ������������ ����� �������




//#define ERROR_TOLERANCE 14

//#define ERROR_TOLERANCE 2


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

#define Add_Health 0x80 //��������� �������� �����
#define Add_Rounds 0x81 //��������� �������� ��������
#define Change_color 0xA9//��������� ������� ���� ������� 
#define Command 0x83 //���������-�������
#define Valid_value 0xE8 //���������� �������� ������������ ����� ���������
#define Set_life 0xA3 //���������� �� ������
#define Set_damage 0xA8 //���������� �� ����
