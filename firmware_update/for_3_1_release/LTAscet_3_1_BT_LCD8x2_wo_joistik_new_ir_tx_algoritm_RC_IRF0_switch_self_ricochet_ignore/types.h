#include "joystick_driver_types.h"

#define RX_BUFFER_SIZE   8 //������ ������� ���������
#define TX_BUFFER_SIZE   8 //������ ������� ���������

#ifndef bool

                        #define bool unsigned char

                        #define true 1

                        #define false 0

#endif






//---------------------------------------------------------------------//


//��������� ������������� ��� ��� ������� ��-���������
enum Rx_Event {	
				NOT_EVENT, 		//��� �������
				RX_COMPLETE, 	//������ �����
				RX_MESSAGE_COMPLITE,//������� ���������
				RX_ERROR		//������ ������ ������
				}; 
typedef enum Rx_Event trx_event;




//��������� ��������� ��� �������� ����������� �� �� ���������

typedef struct IR_Message {
uint8_t ID; //������������ ���������
uint8_t param;//�������� ���������
uint8_t control_byte;//����������� ����
} tir_message;








//--------------------------------------------------

typedef struct IR_Tx_Buffer_Cursor {
uint8_t byte_pos; //����� ����� � ������ �����������
uint8_t bit_mask;//������� ����� ��� ��������� �������� ����
uint16_t bits_for_tx;//������� ��� ����� ��� �������� 

} tir_tx_buffer_cursor;






//---------------------------------------------------------------------//

//��������� ��������� ��� �������� �������������� ������
//� ��� ����� ������� ������������ ��������  � "�����" ������� (IR_ZERO ��� IR_ONE)
typedef struct PlayerID {
uint16_t bit_7;//��������� ��� (������ ������ ���� ����� IR_ZERO)
uint16_t bit_6; 
uint16_t bit_5; 
uint16_t bit_4;
uint16_t bit_3;
uint16_t bit_2;
uint16_t bit_1;
uint16_t bit_0;//������ ���
} tplayer_id;

/*
union player_id_union {
tplayer_id bits;
uint8_t data [8];
};
*/


//��������� ��������� ��� �������� �������������� ������
//� ��� ����� ������� ������������ �������� � "�����" ������� (IR_ZERO ��� IR_ONE) 
typedef struct TeamID{
uint16_t bit_1;
uint16_t bit_0;
} tteam_id;
 


typedef struct Damage{
uint16_t bit_3; 
uint16_t bit_2; 
uint16_t bit_1; 
uint16_t bit_0;

} tdamage;




//������ ������ ��������� ������ ������
typedef struct DataPacket {
uint16_t header; //���������, ������ ������ ���� ����� IR_START (1 ����)
tplayer_id player_id; //����� ��������� ���� ������������� ������ (8 ����)
tteam_id team_id;	  //����� ������������� ������� (2 �����)
tdamage damage;		  //�� � ��������� ����� "����" (4 �����)
uint16_t end_of_data;  //�����, ����������� �����������, ��� ������ ��� �������� ������ ��� (������ ������ ���� ����� 0) (1 ����)
} tdata_packet;

//---------------------------------------------------------------------//

union data_packet_union{
tdata_packet packet;
uint16_t data[16];
};





//������ ���������, � ������� ����� ������� ������, ��������� �� ����������� ������
typedef struct RX_Packet {
uint8_t player_id;	//������������� ��������� � ��� ������
uint8_t team_id;	//������������� (����) ��� �������
uint8_t damage;		//���������� ��� ����
} trx_packet;


//������ ���������, � ������� ����� ������� ����� ���������������� ��������������� �����
typedef struct SoundAddres {
uint8_t *start_adress; //����� ������� ������� 
uint8_t *end_adress; //����� ���������� �������
uint8_t *curr_adress; //����� ���������� �������, ������� ����� ������������� 

} tsnd_adress;


enum typkeyboard_status {
			no_key_pressed,
			key_pressed
			};
typedef enum typkeyboard_status TKEYBOARD_STATUS;

enum typkeyboard_event  {
			no_key_pressing,
			key_pressing,
	        };
typedef enum typkeyboard_event TKEYBOARD_EVENT;


struct pressing_duration
		{ uint16_t key_1      ; //������������ ������������ 
					    //������� ������ 1 ("������� ����")
		  unsigned int key_1_inc:1; //���, ����������� 
					    //��� ����������� ������
					    //������������ ������� ������ 1  
		  uint16_t key_2      ; //������������ ������������ 
					    //������� ������ 2 ("������������")
		  unsigned int key_2_inc:1; //���, ����������� 
					    //��� ����������� ������
					    //������������ ������� ������ 2  
		  
		  
		  
		  unsigned int no_key   ;

		};



enum tfire_mode_status {
single,
queues
};
typedef enum tfire_mode_status TFIRE_MODE_STATUS;





enum tdisplay_batt_mode {
icon,
digit
};
typedef enum tdisplay_batt_mode TDISPLAY_BATT_MODE;




//��������� ��������� ��� �������� ��������� ������ ����������
//����� Touch Memory
typedef struct TM_serial_num {
unsigned char device_code;//��� ����������
unsigned char serial[6]; //�������� ����� �����
} ttm_serial_num;

/*

enum sound_buffer{
sound_buffer_1,
sound_buffer_2
};

typedef enum sound_buffer TSOUND_BUFFER;
*/

typedef struct miles_protocol{
uint16_t carrier_frequency;
uint16_t err_tolerance;
uint16_t ir_zero;
uint16_t ir_one;
uint16_t ir_start;
uint16_t ir_space;
} tmiles_protocol;


enum ir_carrier_frequency{
freq_36KHz,
freq_56KHz
};
typedef enum  ir_carrier_frequency TIR_CARRIER_FREQUENCY;


enum reload_status{
	nothing_to_do,
	waiting_countdown,
	reload_now
};
typedef enum  reload_status TRELOAD_STATUS;


