#include <avr/io.h>        // ���������� ����� ��� ������ �����-�����
#include <util/delay.h>    // ���� ����������� ������������ ��������
#include <avr/interrupt.h> // ����� ������������ ����������
#include <avr/pgmspace.h>  //����� ������� ��������� � ������ ��������
#include <avr/eeprom.h>
#include <string.h>
//#include <util/uart.h>

#include "definition_of_ports_atmega16.h"
#include "hal.h"
#include "miles_protocol.h"
#include "types.h"
#include "usart.h"
#include "menu.h"
//#include "shift_regist_driver.h"






//����������� ���� �������
//extern void init_shift_register(void);
void configuring_ports(void);		//������������ ������
void init_timer2(void); 			//��������� ������� timer2, ����� �������������� ��� ����� � �������� ��-�������
void init_int0(void);				//����������� ������� ���������� ������ INT0
void init_tm(void);				//����������� ������� ���������� ������ INT1
void set_buffer_bit(uint8_t, bool);	//������ �������� ���� � ������ ��-���������
void set_bt_buffer_bit(uint8_t, bool);	//������ �������� ���� � ������ ��� ��-������, ��������� �� ������
void send_ir_package(void);			//���������� �������������� ����� (�������)
void set_player_id(uint8_t);		//������ ������ �������������
void set_team_color(tteam_color);	//������ ���� ����� �������
void set_gun_damage(tgun_damage);	//������ ��������� ������ ������ (����)
void init_var(void);				//����������� ���������� �������� ����������
bool get_buffer_bit(uint8_t);		//��������� �������� ���� � ������ ��-���������
bool get_bt_buffer_bit(uint8_t);	//��������� �������� ���� � ������ ������, ��������� �� ������

inline trx_packet get_packet_value(void); //��������� ������ �� ����������� ������
inline trx_packet get_bt_packet_value(void); //��������� ������ �� ����������� ������
tteam_color team_id(void);//���������� ���� ����� ������� 
tgun_damage gun_damage(void);//���������� ������� ����, ��������� ����� �����
void init_timer0(void); //����������� timer0 �� ����� �������� ���, ��� ������ 
void init_timer1(void); //����������� timer1 �� ������� ������� ����� -8 
void display_life(uint8_t life_value);//���������� ������� ����� �� ������������ 
inline  TKEYBOARD_STATUS get_keyboard_status(void); //���������, ����� �� �����
inline  TKEYBOARD_EVENT test_keyboard(void);//��������� ������� ����������
inline  TKEYBOARD_STATUS get_reload_key_status(void);//���������, ������ �� ������� "������������"
inline  TKEYBOARD_EVENT test_reload_key(void);//��������� ������� ������� "������������"
uint8_t bullets_limit(void);//���������� ����� ��������
TFIRE_MODE_STATUS fire_mode(void);//���������� ������� ����� ���� (���������/���������)
void beep(uint16_t, uint16_t, uint8_t); //������������� ���� (�������, ������������, ���������)
void damage_beep(void); // ������������� ���� ��� �������
void playhitsound(void); //������������� ���� ��� �������
void playclipinsound(void); //������������� ���� ��� ������������� �������
void playclipoutsound(void); //������������� ���� ��� ���������� �������
void write_team_id_to_eeprom(tteam_color);
void invite(void); //�����������  � ���� ��������
//inline trx_packet get_packet_value(void); //��������� ������ �� ����������� 
char* int_to_str(uint8_t x, uint8_t digits);//����������� ����� � ������
char* long_int_to_str(uint16_t x, uint8_t digits);//����������� ����� � ������
volatile void get_int_settings(char* text, uint8_t* var_adress, uint8_t max_value);//�������� �������� �������� � ������� ��������� � ���
void check_eeprom(void);//�������� ������������ �������� ����������, �������� � eeprom
volatile void get_enum_settings(char* text, uint8_t* var_adress, uint8_t* arr_adress, uint8_t max_value);
void display_status(void);//������� �� ������� ������� �������� �����, ��������, ��������� 
void display_life_update(void);//��������� �������� ����� �� �������
void display_bullets_update(void);//��������� ���������� �������� �� �������
void display_clips_update(void);//��������� ���������� ��������� �� �������
void init_adc(void);
uint16_t ReadADC(uint8_t ch);
void display_voltage_update(void);
void display_hit_data(trx_packet hit_packet);//������� ��� � ��� ����� � ����� ���� ����
//void generate_batt_symbols(void);//������� � ������ ��� ������� ���������
void get_ir_power_settings(void);
void get_friendly_fire_settings(void);//���������/���������� "��������������" ����
void get_all_setings(void);
uint8_t get_command_index(void);//��������, ��� �� ������� ������ �� UART
uint8_t char_to_int(char);//����������� ������ � ����� �����
//void command_1_slot(void);
bool play_sound_from_eeprom(uint16_t address, uint16_t data_size); //������������� ���� ��������������� �� eeprom
void play_sound_1(void);//������������� ���� 1
void play_sound_2(void);//������������� ���� 2
void play_sound_3(void);//������������� ���� 3
void play_sound_4(void);//������������� ���� 4
void play_sound_5(void);//������������� ���� 5
void play_sound_6(void);//������������� ���� 6
trx_event parsing_bt_data(void);//����������� �����, ���������� �� ������, �������� �� ���� ������
void hit_processing(trx_packet hit_packet);//������������ ���������
void ir_tx_cursor_home(void);//������������� ������ � ������ ������
void send_message(uint8_t ID, uint8_t DATA); //���������� ���������
void pult_reset_to_defaul(void); //���������� ��������� ��� ������ ������
bool get_pult_default_settings(void);//�������, ���� �� �������� ��������� ������ �� ����������?
void menu_slot_1(void);
void menu_slot_2(void);
//void sound_buffer_update(void);//�������� � �������� ����� ����� ������ �������
//uint8_t read_seimpl_from_sound_buffer(void);//��������� ��������� ������ �� ������
//void play_shot_sound(void);//������������� ���� ��������

uint8_t get_pult_message_index (uint8_t message); //�������� ������ ���������
uint8_t get_pult_command_index (uint8_t command); //�������� ������ �������


//���������� ����������

extern volatile uint16_t timer1; 
extern volatile uint16_t timer2; 

extern volatile bool start_bit_received;				//���� ���� ��������������, ���� ������ �����-���
extern volatile uint16_t high_level_counter; 			//������� ������������ ������� �������� ������ �� ������ ��-���������										//������� ����� �������� ���������
extern volatile uint16_t bit_in_rx_buff; 				//���������� ���, �������� ��-���������� 
extern volatile uint16_t bit_in_bt_rx_buff; 			//���������� ���, �������� �� ������ 
extern volatile trx_event rx_event; 					//������� ��-���������
extern volatile trx_event bt_rx_event; 				//������� BT-���������
extern volatile uint16_t low_level_counter; 			//������� ������������ ������� ������� ������ �� ������ ��-���������										
extern volatile uint8_t rx_buffer[RX_BUFFER_SIZE]; 	//������ ��-���������
extern volatile uint8_t bt_rx_buffer[RX_BUFFER_SIZE]; 	//������ ��� ��-������, ��������� �� ������

extern volatile bool ir_transmitter_on;				//����, ����������� (true) ��� ����������� �������� ������ ����� ��-���� 														//������� ����� �������� ���������
extern volatile uint16_t ir_pulse_counter; 					//�������� ������� "�������" ��-�����
extern volatile int ir_space_counter; 					//�������� ������� ������������ ������������ ��������� ��-����� (����� ����� ������) 
//extern volatile trx_packet rx_packet;

//extern volatile union data_packet_union  data_packet; 	//� ���� ���������� ����� ����������� ����� ������ ��� �������� ����� IR
extern volatile uint8_t cursor_position;				//��� ���������� ����� ������� ����� �������� ������� ������ data_packet.data[x]
//extern volatile union data_packet_union  data_packet; 	//� ���� ���������� ����� ����������� ����� ������ ��� �������� ����� IR
extern uint8_t damage_value [] PROGMEM;
extern volatile trx_packet rx_packet;
extern volatile trx_packet bt_rx_packet;
extern volatile uint8_t life; //������� ����� (��������)
extern volatile uint8_t life_leds_status[4]; //���� ������ ����� ������� ��������� ����������� ���������� ��������
extern volatile TKEYBOARD_EVENT  keyboard_event; //������� ���������� 
extern volatile TKEYBOARD_EVENT  reload_key_event; //������� ������� "������������"
extern volatile TJOYSTICK_EVENT  joystick_event; //������� ��������� 
extern volatile struct pressing_duration key_pressing_duration;//��������� ������ �������� ������������ ������� ������
extern volatile struct joystick_pressing_duration joystick_key_pressing_duration;
extern volatile uint8_t fire_led_status; //������ ���������� ������� �������� 
extern volatile uint16_t bullets;  //���������� �������� � ����
extern volatile uint16_t last_simple;					//���������� ����� ��������� ������� �����
extern volatile bool play_hit_snd; //���� ���� ��������� (true) ��� ��������� (false) ��������������� �������������� ������
extern volatile tsnd_adress snd_adress; //� ���� ���������� ����� ������� ����� ��������� �������, ������� ����� �������������
//extern volatile uint8_t shift_register_buffer; //� ���� ���������� ����� ������� ������ ��� ���������� �������� ����� ���������
//extern const unsigned char pSnd_hit[];

extern volatile EEMEM tteam_color eeprom_team_id;
extern volatile EEMEM uint8_t eeprom_player_id;
extern volatile EEMEM tgun_damage eeprom_damage;
extern volatile EEMEM uint8_t eeprom_bullets_in_clip; // ���������� �������� � ������
extern volatile EEMEM uint8_t eeprom_clips; // ���������� �����
extern volatile EEMEM uint8_t eeprom_reload_duration; // ������������ ����������� (� ��������)
extern volatile uint8_t clips;//���-�� ���������� �����
extern volatile life_in_percent;// ������� "�����" � ��������
extern volatile uint16_t chit_detected_counter; // ������� ������������ ���������� �������
extern volatile bool chit_detected; // ����, ������� �������� true, ���� ������������� ���������� �������
extern volatile bool tm_connect; //����, ������� �������� true, ���� ������������� ���������� ����� ��������� � �����������
//extern volatile TTM_EVENT tm_event; //������� ����������� TouchMemory
extern volatile EEMEM uint16_t eeprom_batt_full_voltage; // ���������� ��������� ���������� ������� (�������� ���)
extern volatile EEMEM uint16_t eeprom_batt_low_voltage; // ���������� ��������� ����������� ������� (�������� ���)
extern volatile uint8_t display_bullets_update_now; //����, �����������, ��� ���� �������� ��������� ���������� ��������
extern volatile TDISPLAY_BATT_MODE display_batt_mode; //����� ����������� ���������� ������� (������/�����) 
extern volatile EEMEM uint8_t eeprom_curr_ir_pin; //���������� �������� �� ���������
extern volatile uint8_t curr_ir_pin; //������� �����, ������� ����� "�������" ��� ��������, ���������� �������� �� ���������
extern volatile EEMEM ttm_serial_num eeprom_tm_serial_num;


extern volatile bool cr_received; //����, ����������� �� ��, ��� �� UART ������� ������ "\r" - ������� �������� 
extern volatile bool bt_header_received; //����, ����������� �� ��, ��� �� UART ������� ������ "h" - ��������� ������ 


//���������� �����
extern volatile unsigned char usartTxBuf[SIZE_BUF];
extern volatile unsigned char txBufTail;
extern volatile unsigned char txBufHead;
extern volatile unsigned char txCount;

//�������� �����
extern volatile unsigned char usartRxBuf[SIZE_BUF];
extern volatile unsigned char rxBufTail;
extern volatile unsigned char rxBufHead;
extern volatile unsigned char rxCount;
extern volatile uint16_t uart_timer; //������ ��� ������� ��������

extern volatile bool fcurr_simple_prepared; //����, �����������, ��� ������ ����� ����������� ��� ���������������
extern volatile uint8_t curr_simple; //������� ����� �����

extern volatile EEMEM uint16_t sound_1_adress;//����� � eeprom ����� ��������
extern volatile EEMEM uint16_t sound_1_size;//����� � ������ ����� ��������
extern volatile EEMEM uint16_t sound_2_adress;//����� � eeprom ����� ��������
extern volatile EEMEM uint16_t sound_2_size;//����� � ������ ����� ��������
extern volatile EEMEM uint16_t sound_3_adress;//����� � eeprom ����� ��������
extern volatile EEMEM uint16_t sound_3_size;//����� � ������ ����� ��������
extern volatile EEMEM uint16_t sound_4_adress;//����� � eeprom ����� ��������
extern volatile EEMEM uint16_t sound_4_size;//����� � ������ ����� ��������
extern volatile EEMEM uint16_t sound_5_adress;//����� � eeprom ����� ��������
extern volatile EEMEM uint16_t sound_5_size;//����� � ������ ����� ��������
extern volatile EEMEM uint16_t sound_6_adress;//����� � eeprom ����� ��������
extern volatile EEMEM uint16_t sound_6_size;//����� � ������ ����� ��������
extern volatile EEMEM bool friendly_fire_enable; //����, �����������, ����������� (true) ��� ��� (false) "�������������" �����
//extern volatile TSOUND_BUFFER curr_sound_buffer; //������� ����� �����, �� �������� ����������� �������
extern volatile uint16_t curr_adress_in_eeprom;//����� � eeprom, ������� � �������� ������� ������ � �������� �����
//extern volatile uint8_t curr_pos_in_sound_buff;//������� ������� � �������� ������
extern volatile uint16_t simples_in_queue;//������� �������� �������� �������������

//extern volatile uint8_t update_suond_buffer_now;//����, �����������, ��� ����� �������� ������ � �������� ������ 
extern volatile bool eeprom_is_open;//����, ���������, ������� �� eeprom � ������ ������
extern volatile uint16_t cut_off_sound; //���������� ����������������� �������, ��� ������� ���� �������� ����
extern const unsigned char Decode2Rus[255-192+1] PROGMEM;
extern volatile bool receiver_on;//����, ������������, ��� ���� ����� �� ������
extern volatile tir_tx_buffer_cursor ir_tx_buffer_cursor; //������ ����������� �� ������
extern volatile uint8_t tx_buffer[TX_BUFFER_SIZE]; 	//������ ��-�����������
extern volatile bool pult_hot_key_mode; //���� true, �� ��������� � ������ "������� ������", ���� false - � ����������� ������
extern volatile bool pult_help_mode; //���� true, �� ��������� � ������ "������"



/***********************************************************
* ���������� ���������� ��� ������ (��� ��������� � eeprom ������ �����������)
***********************************************************/
extern volatile EEMEM uint16_t pult_key_up_command; //������� ��� ������� ��������� "�����" ("UP"), ������ "shift" �� ������
extern volatile EEMEM uint16_t pult_shift_and_key_up_command; //������� ��� ������� ��������� "�����" ("UP"), ������ "shift" ������


extern volatile EEMEM uint16_t pult_key_right_command; //������� ��� ������� ��������� "������" ("RIGHT"), ������ "shift" �� ������
extern volatile EEMEM uint16_t pult_shift_and_key_right_command; //������� ��� ������� ��������� "�����" ("RIGHT"), ������ "shift" ������


extern volatile EEMEM uint16_t pult_key_down_command; //������� ��� ������� ��������� "����" ("DOWN"), ������ "shift" �� ������
extern volatile EEMEM uint16_t pult_shift_and_key_down_command; //������� ��� ������� ��������� "����" ("DOWN"), ������ "shift" ������


extern volatile EEMEM uint16_t pult_key_left_command; //������� ��� ������� ��������� "�����" ("LEFT"), ������ "shift" �� ������
extern volatile EEMEM uint16_t pult_shift_and_key_left_command; //������� ��� ������� ��������� "�����" ("LEFT"), ������ "shift" ������


extern volatile EEMEM uint16_t pult_key_central_command; //������� ��� ������� ��������� "�����������" ("CENTRAL"), ������ "shift" �� ������
extern volatile EEMEM uint16_t pult_shift_and_key_central_command; //������� ��� ������� ��������� "�����������" ("CENTRAL"), ������ "shift" ������

extern volatile EEMEM uint16_t pult_reload_command; //������� ��� ������� "���������" ("RELOAD"), ������ "shift" �� ������
//extern volatile EEMEM uint16_t pult_shift_and_key_reload_command; //������� ��� ������� "���������" ("RELOD"), ������ "shift" ������


//extern const uint8_t messages [] PROGMEM;

//extern const uint8_t commands [] PROGMEM;

