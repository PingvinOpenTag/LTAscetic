#include <avr/io.h>        // Определяет имена для портов ввода-ывода
#include <util/delay.h>    // ���� ����������� ������������ ��������
#include <avr/interrupt.h> // ����� ������������ ����������
#include <avr/pgmspace.h>  //����� ������� ��������� � ������ ��������
#include "types.h"
volatile uint16_t timer1; 


volatile bool start_bit_received;				//���� ���� ��������������, ���� ������ �����-���

volatile uint16_t high_level_counter; 			//������� ������������ ������� �������� ������ �� ������ ��-���������										
												//������� ����� �������� ���������


volatile uint16_t bit_in_rx_buff; 				//���������� ���, �������� ��-���������� 

volatile trx_event rx_event; 					//������� ��-���������

volatile uint16_t low_level_counter; 			//������� ������������ ������� ������� ������ �� ������ ��-���������										
												//������� ����� �������� ���������


volatile uint8_t rx_buffer[RX_BUFFER_SIZE]; 	//������ ��-���������

volatile uint8_t tx_buffer[TX_BUFFER_SIZE]; 	//������ ��-���������


volatile bool ir_transmitter_on;	//����, ����������� (true) ��� ����������� �������� ������ ����� ��-���� 

volatile int ir_pulse_counter; 					//�������� ������� "�������" ��-�����

volatile int ir_space_counter; 					//�������� ������� ������������ ������������ ��������� ��-����� (����� ����� ������) 

volatile union data_packet_union  data_packet; 	//� ���� ���������� ����� ����������� ����� ������ ��� �������� ����� IR


volatile uint8_t cursor_position;				//��� ���������� ����� ������� ����� �������� ������� ������ data_packet.data[x]

volatile union data_packet_union  data_packet; 	//� ���� ���������� ����� ����������� ����� ������ ��� �������� ����� IR

volatile trx_packet rx_packet;

volatile uint8_t life; //������� ����� (��������)

volatile uint8_t life_leds_status[4]; //���� ������ ����� ������� ��������� 

volatile TKEYBOARD_EVENT  keyboard_event; //������� ���������� 

volatile struct pressing_duration key_pressing_duration;//��������� ������ �������� ������������ ������� ������

volatile uint8_t fire_led_status; //������ ���������� ������� �������� 

volatile uint16_t bullets;  //���������� �������� � ����

volatile uint16_t last_simple;					//���������� ����� ��������� ������� �����


//������ ��� ��������� ��������� �������� �����. �������� ���� = damage_value[Damage_xx]
uint8_t damage_value [] PROGMEM = 
{
	1,   // 0000 = 1   (Damage_1)
	2,   // 0001 = 2   (Damage_2)
	4,   // 0010 = 4   (Damage_4)
	5,   // 0011 = 5   (Damage_5)
	7,   // 0100 = 7   (Damage_7)
	10,  // 0101 = 10  (Damage_10)
	15,  // 0110 = 15  (Damage_15)
	17,  // 0111 = 17  (Damage_17)
	20,  // 1000 = 20  (Damage_20)
	25,  // 1001 = 25  (Damage_25)
	30,  // 1010 = 30  (Damage_30)
	35,  // 1011 = 35  (Damage_35)
	40,  // 1100 = 40  (Damage_40)
	50,  // 1101 = 50  (Damage_50)
	75,  // 1110 = 75  (Damage_75)
	100, // 1111 = 100 (Damage_100)

};

