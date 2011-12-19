/**************************************************************************************
* Проверка
***************************************************************************************/

#include <avr/io.h>        // Определяет имена для портов ввода-ывода
#include <util/delay.h>    // Дает возможность формирования задержки
#include <avr/interrupt.h> // Будем использовать прерывания
#include <avr/pgmspace.h>  //Будем хранить константы в памяти программ
#include "types.h"
volatile uint16_t timer1; 


volatile bool start_bit_received;				//Этот флаг устанвливается, если принят старт-бит

volatile uint16_t high_level_counter; 			//Счетчик длительности сигнала высокого уровня на выводе ИК-приемника										
												//который нужно передать следующим


volatile uint16_t bit_in_rx_buff; 				//Количество бит, принятых ИК-приемником 

volatile trx_event rx_event; 					//события ИК-приемника

volatile uint16_t low_level_counter; 			//Счетчик длительности сигнала низкого уровня на выводе ИК-приемника										
												//который нужно передать следующим


volatile uint8_t rx_buffer[RX_BUFFER_SIZE]; 	//Буффер ИК-приемника

volatile uint8_t tx_buffer[TX_BUFFER_SIZE]; 	//Буффер ИК-приемника


volatile bool ir_transmitter_on;	//флаг, разрешаюший (true) или запрещающий передачу данных через ИК-диод 

volatile int ir_pulse_counter; 					//Обратный счетчик "вспышек" ИК-диода

volatile int ir_space_counter; 					//Обратный счетчик длительности выключенного состояния ИК-диода (пауза между битами) 

volatile union data_packet_union  data_packet; 	//В этой переменной будем формировать пакет данных для отправки через IR


volatile uint8_t cursor_position;				//Эта переменная будет хранить индех элемента массива данных data_packet.data[x]

volatile union data_packet_union  data_packet; 	//В этой переменной будем формировать пакет данных для отправки через IR

volatile trx_packet rx_packet;

volatile uint8_t life; //уровень жизни (здоровье)

volatile uint8_t life_leds_status[4]; //этот массив будет хранить состояния 

volatile TKEYBOARD_EVENT  keyboard_event; //события клавиатуры 

volatile struct pressing_duration key_pressing_duration;//структура хранит счетчики длительности нажатия кнопок

volatile uint8_t fire_led_status; //статус светодиода вспышки выстрела 

volatile uint16_t bullets;  //количество патронов в таге

volatile uint16_t last_simple;					//порядковый номер последней выборки звука


//Массив для получения реального значения урона. Реальный урон = damage_value[Damage_xx]
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

