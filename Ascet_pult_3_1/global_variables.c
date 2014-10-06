/**************************************************************************************
* Глобальные переменные
***************************************************************************************/
#include <avr/io.h>        // Определяет имена для портов ввода-ывода
#include <util/delay.h>    // Дает возможность формирования задержки
#include <avr/interrupt.h> // Будем использовать прерывания
#include <avr/pgmspace.h>  //Будем хранить константы в памяти программ
#include <avr/eeprom.h>
#include "types.h"
#include "miles_protocol.h"
//volatile uint16_t timer1; 
volatile uint16_t timer2; 


volatile bool start_bit_received;				//Этот флаг устанвливается, если принят старт-бит

volatile uint16_t high_level_counter; 			//Счетчик длительности сигнала высокого уровня на выводе ИК-приемника										
												//который нужно передать следующим


volatile uint16_t bit_in_rx_buff; 				//Количество бит, принятых ИК-приемником 

volatile uint16_t bit_in_bt_rx_buff; 			//Количество бит, принятых по блютус 


volatile trx_event rx_event; 					//события ИК-приемника

volatile trx_event bt_rx_event; 				//события BT-приемника




volatile uint16_t low_level_counter; 			//Счетчик длительности сигнала низкого уровня на выводе ИК-приемника										
												//который нужно передать следующим


volatile uint8_t rx_buffer[RX_BUFFER_SIZE]; 	//Буффер ИК-приемника

volatile uint8_t bt_rx_buffer[RX_BUFFER_SIZE]; 	//Буффер для ИК-пакета, принятого по блютус

volatile uint8_t tx_buffer[TX_BUFFER_SIZE]; 	//Буффер ИК-передатчика


volatile bool ir_transmitter_on;	//флаг, разрешаюший (true) или запрещающий передачу данных через ИК-диод 

volatile uint16_t ir_pulse_counter; 					//Обратный счетчик "вспышек" ИК-диода

volatile int ir_space_counter; 					//Обратный счетчик длительности выключенного состояния ИК-диода (пауза между битами) 

//volatile union data_packet_union  data_packet; 	//В этой переменной будем формировать пакет данных для отправки через IR


volatile uint8_t cursor_position;				//Эта переменная будет хранить индех элемента массива данных data_packet.data[x]

//volatile union data_packet_union  data_packet; 	//В этой переменной будем формировать пакет данных для отправки через IR

volatile trx_packet rx_packet;

volatile trx_packet bt_rx_packet;

volatile uint8_t life; //уровень жизни (здоровье)

volatile uint8_t life_leds_status[4]; //этот массив будет хранить состояния 

volatile TKEYBOARD_EVENT  keyboard_event; //события клавиатуры 

volatile TKEYBOARD_EVENT  reload_key_event; //события клавиши "Перезарядить"

volatile TJOYSTICK_EVENT  joystick_event; //события джойстика 

volatile struct pressing_duration key_pressing_duration;//структура хранит счетчики длительности нажатия кнопок

volatile struct joystick_pressing_duration joystick_key_pressing_duration;

volatile uint8_t fire_led_status; //статус светодиода вспышки выстрела 

volatile uint8_t bullets;  //количество патронов в магазине

volatile uint16_t last_simple;					//порядковый номер последней выборки звука

volatile bool play_hit_snd; //этот флаг разрешает (true) или запрещает (false) воспроизведение дополнительных звуков

volatile tsnd_adress snd_adress; //в этой переменной будем хранить адрес звукового массива, который нужно воспроизвести


volatile EEMEM tteam_color eeprom_team_id;
//volatile uint8_t shift_register_buffer; //в этой переменной будем хранить данные для сдвигового регистра перед отправкой
volatile EEMEM uint8_t eeprom_player_id;

volatile EEMEM tgun_damage eeprom_damage;

volatile EEMEM uint8_t eeprom_bullets_in_clip; // количество патронов в обойме

volatile EEMEM uint8_t eeprom_clips; // количество обойм при инициализации

volatile EEMEM uint8_t eeprom_reload_duration; // длительность перезарядки (в секундах)

volatile EEMEM uint16_t eeprom_batt_full_voltage; // напряжение полностью заряженной батареи (значение АЦП)

volatile EEMEM uint16_t eeprom_batt_low_voltage; // напряжение полностью разряженной батареи (значение АЦП)

volatile EEMEM uint8_t eeprom_curr_ir_pin; //определяет мощность ИК излучения

volatile EEMEM ttm_serial_num eeprom_tm_serial_num;

volatile uint8_t clips;//кол-во оставшихся обойм

volatile life_in_percent;// остаток "жизни" в процетах

volatile uint16_t chit_detected_counter; // счётчик длительности отключения повязки

volatile bool chit_detected; // флаг, имеющий значение true, если зафиксировано отключение повязки

volatile bool tm_connect; //флаг, имеющий значение true, если зафиксировано поднесение ключа тачмемори к считывателю


volatile uint8_t display_bullets_update_now; //флаг, указывающий, что пора обновить индикацию оставшихся патронов
//volatile TTM_EVENT tm_event; //события считывателя TouchMemory



volatile TDISPLAY_BATT_MODE display_batt_mode; //режим отображения напряжения батареи (иконка/цифры) 


volatile uint8_t curr_ir_pin; //текущая ножка, которую будем "дергать" при выстреле, определяет мощность ИК излучения


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

volatile bool cr_received; //влаг, указывающий на то, что по UART получен символ "\r" - перевод корретки 
volatile bool bt_header_received; //влаг, указывающий на то, что по UART получен символ "h" - заголовок пакета 
volatile uint16_t uart_timer; //таймер для отсчета таймаута
volatile bool fcurr_simple_prepared; //флаг, указывающий, что сеймпл звука подготовлен для воспроизведения
volatile uint8_t curr_simple; //текущий сеймл звука
volatile EEMEM uint16_t sound_1_adress;//адрес в eeprom звука выстрела
volatile EEMEM uint16_t sound_1_size;//длина в байтах звука выстрела
volatile EEMEM uint16_t sound_2_adress;//адрес в eeprom звука выстрела
volatile EEMEM uint16_t sound_2_size;//длина в байтах звука выстрела
volatile EEMEM uint16_t sound_3_adress;//адрес в eeprom звука выстрела
volatile EEMEM uint16_t sound_3_size;//длина в байтах звука выстрела
volatile EEMEM uint16_t sound_4_adress;//адрес в eeprom звука выстрела
volatile EEMEM uint16_t sound_4_size;//длина в байтах звука выстрела
volatile EEMEM uint16_t sound_5_adress;//адрес в eeprom звука выстрела
volatile EEMEM uint16_t sound_5_size;//длина в байтах звука выстрела
volatile EEMEM uint16_t sound_6_adress;//адрес в eeprom звука выстрела
volatile EEMEM uint16_t sound_6_size;//длина в байтах звука выстрела
volatile EEMEM bool friendly_fire_enable; //флаг, указывающий, фиксировать (true) или нет (false) "дружественный" огонь

//volatile TSOUND_BUFFER curr_sound_buffer; //текущий буфер звука
volatile uint16_t curr_adress_in_eeprom;//адрес в eeprom, начиная с которого запишем данные в звуковой буфер
//volatile uint8_t curr_pos_in_sound_buff;//текущая пощиция в звуковом буфере
volatile uint16_t simples_in_queue;//сколько сеймплов осталось воспроизвести
//volatile uint8_t update_suond_buffer_now;//флаг, указывающия, что нужно обновить данные в звуковом буфере 
volatile bool eeprom_is_open;//флаг, указывает, открыта ли eeprom в данный момент
volatile uint16_t cut_off_sound; //количество невоспроизведённых сеймлов, при котором пора отсекать звук
volatile bool receiver_on;//флаг, показывающия, что идет прием ИК пакета
volatile tir_tx_buffer_cursor ir_tx_buffer_cursor; //курсор передающего ИК буфера
volatile bool pult_hot_key_mode; //если true, то находимся в режиме "горячих клавиш", если false - в расширенном режиме
volatile bool pult_help_mode; //если true, то находимся в режиме "помощь"

/***********************************************************
* Глобальные переменные для пульта (все храняться в eeprom пямяти контроллера)
***********************************************************/
volatile EEMEM uint16_t pult_key_up_command; //команда для клавиши джойстика "Вверх" ("UP"), кнопка "shift" не нажата
volatile EEMEM uint16_t pult_shift_and_key_up_command; //команда для клавиши джойстика "Вверх" ("UP"), кнопка "shift" нажата


volatile EEMEM uint16_t pult_key_right_command; //команда для клавиши джойстика "Вправо" ("RIGHT"), кнопка "shift" не нажата
volatile EEMEM uint16_t pult_shift_and_key_right_command; //команда для клавиши джойстика "Вверх" ("RIGHT"), кнопка "shift" нажата


volatile EEMEM uint16_t pult_key_down_command; //команда для клавиши джойстика "Вниз" ("DOWN"), кнопка "shift" не нажата
volatile EEMEM uint16_t pult_shift_and_key_down_command; //команда для клавиши джойстика "Вниз" ("DOWN"), кнопка "shift" нажата


volatile EEMEM uint16_t pult_key_left_command; //команда для клавиши джойстика "Влево" ("LEFT"), кнопка "shift" не нажата
volatile EEMEM uint16_t pult_shift_and_key_left_command; //команда для клавиши джойстика "Влево" ("LEFT"), кнопка "shift" нажата


volatile EEMEM uint16_t pult_key_central_command; //команда для клавиши джойстика "Центральная" ("CENTRAL"), кнопка "shift" не нажата
volatile EEMEM uint16_t pult_shift_and_key_central_command; //команда для клавиши джойстика "Центральная" ("CENTRAL"), кнопка "shift" нажата

volatile EEMEM uint16_t pult_reload_command; //команда для клавиши "Перезаряд" ("RELOAD"), кнопка "shift" не нажата
//volatile EEMEM uint16_t pult_shift_and_key_reload_command; //команда для клавиши "Перезаряд" ("RELOD"), кнопка "shift" нажата




