#include <avr/io.h>        // Определяет имена для портов ввода-ывода
#include <util/delay.h>    // Дает возможность формирования задержки
#include <avr/interrupt.h> // Будем использовать прерывания
#include <avr/pgmspace.h>  //Будем хранить константы в памяти программ

#include "definition_of_ports_atmega16.h"
#include "hal.h"
#include "miles_protocol.h"
#include "types.h"







//Декларируем наши функции

void configuring_ports(void);		//конфигурация портов
void init_timer2(void); 			//Настройка таймера timer2, будет использоваться для приёма и передачи ИК-пакетов
void init_int0(void);				//Настраиваем внешние прерывания вывода INT0
void set_buffer_bit(uint8_t, bool);	//Задаем значение биту в буфере ИК-приемника
void send_ir_package(void);			//Отправляем подготовленный пакет (выстрел)
void set_player_id(uint8_t);		//Задаем игроку идентификатор
void set_team_color(tteam_color);	//Задаем цвет нашей команды
void set_gun_damage(tgun_damage);	//Задаем мощьность нашего оружия (урон)
void init_var(void);				//Присваиваем дефалтовые значения переменным
bool get_buffer_bit(uint8_t);		//Считываем значение бита в буфере ИК-приемника
inline trx_packet get_packet_value(void); //Считываем данные из полученного пакета
tteam_color team_id(void);//Опреднляем цвет нашей команды 
tgun_damage gun_damage(void);//Определяем текущий урон, наносимый нашим тагом
void init_timer0(void); //Настраиваем timer0 на режим быстрого ШИМ, для вывода 
void init_timer1(void); //Настраиваем timer1 на частоту выборки звука -8 
void display_life(uint8_t life_value);//отображаем уровень жизни на светодиодной 
inline  TKEYBOARD_STATUS get_keyboard_status(void); //Проверяем, нажат ли курок
inline  TKEYBOARD_EVENT test_keyboard(void);//Проверяем события клавиатуры
uint8_t bullets_limit(void);//Определяем лимит патронов
TFIRE_MODE_STATUS fire_mode(void);//Определяем текущий режим огня (одиночный/очередями)
void beep(uint16_t, uint16_t, uint8_t); //Воспроизводим звук (частота, длительность, громкость)
void damage_beep(void); // Воспроизводим звук при ранении
void playhitsound(void); //Воспроизводим звук при ранении

//inline trx_packet get_packet_value(void); //Считываем данные из полученного 



//Глобальные переменные

extern volatile uint16_t timer1; 

extern volatile bool start_bit_received;				//Этот флаг устанвливается, если принят старт-бит
extern volatile uint16_t high_level_counter; 			//Счетчик длительности сигнала высокого уровня на выводе ИК-приемника										//который нужно передать следующим
extern volatile uint16_t bit_in_rx_buff; 				//Количество бит, принятых ИК-приемником 
extern volatile trx_event rx_event; 					//события ИК-приемника
extern volatile uint16_t low_level_counter; 			//Счетчик длительности сигнала низкого уровня на выводе ИК-приемника										
extern volatile uint8_t rx_buffer[RX_BUFFER_SIZE]; 	//Буффер ИК-приемника
extern volatile bool ir_transmitter_on;				//флаг, разрешаюший (true) или запрещающий передачу данных через ИК-диод 														//который нужно передать следующим
extern volatile int ir_pulse_counter; 					//Обратный счетчик "вспышек" ИК-диода
extern volatile int ir_space_counter; 					//Обратный счетчик длительности выключенного состояния ИК-диода (пауза между битами) 
//extern volatile trx_packet rx_packet;

extern volatile union data_packet_union  data_packet; 	//В этой переменной будем формировать пакет данных для отправки через IR
extern volatile uint8_t cursor_position;				//Эта переменная будет хранить индех элемента массива данных data_packet.data[x]
extern volatile union data_packet_union  data_packet; 	//В этой переменной будем формировать пакет данных для отправки через IR
extern uint8_t damage_value [] PROGMEM;
extern volatile trx_packet rx_packet;
extern volatile uint8_t life; //уровень жизни (здоровье)
extern volatile uint8_t life_leds_status[4]; //этот массив будет хранить состояния светодиодов индикатора здоровья
extern volatile TKEYBOARD_EVENT  keyboard_event; //события клавиатуры 
extern volatile struct pressing_duration key_pressing_duration;//структура хранит счетчики длительности нажатия кнопок
extern volatile uint8_t fire_led_status; //статус светодиода вспышки выстрела 
extern volatile uint16_t bullets;  //количество патронов в таге
extern volatile uint16_t last_simple;					//порядковый номер последней выборки звука
extern volatile bool play_hit_snd; //этот флаг разрешает (true) или запрещает (false) воспроизведение дополнительных звуков
extern volatile tsnd_adress snd_adress; //в этой переменной будем хранить адрес звукового массива, который нужно воспроизвести

//extern const unsigned char pSnd_hit[];
