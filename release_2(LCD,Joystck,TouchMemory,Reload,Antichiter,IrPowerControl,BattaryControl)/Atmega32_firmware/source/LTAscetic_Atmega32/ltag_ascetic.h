#include <avr/io.h>        // Определяет имена для портов ввода-ывода
#include <util/delay.h>    // Дает возможность формирования задержки
#include <avr/interrupt.h> // Будем использовать прерывания
#include <avr/pgmspace.h>  //Будем хранить константы в памяти программ
#include <avr/eeprom.h>

#include "definition_of_ports_atmega16.h"
#include "hal.h"
#include "miles_protocol.h"
#include "types.h"
//#include "shift_regist_driver.h"






//Декларируем наши функции
//extern void init_shift_register(void);
void configuring_ports(void);		//конфигурация портов
void init_timer2(void); 			//Настройка таймера timer2, будет использоваться для приёма и передачи ИК-пакетов
void init_int0(void);				//Настраиваем внешние прерывания вывода INT0
void init_tm(void);				//Настраиваем внешние прерывания вывода INT1
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
inline  TKEYBOARD_STATUS get_reload_key_status(void);//Проверяем, нажата ли клавиша "Перезарядить"
inline  TKEYBOARD_EVENT test_reload_key(void);//Проверяем события клавиши "Перезарядить"
uint8_t bullets_limit(void);//Определяем лимит патронов
TFIRE_MODE_STATUS fire_mode(void);//Определяем текущий режим огня (одиночный/очередями)
void beep(uint16_t, uint16_t, uint8_t); //Воспроизводим звук (частота, длительность, громкость)
void damage_beep(void); // Воспроизводим звук при ранении
void playhitsound(void); //Воспроизводим звук при ранении
void playclipinsound(void); //Воспроизводим звук при передёргивании затвора
void playclipoutsound(void); //Воспроизводим звук при отпускании затвора
void write_team_id_to_eeprom(tteam_color);
void invite(void); //приглашение  в меню настроек
//inline trx_packet get_packet_value(void); //Считываем данные из полученного 
char* int_to_str(uint8_t x, uint8_t digits);//Преобразуем число в строку
volatile void get_int_settings(char* text, uint8_t* var_adress, uint8_t max_value);//Получаем значения настроек с помощью джойстика и ЖКИ
void check_eeprom(void);//проверим корректность значений переменных, хранимых в eeprom
volatile void get_enum_settings(char* text, uint8_t* var_adress, uint8_t* arr_adress, uint8_t max_value);
void display_status(void);//выводим на дисплей текущее значение жизни, патронов, магазинов 
void display_life_update(void);//обновляем значение жизни на дисплее
void display_bullets_update(void);//обновляем количество патронов на дисплее
void display_clips_update(void);//обновляем количество магазинов на дисплее
void init_adc(void);
uint16_t ReadADC(uint8_t ch);
void display_voltage_update(void);
void display_hit_data(void);//покажем кто в нас попал и какой урон нанёс
//void generate_batt_symbols(void);//создаем в памяти ЖКИ символы батарейки
void get_ir_power_settings(void);
void get_all_setings(void);
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
extern volatile TKEYBOARD_EVENT  reload_key_event; //события клавиши "Перезарядить"
extern volatile TJOYSTICK_EVENT  joystick_event; //события джойстика 
extern volatile struct pressing_duration key_pressing_duration;//структура хранит счетчики длительности нажатия кнопок
extern volatile struct joystick_pressing_duration joystick_key_pressing_duration;
extern volatile uint8_t fire_led_status; //статус светодиода вспышки выстрела 
extern volatile uint16_t bullets;  //количество патронов в таге
extern volatile uint16_t last_simple;					//порядковый номер последней выборки звука
extern volatile bool play_hit_snd; //этот флаг разрешает (true) или запрещает (false) воспроизведение дополнительных звуков
extern volatile tsnd_adress snd_adress; //в этой переменной будем хранить адрес звукового массива, который нужно воспроизвести
//extern volatile uint8_t shift_register_buffer; //в этой переменной будем хранить данные для сдвигового регистра перед отправкой
//extern const unsigned char pSnd_hit[];

extern volatile EEMEM tteam_color eeprom_team_id;
extern volatile EEMEM uint8_t eeprom_player_id;
extern volatile EEMEM tgun_damage eeprom_damage;
extern volatile EEMEM uint8_t eeprom_bullets_in_clip; // количество патронов в обойме
extern volatile EEMEM uint8_t eeprom_clips; // количество обойм
extern volatile EEMEM uint8_t eeprom_reload_duration; // длительность перезарядки (в секундах)
extern volatile uint8_t clips;//кол-во оставшихся обойм
extern volatile life_in_percent;// остаток "жизни" в процетах
extern volatile uint16_t chit_detected_counter; // счётчик длительности отключения повязки
extern volatile bool chit_detected; // флаг, имеющий значение true, если зафиксировано отключение повязки
extern volatile bool tm_connect; //флаг, имеющий значение true, если зафиксировано поднесение ключа тачмемори к считывателю
//extern volatile TTM_EVENT tm_event; //события считывателя TouchMemory
extern volatile EEMEM uint16_t eeprom_batt_full_voltage; // напряжение полностью заряженной батареи (значение АЦП)
extern volatile EEMEM uint16_t eeprom_batt_low_voltage; // напряжение полностью разряженной батареи (значение АЦП)
extern volatile bool display_bullets_update_now; //флаг, указывающий, что пора обновить индикацию оставшихся патронов
extern volatile TDISPLAY_BATT_MODE display_batt_mode; //режим отображения напряжения батареи (иконка/цифры) 
extern volatile EEMEM uint8_t eeprom_curr_ir_pin; //определяет мощность ИК излучения
extern volatile uint8_t curr_ir_pin; //текущая ножка, которую будем "дергать" при выстреле, определяет мощность ИК излучения
extern volatile EEMEM ttm_serial_num eeprom_tm_serial_num;
