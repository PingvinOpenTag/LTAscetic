#include <avr/io.h>        // Определяет имена для портов ввода-ывода
#include <util/delay.h>    // Дает возможность формирования задержки
#include <avr/interrupt.h> // Будем использовать прерывания
#include <avr/pgmspace.h>  //Будем хранить константы в памяти программ
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






//Декларируем наши функции
//extern void init_shift_register(void);
void configuring_ports(void);		//конфигурация портов
void init_timer2(void); 			//Настройка таймера timer2, будет использоваться для приёма и передачи ИК-пакетов
void init_int0(void);				//Настраиваем внешние прерывания вывода INT0
void init_tm(void);				//Настраиваем внешние прерывания вывода INT1
void set_buffer_bit(uint8_t, bool);	//Задаем значение биту в буфере ИК-приемника
void set_bt_buffer_bit(uint8_t, bool);	//Задаем значение биту в буфере для ИК-пакета, принятого по блютус
void send_ir_package(void);			//Отправляем подготовленный пакет (выстрел)
void set_player_id(uint8_t);		//Задаем игроку идентификатор
void set_team_color(tteam_color);	//Задаем цвет нашей команды
void set_gun_damage(tgun_damage);	//Задаем мощьность нашего оружия (урон)
void init_var(void);				//Присваиваем дефалтовые значения переменным
bool get_buffer_bit(uint8_t);		//Считываем значение бита в буфере ИК-приемника
bool get_bt_buffer_bit(uint8_t);	//Считываем значение бита в буфере пакета, принятого по блютус

inline trx_packet get_packet_value(void); //Считываем данные из полученного пакета
inline trx_packet get_bt_packet_value(void); //Считываем данные из полученного пакета
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
char* long_int_to_str(uint16_t x, uint8_t digits);//Преобразуем число в строку
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
void display_hit_data(trx_packet hit_packet);//покажем кто в нас попал и какой урон нанёс
//void generate_batt_symbols(void);//создаем в памяти ЖКИ символы батарейки
void get_ir_power_settings(void);
void get_friendly_fire_settings(void);//включение/отключение "дружественного" огня
void get_all_setings(void);
uint8_t get_command_index(void);//проверим, что за команда пришла по UART
uint8_t char_to_int(char);//преобразуем символ в целое число
//void command_1_slot(void);
bool play_sound_from_eeprom(uint16_t address, uint16_t data_size); //воспроизводим звук непосредственно из eeprom
void play_sound_1(void);//воспроизводим звук 1
void play_sound_2(void);//воспроизводим звук 2
void play_sound_3(void);//воспроизводим звук 3
void play_sound_4(void);//воспроизводим звук 4
void play_sound_5(void);//воспроизводим звук 5
void play_sound_6(void);//воспроизводим звук 6
trx_event parsing_bt_data(void);//анализируем пакет, полученный по блютус, извекаем из него данные
void hit_processing(trx_packet hit_packet);//обрабатываем попадание
void ir_tx_cursor_home(void);//устанавливаем курсор в начало буфера
void send_message(uint8_t ID, uint8_t DATA); //отправляем сообщение
void pult_reset_to_defaul(void); //дефалтовые настройки для кнопок пульта
bool get_pult_default_settings(void);//спросим, надо ли сбросить настройки пульта на дефолтовые?
void menu_slot_1(void);
void menu_slot_2(void);
//void sound_buffer_update(void);//подкинем в звуковой буфер новую партию сейплов
//uint8_t read_seimpl_from_sound_buffer(void);//считываем очередной сеймпл из буфера
//void play_shot_sound(void);//воспроизводим звук выстрела

uint8_t get_pult_message_index (uint8_t message); //получаем индекс сообщения
uint8_t get_pult_command_index (uint8_t command); //получаем индекс команды


//Глобальные переменные

extern volatile uint16_t timer1; 
extern volatile uint16_t timer2; 

extern volatile bool start_bit_received;				//Этот флаг устанвливается, если принят старт-бит
extern volatile uint16_t high_level_counter; 			//Счетчик длительности сигнала высокого уровня на выводе ИК-приемника										//который нужно передать следующим
extern volatile uint16_t bit_in_rx_buff; 				//Количество бит, принятых ИК-приемником 
extern volatile uint16_t bit_in_bt_rx_buff; 			//Количество бит, принятых по блютус 
extern volatile trx_event rx_event; 					//события ИК-приемника
extern volatile trx_event bt_rx_event; 				//события BT-приемника
extern volatile uint16_t low_level_counter; 			//Счетчик длительности сигнала низкого уровня на выводе ИК-приемника										
extern volatile uint8_t rx_buffer[RX_BUFFER_SIZE]; 	//Буффер ИК-приемника
extern volatile uint8_t bt_rx_buffer[RX_BUFFER_SIZE]; 	//Буффер для ИК-пакета, принятого по блютус

extern volatile bool ir_transmitter_on;				//флаг, разрешаюший (true) или запрещающий передачу данных через ИК-диод 														//который нужно передать следующим
extern volatile uint16_t ir_pulse_counter; 					//Обратный счетчик "вспышек" ИК-диода
extern volatile int ir_space_counter; 					//Обратный счетчик длительности выключенного состояния ИК-диода (пауза между битами) 
//extern volatile trx_packet rx_packet;

//extern volatile union data_packet_union  data_packet; 	//В этой переменной будем формировать пакет данных для отправки через IR
extern volatile uint8_t cursor_position;				//Эта переменная будет хранить индех элемента массива данных data_packet.data[x]
//extern volatile union data_packet_union  data_packet; 	//В этой переменной будем формировать пакет данных для отправки через IR
extern uint8_t damage_value [] PROGMEM;
extern volatile trx_packet rx_packet;
extern volatile trx_packet bt_rx_packet;
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
extern volatile uint8_t display_bullets_update_now; //флаг, указывающий, что пора обновить индикацию оставшихся патронов
extern volatile TDISPLAY_BATT_MODE display_batt_mode; //режим отображения напряжения батареи (иконка/цифры) 
extern volatile EEMEM uint8_t eeprom_curr_ir_pin; //определяет мощность ИК излучения
extern volatile uint8_t curr_ir_pin; //текущая ножка, которую будем "дергать" при выстреле, определяет мощность ИК излучения
extern volatile EEMEM ttm_serial_num eeprom_tm_serial_num;


extern volatile bool cr_received; //влаг, указывающий на то, что по UART получен символ "\r" - перевод корретки 
extern volatile bool bt_header_received; //влаг, указывающий на то, что по UART получен символ "h" - заголовок пакета 


//передающий буфер
extern volatile unsigned char usartTxBuf[SIZE_BUF];
extern volatile unsigned char txBufTail;
extern volatile unsigned char txBufHead;
extern volatile unsigned char txCount;

//приемный буфер
extern volatile unsigned char usartRxBuf[SIZE_BUF];
extern volatile unsigned char rxBufTail;
extern volatile unsigned char rxBufHead;
extern volatile unsigned char rxCount;
extern volatile uint16_t uart_timer; //таймер для отсчета таймаута

extern volatile bool fcurr_simple_prepared; //флаг, указывающий, что сеймпл звука подготовлен для воспроизведения
extern volatile uint8_t curr_simple; //текущий сеймл звука

extern volatile EEMEM uint16_t sound_1_adress;//адрес в eeprom звука выстрела
extern volatile EEMEM uint16_t sound_1_size;//длина в байтах звука выстрела
extern volatile EEMEM uint16_t sound_2_adress;//адрес в eeprom звука выстрела
extern volatile EEMEM uint16_t sound_2_size;//длина в байтах звука выстрела
extern volatile EEMEM uint16_t sound_3_adress;//адрес в eeprom звука выстрела
extern volatile EEMEM uint16_t sound_3_size;//длина в байтах звука выстрела
extern volatile EEMEM uint16_t sound_4_adress;//адрес в eeprom звука выстрела
extern volatile EEMEM uint16_t sound_4_size;//длина в байтах звука выстрела
extern volatile EEMEM uint16_t sound_5_adress;//адрес в eeprom звука выстрела
extern volatile EEMEM uint16_t sound_5_size;//длина в байтах звука выстрела
extern volatile EEMEM uint16_t sound_6_adress;//адрес в eeprom звука выстрела
extern volatile EEMEM uint16_t sound_6_size;//длина в байтах звука выстрела
extern volatile EEMEM bool friendly_fire_enable; //флаг, указывающий, фиксировать (true) или нет (false) "дружественный" огонь
//extern volatile TSOUND_BUFFER curr_sound_buffer; //текущий буфер звука, из которого считываются сеймплы
extern volatile uint16_t curr_adress_in_eeprom;//адрес в eeprom, начиная с которого запишем данные в звуковой буфер
//extern volatile uint8_t curr_pos_in_sound_buff;//текущая пощиция в звуковом буфере
extern volatile uint16_t simples_in_queue;//сколько сеймплов осталось воспроизвести

//extern volatile uint8_t update_suond_buffer_now;//флаг, указывающия, что нужно обновить данные в звуковом буфере 
extern volatile bool eeprom_is_open;//флаг, указывает, открыта ли eeprom в данный момент
extern volatile uint16_t cut_off_sound; //количество невоспроизведённых сеймлов, при котором пора отсекать звук
extern const unsigned char Decode2Rus[255-192+1] PROGMEM;
extern volatile bool receiver_on;//флаг, показывающия, что идет прием ИК пакета
extern volatile tir_tx_buffer_cursor ir_tx_buffer_cursor; //курсор передающего ИК буфера
extern volatile uint8_t tx_buffer[TX_BUFFER_SIZE]; 	//Буффер ИК-передатчика
extern volatile bool pult_hot_key_mode; //если true, то находимся в режиме "горячих клавиш", если false - в расширенном режиме
extern volatile bool pult_help_mode; //если true, то находимся в режиме "помощь"



/***********************************************************
* Глобальные переменные для пульта (все храняться в eeprom пямяти контроллера)
***********************************************************/
extern volatile EEMEM uint16_t pult_key_up_command; //команда для клавиши джойстика "Вверх" ("UP"), кнопка "shift" не нажата
extern volatile EEMEM uint16_t pult_shift_and_key_up_command; //команда для клавиши джойстика "Вверх" ("UP"), кнопка "shift" нажата


extern volatile EEMEM uint16_t pult_key_right_command; //команда для клавиши джойстика "Вправо" ("RIGHT"), кнопка "shift" не нажата
extern volatile EEMEM uint16_t pult_shift_and_key_right_command; //команда для клавиши джойстика "Вверх" ("RIGHT"), кнопка "shift" нажата


extern volatile EEMEM uint16_t pult_key_down_command; //команда для клавиши джойстика "Вниз" ("DOWN"), кнопка "shift" не нажата
extern volatile EEMEM uint16_t pult_shift_and_key_down_command; //команда для клавиши джойстика "Вниз" ("DOWN"), кнопка "shift" нажата


extern volatile EEMEM uint16_t pult_key_left_command; //команда для клавиши джойстика "Влево" ("LEFT"), кнопка "shift" не нажата
extern volatile EEMEM uint16_t pult_shift_and_key_left_command; //команда для клавиши джойстика "Влево" ("LEFT"), кнопка "shift" нажата


extern volatile EEMEM uint16_t pult_key_central_command; //команда для клавиши джойстика "Центральная" ("CENTRAL"), кнопка "shift" не нажата
extern volatile EEMEM uint16_t pult_shift_and_key_central_command; //команда для клавиши джойстика "Центральная" ("CENTRAL"), кнопка "shift" нажата

extern volatile EEMEM uint16_t pult_reload_command; //команда для клавиши "Перезаряд" ("RELOAD"), кнопка "shift" не нажата
//extern volatile EEMEM uint16_t pult_shift_and_key_reload_command; //команда для клавиши "Перезаряд" ("RELOD"), кнопка "shift" нажата


//extern const uint8_t messages [] PROGMEM;

//extern const uint8_t commands [] PROGMEM;

