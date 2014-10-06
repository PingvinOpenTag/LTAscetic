#include "ltag_ascetic.h"
#include "lcd_driver.h"
#include "tm_driver.h"
#include "i2c_eeprom.h"
#include "commands.h"




int main (void) {
configuring_ports();
get_ir_protocol();
init_timer2();
init_int0();
init_tm();	//инициализируем считыватель тачмемори
init_timer1();
init_timer0();

init_var();                		//Задаём значения переменным

init_lcd(); 
lcd_clrscr();
init_joystick();
init_adc();

USART_Init();
EnableRxInt();
EnableTxInt();


eeInit();


_delay_ms(15); 
lcd_puts("LTAscet");
lcd_gotoxy(0, 1);
lcd_puts("Russia");
lcd_generate_additional_symbols();
//lcd_puts(int_to_str(4,3));
//lcd_clrscr();
//lcd_generate_batt_symbols();


/*
lcd_putc(0);

lcd_putc(1);
lcd_putc(2);
lcd_putc(3);
lcd_putc(4);
lcd_putc(5);
*/

sei(); 

life_leds_status[0] = ON;
timer2 = 0;
while (timer2 < 1000);

life_leds_status[1] = ON;
timer2 = 0;
while (timer2 < 1000);

//LIFE_LED3_ON;
life_leds_status[2] = ON;
timer2 = 0;
while (timer2 < 1000);

//LIFE_LED4_ON;
life_leds_status[3] = ON;
timer2 = 0;
while (timer2 < 1000);

//FIRE_LED_ON;
fire_led_status = ON;
timer2 = 0;
while (timer2 < 1000);

BULLETS_OUT_LED_ON;
timer2 = 0;
while (timer2 < 1000);


BT_STATE_LED_ON;
timer2 = 0;
while (timer2 < 1000);

WOUND_LED_ON;
timer2 = 0;
while (timer2 < 1000);

FIRE_LED_ON;
timer2 = 0;
while (timer2 < 1000);



lcd_bl_on();
timer2 = 0;
while (timer2 < 1000);


lcd_bl_off();
timer2 = 0;
while (timer2 < 1000);

//LIFE_LED1_OFF;
life_leds_status[0] = OFF;
timer2 = 0;
while (timer2 < 1000);

//LIFE_LED2_OFF;
life_leds_status[1] = OFF;
timer2 = 0;
while (timer2 < 1000);

//LIFE_LED3_OFF;
life_leds_status[2] = OFF;
timer2 = 0;
while (timer2 < 1000);

//LIFE_LED4_OFF;
life_leds_status[3] = OFF;
timer2 = 0;
while (timer2 < 1000);

FIRE_LED_OFF;
timer2 = 0;
while (timer2 < 1000);

BT_STATE_LED_OFF;
timer2 = 0;
while (timer2 < 1000);

BULLETS_OUT_LED_OFF;
timer2 = 0;
while (timer2 < 1000);

WOUND_LED_OFF;
timer2 = 0;
while (timer2 < 1000);

timer2 = 0;
while (timer2 < 1000);
//life_led1_status = FQR_1HZ;
//life_led2_status = FQR_2HZ;
//life_led3_status = FQR_4HZ;
//life_led4_status = ON;

display_life(life);
beep(1000, 3, 128);

USART_SendStr("Hello!\n\r");
USART_SendStr("Привет!\n\r");
USART_SendStrP(command_0);
USART_SendStrP(command_1);

//play_sound_from_eeprom(0,27264);
/*
play_shot_sound();
while(1){
if(update_suond_buffer_now)
	{
		sound_buffer_update();
	}
};
*/
/*
volatile bool ok;

volatile uint8_t data_tmp;
data_tmp = 0;
ok = open_eeprom(0);
ok = read_eeprom_byte(&data_tmp);
ok = read_eeprom_byte(&data_tmp);
ok = read_eeprom_byte(&data_tmp);
ok = close_eeprom(&data_tmp);
simples_in_queue = eeprom_read_word(&sound_1_size);
timer1 = 0;
while (timer1 < 10000);
while(eeprom_is_open);
simples_in_queue = eeprom_read_word(&sound_1_size);
*/


invite();
cut_off_sound = (eeprom_read_word(&sound_1_size)/100)*(100-CUT_OFF_SOUNT);
joystick_event=no_pressing;
display_status();
USART_FlushRxBuf();
bt_header_received=false;
timer2 = 0;
while (timer2 < 1000);


while(1){

////volatile uint16_t adc_data;
////volatile uint16_t batt_voltage;
////adc_data=ReadADC(ADC_CHANNEL);
//adc_data=(adc_data/4)*7.5;
//USART_SendStr("Hello!\n\r");
if (simples_in_queue==0) display_voltage_update();
//if(update_suond_buffer_now)
//	{
//		sound_buffer_update();
//	}
//batt_voltage = (adc_data/4)*7.5;
//batt_voltage = batt_voltage/10;

if (chit_detected)
	{
		lcd_clrscr();
		lcd_puts(" Ошибка ");
		lcd_gotoxy(0, 1);
		lcd_puts("датчика!");
		while (chit_detected)
		{
			beep(1000, 3, 128);
			beep(500, 3, 128); //Воспроизводим звук (частота, длительность, громкость)
		};
		
		USART_FlushRxBuf();
		keyboard_event=no_key_pressing;
		reload_key_event=no_key_pressing;
		joystick_event = no_pressing;
		display_status();
	}

switch(keyboard_event)
	{
	 	case no_key_pressing: break;
		case key_pressing:
		{
			if (bullets > 0)
			{
				bullets--;//уменьшаем на 1 количество патронов		
				//last_simple = 0;//воспроизводим звук выстрела
				
				if (simples_in_queue>1) //если звук выстрла воспроизводится
				{
					simples_in_queue=1;//закроем eeprom
					while (eeprom_is_open);//дождемся, пока eerom закроется
				}
				 
				simples_in_queue = eeprom_read_word(&sound_1_size);
				
				
				send_ir_package();		//Производим "выстрел"
			//	display_bullets_update();
				
	 		}
			else
			{
				if (simples_in_queue==0) //если звук выстрла не воспроизводится
				{
					play_sound_5();
				}
			}
			keyboard_event=no_key_pressing; 
		} 
        break;
  

		default:keyboard_event=no_key_pressing;	
	}


switch(reload_key_event)
	{
	 	case no_key_pressing: break;
		case key_pressing:
		{
			if ((clips > 0)&&(simples_in_queue==0))//если обоймы не кончилсь и не воспроизводиться звук выстрела
			{
				playclipinsound();
				clips--;//уменьшаем на 1 количество патронов
				bullets = eeprom_read_byte(&eeprom_bullets_in_clip);
				display_clips_update();
				display_bullets_update();
				for (int i=0; i< eeprom_read_byte(&eeprom_reload_duration); i++){
				timer2 = 0;
				while (timer2 < 8000);
				}
				//while (timer1 < 10000);
				playclipoutsound();
				BULLETS_OUT_LED_OFF;
		//		last_simple = 0;//воспроизводим звук выстрела
		//		send_ir_package();		//Производим "выстрел"
				
	 		}
			reload_key_event=no_key_pressing; 
		} 
        break;
  

		default:reload_key_event=no_key_pressing;	
	}


	{
		switch(rx_event)//выясним, какое именно событие произошло
					{
						case RX_COMPLETE: 	//получен пакет
						{
						//	cli();
						/*********************************************************
							WOUND_LED_ON; //включаем вспомогательный светодиод
							timer1=0;
							while(timer1 < 35000);
							WOUND_LED_OFF;	//выключаем вспомогательный светодиод
						************************************************************/
							
							rx_event = NOT_EVENT;	
							if(!get_buffer_bit(0)) //если этот бит равен 0, то это пакет с данными (выстрел)
							{
						
								
	//							uint8_t player_id;
								rx_packet = get_packet_value();
//								volatile int gg;
//								gg++;


								hit_processing(rx_packet);
								rx_event = NOT_EVENT;

							}
							
						
						
						//	sei();
							break;
						}
						
						case RX_MESSAGE_COMPLITE://принято сообщение
						{
							rx_event = NOT_EVENT;							
							if(get_buffer_bit(0)) //если этот бит равен 1, то это пакет с командой
                        	{
                           		ir_message = get_ir_message_from_buffer();//выдергиваем значения команды из буфера ИК приемника
                        
                           	 if (ir_message.control_byte ==Valid_value )//сообщение принято корректно (контрольный байт принят без ошибок)
							 {
								switch(ir_message.ID)//если имя команды
                           		{
                              		case Add_Health: //добавить "жизни"
                              		{
										//код для добавления жизни
                                 		break;
                              		}
                              		case Add_Rounds://добавить "патронов"
                              		{
                                 
								 		//код для добавления патронов
                                 		break;
                              		}

                              		case Command://какая то дополнительноя команда
                              		{
                                    	
										switch(ir_message.param)//выясним, какая это командв
										{
											case 0x05://начать новую игру немедленно
											{
													
													
													if (simples_in_queue>1) //если звук выстрла воспроизводится
													{
														simples_in_queue=1;//закроем eeprom
														while (eeprom_is_open);//дождемся, пока eerom закроется
													}
													init_var(); //инициализируем переменные
													joystick_event=no_pressing; //очищаем события джойстика
													keyboard_event=no_key_pressing;//очищаем события триггера
													reload_key_event=no_key_pressing;//очищаем события перезарядки
													rx_event = NOT_EVENT;   //очищаем события ИК приемника
													display_status();//обновляем информацию на дисплее
													
													WOUND_LED_ON;
													playstartsound();
													//код обработки дополнительной команды
                                 					WOUND_LED_OFF;
												
												break;
											}
											case 0x00://"выключить" игрока 
											{
												
												
												break;
												
											}
											default: break;
										
										}
										
									
										break;
                              		}
                           		}
                        	 }

                        	}
							else//контрольный байт сообщения не корректный - ошибка приема
							{
							}
							
							
							
							
							rx_event = NOT_EVENT;
							break;

						}
						
						
						case RX_ERROR:		//ошибка приема
						{
                            if((!ir_error_ignore)&&(!eeprom_is_open))//если не надо игнорировать ошибку и звук не воспроизводиться уже
                            {
						//	cli();
							BULLETS_OUT_LED_ON;
							/*
							timer2=0;
							while(timer2 < 4000);
							*/
							play_sound_8();
							BULLETS_OUT_LED_OFF;
                            }
                            rx_event = NOT_EVENT;
						//	sei();
							break;
						}
						
						case NOT_EVENT:		//ошибка приема
						{
						//	cli();
						//	rx_event = NOT_EVENT;	
						//	sei();
							break;
						}



					}



	}

if (rxCount>0)//если буфер BT приемника не пустой
	{
		test_bt_data();
	}
	

//		unsigned char tmp_char;
//		tmp_char = USART_GetChar();
//		USART_FlushRxBuf();
//		cr_received=false;



//	timer1=0;				//Сделаем паузу
//	while(timer1 < 65000);	//Подождем, пока не произойдет 65000 прерываний таймера (чуть меньше секунды)

	switch(joystick_event)
	{
		case key_up_pressing: 
			{
		//	lcd_clrscr();
	//		lcd_gotoxy(0, 0);
	
	//		if ((result+10)<=max_value) result=result+10;
	//		lcd_gotoxy(0, 1);
	//		lcd_puts(int_to_str(result,3));
		/*
			uint16_t adc_data;
			uint16_t batt_voltage;
			adc_data=ReadADC(ADC_CHANNEL);
			adc_data=(adc_data/4)*7.5;
			display_voltage_update(adc_data);
		*/
			switch(display_batt_mode)
			{
				case icon: display_batt_mode=digit;
				break;
				case digit: display_batt_mode=icon;
			
			}
			
			
			
			joystick_event = no_pressing;
			}
	
		break;
		case key_right_pressing: 
			{
			//lcd_clrscr();
			//lcd_home();
//			if ((result)<max_value) result++;
//			lcd_gotoxy(0, 1);
//			lcd_puts(int_to_str(pgm_read_byte(arr_adress+result),3));
			//lcd_puts("Нажата кнопка \n");
			//lcd_puts("Вправо");
			joystick_event = no_pressing;
			}
		break;
		case key_down_pressing: 
			{
	//		lcd_clrscr();
	//		lcd_gotoxy(0, 0);
	//		if((result-9)>0) result=result-10;
	//		lcd_gotoxy(0, 1);
	//		lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
			}
		break;
		case key_left_pressing: 
			{
			//lcd_clrscr();
			//lcd_gotoxy(0, 0);
//			if ((result)>0) result--;
			//lcd_puts("Нажата кнопка \n");
			//lcd_puts("Влево");
//			lcd_gotoxy(0, 1);
//			lcd_puts(int_to_str(pgm_read_byte(arr_adress+result),3));
			joystick_event = no_pressing;
			}
		break;
		case key_central_pressing: 
			{
			joystick_event = no_pressing;
			}
		break;
		default: joystick_event = no_pressing;
	
	
		}
	
	if (display_bullets_update_now>0) 
		{
			display_bullets_update();
			display_bullets_update_now--;
		}

	};
return 0;

}




void configuring_ports(){
//IR_LED_DDR |= IR_LED_PIN; //ножку, на которой подключен ИК диот переводим в режим "выход"
FIRE_LED_DDR |= FIRE_LED_PIN; //ножку, на которой подключен ИК диот переводим в режим "выход"
BT_STATE_LED_DDR |= BT_STATE_LED_PIN;//ножку, к которой подключен сетодиод статуса блютус соединения - как выход


IR_LED_HIGH_POWER_DDR|=IR_LED_HIGH_POWER_PIN;//ножку, на которой подключен ИК диот переводим в режим "выход"
IR_LED_HIGH_POWER_OFF;
IR_LED_LOW_POWER_DDR|=IR_LED_LOW_POWER_PIN;//ножку, на которой подключен ИК диот переводим в режим "выход"
IR_LED_LOW_POWER_OFF;
/*
LIFE_LEDS_DDR |= LIFE_LED1_PIN  //ножки, на которых
				| LIFE_LED2_PIN //подключены светодиоды,
				| LIFE_LED3_PIN //отображающие уровень "жизни"
				| LIFE_LED4_PIN;//тоже настраиваем как выходы

*/


LIFE_LED1_DDR |= LIFE_LED1_PIN;
LIFE_LED2_DDR |= LIFE_LED2_PIN;
LIFE_LED3_DDR |= LIFE_LED3_PIN;
LIFE_LED4_DDR |= LIFE_LED4_PIN;


SOUND_DDR |= SOUND_PIN; //настраиваем выход ШИМ (АЦП)
SOUND_CONTROL_DDR|= SOUND_CONTROL_PIN;//настраиваем вывод, контролирующий усилитель, как "выход" 
SOUND_ON;//включаем усилитель



BULLETS_OUT_LED_DDR|=BULLETS_OUT_LED_PIN;
//DDRA |= (1 << 4)|(1<<5)|(1<<6)|(1<<7); // Устанавливаем порт PORTA.1 как выход                                                                                                                                                                    DDRB = 1<<DDB3;						//PB3 (OC0) set as output
//DDRD |= (1 << 7);//светодиод на повязке
DDRB |= (1 << 2);// отключение усилителя
PORTB &=~(1 << 2);
//PORTD &= ~(1 << 7);//выключаем светодиод на повязке
WOUND_LED_DDR |= WOUND_LED_PIN;
TSOP_DDR &=~TSOP_PIN; //вывод, к которому подключен ИК-датчик настраиваем как "вход"
RELOAD_KEY_DDR &=~RELOAD_KEY_PIN; //вывод, на котором сидит кнопка "перезарядка" как "вход"
RELOAD_KEY_PORT |= RELOAD_KEY_PIN;//включаем подтягивающий резистор

FIRE_KEY_DDR&=~FIRE_KEY_PIN;
FIRE_KEY_PORT|=FIRE_KEY_PIN;

FIRE_MODE_KEY_DDR&=~FIRE_MODE_KEY_PIN;
FIRE_MODE_KEY_PORT|=FIRE_MODE_KEY_PIN;


BT_STATE_PORT&=~BT_STATE_PIN;

ADC_DDR&=~ADC_PIN;
ADC_PORT&=~ADC_PIN;
}


/**************************************************************************************
* Функция выполняет настройку таймера timer2
* Режим работы таймер - СТС (сброс при совпадении)
* тактирование - без делителя, с частотой кварца
* регист сравнения будет иметь такое значение, чтобы прерывания
* генерировались с удвоенной частотой несущей ИК-приемника 
***************************************************************************************/


void init_timer2(void){
OCR2 = (F_CPU/1000)/(miles_protocol.carrier_frequency/1000)/2-1;       //Прерывания будут генерироваться с частотой, вдвое большей, чем частота несущей 
TCCR2 = _BV(CS20)|_BV(WGM21); // Режим работы таймер - СТС (сброс при совпадении)
                              // Тактирование с частотой резонатора (7 372 800 Гц)
//TIMSK |= _BV(OCIE2);          // Разрешаем прерывания по захвату/сравнению
TIMSK &=~_BV(OCIE2);  
}



void reinit_timer2(void){
OCR2 = (F_CPU/1000)/(miles_protocol.carrier_frequency/1000)/2-1;   

}
/**************************************************************************************
* Функция выполняет настройку внешних прерываний вывода INT0
***************************************************************************************/
void init_int0(void){
DDRD &=~(1<<2); 				//Настраиваем вывод INT0 как вход
MCUCR |=_BV(ISC01);				//Прерывания будут генерироваться 
MCUCR &=~_BV(ISC00);			//по спаду импульса
GICR |=_BV(INT0); 				//Разрешаем внешние прерывания на INT0

}



void set_buffer_bit(uint8_t index, bool value){	//Задаем значение биту в буфере ИК-приемника
uint8_t byte_index;
uint8_t bit_index;
byte_index = index/8; //Определяем, в каком байте нахадится нужный бит
bit_index = index - (byte_index*8);//Определяем номер бита в байте
if(value) 
		{
			rx_buffer[byte_index] |= (1<<(7-bit_index));
		}
else	{
			rx_buffer[byte_index] &= ~(1<<(7-bit_index));
		}
}


void set_bt_buffer_bit(uint8_t index, bool value){	//Задаем значение биту в буфере ИК-приемника
uint8_t byte_index;
uint8_t bit_index;
byte_index = index/8; //Определяем, в каком байте нахадится нужный бит
bit_index = index - (byte_index*8);//Определяем номер бита в байте
if(value) 
		{
			bt_rx_buffer[byte_index] |= (1<<(7-bit_index));
		}
else	{
			bt_rx_buffer[byte_index] &= ~(1<<(7-bit_index));
		}
}



/*

inline trx_packet get_packet_value(){ //Считываем данные из полученного пакета
trx_packet result;
uint8_t byte_tmp;

result.player_id = rx_buffer[0];
byte_tmp = rx_buffer[1];
byte_tmp = byte_tmp << 2; //избавляемся от бит цвета команды
byte_tmp = byte_tmp >> 4;
result.damage = pgm_read_byte(&(damage_value[byte_tmp]));
result.team_id = rx_buffer[1]>>6;

return result;
}
*/




/**************************************************************************************
* Функця производит "выстрел"
* устанавлвает курсор на позицию начала блока данных data_packet.data[0]
* и разрешает передачу данных
* функция возвращает управление  только после отправки всех данных 
***************************************************************************************/


void send_ir_package(void){ //Отправляем пакет ("стреляем")
//ir_pulse_counter=IR_START;
//cursor_position = 0; 		//Курсор - на начало блока данных
ir_error_ignore = DEFAULT_IR_ERROR_IGNORE;
ir_tx_cursor_home();//Курсор - на начало буфера
ir_tx_buffer_cursor.bits_for_tx=14;//"выстрел" состоит из 14 бит
 
ir_transmitter_on = true;	//Разрешаем передачу
TIMSK |= _BV(OCIE2);          // Разрешаем прерывания по захвату/сравнению
FIRE_LED_ON;


//while (ir_transmitter_on);	//Ждем, пока пакет отправиться
}

/**************************************************************************************
* Установка идентификатора игрока
* в качестве аргумента функции указывается идентификационный номер игрока (от 1 до 127)
* в результате выполнения функции в глобальной переменной data_packet.player_id
* будут соответствующим образом инициированы  data_packet.player_id.(bit_0 ... bit_7) 
***************************************************************************************/
void set_player_id(uint8_t ID){
tx_buffer[0]= ID;
tx_buffer[0] &=~(1<<7);//седьмой бит в выстреле всегда равен "0"

}



/**************************************************************************************
* Установка идентификатора (цвета) команды
* в качестве аргумента функции указывается идентификационный номер (цвет) команды (от 0 до 3)
* в результате выполнения функции в глобальной переменной data_packet.team_id
* будут соответствующим образом инициированы  data_packet.team_id.(bit_0 и bit_1) 
***************************************************************************************/



void set_team_color(tteam_color  color){
tx_buffer[1] &=~((1<<7)|(1<<6));//обнуляем два старших бита 
tx_buffer[1] |=(color <<6);//устанавливаем 6 и 7 биты в соответствии с цветом команды
}




/**************************************************************************************
* Установка установка мощьности нашего оружия (наносимый урон)
* в качестве аргумента функции указывается наносимый урон
* в результате выполнения функции в глобальной переменной data_packet.damage
* будут соответствующим образом инициированы  data_packet.damage.(bit_0 и bit_3) 
***************************************************************************************/


void set_gun_damage(tgun_damage damage){
tx_buffer[1] &=~((1<<5)|(1<<4)|(1<<3)|(1<<2));//обнуляем биты урона 
tx_buffer[1] |=(damage << 2);
}




void init_var(void){            //Задаём значения переменным
check_eeprom();
last_simple = 0xFFFF; //иначе будет звук выстрела при сбросе
snd_adress.curr_adress = (uint8_t*)0xFFFF; //иначе бубдет попытка воспроизвести звук
ir_transmitter_on=false; 	//Запретим пока передачу данных (поскольку данные ещё не сформированны)
set_player_id(eeprom_read_byte(&eeprom_player_id));	//Устанавливаем идентификатор игрока
set_team_color(team_id());	//Устанавливаем идентификатор (цвет) команды
set_gun_damage(gun_damage());		//Устанавливаем мощьность оружия (урон)

clips=eeprom_read_byte(&eeprom_clips);//Устанавливаем количество обойм
bullets=0; //eeprom_read_byte(&eeprom_bullets_in_clip);//Устанавливаем количество патронов
//data_packet.packet.header = IR_START;		//Устанавливаем  заголовку (старт биту) необходимую длительность
//data_packet.packet.end_of_data = 0;		//0 - это указание передатчику, что данных для передачи больше нет
cursor_position = 0; //Курсор - на начало блока данных
start_bit_received = false;//Старт бит ещё не принят
bit_in_rx_buff = 0;//Буфер приемника пуст
bit_in_bt_rx_buff = 0;//Буфер приемника блютус пуст
rx_event = NOT_EVENT;//Сбрасываем события приемника
bt_rx_event = NOT_EVENT;//Сбрасываем события BT приемника
//reset_clock(); //обнуляем часы
life = 8;//здоровье -100% выводим на диоды
life_in_percent = 100;//эту выводим на дисплей
key_pressing_duration.key_1    =0;//обнуляем счетчики 
						  //длительности
						  //непрерывного нажатия кнопок
key_pressing_duration.key_1_inc=1;//разрешаем отсчет длительности
key_pressing_duration.key_2    =0;//обнуляем счетчики 
						  //длительности
						  //непрерывного нажатия кнопок
key_pressing_duration.key_2_inc=1;//разрешаем отсчет длительности
chit_detected_counter=0;
chit_detected = false;
display_bullets_update_now = 0;
display_batt_mode = icon;
curr_ir_pin = eeprom_read_byte(&eeprom_curr_ir_pin);//Устанавливаем мощность ИК излучения
cr_received = false; //символ "\r" ещё не получен
bt_header_received = false; //символ "h" ещё не получен
simples_in_queue =0;
//update_suond_buffer_now = 0;
eeprom_is_open = false;
receiver_on = false;
ir_error_ignore = 0; //не игнорируем ошибочные пакеты

}




bool get_buffer_bit(uint8_t index){		//Считываем значение бита в буфере ИК-приемника
uint8_t byte_index;
uint8_t bit_index;
byte_index = index/8; //Определяем, в каком байте нахадится нужный бит
bit_index = index - (byte_index*8);//Определяем номер бита в байте
if(rx_buffer[byte_index]&(1<<(7-bit_index))) return true;
else return false;


}

bool get_bt_buffer_bit(uint8_t index){		//Считываем значение бита в буфере ИК-приемника
uint8_t byte_index;
uint8_t bit_index;
byte_index = index/8; //Определяем, в каком байте нахадится нужный бит
bit_index = index - (byte_index*8);//Определяем номер бита в байте
if(bt_rx_buffer[byte_index]&(1<<(7-bit_index))) return true;
else return false;


}
inline trx_packet get_packet_value(){ //Считываем данные из полученного пакета
trx_packet result;
uint8_t byte_tmp;

result.player_id = rx_buffer[0];
byte_tmp = rx_buffer[1];
byte_tmp = byte_tmp << 2; //избавляемся от бит цвета команды
byte_tmp = byte_tmp >> 4;
result.damage = pgm_read_byte(&(damage_value[byte_tmp]));
result.team_id = rx_buffer[1]>>6;

return result;
}



inline trx_packet get_bt_packet_value(){ //Считываем данные из полученного пакета
trx_packet result;
uint8_t byte_tmp;

result.player_id = bt_rx_buffer[0];
byte_tmp = bt_rx_buffer[1];
byte_tmp = byte_tmp << 2; //избавляемся от бит цвета команды
byte_tmp = byte_tmp >> 4;
result.damage = pgm_read_byte(&(damage_value[byte_tmp]));
result.team_id = bt_rx_buffer[1]>>6;

return result;
}




tteam_color team_id()//Опреднляем цвет нашей команды 
{


tteam_color result;
/*
	switch (SW_TEAM_IN&SW_TEAM_MASK) //проверим состояние переключателя "DAMAGE"
	{
		case SW_TEAM_KEY1_PIN: //1-й ключ в состоянии OFF (разомкнут), а второй замкнут (ON)
		{
			result = Blue;
			//return result;
			break;
		}
		case SW_TEAM_KEY2_PIN://2-й ключ в состоянии OFF (разомкнут), а первый замкнут (ON)
		{
			result = Yellow;
			//return result;
			break;
		}
		
		case SW_TEAM_KEY1_PIN|SW_TEAM_KEY2_PIN: //оба ключа в состоянии OFF
		{
			result = Red;
			//return result;
			break;
		}

		case 0: //оба ключа в состоянии ON
		{
			result = Green;
			//return result;
			break;
		}
		default: result = Red;

	}
*/
result = eeprom_read_byte(&eeprom_team_id);
return result;
}



void write_team_id_to_eeprom(tteam_color color){
eeprom_write_byte(&eeprom_team_id, color);

}


/**************************************************************************************
* Функция выполняет настройку таймера timer0
* Режим работы таймер - СТС (сброс при совпадении)
* тактирование - с предделителем на 1024
* регист сравнения будет иметь такое значение, чтобы прерывания
* генерировались 100 прерываний в секунду 
***************************************************************************************/

void init_timer0(void){

//OCR0 = F_CPU/1024/100-1;		// Прерывания должны генерироваться с частотой 100 Гц

OCR0 = 128; //Скважность = 0,5
TCCR0 = _BV(WGM01)| _BV(WGM00);	// Режим работы таймер - fast PWM (быстрый ШИМ)
TCCR0 |=  _BV(CS00);            // Тактирование с частотой резонвтора 8 МГц
TCCR0 |=  _BV(COM01);    		//Неинвертированный режим ШИМ






//TIMSK |= _BV(OCIE0);          // Разрешаем прерывания по захвату/сравнению
		                      // Разрешаем прерывания глобально

}


tgun_damage gun_damage()//Определяем текущий урон, наносимый нашим тагом
{
/*tgun_damage result;
	switch (SW_DAMAGE_IN&SW_DAMAGE_MASK) //проверим состояние переключателя "DAMAGE"
	{
		case SW_DAMAGE_KEY1_PIN: //1-й ключ в состоянии OFF (разомкнут), а второй замкнут (ON)
		{
			result = Damage_50;
			//return result;
			break;
		}
		case SW_DAMAGE_KEY2_PIN://2-й ключ в состоянии OFF (разомкнут), а первый замкнут (ON)
		{
			result = Damage_25;
			//return result;
			break;
		}
		
		case SW_DAMAGE_KEY1_PIN|SW_DAMAGE_KEY2_PIN: //оба ключа в состоянии OFF
		{
			result = Damage_10;
			//return result;
			break;
		}

		case 0: //оба ключа в состоянии ON
		{
			result = Damage_100;
			//return result;
			break;
		}
		default: result = Damage_25;

	}

return result;
*/
return eeprom_read_byte(&eeprom_damage);
}



void init_timer1(void){ //Настраиваем timer1 на частоту выборки звука -8 
TCCR1A &=~_BV(WGM10); //режим работы таймера - CTC (сброс при совпадении)
TCCR1A &=~_BV(WGM11);
TCCR1B |=_BV(WGM12); 
TCCR1B &=~_BV(WGM13); 
TCCR1A &=~_BV(COM1A0);//отключаем таймер от вывода OC1A
TCCR1A &=~_BV(COM1A1);
TCCR1B &=~_BV(COM1B0);//отключаем таймер от вывода OC1B
TCCR1B &=~_BV(COM1B1);

TCCR1B &=~_BV(CS10); //делитель = 8
TCCR1B |=_BV(CS11);
TCCR1B &=~_BV(CS12); 
//OCR1AL=60;
//OCR1AL=124;
OCR1AL=(F_CPU/8000)/8-1; // настраиваем на частоту выборки звука - 8 кГц
//OCR1AL=(F_CPU/16000)/8-1; // настраиваем на частоту выборки звука - 8 кГц
//OCR1AL=248; 
//OCR1AH=0x27;
//OCR1AL=0x0F;


TIMSK |= _BV(OCIE1A);  
//TIMSK |= _BV(OCIE1B);  

}



void display_life(uint8_t life_value) //отображаем уровень жизни на светодиодной линейке
{

uint8_t integer_part;
	for (int i=0; i<4; i++)
	{life_leds_status[i] = OFF;}

integer_part = life_value/2;
for (int i=0; i<integer_part; i++)
{
	life_leds_status[i] = ON;
}


if ((life_value-integer_part*2)>0) 
 life_leds_status[integer_part] = FQR_2HZ;
 
}

/*
uint8_t bullets_limit(void)//Определяем лимит патронов
{

uint16_t result;
	switch (SW_BULLETS_LIMIT_IN&SW_BULLETS_LIMIT_MASK) //проверим состояние переключателя "BULLETS_LIMIT"
	{
		case SW_BULLETS_LIMIT_KEY1_PIN: //1-й ключ в состоянии OFF (разомкнут), а второй замкнут (ON)
		{
			result = 64;
			//return result;
			break;
		}
		case SW_BULLETS_LIMIT_KEY2_PIN://2-й ключ в состоянии OFF (разомкнут), а первый замкнут (ON)
		{
			result = 32;
			//return result;
			break;
		}
		
		case SW_BULLETS_LIMIT_KEY1_PIN|SW_BULLETS_LIMIT_KEY2_PIN: //оба ключа в состоянии OFF
		{
			result = 16;
			//return result;
			break;
		}

		case 0: //оба ключа в состоянии ON
		{
			result = 128;
			//return result;
			break;
		}
		default: result = 16;

	}

return result;


}
*/





TFIRE_MODE_STATUS fire_mode()//Определяем текущий режим огня (одиночный/очередями)
{
TFIRE_MODE_STATUS result;
if (FIRE_MODE_KEY_IN&FIRE_MODE_KEY_PIN) result = single;
else  result = queues;
return result;

}




void beep(uint16_t fqr, uint16_t count, uint8_t value) //Воспроизводим звук (частота, длительность, громкость)
{
uint16_t last_simple_tmp;
uint8_t devider; //делитель, будет иметь значение, указывающее, на сколько нужно поделить частоту 8 кГц чтобы получить нужную (fqr)
uint16_t beep_counter; //длительность звука в циклах (один цикл равен периоду колибаний)

if (fqr > 4000) return; //если запрашиваемая частота выше 4 кГц то мы её воспроизвести не сможем, выходим

last_simple_tmp = last_simple; //сохраним значение последнего семпла звука выстрела (вдруг звук воспроизводится в это время)
last_simple = 0xFFFF; //иначе будет звук выстрела


devider = 4000/fqr; 
if (count > 160) count = 160; //ограничим время воспроизведения 16 секундами
beep_counter = (fqr/10)*count; //количество циклов (периодов колебаний)

for (uint16_t i=0; i < beep_counter; i++)
	{
		OCR0 = value;
		timer2=0;
		while (timer2 < devider);
		OCR0 = 0;
		timer2=0;
		while (timer2 < devider);

	}


last_simple = last_simple_tmp;
}

inline void damage_beep(void) // Воспроизводим звук при ранении
{
WOUND_LED_ON; //включаем вспомогательный светодиод

beep(1000, 3, 128);
beep(500, 3, 128);
WOUND_LED_OFF;

}


void playhitsound(void) //Воспроизводим звук при ранении

{
if (simples_in_queue>1) //если звук выстрла воспроизводится
				{
					simples_in_queue=1;//закроем eeprom
					while (eeprom_is_open);//дождемся, пока eerom закроется
				}
play_sound_2();
/*
snd_adress.start_adress = &pSnd_hit[0];

snd_adress.end_adress = &pSnd_hit[sizeof(pSnd_hit)-1];

snd_adress.curr_adress = &pSnd_hit[0];

play_hit_snd = true; //разрешаем воспроизведенме звука

while (play_hit_snd);// ждем окончания воспроизведения звука

*/}




void playgameoversound(void)//Воспроизводим звук когда жизни ноль
{
if (simples_in_queue>1) //если звук выстрла воспроизводится
				{
					simples_in_queue=1;//закроем eeprom
					while (eeprom_is_open);//дождемся, пока eerom закроется
				}
play_sound_7();
}



void playstartsound(void)//Воспроизводим звук "старт игры"
{

if (simples_in_queue>1) //если звук выстрла воспроизводится
				{
					simples_in_queue=1;//закроем eeprom
					while (eeprom_is_open);//дождемся, пока eerom закроется
				}
play_sound_6();


}



void playclipinsound(void) //Воспроизводим звук при передёргивании затвора
{
//play_hit_snd = true; //разрешаем воспроизведенме звука
play_sound_3();
//play_hit_snd = false; 
/*
snd_adress.start_adress = &clipinSnd[0];

snd_adress.end_adress = &clipinSnd[sizeof(clipinSnd)-1];

snd_adress.curr_adress = &clipinSnd[0];

play_hit_snd = true; //разрешаем воспроизведенме звука

while (play_hit_snd);// ждем окончания воспроизведения звука
*/
}


void playclipoutsound(void) //Воспроизводим звук при отпускании затвора
{
//play_hit_snd = true; //разрешаем воспроизведенме звука
play_sound_4();
//play_hit_snd = false; 
/*
snd_adress.start_adress = &clipoutSnd[0];

snd_adress.end_adress = &clipoutSnd[sizeof(clipinSnd)-1];

snd_adress.curr_adress = &clipoutSnd[0];

play_hit_snd = true; //разрешаем воспроизведенме звука

while (play_hit_snd);// ждем окончания воспроизведения звука
*/
}


void invite(){ //приглашение  в меню настроек

volatile uint8_t countdown = 5; //счётчик обратного отсчёта

lcd_clrscr();
lcd_home();
if ((eeprom_read_byte(&eeprom_tm_serial_num.device_code)==0)||(eeprom_read_byte(&eeprom_tm_serial_num.device_code)==0xFF))
	/* если ключ ещё не записан*/
	{//[if]
		joystick_event = no_pressing;
		lcd_puts("Запись");
		lcd_gotoxy(0, 1);
		lcd_puts("ключа ТМ");
		//timer1 = 0;
		while (/*(joystick_event!=key_central_pressing)*/(reload_key_event!=key_pressing)&&(eeprom_read_byte(&eeprom_tm_serial_num.device_code)==0)||(eeprom_read_byte(&eeprom_tm_serial_num.device_code)==0xFF)) //пока не нажата центральная кнопка или не записан ключ
		{//[while]
				
				while ((cr_received==false)&&(keyboard_event==no_key_pressing)&&(reload_key_event==no_key_pressing)&&(tm_event == no_tm_event)){};                     
				//while ((cr_received==false)&&/*(joystick_event==no_pressing)*/(reload_key_event==no_key_pressing)&&(tm_event == no_tm_event)){};
				if (cr_received)
				{	
					parsing_command();
					

				}
				if (keyboard_event==key_pressing)
				{
					if (BT_STATE_IN&BT_STATE_PIN)//если блютус соединение установлено
					{
						while(!(FIRE_KEY_IN&FIRE_KEY_PIN))//пока нажат курок
						{
							USART_PutChar('1');//посылаем повязке символ '1' - мигнуть диодом на повязке
							timer2 = 0;
							while (timer2 < 6000);

						}
					}
					
					
					keyboard_event=no_key_pressing;
				}	
				switch(tm_event)
				{//[switch]
					case no_tm_event: 
					{
					
					}
					break;
					
					case tm_crc_error: 
					{
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ошибка");
						lcd_gotoxy(0, 1);
						lcd_puts("CRC");
						timer2 = 0;
						while (timer2 < 6000){};
						lcd_clrscr();
						lcd_home();
						lcd_puts("Запись");
						lcd_gotoxy(0, 1);
						lcd_puts("ключа ТМ");
						tm_event=no_tm_event;
					}
					break;

					case tm_crc_ok: 
					{
					
						
						eeprom_write_byte(&eeprom_tm_serial_num.device_code,device_code);
						for (int i = 0; i<6; i++ )
						{
							eeprom_write_byte(&eeprom_tm_serial_num.serial[i],tm_code[i]);
						}
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ключ ТМ");
						lcd_gotoxy(0, 1);
						lcd_puts("записан!");
						timer2 = 0;
						while (timer2 < 6000){};
						tm_event=no_tm_event;
					
					}

					break;
				
				}//[/switch]
				if (reload_key_event==key_pressing) 
			//	if (joystick_event==key_central_pressing) 
				break;	

			//	if (joystick_event!=key_central_pressing) joystick_event = no_pressing;
		
			};//[/while]
		
//if (joystick_event==key_central_pressing)
if (reload_key_event==key_pressing)
		/*если вышли по нажатию кнопки перезаряда*/
	{
		reload_key_event=no_key_pressing;
	//	joystick_event = no_pressing;
		lcd_clrscr();
		lcd_home();

		lcd_puts("Перезар.\nнастр. 5");

		//lcd_puts("Вправо");
		while ((countdown > 0)&&(reload_key_event==no_key_pressing))//пока не кончиться обратный отсчёт или не нажмун кнопку джойстика
		{
			timer2 = 0;
			while ((timer2 < 6000)&&(reload_key_event==no_key_pressing)){};
			if (reload_key_event!=no_key_pressing) break; //если нажата кнопка, выходим из цикла 
			lcd_gotoxy(7, 1);
			countdown--;
			lcd_puts(int_to_str(countdown,0));
		}

		if (reload_key_event==key_pressing) 
		{
		
			get_all_setings();
			/*
			get_int_settings("Идент. игрока:", &eeprom_player_id, 127); //нажата центральная кнопка
			set_player_id(eeprom_read_byte(&eeprom_player_id));	//Устанавливаем идентификатор игрока
			get_int_settings("Идент. команды:", &eeprom_team_id, 3); //нажата центральная кнопка
			set_team_color(team_id());	//Устанавливаем идентификатор (цвет) команды
			get_enum_settings("Наносимый урон:", &eeprom_damage, &damage_value, Damage_100);
			set_gun_damage(gun_damage());		//Устанавливаем мощьность оружия (урон)
			get_int_settings("Емкость магазина:", &eeprom_bullets_in_clip, 90); //нажата центральная кнопка 
			get_int_settings("Магазинов:", &eeprom_clips, 90);
			get_int_settings("Время перезаряда:", &eeprom_reload_duration, 8);
			*/
	//	return;
		}
	}


//	bullets = eeprom_read_byte(&eeprom_bullets_in_clip);
//	BULLETS_OUT_LED_OFF;
	bullets = 0;
	BULLETS_OUT_LED_ON;
	
	clips = eeprom_read_byte(&eeprom_clips);
	joystick_event=no_pressing;
	keyboard_event=no_key_pressing;
	tm_event=no_tm_event;




	}//[/if]


if ((eeprom_read_byte(&eeprom_tm_serial_num.device_code)!=0)&&(eeprom_read_byte(&eeprom_tm_serial_num.device_code)!=0xFF))

/*если ТМ ключ уже занесён в память*/

	{//[if]
		
		volatile uint8_t tm_valide=0;
		while (!tm_valide)
		{//[while]
		lcd_clrscr();
		lcd_home();
		lcd_puts("Активац.\nключом");
		//lcd_gotoxy(0, 1);
		//lcd_puts("приложите ключ");
		while ((cr_received==false)&&(keyboard_event==no_key_pressing)&&(tm_event == no_tm_event)){};
		if (cr_received)
				{	
					parsing_command();
					

				}	
		if (keyboard_event==key_pressing)
				{
					if (BT_STATE_IN&BT_STATE_PIN)//если блютус соединение установлено
					{
						while(!(FIRE_KEY_IN&FIRE_KEY_PIN))//пока нажат курок
						{
							USART_PutChar('1');//посылаем повязке символ '1' - мигнуть диодом на повязке
							timer2 = 0;
							while (timer2 < 6000);

						}
					}
					
					
					keyboard_event=no_key_pressing;
				}
		switch(tm_event)
				{//[switch]
					case no_tm_event: 
					{

					}
					break;
					
					case tm_crc_error: 
					{
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ошибка");
						lcd_gotoxy(0, 1);
						lcd_puts("CRC");
						timer2 = 0;
						while (timer2 < 6000){};
						tm_event=no_tm_event;
					}
					break;

					case tm_crc_ok: 
					{
					
						if (tm_verification()) 	
						{
							tm_valide=1;				
							lcd_clrscr();
							lcd_home();
							lcd_puts("Удачи!");
							timer2 = 0;
							while (timer2 < 6000){};
							tm_event=no_tm_event;
							break;
						}
						lcd_clrscr();
						lcd_home();
						lcd_puts("Не тот");
						lcd_gotoxy(0, 1);
						lcd_puts("ключ");
						timer2 = 0;
						while (timer2 < 6000){};
						tm_event=no_tm_event;

					}

					break;
				
				}//[/switch]
			}//[while]
		lcd_clrscr();
		lcd_home();
		joystick_event=no_pressing;
		keyboard_event=no_key_pressing;
		tm_event=no_tm_event;
lcd_puts("Ключ для\nнастр. 5");
//lcd_puts("Вправо");
while ((countdown > 0)&&(tm_event == no_tm_event)&&(joystick_event==no_pressing))//пока не кончиться обратный отсчёт или не нажмун кнопку джойстика
	{
		timer2 = 0;
		while ((timer2 < 6000)&&(joystick_event==no_pressing)){};
		if (joystick_event!=no_pressing) break; //если нажата кнопка, выходим из цикла 
		lcd_gotoxy(7, 1);
		countdown--;
		lcd_puts(int_to_str(countdown,0));
	}


switch(tm_event)
				{//[switch]
					case no_tm_event: 
					{

					}
					break;
					
					case tm_crc_error: 
					{
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ошибка");
						lcd_gotoxy(0, 1);
						lcd_puts("CRC");
						timer2 = 0;
						while (timer2 < 6000){};
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ключ для");
						lcd_gotoxy(0, 1);
						lcd_puts("настроек");
						tm_event=no_tm_event;
					}
					break;

					case tm_crc_ok: 
					{
					
						if (tm_verification()) 	//ключ верный
						{
							get_all_setings();
							tm_event=no_tm_event;
							break;
						}
						//чужой ключ
						lcd_clrscr();
						lcd_home();
						lcd_puts("Не тот");
						lcd_gotoxy(0, 1);
						lcd_puts("ключ");
						timer2 = 0;
						while (timer2 < 6000){};
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ключ для");
						lcd_gotoxy(0, 1);
						lcd_puts("настроек");
						tm_event=no_tm_event;

					
					}

					break;
				
				}//[/switch]

/*
if (joystick_event==key_central_pressing) 
	{
		
		get_int_settings("Идент. игрока:", &eeprom_player_id, 127); //нажата центральная кнопка
		get_int_settings("Идент. команды:", &eeprom_team_id, 3); //нажата центральная кнопка
		get_enum_settings("Наносимый урон:", &eeprom_damage, &damage_value, Damage_100);
		get_int_settings("Емкость магазина:", &eeprom_bullets_in_clip, 90); //нажата центральная кнопка 
		get_int_settings("Магазинов:", &eeprom_clips, 90);
		get_int_settings("Время перезаряда:", &eeprom_reload_duration, 8);
	//	return;
	}

*/


//	bullets = eeprom_read_byte(&eeprom_bullets_in_clip);
//	BULLETS_OUT_LED_OFF;

	bullets = 0;
	BULLETS_OUT_LED_ON;


	clips = eeprom_read_byte(&eeprom_clips);
	joystick_event=no_pressing;
	keyboard_event=no_key_pressing;
	}






}



/*

void invite(){ //приглашение  в меню настроек

volatile uint8_t countdown = 5; //счётчик обратного отсчёта

lcd_clrscr();
lcd_home();
if ((eeprom_read_byte(&eeprom_tm_serial_num.device_code)==0)||(eeprom_read_byte(&eeprom_tm_serial_num.device_code)==0xFF))
	// если ключ ещё не записан
	{//[if]
		joystick_event = no_pressing;
		lcd_puts("Запись ключа ТМ");
		lcd_gotoxy(0, 1);
		lcd_puts("Центр.кн.-отмена");
		//timer1 = 0;
		while ((joystick_event!=key_central_pressing)&&(eeprom_read_byte(&eeprom_tm_serial_num.device_code)==0)||(eeprom_read_byte(&eeprom_tm_serial_num.device_code)==0xFF)) //пока не нажата центральная кнопка или не записан ключ
		{//[while]
				
				while ((cr_received==false)&&(keyboard_event==no_key_pressing)&&(joystick_event==no_pressing)&&(tm_event == no_tm_event)){};
				if (cr_received)
				{	
					parsing_command();
					

				}
				if (keyboard_event==key_pressing)
				{
					if (BT_STATE_IN&BT_STATE_PIN)//если блютус соединение установлено
					{
						while(!(FIRE_KEY_IN&FIRE_KEY_PIN))//пока нажат курок
						{
							USART_PutChar('1');//посылаем повязке символ '1' - мигнуть диодом на повязке
							timer2 = 0;
							while (timer2 < 6000);

						}
					}
					
					
					keyboard_event=no_key_pressing;
				}	
				switch(tm_event)
				{//[switch]
					case no_tm_event: 
					{
					
					}
					break;
					
					case tm_crc_error: 
					{
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ошибка CRC");
						timer2 = 0;
						while (timer2 < 6000){};
						lcd_clrscr();
						lcd_home();
						lcd_puts("Запись ключа ТМ");
						lcd_gotoxy(0, 1);
						lcd_puts("Центр.кн.-отмена");
						tm_event=no_tm_event;
					}
					break;

					case tm_crc_ok: 
					{
					
						
						eeprom_write_byte(&eeprom_tm_serial_num.device_code,device_code);
						for (int i = 0; i<6; i++ )
						{
							eeprom_write_byte(&eeprom_tm_serial_num.serial[i],tm_code[i]);
						}
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ключ ТМ записан!");
						timer2 = 0;
						while (timer2 < 6000){};
						tm_event=no_tm_event;
					
					}

					break;
				
				}//[/switch]
				if (joystick_event==key_central_pressing) 
				break;	

				if (joystick_event!=key_central_pressing) joystick_event = no_pressing;
		
			};//[/while]
		
if (joystick_event==key_central_pressing)
		//если вышли по нажатию центральной кнопки
	{
		joystick_event = no_pressing;
		lcd_clrscr();
		lcd_home();

		lcd_puts("Для настроек жми\nцентр. кнопку 5");
		//lcd_puts("Вправо");
		while ((countdown > 0)&&(joystick_event==no_pressing))//пока не кончиться обратный отсчёт или не нажмун кнопку джойстика
		{
			timer2 = 0;
			while ((timer2 < 6000)&&(joystick_event==no_pressing)){};
			if (joystick_event!=no_pressing) break; //если нажата кнопка, выходим из цикла 
			lcd_gotoxy(14, 1);
			countdown--;
			lcd_puts(int_to_str(countdown,0));
		}

		if (joystick_event==key_central_pressing) 
		{
		
			get_all_setings();
		
		}
	}


//	bullets = eeprom_read_byte(&eeprom_bullets_in_clip);
//	BULLETS_OUT_LED_OFF;
	bullets = 0;
	BULLETS_OUT_LED_ON;
	
	clips = eeprom_read_byte(&eeprom_clips);
	joystick_event=no_pressing;
	keyboard_event=no_key_pressing;
	tm_event=no_tm_event;




	}//[/if]


if ((eeprom_read_byte(&eeprom_tm_serial_num.device_code)!=0)&&(eeprom_read_byte(&eeprom_tm_serial_num.device_code)!=0xFF))

//если ТМ ключ уже занесён в память

	{//[if]
		
		volatile uint8_t tm_valide=0;
		while (!tm_valide)
		{//[while]
		lcd_clrscr();
		lcd_home();
		lcd_puts("Для активации\nприложи ключ");
		//lcd_gotoxy(0, 1);
		//lcd_puts("приложите ключ");
		while ((cr_received==false)&&(keyboard_event==no_key_pressing)&&(tm_event == no_tm_event)){};
		if (cr_received)
				{	
					parsing_command();
					

				}	
		if (keyboard_event==key_pressing)
				{
					if (BT_STATE_IN&BT_STATE_PIN)//если блютус соединение установлено
					{
						while(!(FIRE_KEY_IN&FIRE_KEY_PIN))//пока нажат курок
						{
							USART_PutChar('1');//посылаем повязке символ '1' - мигнуть диодом на повязке
							timer2 = 0;
							while (timer2 < 6000);

						}
					}
					
					
					keyboard_event=no_key_pressing;
				}	
		switch(tm_event)
				{//[switch]
					case no_tm_event: 
					{

					}
					break;
					
					case tm_crc_error: 
					{
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ошибка CRC");
						timer2 = 0;
						while (timer2 < 6000){};
						tm_event=no_tm_event;
					}
					break;

					case tm_crc_ok: 
					{
					
						if (tm_verification()) 	
						{
							tm_valide=1;				
							lcd_clrscr();
							lcd_home();
							lcd_puts("Удачи!");
							timer2 = 0;
							while (timer2 < 6000){};
							tm_event=no_tm_event;
							break;
						}
						lcd_clrscr();
						lcd_home();
						lcd_puts("Не тот ключ");
						timer2 = 0;
						while (timer2 < 6000){};
						tm_event=no_tm_event;

					}

					break;
				
				}//[/switch]
			}//[while]
		lcd_clrscr();
		lcd_home();
		joystick_event=no_pressing;
		keyboard_event=no_key_pressing;
		tm_event=no_tm_event;
lcd_puts("Для настроек\nприложи ключ  5");
//lcd_puts("Вправо");
while ((countdown > 0)&&(tm_event == no_tm_event)&&(joystick_event==no_pressing))//пока не кончиться обратный отсчёт или не нажмун кнопку джойстика
	{
		timer2 = 0;
		while ((timer2 < 6000)&&(joystick_event==no_pressing)){};
		if (joystick_event!=no_pressing) break; //если нажата кнопка, выходим из цикла 
		lcd_gotoxy(14, 1);
		countdown--;
		lcd_puts(int_to_str(countdown,0));
	}


switch(tm_event)
				{//[switch]
					case no_tm_event: 
					{

					}
					break;
					
					case tm_crc_error: 
					{
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ошибка CRC");
						timer2 = 0;
						while (timer2 < 6000){};
						lcd_clrscr();
						lcd_home();
						lcd_puts("Для настроек");
						lcd_gotoxy(0, 1);
						lcd_puts("приложи ключ");
						tm_event=no_tm_event;
					}
					break;

					case tm_crc_ok: 
					{
					
						if (tm_verification()) 	//ключ верный
						{
							get_all_setings();
							tm_event=no_tm_event;
							break;
						}
						//чужой ключ
						lcd_clrscr();
						lcd_home();
						lcd_puts("Не тот ключ");
						timer2 = 0;
						while (timer2 < 6000){};
						lcd_clrscr();
						lcd_home();
						lcd_puts("Для настроек");
						lcd_gotoxy(0, 1);
						lcd_puts("приложи ключ");
						tm_event=no_tm_event;

					
					}

					break;
				
				}//[/switch]



//	bullets = eeprom_read_byte(&eeprom_bullets_in_clip);
//	BULLETS_OUT_LED_OFF;

	bullets = 0;
	BULLETS_OUT_LED_ON;


	clips = eeprom_read_byte(&eeprom_clips);
	joystick_event=no_pressing;
	keyboard_event=no_key_pressing;
	}






}

*/


char numbers[] PROGMEM={"0123456789"};
char* int_to_str(uint8_t x, uint8_t digits){
//const char numbers[10]="0123456789";

static volatile char str[6];



volatile uint8_t celoe, ostatok;
celoe=x;
//tmp = celoe/100;
//str[0]=pgm_read_byte(&numbers[tmp]);
//tmp = celoe - t100;
int digits_tmp;
digits_tmp=digits;
if (digits == 0) digits_tmp=3;
      for (int i=(digits_tmp-1); i>=0; i--)
      {   
      //volatile long int tmp;
	 // tmp = celoe;
      ostatok= celoe%10;
	  celoe  = celoe/10;
	  //ostatok= tmp - celoe*10;   
      str[i]= pgm_read_byte(&numbers[ostatok]);
      }
      str[digits_tmp]='\0';
	  
	  
	  
if (digits == 0)	
{
        while ((str[0]=='0')&str[1] !='\0') for (int i=0; i < 6; i++) str[i]=str[i+1];
}

//str[0]=numbers[0];
//str[0]=pgm_read_byte(&numbers[0]);
//str[1]=numbers[1];//pgm_read_byte(&(numbers[1]));
//str[2]=numbers[2];//pgm_read_byte(&(numbers[2]));
//str[3]='\0';

      return &str;      

}

char* long_int_to_str(uint16_t x, uint8_t digits){
//const char numbers[10]="0123456789";

static volatile char str[6];



volatile uint16_t celoe, ostatok;
celoe=x;
//tmp = celoe/100;
//str[0]=pgm_read_byte(&numbers[tmp]);
//tmp = celoe - t100;
int digits_tmp;
digits_tmp=digits;
if (digits == 0) digits_tmp=5;
      for (int i=(digits_tmp-1); i>=0; i--)
      {   
      //volatile long int tmp;
	 // tmp = celoe;
      ostatok= celoe%10;
	  celoe  = celoe/10;
	  //ostatok= tmp - celoe*10;   
      str[i]= pgm_read_byte(&numbers[ostatok]);
      }
      str[digits_tmp]='\0';
	  
	  
	  
if (digits == 0)	
{
        while ((str[0]=='0')&str[1] !='\0') for (int i=0; i < 6; i++) str[i]=str[i+1];
}

//str[0]=numbers[0];
//str[0]=pgm_read_byte(&numbers[0]);
//str[1]=numbers[1];//pgm_read_byte(&(numbers[1]));
//str[2]=numbers[2];//pgm_read_byte(&(numbers[2]));
//str[3]='\0';

      return &str;      

}

volatile void get_int_settings(char* text, uint8_t* var_adress, uint8_t max_value){//Получаем значения настроек с помощью джойстика и ЖКИ
uint8_t result;
uint8_t cursor_pos=4;//позиия курсора
volatile bool quit=false; 

uint8_t pos0_digit;
uint8_t pos1_digit;
uint8_t pos2_digit;



//joystick_event=no_pressing;
reload_key_event = no_key_pressing;
keyboard_event = no_key_pressing;
result = eeprom_read_byte(var_adress);
if (result>max_value) result = max_value;

pos0_digit = result/100; //сотни
pos1_digit = result/10-pos0_digit*10;//десятки
pos2_digit = result-pos0_digit*100-pos1_digit*10;//единицы


	
lcd_clrscr();

lcd_command(1<<LCD_ON);//выключаем дисплей
lcd_command(LCD_DISP_ON_BLINK/*LCD_DISP_ON|LCD_ON_CURSOR|LCD_ON_BLINK*/);//включаем дисплей, курсор выключен


lcd_puts(text);
lcd_gotoxy(0, 1);
lcd_puts(int_to_str(result,3));
lcd_puts(" Дал.");
lcd_gotoxy(cursor_pos, 1);

while  (!quit)//пока не вышли из настроек
{
//	while  (joystick_event==no_pressing){};
	while  ((reload_key_event == no_key_pressing)&&(keyboard_event!=key_pressing)){};
	switch(cursor_pos)//смотрим, в какой позиции курсор (сотни, десятки или единицы)
		{
			case 0: //сотни
			{
				if(reload_key_event == key_pressing){ cursor_pos=4;lcd_gotoxy(cursor_pos, 1);break;}
				if(keyboard_event== key_pressing)
				{
					if ((pos0_digit+1) <=  max_value/100) pos0_digit++;
					else pos0_digit=0;
					result = pos0_digit*100+pos1_digit*10+pos2_digit;
					lcd_gotoxy(0, 1);
					lcd_puts(int_to_str(result,3));
					lcd_gotoxy(cursor_pos, 1);
					break;
				}
				
			
			}
			break;
			case 1: //десятки
			{			
				if(reload_key_event == key_pressing){  lcd_gotoxy(--cursor_pos, 1);break;}
				if(keyboard_event== key_pressing)
				{
					if (((pos0_digit*100+(pos1_digit+1)*10+pos2_digit) <= max_value)&&(pos1_digit<9)) pos1_digit++;
					else pos1_digit=0;
					result = pos0_digit*100+pos1_digit*10+pos2_digit;
					lcd_gotoxy(0, 1);
					lcd_puts(int_to_str(result,3));
					lcd_gotoxy(cursor_pos, 1);
					break;
				}
			
			
			
			
			
			}
			break;
			case 2: //единицы
			{
				if(reload_key_event == key_pressing){  /*cursor_pos=4;*/ lcd_gotoxy(--cursor_pos, 1);break;}
			
				if(keyboard_event== key_pressing)
				{
					if (((pos0_digit*100+pos1_digit*10+pos2_digit+1) <= max_value)&&(pos2_digit<9)) pos2_digit++;
					else pos2_digit=0;
					result = pos0_digit*100+pos1_digit*10+pos2_digit;
					lcd_gotoxy(0, 1);
					lcd_puts(int_to_str(result,3));
					lcd_gotoxy(cursor_pos, 1);
					break;
				}
			
			
			
			
			}
			break;
			case 4: //позиция выхода из данного меню настроек
			{
				if(reload_key_event == key_pressing){  cursor_pos=2; lcd_gotoxy(cursor_pos, 1);}
				if(keyboard_event== key_pressing)
				{
					quit = true;
				}
	
	
			}
			break;

		}
	reload_key_event = no_key_pressing; 
	keyboard_event= no_key_pressing; 
	

}

if (result != eeprom_read_byte(var_adress))
	{
	
	lcd_clrscr();
	lcd_puts("Сохранен");
	eeprom_write_byte(var_adress, result);	
	}
}



volatile void get_enum_settings(char* text, uint8_t* var_adress, uint8_t* arr_adress, uint8_t max_value)
{
uint8_t result;
uint8_t value;
joystick_event=no_pressing;
reload_key_event = no_key_pressing;
keyboard_event = no_key_pressing;
result = eeprom_read_byte(var_adress);
value = pgm_read_byte(arr_adress+result);



lcd_clrscr();
lcd_command(1<<LCD_ON);//выключаем дисплей
lcd_command(LCD_DISP_ON);//включаем дисплей, курсор выключен
lcd_puts(text);
lcd_gotoxy(0, 1);
lcd_puts(int_to_str(value,3));
lcd_puts(" П-OK");

while  (/*joystick_event!=key_central_pressing*/reload_key_event != key_pressing)
{
	while  (/*joystick_event==no_pressing*/(keyboard_event != key_pressing)&&(reload_key_event != key_pressing)){};

	if (keyboard_event == key_pressing)
	{
		if ((result)<max_value) result++;
	//	else result = eeprom_read_byte(var_adress);
		else result = 0;	
			keyboard_event=no_key_pressing;
			lcd_gotoxy(0, 1);
			lcd_puts(int_to_str(pgm_read_byte(arr_adress+result),3));
	}
	else;

}
reload_key_event=no_key_pressing;
if (result != eeprom_read_byte(var_adress))
	{
	
	lcd_clrscr();
	lcd_puts("Сохранен");
	eeprom_write_byte(var_adress, result);	
	}

}



/*
volatile void get_int_settings(char* text, uint8_t* var_adress, uint8_t max_value){//Получаем значения настроек с помощью джойстика и ЖКИ
uint8_t result;
joystick_event=no_pressing;
result = eeprom_read_byte(var_adress);
if (result>max_value) result = max_value;

lcd_clrscr();
lcd_puts(text);
lcd_gotoxy(0, 1);
lcd_puts(int_to_str(result,3));
lcd_puts(" центр.кн.-OK");

while  (joystick_event!=key_central_pressing)
{
	while  (joystick_event==no_pressing){};
	switch(joystick_event){
	case key_up_pressing: 
		{
		//	lcd_clrscr();
	//		lcd_gotoxy(0, 0);
	
			if ((result+10)<=max_value) result=result+10;
			lcd_gotoxy(0, 1);
			lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
		}
	
		break;
		case key_right_pressing: 
		{
			//lcd_clrscr();
			//lcd_home();
			if ((result)<max_value) result++;
			lcd_gotoxy(0, 1);
			lcd_puts(int_to_str(result,3));
			//lcd_puts("Нажата кнопка \n");
			//lcd_puts("Вправо");
			joystick_event = no_pressing;
		}
		break;
		case key_down_pressing: 
		{
	//		lcd_clrscr();
	//		lcd_gotoxy(0, 0);
			if((result-9)>0) result=result-10;
			lcd_gotoxy(0, 1);
			lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
		}
		break;
		case key_left_pressing: 
		{
			//lcd_clrscr();
			//lcd_gotoxy(0, 0);
			if ((result)>0) result--;
			//lcd_puts("Нажата кнопка \n");
			//lcd_puts("Влево");
			lcd_gotoxy(0, 1);
			lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
		}
		break;
		case key_central_pressing: 
		{
			
		}
		break;
		default: joystick_event = no_pressing;
	
	
	}

}

if (result != eeprom_read_byte(var_adress))
	{
	
	lcd_clrscr();
	lcd_puts("Сохранение...");
	eeprom_write_byte(var_adress, result);	
	}
}


*/

/*
volatile void get_enum_settings(char* text, uint8_t* var_adress, uint8_t* arr_adress, uint8_t max_value)
{
uint8_t result;
uint8_t value;
joystick_event=no_pressing;
result = eeprom_read_byte(var_adress);
value = pgm_read_byte(arr_adress+result);

lcd_clrscr();
lcd_puts(text);
lcd_gotoxy(0, 1);
lcd_puts(int_to_str(value,3));
lcd_puts(" центр.кн.-OK");

while  (joystick_event!=key_central_pressing)
{
	while  (joystick_event==no_pressing){};
	switch(joystick_event){
	case key_up_pressing: 
		{
		//	lcd_clrscr();
	//		lcd_gotoxy(0, 0);
	
	//		if ((result+10)<=max_value) result=result+10;
	//		lcd_gotoxy(0, 1);
	//		lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
		}
	
		break;
		case key_right_pressing: 
		{
			//lcd_clrscr();
			//lcd_home();
			if ((result)<max_value) result++;
			lcd_gotoxy(0, 1);
			lcd_puts(int_to_str(pgm_read_byte(arr_adress+result),3));
			//lcd_puts("Нажата кнопка \n");
			//lcd_puts("Вправо");
			joystick_event = no_pressing;
		}
		break;
		case key_down_pressing: 
		{
	//		lcd_clrscr();
	//		lcd_gotoxy(0, 0);
	//		if((result-9)>0) result=result-10;
	//		lcd_gotoxy(0, 1);
	//		lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
		}
		break;
		case key_left_pressing: 
		{
			//lcd_clrscr();
			//lcd_gotoxy(0, 0);
			if ((result)>0) result--;
			//lcd_puts("Нажата кнопка \n");
			//lcd_puts("Влево");
			lcd_gotoxy(0, 1);
			lcd_puts(int_to_str(pgm_read_byte(arr_adress+result),3));
			joystick_event = no_pressing;
		}
		break;
		case key_central_pressing: 
		{
			
		}
		break;
		default: joystick_event = no_pressing;
	
	
	}
}

if (result != eeprom_read_byte(var_adress))
	{
	
	lcd_clrscr();
	lcd_puts("Сохранение...");
	eeprom_write_byte(var_adress, result);	
	}

}

*/

void check_eeprom(){

if (eeprom_read_byte(&eeprom_player_id)>127) eeprom_write_byte(&eeprom_player_id,0); 
if (eeprom_read_byte(&eeprom_team_id)>3) eeprom_write_byte(&eeprom_team_id,0); 
if (eeprom_read_byte(&eeprom_damage)>Damage_100) eeprom_write_byte(&eeprom_damage,Damage_10); 
if (eeprom_read_byte(&eeprom_bullets_in_clip)>90) eeprom_write_byte(&eeprom_bullets_in_clip,30); 
if (eeprom_read_byte(&eeprom_clips)>90) eeprom_write_byte(&eeprom_clips,15); 
if (eeprom_read_byte(&eeprom_reload_duration)>8) eeprom_write_byte(&eeprom_reload_duration,1); 
if (eeprom_read_word(&eeprom_batt_full_voltage)==0xFFFF) eeprom_write_word(&eeprom_batt_full_voltage,(DEFAULT_BATT_FULL_VOLTAGE)); 
if (eeprom_read_word(&eeprom_batt_low_voltage)==0xFFFF) eeprom_write_word(&eeprom_batt_low_voltage,(DEFAULT_BATT_LOW_VOLTAGE)); 
if ((eeprom_read_byte(&eeprom_ir_carr_freq)!=freq_36KHz)&&(eeprom_read_byte(&eeprom_ir_carr_freq)!=freq_56KHz)) eeprom_write_byte(&eeprom_ir_carr_freq,freq_36KHz); 
if ((eeprom_read_byte(&eeprom_curr_ir_pin)!=IR_LED_HIGH_POWER_PIN)&&(eeprom_read_byte(&eeprom_curr_ir_pin)!=IR_LED_LOW_POWER_PIN)) eeprom_write_byte(&eeprom_curr_ir_pin,IR_LED_HIGH_POWER_PIN); 
if (eeprom_read_byte(&friendly_fire_enable)>1) eeprom_write_byte(&friendly_fire_enable,1); 
}





void display_status()//выводим на дисплей текущее значение жизни, патронов, магазинов 
{
	lcd_clrscr();
	lcd_putc(1);
	lcd_puts(int_to_str(life_in_percent,0));
	//lcd_puts("% ");
	lcd_gotoxy(7, 0);
	lcd_putc(0);
	lcd_gotoxy(0, 1);
	lcd_putc(2);
	lcd_puts(int_to_str(bullets,3));
	lcd_putc('/');
	lcd_gotoxy(5, 1);
	lcd_putc(3);
	lcd_puts(int_to_str(clips,2));
	lcd_puts(" ");

/*
lcd_putc(1);
lcd_puts("100\n");
lcd_putc(2);
lcd_puts("030/");

lcd_putc(3);
lcd_puts("15 ");
//lcd_putc(3);
//lcd_putc(4);
//lcd_putc(5);
lcd_gotoxy(7, 0);
lcd_putc(0);
*/


}


void display_life_update(){//обновляем значение жизни на дисплее
lcd_gotoxy(1, 0);
lcd_puts(int_to_str(life_in_percent,3));
//lcd_puts("%   ");
}



void display_bullets_update(){//обновляем значение жизни на дисплее
lcd_gotoxy(1, 1);
lcd_puts(int_to_str(bullets,3));
//lcd_puts(" ");
}

void display_clips_update(){//обновляем значение жизни на дисплее
lcd_gotoxy(6, 1);
lcd_puts(int_to_str(clips,2));
lcd_puts(" ");
}


void display_voltage_update(){//обновляем значение жизни на дисплее

volatile uint16_t adc_data;
//volatile uint16_t batt_voltage;
adc_data=ReadADC(ADC_CHANNEL);


uint16_t delta; //разница между значениями АЦП при полностью заряженной и полностью разряженной батареи
uint8_t curr_batt_level; //текузий уровеь состояния батареи (1 из 6 возможных)
uint16_t full_level, low_level;//значения полностью заряженной и полностью разряженной батареи в значениях АЦП
full_level = (eeprom_read_word(&eeprom_batt_full_voltage)*4)/75;
low_level = (eeprom_read_word(&eeprom_batt_low_voltage)*4)/75;


//delta=((eeprom_read_word(&eeprom_batt_full_voltage)-eeprom_read_word(&eeprom_batt_low_voltage))*4)/75;
delta = full_level - low_level;



switch(display_batt_mode)
{
	case icon:
	{
		lcd_gotoxy(5, 0);
		lcd_puts("  ");
		lcd_putc(0);
	//	lcd_gotoxy(15, 0);
		lcd_gotoxy(7, 0);
		if (adc_data < low_level) 
		{
//		curr_batt_level=0;
			lcd_generate_bat_level_symbols(0);
			return;
		}
		if (adc_data >full_level)
		{
			lcd_generate_bat_level_symbols(5);
			return;
		}
		curr_batt_level = (6*(adc_data - low_level))/delta;
		//lcd_putc(curr_batt_level);
		lcd_generate_bat_level_symbols(curr_batt_level);
		return;


	}
	break;
	case digit:
	{
		lcd_gotoxy(5, 0);
		adc_data=(adc_data*15)/8;
		lcd_puts(int_to_str(adc_data/100,0));
		lcd_puts(",");
		lcd_puts(int_to_str((adc_data%100)/10,0));
		lcd_puts(" ");
		return;
	
	}
	break;

}




/*
lcd_gotoxy(12, 0);
lcd_puts(int_to_str(viltage/100,0));
lcd_puts(",");
lcd_puts(int_to_str(viltage%100,0));
lcd_puts(" ");
*/




}



void init_adc(void)
{

ADMUX=((1<<REFS0)|(1<<REFS1));//выбираем внутренний источник опорного напряжения
ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Rrescalar div factor =128


}


uint16_t ReadADC(uint8_t ch)
{
   //Select ADC Channel ch must be 0-7
   ch=ch&0b00000111;
   ADMUX|=ch;

   //Start Single conversion
   ADCSRA|=(1<<ADSC);

   //Wait for conversion to complete
   while(!(ADCSRA & (1<<ADIF)));

   //Clear ADIF by writing one to it
   //Note you may be wondering why we have write one to clear it
   //This is standard way of clearing bits in io as said in datasheets.
   //The code writes '1' but it result in setting bit to '0' !!!

   ADCSRA|=(1<<ADIF);

   return(ADC);
}


/********************************************
*покажем кто в нас попал и какой урон нанёс
********************************************/
/*
void display_hit_data(trx_packet hit_packet)
{
lcd_clrscr();
lcd_home();
lcd_puts("Урон ");
lcd_puts(int_to_str(hit_packet.damage,0));
lcd_puts("% нанес");
lcd_gotoxy(0, 1);
lcd_puts("иг. ");
lcd_puts(int_to_str(hit_packet.player_id,0));
lcd_puts(" ком. ");
lcd_puts(int_to_str(hit_packet.team_id,0));
}
*/


/********************************************
*покажем кто в нас попал и какой урон нанёс
********************************************/

void display_hit_data(trx_packet hit_packet)
{
lcd_clrscr();
lcd_home();
lcd_puts("Урон ");
lcd_puts(int_to_str(hit_packet.damage,0));
//lcd_puts("%");
lcd_gotoxy(0, 1);
lcd_putc(4);
lcd_puts(int_to_str(hit_packet.player_id,0));
lcd_gotoxy(5, 1);
lcd_putc(5);
lcd_puts(int_to_str(hit_packet.team_id,0));
}




void get_ir_power_settings(void)
{
uint8_t result;
uint8_t value;
joystick_event=no_pressing;
reload_key_event = no_key_pressing;
keyboard_event = no_key_pressing;
result = eeprom_read_byte(&eeprom_curr_ir_pin);
lcd_clrscr();
lcd_command(1<<LCD_ON);//выключаем дисплей
lcd_command(LCD_DISP_ON);//включаем дисплей, курсор выключен

lcd_puts("Мощн. ИК");
lcd_gotoxy(0, 1);
if (result==IR_LED_HIGH_POWER_PIN) lcd_puts("ул. ");
if (result==IR_LED_LOW_POWER_PIN) lcd_puts("пом.");

lcd_puts("П-OK");

while  (reload_key_event != key_pressing)
{
	while  ((reload_key_event != key_pressing)&&(keyboard_event !=key_pressing)/*joystick_event==no_pressing*/){};

	if (keyboard_event ==key_pressing)
	{
		keyboard_event = no_key_pressing;
		if (result==IR_LED_LOW_POWER_PIN)
			{
				result=IR_LED_HIGH_POWER_PIN;
				lcd_gotoxy(0, 1);
				lcd_puts("ул. ");
			}
		else
		{		
				result=IR_LED_LOW_POWER_PIN;
				lcd_gotoxy(0, 1);
				lcd_puts("пом.");
		}
	
	}
	else;

}
reload_key_event = no_key_pressing;

if (result != eeprom_read_byte(&eeprom_curr_ir_pin))
	{
	
	lcd_clrscr();
	lcd_puts("Сохранен");
	eeprom_write_byte(&eeprom_curr_ir_pin, result);	
	}

}



/*
void get_ir_power_settings(void)
{
uint8_t result;
uint8_t value;
joystick_event=no_pressing;
result = eeprom_read_byte(&eeprom_curr_ir_pin);
lcd_clrscr();
lcd_puts("Мощность ИК для");
lcd_gotoxy(0, 1);
if (result==IR_LED_HIGH_POWER_PIN) lcd_puts("улицы");
if (result==IR_LED_LOW_POWER_PIN) lcd_puts("помещ");

lcd_puts(" цен.кн.-OK");

while  (joystick_event!=key_central_pressing)
{
	while  (joystick_event==no_pressing){};
	switch(joystick_event){
	case key_up_pressing: 
		{
		//	lcd_clrscr();
	//		lcd_gotoxy(0, 0);
	
	//		if ((result+10)<=max_value) result=result+10;
	//		lcd_gotoxy(0, 1);
	//		lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
		}
	
		break;
		case key_right_pressing: 
		{
			//lcd_clrscr();
			//lcd_home();
		
		//	if ((result)<max_value) result++;
			if (result==IR_LED_LOW_POWER_PIN)
			{
				result=IR_LED_HIGH_POWER_PIN;
				lcd_gotoxy(0, 1);
				lcd_puts("улицы");
			}
			
			joystick_event = no_pressing;
		}
		break;
		case key_down_pressing: 
		{
	//		lcd_clrscr();
	//		lcd_gotoxy(0, 0);
	//		if((result-9)>0) result=result-10;
	//		lcd_gotoxy(0, 1);
	//		lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
		}
		break;
		case key_left_pressing: 
		{
			//lcd_clrscr();
			//lcd_gotoxy(0, 0);
			
			if (result==IR_LED_HIGH_POWER_PIN) 
			{
				result=IR_LED_LOW_POWER_PIN;
				lcd_gotoxy(0, 1);
				lcd_puts("помещ");
			}
			
			joystick_event = no_pressing;
		}
		break;
		case key_central_pressing: 
		{
			
		}
		break;
		default: joystick_event = no_pressing;
	
	
	}
}

if (result != eeprom_read_byte(&eeprom_curr_ir_pin))
	{
	
	lcd_clrscr();
	lcd_puts("Сохранение...");
	eeprom_write_byte(&eeprom_curr_ir_pin, result);	
	}

}


*/

/*
void get_friendly_fire_settings(void)//включение/отключение "дружественного" огня
{
uint8_t result;
uint8_t value;
joystick_event=no_pressing;
result = eeprom_read_byte(&friendly_fire_enable);
lcd_clrscr();
lcd_puts("Дружеств. огонь:");
lcd_gotoxy(0, 1);
if (result) lcd_puts("Да ");
else lcd_puts("Нет");

lcd_puts(" цен.кн.-OK");

while  (joystick_event!=key_central_pressing)
{
	while  (joystick_event==no_pressing){};
	switch(joystick_event){
	case key_up_pressing: 
		{
		//	lcd_clrscr();
	//		lcd_gotoxy(0, 0);
	
	//		if ((result+10)<=max_value) result=result+10;
	//		lcd_gotoxy(0, 1);
	//		lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
		}
	
		break;
		case key_right_pressing: 
		{
			//lcd_clrscr();
			//lcd_home();
		
		//	if ((result)<max_value) result++;
			if (result==false)
			{
				result=true;
				lcd_gotoxy(0, 1);
				lcd_puts("Да ");
			}
			
			joystick_event = no_pressing;
		}
		break;
		case key_down_pressing: 
		{
	//		lcd_clrscr();
	//		lcd_gotoxy(0, 0);
	//		if((result-9)>0) result=result-10;
	//		lcd_gotoxy(0, 1);
	//		lcd_puts(int_to_str(result,3));
			joystick_event = no_pressing;
		}
		break;
		case key_left_pressing: 
		{
			//lcd_clrscr();
			//lcd_gotoxy(0, 0);
			
			if (result) 
			{
				result=false;
				lcd_gotoxy(0, 1);
				lcd_puts("Нет");
			}
			
			joystick_event = no_pressing;
		}
		break;
		case key_central_pressing: 
		{
			
		}
		break;
		default: joystick_event = no_pressing;
	
	
	}
}

if (result != eeprom_read_byte(&friendly_fire_enable))
	{
	
	lcd_clrscr();
	lcd_puts("Сохранение...");
	eeprom_write_byte(&friendly_fire_enable, result);	
	}

}


*/

void get_friendly_fire_settings(void)//включение/отключение "дружественного" огня
{
uint8_t result;
uint8_t value;
joystick_event=no_pressing;
reload_key_event = no_key_pressing;
keyboard_event = no_key_pressing;
result = eeprom_read_byte(&friendly_fire_enable);
lcd_clrscr();
lcd_command(1<<LCD_ON);//выключаем дисплей
lcd_command(LCD_DISP_ON);//включаем дисплей, курсор выключен
lcd_puts("Друж.ог.");
lcd_gotoxy(0, 1);
if (result) lcd_puts("Да ");
else lcd_puts("Нет");

lcd_puts(" П-OK");

while  (/*joystick_event!=key_central_pressing*/reload_key_event != key_pressing)
{
	while  ((reload_key_event != key_pressing)&&(keyboard_event !=key_pressing)/*joystick_event==no_pressing*/){};
	if (keyboard_event ==key_pressing)
	{
		keyboard_event = no_key_pressing;
		if (result)
		{
			result=false;
			lcd_gotoxy(0, 1);
			lcd_puts("Нет");	
		}
		else
		{		
			result=true;
			lcd_gotoxy(0, 1);
			lcd_puts("Да ");
		}
	
	}
	else;

}
reload_key_event = no_key_pressing;
if (result != eeprom_read_byte(&friendly_fire_enable))
	{
	
	lcd_clrscr();
	lcd_puts("Сохранен");
	eeprom_write_byte(&friendly_fire_enable, result);	
	}

}



void get_all_setings(void)
{
	get_int_settings("ID игр.", &eeprom_player_id, 127); //нажата центральная кнопка
	set_player_id(eeprom_read_byte(&eeprom_player_id));	//Устанавливаем идентификатор игрока
	get_int_settings("ID ком.", &eeprom_team_id, 3); //нажата центральная кнопка
	set_team_color(team_id());	//Устанавливаем идентификатор (цвет) команды
	get_enum_settings("Урон", &eeprom_damage, &damage_value, Damage_100);
	set_gun_damage(gun_damage());		//Устанавливаем мощьность оружия (урон)
	get_enum_settings("IR_F KHz", &eeprom_ir_carr_freq, &ir_f0_value, freq_56KHz);	
	get_ir_protocol();
	reinit_timer2();
	get_int_settings("Патронов", &eeprom_bullets_in_clip, 90); //нажата центральная кнопка 
	get_int_settings("Обойм", &eeprom_clips, 90);
	get_int_settings("Перез. с", &eeprom_reload_duration, 8);
	get_ir_power_settings();
	curr_ir_pin=eeprom_read_byte(&eeprom_curr_ir_pin);
	get_friendly_fire_settings();
}




uint8_t get_command_index(void)//проверим, что за команда пришла по UART
{
volatile uint16_t delta;
volatile char* cursor_pos;
volatile char* buff_pos;

volatile char cmd_buff[32];
uint8_t comand_len;
//char test_str[]="Also firmware";

	

	for (int index=0; index <(sizeof(commandsPointers)/sizeof(char*)); index++)
	{
		
		unsigned char sym;
		uint8_t sym_index = 0;
		char* psym; //указатель на первый символ команды в памяти программ
//		char* pos;
//		char* pos_buff;
		psym = (char*)(pgm_read_word(&(commandsPointers[index])));
		comand_len = 0;
		while((pgm_read_byte(psym)!=0))
		{
			sym = pgm_read_byte(psym);
			cmd_buff[sym_index++] = sym; //копируем команду в буфер
			comand_len++;
			psym++;
		}
		
		cursor_pos = memmem(&(usartRxBuf[rxBufHead]),rxCount,cmd_buff,comand_len);
		buff_pos = &usartRxBuf[0];
		if (memmem(&(usartRxBuf[rxBufHead]),rxCount,cmd_buff,comand_len)!=NULL) 
		{
			delta = (uint16_t)(cursor_pos) - (uint16_t)(buff_pos);
			
			rxBufHead = rxBufHead+comand_len+(unsigned char)(delta);
			rxCount = rxCount-comand_len-(unsigned char)(delta);
//			cursor_pos+=comand_len;
			return index;	
		}
//		if (memmem(test_str,13,cmd_buff,comand_len)!=NULL) return 33;	
		
		//while (sym = pgm_read_byte(p))


	
	}

	return 255; //не найдена такая команда в списке команд
}






void get_int_argument_value(uint8_t* var_adress, uint8_t min_val, uint8_t max_val)
{

bool param_not_empty = false;
volatile char ch_tmp;
volatile uint8_t result = 0;

while(usartRxBuf[rxBufHead] !='\r')

	{

		ch_tmp = usartRxBuf[rxBufHead++];
		if (ch_tmp==' ') continue; //игнорируем пробелы
		if ((ch_tmp >= '0')&&(ch_tmp<= '9'))
		{
			result = result*10+char_to_int(ch_tmp);
			param_not_empty = true;
		}		
		else 
		{
			USART_SendStrP(parameter_invalid_error);
			return;
			
			//USART_SendStr("ERROR\n\r");
			//return invalid;//недопустимый аргумент

		}
	}
if (!param_not_empty) 
	{
		USART_SendStrP(parameter_empty_error);
		return;//пустой аргумент
	}

if ((result>max_val)||(result<min_val))
	{
		USART_SendStrP(parameter_out_of_range_error);
		return;//аргумент больше максимально допустимого значения
	} 

eeprom_write_byte(var_adress, result);	
USART_SendStr("OK\r\n");


}


void get_word_argument_value(uint8_t* var_adress, uint16_t min_val, uint16_t max_val)
{

bool param_not_empty = false;
volatile char ch_tmp;
volatile uint16_t result = 0;

while(usartRxBuf[rxBufHead] !='\r')

	{

		ch_tmp = usartRxBuf[rxBufHead++];
		if (ch_tmp==' ') continue; //игнорируем пробелы
		if ((ch_tmp >= '0')&&(ch_tmp<= '9'))
		{
			result = result*10+char_to_int(ch_tmp);
			param_not_empty = true;
		}		
		else 
		{
			USART_SendStrP(parameter_invalid_error);
			return;
			
			//USART_SendStr("ERROR\n\r");
			//return invalid;//недопустимый аргумент

		}
	}
if (!param_not_empty) 
	{
		USART_SendStrP(parameter_empty_error);
		return;//пустой аргумент
	}

if ((result>max_val)||(result<min_val))
	{
		USART_SendStrP(parameter_out_of_range_error);
		return;//аргумент больше максимально допустимого значения
	} 

eeprom_write_word(var_adress, result);	
USART_SendStr("OK\r\n");


}






void command_0_slot(void){//bullets_in_clip=
get_int_argument_value(&eeprom_bullets_in_clip, 0, MAX_BULL_IN_CLIP);
/*
bool param_not_empty = false;
volatile char ch_tmp;
volatile uint8_t result = 0;
while(usartRxBuf[rxBufHead] !='\r')

	{

		ch_tmp = usartRxBuf[rxBufHead++];
		if (ch_tmp==' ') continue; //игнорируем пробелы
		if ((ch_tmp >= '0')&&(ch_tmp<= '9'))
		{
			result = result*10+char_to_int(ch_tmp);
			param_not_empty = true;
		}		
		else 
		{
			USART_SendStr("ERROR\n\r");
			return;
		}
	}

if (!param_not_empty) 
{
	USART_SendStrP(parameter_empty_error);
	return;
}
//lcd_gotoxy(7, 0);
//lcd_puts(int_to_str(result,0));
eeprom_write_byte(&eeprom_bullets_in_clip, result);	
USART_SendStr("OK\r\n");
*/
}




void command_1_slot(void){
USART_SendStr(int_to_str(eeprom_read_byte(&eeprom_bullets_in_clip),0));
USART_SendStr("\r\nOK\r\n");





}
void command_2_slot(void){
USART_FlushTxBuf();
USART_SendStrP(protocol);
USART_SendStr("OK\r\n");

}
uint8_t char_to_int(char c)
     {
       switch(c)
       {
         case '0': return 0;
         case '1': return 1;
         case '2': return 2;
         case '3': return 3;
         case '4': return 4;
         case '5': return 5;
         case '6': return 6;
         case '7': return 7;
         case '8': return 8;
         case '9': return 9;
         default : return 0x55;
         
       }
     }


void command_3_slot(void){
bool param_not_empty = false;
volatile char ch_tmp;
volatile uint8_t result = 0;
while(usartRxBuf[rxBufHead] !='\r')

	{

		ch_tmp = usartRxBuf[rxBufHead++];
		if (ch_tmp==' ') continue; //игнорируем пробелы
		if ((ch_tmp >= '0')&&(ch_tmp<= '9'))
		{
			result = result*10+char_to_int(ch_tmp);
			param_not_empty = true;
		}		
		else 
		{
			USART_SendStr("ERROR\n\r");
			return;
		}
	}

if (!param_not_empty) 
{
	USART_SendStrP(parameter_empty_error);
	return;
}
//lcd_gotoxy(7, 0);
//lcd_puts(int_to_str(result,0));
eeprom_write_byte(&eeprom_clips, result);	
USART_SendStr("OK\r\n");
}

void command_4_slot(void){
USART_SendStr(int_to_str(eeprom_read_byte(&eeprom_clips),0));
USART_SendStr("\r\nOK\r\n");
}


void command_5_slot(void){
bool param_not_empty = false;
volatile char ch_tmp;
volatile uint16_t result = 0;
while(usartRxBuf[rxBufHead] !='\r')

	{

		ch_tmp = usartRxBuf[rxBufHead++];
		if (ch_tmp==' ') continue; //игнорируем пробелы
		if ((ch_tmp >= '0')&&(ch_tmp<= '9'))
		{
			result = result*10+char_to_int(ch_tmp);
			param_not_empty = true;
		}		
		else 
		{
			USART_SendStr("ERROR\n\r");
			return;
		}
	}

if (!param_not_empty) 
{
	USART_SendStrP(parameter_empty_error);
	return;
}
if (result>512) 
{
	USART_SendStrP(parameter_out_of_range_error);
	return;
}
USART_FlushRxBuf();
//uart_timer = 0;
//while (uart_timer < 40);
USART_SendStr("OK\r\n");

uart_timer = 0;
while ((uart_timer < 650)&&(rxCount<128)); //ждем приема 128 байт
if (rxCount>=128)
	{
		if(eeWriteBytes(&usartRxBuf[0], result*128, 128)) 
		{
//			uart_timer = 0;
//			while (uart_timer < 40);
			USART_SendStr("OK\r\n");
		}
		else 
		{
			USART_SendStr("ERROR:eeprom write error\r\n");
		}
	} 
else 
	{
		USART_SendStr("ERROR\r\n");
	}




}

void command_6_slot(void){
bool param_not_empty = false;
volatile char ch_tmp;
volatile uint16_t result = 0;
while(usartRxBuf[rxBufHead] !='\r')

	{

		ch_tmp = usartRxBuf[rxBufHead++];
		if (ch_tmp==' ') continue; //игнорируем пробелы
		if ((ch_tmp >= '0')&&(ch_tmp<= '9'))
		{
			result = result*10+char_to_int(ch_tmp);
			param_not_empty = true;
		}		
		else 
		{
			USART_SendStr("ERROR\n\r");
			return;
		}
	}

if (!param_not_empty) 
{
	USART_SendStrP(parameter_empty_error);
	return;
}
if (result>512) 
{
	USART_SendStrP(parameter_out_of_range_error);
	return;
}
USART_FlushTxBuf();
	if (eeReadBytes(&usartTxBuf[0],result*128,128))
		{
	
			for (int i = 0; i<128; i++)
			{
				USART_PutChar(usartTxBuf[i]);

			}
//				USART_SendStr("OK\r\n");

				/*
				txCount   = 127;
				
				if(((UCSRA & (1<<UDRE)) != 0)) 
  				{
  					UDR = usartTxBuf[0];
					while (txCount); //пока все не отправим
					USART_SendStr("OK\r\n");
  				}
  				else
				{
					
				}
				*/

	
		}
	else
		{
				USART_SendStr("ERROR:eeprom read error\r\n");
		}

}











uint16_t str_to_int()
{
	
}

void apply_sound_get_command(uint16_t* var_adress) {
USART_SendStr(long_int_to_str(eeprom_read_word(var_adress),0));
USART_SendStr("\r\nOK\r\n");
}



void apply_sound_set_command(uint16_t* var_adress) {
bool param_not_empty = false;
volatile char ch_tmp;
volatile uint16_t result = 0;
while(usartRxBuf[rxBufHead] !='\r')

	{

		ch_tmp = usartRxBuf[rxBufHead++];
		if (ch_tmp==' ') continue; //игнорируем пробелы
		if ((ch_tmp >= '0')&&(ch_tmp<= '9'))
		{
			result = result*10+char_to_int(ch_tmp);
			param_not_empty = true;
		}		
		else 
		{
			USART_SendStr("ERROR\n\r");
			return;
		}
	}

if (!param_not_empty) 
{
	USART_SendStrP(parameter_empty_error);
	return;
}
//lcd_gotoxy(7, 0);
//lcd_puts(int_to_str(result,0));
eeprom_write_word(var_adress, result);	
USART_SendStr("OK\r\n");
}

void command_7_slot(void){//"sound_1_adress=";
apply_sound_set_command(&sound_1_adress);
}


void command_8_slot(void){//"sound_1_adress?";
apply_sound_get_command(&sound_1_adress);
}


void command_9_slot(void){//"sound_1_size="
apply_sound_set_command(&sound_1_size);
}



void command_10_slot(void){//"sound_1_size?"
apply_sound_get_command(&sound_1_size);
}


void command_11_slot(void){//"sound_2_adress="
apply_sound_set_command(&sound_2_adress);
}

void command_12_slot(void){//"sound_2_adress?"
apply_sound_get_command(&sound_2_adress);
}


void command_13_slot(void){//"sound_2_size="
apply_sound_set_command(&sound_2_size);


}

void command_14_slot(void){//"sound_2_size?";
apply_sound_get_command(&sound_2_size);
}


void command_15_slot(void){//"sound_3_adress="
apply_sound_set_command(&sound_3_adress);
}

void command_16_slot(void){//"sound_3_adress?"
apply_sound_get_command(&sound_3_adress);
}


void command_17_slot(void){//"sound_3_size="
apply_sound_set_command(&sound_3_size);


}

void command_18_slot(void){//"sound_3_size?"
apply_sound_get_command(&sound_3_size);
}

void command_19_slot(void){//"sound_4_adress="
apply_sound_set_command(&sound_4_adress);
}

void command_20_slot(void){//"sound_4_adress?"
apply_sound_get_command(&sound_4_adress);
}


void command_21_slot(void){//"sound_4_size=";
apply_sound_set_command(&sound_4_size);
}

void command_22_slot(void){//"sound_4_size?"
apply_sound_get_command(&sound_4_size);
}

void command_23_slot(void){//"sound_5_adress="
apply_sound_set_command(&sound_5_adress);
}

void command_24_slot(void){//"sound_5_adress?"
apply_sound_get_command(&sound_5_adress);
}


void command_25_slot(void){//"sound_5_size="
apply_sound_set_command(&sound_5_size);
}

void command_26_slot(void){//"sound_5_size?"
apply_sound_get_command(&sound_5_size);
}


void command_27_slot(void){//"sound_6_adress="
apply_sound_set_command(&sound_6_adress);
}

void command_28_slot(void){//"sound_6_adress?"
apply_sound_get_command(&sound_6_adress);
}


void command_29_slot(void){//"sound_6_size="
apply_sound_set_command(&sound_6_size);
}

void command_30_slot(void){//"sound_6_size?"
apply_sound_get_command(&sound_6_size);
}


void command_31_slot(void){//"play_sound"
bool param_not_empty = false;
volatile char ch_tmp;
volatile uint8_t result = 0;
while(usartRxBuf[rxBufHead] !='\r')

	{

		ch_tmp = usartRxBuf[rxBufHead++];
		if (ch_tmp==' ') continue; //игнорируем пробелы
		if ((ch_tmp >= '0')&&(ch_tmp<= '9'))
		{
			result = result*10+char_to_int(ch_tmp);
			param_not_empty = true;
		}		
		else 
		{
			USART_SendStr("ERROR\n\r");
			return;
		}
	}

if (!param_not_empty) 
{
	USART_SendStrP(parameter_empty_error);
	return;
}
if ((result > 10)||(result==0))
{
	USART_SendStrP(parameter_out_of_range_error);
	return;
}
//lcd_gotoxy(7, 0);
//lcd_puts(int_to_str(result,0));
switch (result)
	{
		case 1:
		{
			play_sound_1();
		}
		break;
		case 2:
		{
			play_sound_2();
		}
		break;
		case 3:
		{
			play_sound_3();
		}
		break;
		case 4:
		{
			play_sound_4();
		}
		break;
		case 5:
		{
			play_sound_5();
		}
		break;
		case 6:
		{
			play_sound_6();
		}
		break;
		case 7:
		{
			play_sound_7();
		}
		break;

		case 8:
		{
			play_sound_8();
		}
		break;
		
		case 9:
		{
			play_sound_9();
		}
		break;

		case 10:
		{
			play_sound_10();
		}
		break;

		default:
		break;
	}



	
USART_SendStr("OK\r\n");
}




void command_32_slot(void){//"play_shot_sound"
play_sound_1();
//playhitsound();
//play_shot_sound();
USART_SendStr("OK\r\n");
}


void command_33_slot(void){//player_id=
get_int_argument_value(&eeprom_player_id, 0, 127);


}


void command_34_slot(void){//player_id?
USART_SendStr(int_to_str(eeprom_read_byte(&eeprom_player_id),0));
USART_SendStr("\r\nOK\r\n");

}

void command_35_slot(void){//damage=
get_int_argument_value(&eeprom_damage, 0, 15);


}


void command_36_slot(void){//damage?
USART_SendStr(int_to_str(eeprom_read_byte(&eeprom_damage),0));
USART_SendStr("\r\nOK\r\n");

}

void command_37_slot(void){//ir_power=
//get_int_argument_value(&eeprom_damage, 0, 15);
bool param_not_empty = false;
volatile char ch_tmp;
volatile uint8_t result = 0;

while(usartRxBuf[rxBufHead] !='\r')

	{

		ch_tmp = usartRxBuf[rxBufHead++];
		if (ch_tmp==' ') continue; //игнорируем пробелы
		if ((ch_tmp >= '0')&&(ch_tmp<= '9'))
		{
			result = result*10+char_to_int(ch_tmp);
			param_not_empty = true;
		}		
		else 
		{
			USART_SendStrP(parameter_invalid_error);
			return;
			
			//USART_SendStr("ERROR\n\r");
			//return invalid;//недопустимый аргумент

		}
	}
if (!param_not_empty) 
	{
		USART_SendStrP(parameter_empty_error);
		return;//пустой аргумент
	}

if ((result>1)||(result<0))
	{
		USART_SendStrP(parameter_out_of_range_error);
		return;//аргумент больше максимально допустимого значения
	} 

switch (result)
	{
		case 0: eeprom_write_byte(&eeprom_curr_ir_pin, IR_LED_LOW_POWER_PIN);	
		break;
		case 1:  eeprom_write_byte(&eeprom_curr_ir_pin, IR_LED_HIGH_POWER_PIN);
		break;
	}
		

USART_SendStrP(ok_string);


}


void command_38_slot(void){//ir_power?

switch (eeprom_read_byte(&eeprom_curr_ir_pin))
	{
		case IR_LED_LOW_POWER_PIN: USART_SendStr("0");
		break;
		case IR_LED_HIGH_POWER_PIN: USART_SendStr("1");
	} 

USART_SendStrP(ok_string);

}



void command_39_slot(void){//friendly_fire=
get_int_argument_value(&friendly_fire_enable, 0, 1);
}

void command_40_slot(void){//friendly_fire?
USART_SendStr(int_to_str(eeprom_read_byte(&friendly_fire_enable),0));
USART_SendStrP(ok_string);
}


void command_41_slot(void){//team_id=
get_int_argument_value(&eeprom_team_id, 0, 3);

}

void command_42_slot(void){//team_id?
USART_SendStr(int_to_str(eeprom_read_byte(&eeprom_team_id),0));
USART_SendStrP(ok_string);
}


void command_43_slot(void){//batt_full_voltage=
get_word_argument_value(&eeprom_batt_full_voltage, 3000, 45000);
}

void command_44_slot(void){//batt_full_voltage?
USART_SendStr(long_int_to_str(eeprom_read_word(&eeprom_batt_full_voltage),0));
USART_SendStrP(ok_string);
}

void command_45_slot(void){//batt_low_voltage=
get_word_argument_value(&eeprom_batt_low_voltage, 2700, 45000);
}

void command_46_slot(void){//batt_low_voltage?
USART_SendStr(long_int_to_str(eeprom_read_word(&eeprom_batt_low_voltage),0));
USART_SendStrP(ok_string);
}


void command_47_slot(void){//"sound_7_adress="
apply_sound_set_command(&sound_7_adress);
}

void command_48_slot(void){//"sound_7_adress?"
apply_sound_get_command(&sound_7_adress);
}


void command_49_slot(void){//"sound_7_size="
apply_sound_set_command(&sound_7_size);
}

void command_50_slot(void){//"sound_7_size?"
apply_sound_get_command(&sound_7_size);
}

void command_51_slot(void){//"sound_8_adress="
apply_sound_set_command(&sound_8_adress);
}

void command_52_slot(void){//"sound_8_adress?"
apply_sound_get_command(&sound_8_adress);
}


void command_53_slot(void){//"sound_8_size="
apply_sound_set_command(&sound_8_size);
}

void command_54_slot(void){//"sound_8_size?"
apply_sound_get_command(&sound_8_size);
}

void command_55_slot(void){//"sound_9_adress="
apply_sound_set_command(&sound_9_adress);
}

void command_56_slot(void){//"sound_9_adress?"
apply_sound_get_command(&sound_9_adress);
}


void command_57_slot(void){//"sound_9_size="
apply_sound_set_command(&sound_9_size);
}

void command_58_slot(void){//"sound_9_size?"
apply_sound_get_command(&sound_9_size);
}

void command_59_slot(void){//"sound_10_adress="
apply_sound_set_command(&sound_10_adress);
}

void command_60_slot(void){//"sound_10_adress?"
apply_sound_get_command(&sound_10_adress);
}


void command_61_slot(void){//"sound_10_size="
apply_sound_set_command(&sound_10_size);
}

void command_62_slot(void){//"sound_10_size?"
apply_sound_get_command(&sound_10_size);
}



void command_63_slot(void){//ir_carrier_frequency=
get_int_argument_value(&eeprom_ir_carr_freq, 0, 1);
get_ir_protocol();
reinit_timer2();
}

void command_64_slot(void){//ir_carrier_frequency?
USART_SendStr(int_to_str(eeprom_read_byte(&eeprom_ir_carr_freq),0));
USART_SendStr("\r\nOK\r\n");

}


void command_65_slot(void){//clear_tm

	eeprom_write_byte(&eeprom_tm_serial_num.device_code,0xFF);
						for (int i = 0; i<6; i++ )
						{
							eeprom_write_byte(&eeprom_tm_serial_num.serial[i],0xFF);
						}
						lcd_clrscr();
						lcd_home();
						lcd_puts("Ключ ТМ");
						lcd_gotoxy(0, 1);
						lcd_puts("удален!");
						USART_SendStr("\r\nOK\r\n");
						timer2 = 0;
						while (timer2 < 6000){};
						



}


void parsing_command(void)
{
uint8_t cmd_index;
					DisableRxInt();
					cmd_index = get_command_index();
					switch(cmd_index)
					{
						case 0: command_0_slot();
						break;
						case 1: command_1_slot();
						break;
						case 2: command_2_slot();
						break;
						case 3: command_3_slot();
						break;
						case 4: command_4_slot();
						break;
						case 5: command_5_slot();
						break;
						case 6: command_6_slot();
						break;
						case 7: command_7_slot();
						break;
						case 8: command_8_slot();
						break;
						case 9: command_9_slot();
						break;
						case 10: command_10_slot();
						break;
						case 11: command_11_slot();
						break;
						case 12: command_12_slot();
						break;
						case 13: command_13_slot();
						break;
						case 14: command_14_slot();
						break;
						case 15: command_15_slot();
						break;
						case 16: command_16_slot();
						break;
						case 17: command_17_slot();
						break;
						case 18: command_18_slot();
						break;
						case 19: command_19_slot();
						break;
						case 20: command_20_slot();
						break;
						case 21: command_21_slot();
						break;
						case 22: command_22_slot();
						break;
						case 23: command_23_slot();
						break;
						case 24: command_24_slot();
						break;
						case 25: command_25_slot();
						break;
						case 26: command_26_slot();
						break;
						case 27: command_27_slot();
						break;
						case 28: command_28_slot();
						break;
						case 29: command_29_slot();
						break;
						case 30: command_30_slot();
						break;
						case 31: command_31_slot();
						break;		
						case 32: command_32_slot();
						break;		
						case 33: command_33_slot();
						break;
						case 34: command_34_slot();
						break;
						case 35: command_35_slot();
						break;
						case 36: command_36_slot();
						break;
						case 37: command_37_slot();
						break;
						case 38: command_38_slot();
						break;
						case 39: command_39_slot();
						break;
						case 40: command_40_slot();
						break;
						case 41: command_41_slot();
						break;
						case 42: command_42_slot();
						break;
						case 43: command_43_slot();
						break;
						case 44: command_44_slot();
						break;	
						case 45: command_45_slot();
						break;
						case 46: command_46_slot();
						break;	
						case 47: command_47_slot();
						break;
						case 48: command_48_slot();
						break;
						case 49: command_49_slot();
						break;
						case 50: command_50_slot();
						break;
						case 51: command_51_slot();
						break;
						case 52: command_52_slot();
						break;
						case 53: command_53_slot();
						break;
						case 54: command_54_slot();
						break;
						case 55: command_55_slot();
						break;
						case 56: command_56_slot();
						break;
						case 57: command_57_slot();
						break;
						case 58: command_58_slot();
						break;
						case 59: command_59_slot();
						break;
						case 60: command_60_slot();
						break;
						case 61: command_61_slot();
						break;
						case 62: command_62_slot();
						break;
						case 63: command_63_slot();
						break;
						case 64: command_64_slot();
						break;
						case 65: command_65_slot();
						break;


														
						default:
						{
							USART_SendStrP(unknown_command_error);
						}

					}
				
					cr_received = false;
					USART_FlushRxBuf();


}



bool play_sound_from_eeprom(uint16_t address, uint16_t data_size) //воспроизводим звук непосредственно из eeprom

{ 

//	    uint8_t data; //Переменная, в которую запишем прочитанный байт
	 
	//Точно такой же кусок кода, как и в eeWriteByte...
	/*****УСТАНАВЛИВАЕМ СВЯЗЬ С ВЕДОМЫМ********/
	    do
	    {
	        TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	        while(!(TWCR & (1<<TWINT)));
	 
	        if((TWSR & 0xF8) != TW_START)
	            return false;
	 
	        TWDR = (slaveAddressConst<<4) + (slaveAddressVar<<1) + WRITEFLAG;
	        TWCR=(1<<TWINT)|(1<<TWEN);
	 
	        while(!(TWCR & (1<<TWINT)));
	 
	    }while((TWSR & 0xF8) != TW_MT_SLA_ACK);
	 
	/*****ПЕРЕДАЕМ АДРЕС ЧТЕНИЯ********/
	    TWDR=(address>>8);
	    TWCR=(1<<TWINT)|(1<<TWEN);
	    while(!(TWCR & (1<<TWINT)));
	 
	    if((TWSR & 0xF8) != TW_MT_DATA_ACK)
	        return false;
	 
	    TWDR=(address);
	    TWCR=(1<<TWINT)|(1<<TWEN);
	    while(!(TWCR & (1<<TWINT)));
	 
	    if((TWSR & 0xF8) != TW_MT_DATA_ACK)
	        return false;
	 
	/*****ПЕРЕХОД В РЕЖИМ ЧТЕНИЯ********/
	/*Необходимо опять «связаться» с ведомым, т.к. ранее мы отсылали адресный пакет (slaveAddressConst<<4) + (slaveAddressVar<<1) + WRITEFLAG, чтобы записать адрес чтения байта данных. А теперь нужно перейти в режим чтения (мы же хотим прочитать байт данных), для этого отсылаем новый пакет (slaveAddressConst<<4) + (slaveAddressVar<<1) + READFLAG.*/
	 
	    //Повтор условия начала передачи
	    TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	//ждем выполнения текущей операции
	    while(!(TWCR & (1<<TWINT)));
	 
	/*Проверяем статус. Условие повтора начала передачи (0x10=TW_REP_START) должно подтвердиться*/
	    if((TWSR & 0xF8) != TW_REP_START)
	        return false;
	 
	    /*Записываем адрес ведомого (7 битов) и в конце бит чтения (1)*/
	    //TWDR=0b1010’000’1;
	    TWDR = (slaveAddressConst<<4) + (slaveAddressVar<<1) + READFLAG;        
	 
	//Отправляем..
	    TWCR=(1<<TWINT)|(1<<TWEN);
	    while(!(TWCR & (1<<TWINT)));
	 
	/*Проверяем, нашелся ли ведомый с адресом 1010’000 и готов ли он работать на чтение*/
	    if((TWSR & 0xF8) != TW_MR_SLA_ACK)
	        return false;
	 
	/*****СЧИТЫВАЕМ БАЙТ ДАННЫХ********/
	 for(uint16_t i=0; i < (data_size - 1); i++)//считаем все байты, кроме последнего
	{

	/*Начинаем прием данных с помощью очистки флага прерывания TWINT. Читаемый байт записывается в регистр TWDR.*/
	    TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	 
	    //Ждем окончания приема..
	    while(!(TWCR & (1<<TWINT)));
	 
	/*Проверяем статус. По протоколу, прием данных должен оканчиваться без подтверждения со стороны ведущего (TW_MR_DATA_NACK = 0x58)*/
	    if((TWSR & 0xF8) != TW_MR_DATA_ACK)
	        return false;
	 
	    /*Присваиваем переменной data значение, считанное в регистр данных TWDR*/
	    while (fcurr_simple_prepared); //ждем, когда прерывание воспроизведет сеймл
		curr_simple = TWDR; //скармливаем прерыванию очередной сеймпл
		fcurr_simple_prepared = true;//сообщаем прерыванию, что очередной сейпл готов
//		*buffer = TWDR;
//		buffer++;
		//data=TWDR;
	}

	 /*Начинаем прием данных с помощью очистки флага прерывания TWINT. Читаемый байт записывается в регистр TWDR.*/
	    TWCR=(1<<TWINT)|(1<<TWEN);
	 
	    //Ждем окончания приема..
	    while(!(TWCR & (1<<TWINT)));
	 
	/*Проверяем статус. По протоколу, прием данных должен оканчиваться без подтверждения со стороны ведущего (TW_MR_DATA_NACK = 0x58)*/
	    if((TWSR & 0xF8) != TW_MR_DATA_NACK)
	        return false;
	 
	    /*Присваиваем переменной data значение, считанное в регистр данных TWDR*/
	    

		while (fcurr_simple_prepared); //ждем, когда прерывание воспроизведет сеймл
		curr_simple = TWDR; //скармливаем прерыванию очередной сеймпл
		fcurr_simple_prepared = true;//сообщаем прерыванию, что очередной сейпл готов
//		*buffer = TWDR;


	    /*Устанавливаем условие завершения передачи данных (СТОП)*/
	    TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	 
	    //Ждем установки условия СТОП
	    while(TWCR & (1<<TWSTO));
	 
    //Возвращаем считанный байт
    OCR0 = 0;
	curr_simple=0;
	fcurr_simple_prepared=false;
	return true;
}

void play_sound_1(){//воспроизводим звук 1
play_sound_from_eeprom(eeprom_read_word(&sound_1_adress),eeprom_read_word(&sound_1_size));
}
void play_sound_2(){//воспроизводим звук 2
play_sound_from_eeprom(eeprom_read_word(&sound_2_adress),eeprom_read_word(&sound_2_size));
}

void play_sound_3(){//воспроизводим звук 3
play_sound_from_eeprom(eeprom_read_word(&sound_3_adress),eeprom_read_word(&sound_3_size));
}
void play_sound_4(){//воспроизводим звук 4
play_sound_from_eeprom(eeprom_read_word(&sound_4_adress),eeprom_read_word(&sound_4_size));
}
void play_sound_5(){//воспроизводим звук 5
play_sound_from_eeprom(eeprom_read_word(&sound_5_adress),eeprom_read_word(&sound_5_size));
}

void play_sound_6(){//воспроизводим звук 6
play_sound_from_eeprom(eeprom_read_word(&sound_6_adress),eeprom_read_word(&sound_6_size));
}

void play_sound_7(){//воспроизводим звук 7
play_sound_from_eeprom(eeprom_read_word(&sound_7_adress),eeprom_read_word(&sound_7_size));
}

void play_sound_8(){//воспроизводим звук 8
play_sound_from_eeprom(eeprom_read_word(&sound_8_adress),eeprom_read_word(&sound_8_size));
}

void play_sound_9(){//воспроизводим звук 6
play_sound_from_eeprom(eeprom_read_word(&sound_9_adress),eeprom_read_word(&sound_9_size));
}

void play_sound_10(){//воспроизводим звук 6
play_sound_from_eeprom(eeprom_read_word(&sound_10_adress),eeprom_read_word(&sound_10_size));
}

/*
void sound_buffer_update(){//подкинем в звуковой буфер новую партию сейплов
switch (curr_sound_buffer)
	{
		case sound_buffer_1://сейчас сеймплы считываются с первого звукового буфера
		{
			eeReadBytes(&usartTxBuf[0],curr_adress_in_eeprom,256);

		}
		break;
		case sound_buffer_2://сейчас сеймплы считываются со второго звукового буфера
		{
			eeReadBytes(&usartRxBuf[0],curr_adress_in_eeprom,256);

		}
		break;
	}

curr_adress_in_eeprom =curr_adress_in_eeprom + 256; 
update_suond_buffer_now = false;//событие обработали, сбросим флаг	
}
*/



/*

void play_shot_sound(void){
curr_pos_in_sound_buff=0;
curr_sound_buffer=sound_buffer_1;
update_suond_buffer_now = false;
curr_adress_in_eeprom = eeprom_read_word(&sound_1_adress);
eeReadBytes(&usartRxBuf[0],curr_adress_in_eeprom,256);//заполняем буфер 1
curr_adress_in_eeprom = curr_adress_in_eeprom + 256; 
eeReadBytes(&usartTxBuf[0],curr_adress_in_eeprom,256);//заполняем буфер 2
curr_adress_in_eeprom = curr_adress_in_eeprom + 256; 
simples_in_queue = eeprom_read_word(&sound_1_size);//погнали :-)
};

*/


/*****************************************
* Извлекаем данние из пакета, принятого по блютус
******************************************/

trx_event parsing_bt_data(void) //анализируем пакет, полученный по блютус, извекаем из него данные
{
	volatile unsigned char tmp_char;
	trx_event result;

	result = NOT_EVENT;
	
	while (rxCount)//пока не считаем все символы из буфера
	{
		tmp_char = USART_GetChar();
		switch(tmp_char)
		{
			case 'h'://нашли заголовок пакета
			{
				bt_header_received = true; //фиксируем получение заголовка
				bit_in_bt_rx_buff=0;//ставим курсор в начало буфера пакета
			}
			break;
			case '0'://получен бит "0"
			{
				if(bt_header_received) set_bt_buffer_bit(bit_in_bt_rx_buff++, false);//если заголовок получен, заносим значение бита в буфер пакета
			}
			break;
			case '1'://получен бит "1"
			{
				if(bt_header_received) set_bt_buffer_bit(bit_in_bt_rx_buff++, true);//если заголовок получен, заносим значение бита в буфер пакета
			}
			break;
			case 'e'://пакет битый
			{
				bt_header_received = false;//нужно искать следующий заголовок
				return RX_ERROR;//выходим с ошибкой
			}
			break;
			case 't'://таймаут приема бита 
			{
				
				if((bt_header_received)&&(bit_in_bt_rx_buff>0))//если заголовок получен и буфер пакета не пустой
				{				
						
						
						
					switch(bit_in_bt_rx_buff)//проверим, сколько бит принято
					{
						case 14:
						{
							result = RX_COMPLETE;			//Генерим событие "принят пакет"
							break;	
						}
						case 24:
						{
							result	= RX_MESSAGE_COMPLITE;//принято сообщение;
							break;	
						}
						default:
						{
							result = RX_ERROR;			//генерируем событие - "ошибка приёма"
						}
					}						

						bt_header_received = false;//заголовок обработали
				//		return RX_COMPLETE;//выходим с сообщением о получении пакета
				}
				else 	
				{
					bt_header_received = false;//заголовок обработали
				}
			}
			break;

		}



	}

//всё считали, собйтий нет
return result;

/*	
	
	while ((tmp_char!='h')&&(tmp_char!='\r')){tmp_char = USART_GetChar();}// ищем заголовок
	switch(tmp_char)
	{
		case '\r': //нет заголовка в принятом пакете
		{
			return false;
		}
		break;
		case 'h': //нашли заголовок
		{
			bit_in_bt_rx_buff=0; //курсор на начало буфера
			while (rxCount)
			{
				tmp_char = USART_GetChar();
				switch (tmp_char) 
				{
					case '0': 	set_bt_buffer_bit(bit_in_bt_rx_buff++, false);
					break;
					case '1': 	set_bt_buffer_bit(bit_in_bt_rx_buff++, true);
					break;
					case '\r': return true;
					default: return false; //ошибка према бита или что то левое
				}


			}
		//	return true; //данные успешно преобразованы и помещены в буфер пакета
		}
	}
*/


}



/****************************************
* обрабатываем попадание
*****************************************/



void hit_processing(trx_packet hit_packet)//обрабатываем попадание
{

if ((hit_packet.team_id != team_id())||(eeprom_read_byte(&friendly_fire_enable)&&(hit_packet.player_id != eeprom_read_byte(&eeprom_player_id))))//"пуля" прилетела от игрока другой, не нашей, команды
								{
									WOUND_LED_ON; //включаем вспомогательный светодиод						
									USART_SendStr("111");
									lcd_bl_on();
									display_hit_data(hit_packet);
								//	playhitsound();

					//				WOUND_LED_OFF;





								if (life_in_percent > hit_packet.damage) 
									{
										life_in_percent = life_in_percent-hit_packet.damage;
										life = (life_in_percent*10)/125;
										if ((life==0)&&(life_in_percent>0)) life=1;
										playhitsound();
										WOUND_LED_OFF;
										keyboard_event=no_key_pressing; 
									}
									else 
									{	
										life = 0;
										life_in_percent=0;
										WOUND_LED_ON;
										display_life(life);//отобразим уровень жизни на диодах
										display_life_update();//отобразим уровень жизни на ЖКИ
										volatile uint8_t keypress_cntr; //счетчик циклов, в течении которых курок был нажат
										keypress_cntr = 0;
										playgameoversound();
										
										
										
										if ((eeprom_read_byte(&eeprom_tm_serial_num.device_code)!=0)&&(eeprom_read_byte(&eeprom_tm_serial_num.device_code)!=0xFF))

										/*если ТМ ключ уже занесён в память*/
										{

											joystick_event=no_pressing;
											keyboard_event=no_key_pressing;
											tm_event=no_tm_event;
											uint8_t tm_valide;
											tm_valide=0;
											lcd_bl_off();
											
											
											while (!tm_valide)
											{//[while]
												lcd_clrscr();
												lcd_home();
												lcd_puts("Для активации");
												lcd_gotoxy(0, 1);
												lcd_puts("приложите ключ");
												while (tm_event == no_tm_event)
												{
													WOUND_LED_INVERT;
													USART_PutChar('1');
													timer2 = 0;
													while (timer2 < 1000);
													WOUND_LED_INVERT;
													timer2 = 0;
													while (timer2 < 1000);	
												
												};
												switch(tm_event)
												{//[switch]
													case no_tm_event: 
													{
															
													}
													break;
					
													case tm_crc_error: 
													{
														lcd_clrscr();
														lcd_home();
														lcd_puts("Ошибка CRC");
														timer2 = 0;
														while (timer2 < 6000){};
														tm_event=no_tm_event;
													}
													break;

													case tm_crc_ok: 
													{
					
														if (tm_verification()) 	
														{ 	tm_valide=1;
															lcd_clrscr();
															lcd_home();
															lcd_puts("Удачи!");
															timer2 = 0;
															while (timer2 < 6000){};
				
															tm_event=no_tm_event;
															break;
														}	
														lcd_clrscr();
														lcd_home();
														lcd_puts("Не тот ключ");
														timer2 = 0;
														while (timer2 < 6000){};
													/*
														lcd_clrscr();
														lcd_home();
														lcd_puts("Для активации");
														lcd_gotoxy(0, 1);
														lcd_puts("приложи ключ");
													*/
														tm_event=no_tm_event;
														
																		
													}

														break;
				
												}//[/switch]
											}//[while]
										
										}//end if
									
									else //тач мемори не записан в память
									{ 
										
										lcd_bl_off();
										display_status();
										rx_event = rx_event = NOT_EVENT;
										ir_message.control_byte = 0;

										while(!((ir_message.control_byte ==Valid_value)&&(ir_message.ID==Command)&&(ir_message.param==0x05)))//пока не получим команду "старт"
										{
											rx_event = rx_event = NOT_EVENT;
											while((rxCount==0)&&(rx_event != RX_MESSAGE_COMPLITE))//пока нет никаких команд пульта
											{
												WOUND_LED_INVERT;
												USART_PutChar('1');
												timer2 = 0;
												while (timer2 < 1000);
												WOUND_LED_INVERT;
												timer2 = 0;
												while (timer2 < 1000);
											}
										
									
										if (rx_event == RX_MESSAGE_COMPLITE) ir_message = get_ir_message_from_buffer();//пришла команда пульта, выдергиваем значения команды из буфера ИК приемника	
										else {
												
													if (rxCount>0) finde_message_in_bt_buffer();
												
												
											  }	
											
											
											
												
										}




								/*
										while(keypress_cntr < 20)
										{
											WOUND_LED_INVERT;
											USART_PutChar('1');
											timer2 = 0;
											while (timer2 < 1000);
											WOUND_LED_INVERT;
											timer2 = 0;
											while (timer2 < 1000);
											switch (FIRE_KEY_IN&FIRE_KEY_PIN) //проверяем, нажат ли курок
											{
												case 0:  keypress_cntr++ ; break;
												case FIRE_KEY_PIN: keypress_cntr = 0; break;
												default: keypress_cntr = 0;	
											}
						
										}
									*/
									}	
									//"оживаем" - начинаем новую игру
										
										if (simples_in_queue>1) //если звук выстрла воспроизводится
										{
											simples_in_queue=1;//закроем eeprom
											while (eeprom_is_open);//дождемся, пока eerom закроется
										}
										init_var(); //инициализируем переменные
													joystick_event=no_pressing; //очищаем события джойстика
													keyboard_event=no_key_pressing;//очищаем события триггера
													reload_key_event=no_key_pressing;//очищаем события перезарядки
													rx_event = NOT_EVENT;   //очищаем события ИК приемника
													display_status();//обновляем информацию на дисплее
													
													WOUND_LED_ON;
													playstartsound();
													//код обработки дополнительной команды
                                 					WOUND_LED_OFF;
									/*	
										WOUND_LED_OFF;
										init_var();//"оживаем" - начинаем новую игру
										joystick_event=no_pressing;
										keyboard_event=no_key_pressing;
										tm_event=no_tm_event;
									*/
									//	display_status();
									}
								
								display_life(life);//отобразим уровень жизни на диодах
//								display_life_update();//отобразим уровень жизни на ЖКИ
								lcd_bl_off();
								display_status();
								}



}



void ir_tx_cursor_home(void){//устанавливаем курсор в начало буфера
ir_tx_buffer_cursor.byte_pos = 0;
ir_tx_buffer_cursor.bit_mask = (1<<7);
ir_pulse_counter = IR_START; //передадим заголовоке
ir_space_counter = IR_SPACE;// и заголовок
}




tir_message get_ir_message_from_buffer(void) //извлекаем из буфера ИК приемника полученное сообщение 
{
	tir_message msg_tmp;
	msg_tmp.ID = rx_buffer[0];//имя функции в первом принятом байте (индекс 0)
	msg_tmp.param = rx_buffer[1];//параметры функции во втором принятом байте (индекс 1)
	msg_tmp.control_byte = rx_buffer[2];//контрольный байт
	return msg_tmp;

}



tir_message get_ir_message_from_bt_buffer(void) //извлекаем из буфера ИК приемника полученное сообщение 
{
	tir_message msg_tmp;
	msg_tmp.ID = bt_rx_buffer[0];//имя функции в первом принятом байте (индекс 0)
	msg_tmp.param = bt_rx_buffer[1];//параметры функции во втором принятом байте (индекс 1)
	msg_tmp.control_byte = bt_rx_buffer[2];//контрольный байт
	return msg_tmp;

}




void get_ir_protocol(void){

switch (eeprom_read_byte(&eeprom_ir_carr_freq))
	{
		case freq_36KHz: 
		{
			miles_protocol.carrier_frequency = 36000;
			miles_protocol.err_tolerance = ERROR_TOLERANCE_FOR_36KHZ;
			
		}
		break;
		case freq_56KHz:
		{
			miles_protocol.carrier_frequency = 56000;
			miles_protocol.err_tolerance = ERROR_TOLERANCE_FOR_56KHZ;

		}
		break;
		default: 
		{
			miles_protocol.carrier_frequency = DEFAULT_IR_F0; //дефалтовое значение
			miles_protocol.err_tolerance = DEFAULT_ERROR_TOLERANCE;
		}
	}


//uint16_t f_tmp;
//f_tmp = miles_protocol.carrier_frequency/1000;

miles_protocol.ir_zero = ((IR_ZERO_BIT_DURATION/100)*(miles_protocol.carrier_frequency/1000))/5;
miles_protocol.ir_one = ((IR_ONE_BIT_DURATION/100)*(miles_protocol.carrier_frequency/1000))/5;
miles_protocol.ir_start = ((IR_START_BIT_DURATION/100)*(miles_protocol.carrier_frequency/1000))/5;
miles_protocol.ir_space = ((IR_SPACE_DURATION/100)*(miles_protocol.carrier_frequency/1000))/5;

}






void test_bt_data()
{
		
		switch(parsing_bt_data())//проверим роиемный буфер
		{
			case RX_COMPLETE: 	//получен пакет
			{
				if(!get_bt_buffer_bit(0)) //если этот бит равен 0, то это пакет с данными (выстрел)
				{
					bt_rx_packet = get_bt_packet_value();
					hit_processing(bt_rx_packet);
					USART_FlushRxBuf();
					bt_header_received=false;
				}

			}
			break;
			case RX_ERROR:		//ошибка приема
			{
				    if((!ir_error_ignore)&&(!eeprom_is_open))	
					{
						play_sound_8();
						keyboard_event=no_key_pressing; 
					}
			}
			break;
			case RX_MESSAGE_COMPLITE://принято сообщение
						{
					//		rx_event = NOT_EVENT;							
							if(get_bt_buffer_bit(0)) //если этот бит равен 1, то это пакет с командой
                        	{
                           		ir_message = get_ir_message_from_bt_buffer();//выдергиваем значения команды из буфера ИК приемника
                        
                           	 if (ir_message.control_byte ==Valid_value )//сообщение принято корректно (контрольный байт принят без ошибок)
							 {
								switch(ir_message.ID)//если имя команды
                           		{
                              		case Add_Health: //добавить "жизни"
                              		{
										//код для добавления жизни
                                 		break;
                              		}
                              		case Add_Rounds://добавить "патронов"
                              		{
                                 
								 		//код для добавления патронов
                                 		break;
                              		}

                              		case Command://какая то дополнительноя команда
                              		{
                                    	
										switch(ir_message.param)//выясним, какая это командв
										{
											case 0x05://начать новую игру немедленно
											{
													if (simples_in_queue>1) //если звук выстрла воспроизводится
													{
														simples_in_queue=1;//закроем eeprom
														while (eeprom_is_open);//дождемся, пока eerom закроется
													}
													
													init_var(); //инициализируем переменные
													joystick_event=no_pressing; //очищаем события джойстика
													keyboard_event=no_key_pressing;//очищаем события триггера
													reload_key_event=no_key_pressing;//очищаем события перезарядки
													rx_event = NOT_EVENT;   //очищаем события ИК приемника
													display_status();//обновляем информацию на дисплее
													display_life(life);//отобразим уровень жизни на диодах
													WOUND_LED_ON;
													playstartsound();//Воспроизводим звук "старт игры"
													//код обработки дополнительной команды
                                 					WOUND_LED_OFF;
												
												break;
											}
											case 0x00://"выключить" игрока 
											{
												
												
												break;
												
											}
											default: break;
										
										}
										
									
										break;
                              		}
                           		}
                        	 }

                        	}
							else//контрольный байт сообщения не корректный - ошибка приема
							{
							}
							
							
							
							
							rx_event = NOT_EVENT;
							break;

						}
						
			
			
			
			
			break;
			case NOT_EVENT:		//нет ничего интересного ;-)
			{
			}
			break;
		}
}	
	
void finde_message_in_bt_buffer(void)//ищем сообщение в приемном буфере блютус
{
		
		switch(parsing_bt_data())//проверим роиемный буфер
		{
			case RX_COMPLETE: 	//получен пакет
			{
			}
			break;
			case RX_ERROR:		//ошибка приема
			{
			}
			
			case RX_MESSAGE_COMPLITE://принято сообщение
			{
					//		rx_event = NOT_EVENT;							
							if(get_bt_buffer_bit(0)) //если этот бит равен 1, то это пакет с командой
                        	{
                           		ir_message = get_ir_message_from_bt_buffer();//выдергиваем значения команды из буфера ИК приемника	
								rx_event = NOT_EVENT;
								break;

							}
						
			}
			
			
			
			break;
			case NOT_EVENT:		//нет ничего интересного ;-)
			{
			}
			break;
		}
}	
	
