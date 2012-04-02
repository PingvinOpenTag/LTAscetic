#include "ltag_ascetic.h"
#include "hitraw.h"

int main (void) {
configuring_ports();

init_timer2();
init_int0();
init_timer1();
init_timer0();
init_var();                		//Задаём значения переменным

sei(); 

life_leds_status[0] = ON;
timer1 = 0;
while (timer1 < 10000);

life_leds_status[1] = ON;
timer1 = 0;
while (timer1 < 10000);

//LIFE_LED3_ON;
life_leds_status[2] = ON;
timer1 = 0;
while (timer1 < 10000);

//LIFE_LED4_ON;
life_leds_status[3] = ON;
timer1 = 0;
while (timer1 < 10000);

//FIRE_LED_ON;
fire_led_status = ON;
timer1 = 0;
while (timer1 < 10000);

BULLETS_OUT_LED_ON;
timer1 = 0;
while (timer1 < 10000);

WOUND_LED_ON;
timer1 = 0;
while (timer1 < 10000);


//LIFE_LED1_OFF;
life_leds_status[0] = OFF;
timer1 = 0;
while (timer1 < 10000);

//LIFE_LED2_OFF;
life_leds_status[1] = OFF;
timer1 = 0;
while (timer1 < 10000);

//LIFE_LED3_OFF;
life_leds_status[2] = OFF;
timer1 = 0;
while (timer1 < 10000);

//LIFE_LED4_OFF;
life_leds_status[3] = OFF;
timer1 = 0;
while (timer1 < 10000);

//FIRE_LED_OFF;
fire_led_status = OFF;
timer1 = 0;
while (timer1 < 10000);

BULLETS_OUT_LED_OFF;
timer1 = 0;
while (timer1 < 10000);

WOUND_LED_OFF;

//life_led1_status = FQR_1HZ;
//life_led2_status = FQR_2HZ;
//life_led3_status = FQR_4HZ;
//life_led4_status = ON;

display_life(life);
beep(1000, 3, 128);

while(1){

switch(keyboard_event)
	{
	 	case no_key_pressing: break;
		case key_pressing:
		{
			if (bullets > 0)
			{
				bullets--;//уменьшаем на 1 количество патронов
				last_simple = 0;//воспроизводим звук выстрела
				send_ir_package();		//Производим "выстрел"
				
	 		}
			keyboard_event=no_key_pressing; 
		} 
        break;
  

		default:keyboard_event=no_key_pressing;	
	}

//while(rx_event==NOT_EVENT){};

//	send_ir_package();		//Производим "выстрел"
			//_delay_ms(1000);    	// секундная задержка
////	timer1=0;
////	while((rx_event==NOT_EVENT)&&(timer1 < 65000)){}; //ждем, пока не получим события или закончится таймаут
////	if(rx_event)// если есть события приемника


//if (rx_event != NOT_EVENT) 


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
								volatile int gg;
								gg++;

								if (rx_packet.team_id != team_id())//"пуля" прилетела от игрока другой, не нашей, команды
								{
									WOUND_LED_ON; //включаем вспомогательный светодиод						

									playhitsound();

									WOUND_LED_OFF;


//									damage_beep();
									uint8_t damage_tmp;
									switch(rx_packet.damage)
									{
										case 10:
										{
											damage_tmp = 1;
											break;
										}

										case 25:
										{
											damage_tmp = 2;
											break;
										}
																			
										case 50:
										{
											damage_tmp = 4;
											break;
										}
										case 100:
										{
											damage_tmp = 8;
											break;
										}
										default: damage_tmp = 1;
									}

									
								if (life > damage_tmp) life = life-damage_tmp;
									else 
									{	
										life = 0;
										WOUND_LED_ON;
										display_life(life);//отобразим уровень жизни на диодах
										volatile uint8_t keypress_cntr; //счетчик циклов, в течении которых курок был нажат
										keypress_cntr = 0;
										while(keypress_cntr < 20)
										{
											WOUND_LED_INVERT;
											timer1 = 0;
											while (timer1 < 10000);
											switch (FIRE_KEY_IN&FIRE_KEY_PIN) //проверяем, нажат ли курок
											{
												case 0:  keypress_cntr++ ; break;
												case FIRE_KEY_PIN: keypress_cntr = 0; break;
												default: keypress_cntr = 0;	
											}
						
										}

										init_var();//"оживаем" - начинаем новую игру
									}
								
								display_life(life);//отобразим уровень жизни на диодах

								
								}

									
								



	//							rx_player_id = rx_buffer[0];
	//							rx_damage = get_damage_from_rx_paket();

							

							}
							
						
						
						//	sei();
							break;
						}
						
						case RX_ERROR:		//ошибка приема
						{
						//	cli();
							BULLETS_OUT_LED_ON;
							timer1=0;
							while(timer1 < 35000);
							BULLETS_OUT_LED_OFF;
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

//	timer1=0;				//Сделаем паузу
//	while(timer1 < 65000);	//Подождем, пока не произойдет 65000 прерываний таймера (чуть меньше секунды)



};
return 0;

}




void configuring_ports(){
IR_LED_DDR |= IR_LED_PIN; //ножку, на которой подключен ИК диот переводим в режим "выход"
LIFE_LEDS_DDR |= LIFE_LED1_PIN  //ножки, на которых
				| LIFE_LED2_PIN //подключены светодиоды,
				| LIFE_LED3_PIN //отображающие уровень "жизни"
				| LIFE_LED4_PIN;//тоже настраиваем как выходы

SOUND_DDR |= SOUND_PIN; //настраиваем выход ШИМ (АЦП)
BULLETS_OUT_LED_DDR|=BULLETS_OUT_LED_PIN;
//DDRA |= (1 << 4)|(1<<5)|(1<<6)|(1<<7); // Устанавливаем порт PORTA.1 как выход                                                                                                                                                                    DDRB = 1<<DDB3;						//PB3 (OC0) set as output
//DDRD |= (1 << 7);//светодиод на повязке
DDRB |= (1 << 2);// отключение усилителя
PORTB &=~(1 << 2);
//PORTD &= ~(1 << 7);//выключаем светодиод на повязке
WOUND_LED_DDR |= WOUND_LED_PIN;
TSOP_DDR &=~TSOP_PIN; //вывод, к которому подключен ИК-датчик настраиваем как "вход"

}


/**************************************************************************************
* Функция выполняет настройку таймера timer2
* Режим работы таймер - СТС (сброс при совпадении)
* тактирование - без делителя, с частотой кварца
* регист сравнения будет иметь такое значение, чтобы прерывания
* генерировались с удвоенной частотой несущей ИК-приемника 
***************************************************************************************/


void init_timer2(void){
OCR2 = F_CPU/IR_F0/2-1;       //Прерывания будут генерироваться с частотой, вдвое большей, чем частота несущей 
TCCR2 = _BV(CS20)|_BV(WGM21); // Режим работы таймер - СТС (сброс при совпадении)
                              // Тактирование с частотой резонатора (7 372 800 Гц)
TIMSK |= _BV(OCIE2);          // Разрешаем прерывания по захвату/сравнению
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
cursor_position = 0; 		//Курсор - на начало блока данных
ir_transmitter_on = true;	//Разрешаем передачу
//while (ir_transmitter_on);	//Ждем, пока пакет отправиться
}

/**************************************************************************************
* Установка идентификатора игрока
* в качестве аргумента функции указывается идентификационный номер игрока (от 1 до 127)
* в результате выполнения функции в глобальной переменной data_packet.player_id
* будут соответствующим образом инициированы  data_packet.player_id.(bit_0 ... bit_7) 
***************************************************************************************/
void set_player_id(uint8_t ID){

uint8_t *p_id_bit; 							//указатель на биты структуры player_id
p_id_bit = &data_packet.packet.player_id.bit_6; 	//указывает на 6 "бит" структуры 
for (int i=0; i < 7; i++) { 				//надо узнать значения 7 младших бит ID
ID = ID << 1; 								//сдвигаем влево на один бит
if (ID&(1<<7)) 								//если старший бит = 1
	{
		*p_id_bit++ = IR_ONE; 				//присваиваем соответствующее значение  data_packet.player_id.bit_x
	}
else 
	{
		*p_id_bit++ = IR_ZERO; 
	}

}

data_packet.packet.player_id.bit_7 = IR_ZERO; //согласно протоколу, этот "бит" должен быть равен 0 

}



/**************************************************************************************
* Установка идентификатора (цвета) команды
* в качестве аргумента функции указывается идентификационный номер (цвет) команды (от 0 до 3)
* в результате выполнения функции в глобальной переменной data_packet.team_id
* будут соответствующим образом инициированы  data_packet.team_id.(bit_0 и bit_1) 
***************************************************************************************/



void set_team_color(tteam_color  color){
switch(color){

		case Red : { //По протоколу 00 = Red
						data_packet.packet.team_id.bit_0 = IR_ZERO;
						data_packet.packet.team_id.bit_1 = IR_ZERO;
						break;	
					}
		case Blue: { //По протоколу 01 = Blue
						data_packet.packet.team_id.bit_0 = IR_ONE;
						data_packet.packet.team_id.bit_1 = IR_ZERO;
						break;	
					}
		case Yellow: { //По протоколу 10 = Yellow
						data_packet.packet.team_id.bit_0 = IR_ZERO;
						data_packet.packet.team_id.bit_1 = IR_ONE;
						break;	
					}
		case Green: { //По протоколу 11 = Green
						data_packet.packet.team_id.bit_0 = IR_ONE;
						data_packet.packet.team_id.bit_1 = IR_ONE;
						break;	
					}


			}




}




/**************************************************************************************
* Установка установка мощьности нашего оружия (наносимый урон)
* в качестве аргумента функции указывается наносимый урон
* в результате выполнения функции в глобальной переменной data_packet.damage
* будут соответствующим образом инициированы  data_packet.damage.(bit_0 и bit_3) 
***************************************************************************************/


void set_gun_damage(tgun_damage damage){

switch(damage){
		case Damage_1:{  //По протоколу 0000 = 1
						data_packet.packet.damage.bit_0 = IR_ZERO;
						data_packet.packet.damage.bit_1 = IR_ZERO;
						data_packet.packet.damage.bit_2 = IR_ZERO;
						data_packet.packet.damage.bit_3 = IR_ZERO;
						break;
						}
		case Damage_2:{  //По протоколу 0001 = 2
						data_packet.packet.damage.bit_0 = IR_ONE;
						data_packet.packet.damage.bit_1 = IR_ZERO;
						data_packet.packet.damage.bit_2 = IR_ZERO;
						data_packet.packet.damage.bit_3 = IR_ZERO;
						break;
						}

		case Damage_4:{  //По протоколу 0010 = 4
						data_packet.packet.damage.bit_0 = IR_ZERO;
						data_packet.packet.damage.bit_1 = IR_ONE;
						data_packet.packet.damage.bit_2 = IR_ZERO;
						data_packet.packet.damage.bit_3 = IR_ZERO;
						break;
						}

		case Damage_5:{  //По протоколу 0011 = 5
						data_packet.packet.damage.bit_0 = IR_ONE;
						data_packet.packet.damage.bit_1 = IR_ONE;
						data_packet.packet.damage.bit_2 = IR_ZERO;
						data_packet.packet.damage.bit_3 = IR_ZERO;
						break;
						}
		case Damage_7:{  //По протоколу 0100 = 7
						data_packet.packet.damage.bit_0 = IR_ZERO;
						data_packet.packet.damage.bit_1 = IR_ZERO;
						data_packet.packet.damage.bit_2 = IR_ONE;
						data_packet.packet.damage.bit_3 = IR_ZERO;
						break;
						}
		case Damage_10:{  //По протоколу 0101 = 10
						data_packet.packet.damage.bit_0 = IR_ONE;
						data_packet.packet.damage.bit_1 = IR_ZERO;
						data_packet.packet.damage.bit_2 = IR_ONE;
						data_packet.packet.damage.bit_3 = IR_ZERO;
						break;
						}
		case Damage_15:{  //По протоколу 0110 = 15
						data_packet.packet.damage.bit_0 = IR_ZERO;
						data_packet.packet.damage.bit_1 = IR_ONE;
						data_packet.packet.damage.bit_2 = IR_ONE;
						data_packet.packet.damage.bit_3 = IR_ZERO;
						break;
						}
		case Damage_17:{  //По протоколу 0111 = 17
						data_packet.packet.damage.bit_0 = IR_ONE;
						data_packet.packet.damage.bit_1 = IR_ONE;
						data_packet.packet.damage.bit_2 = IR_ONE;
						data_packet.packet.damage.bit_3 = IR_ZERO;
						break;
						}
		case Damage_20:{  //По протоколу 1000 = 20
						data_packet.packet.damage.bit_0 = IR_ZERO;
						data_packet.packet.damage.bit_1 = IR_ZERO;
						data_packet.packet.damage.bit_2 = IR_ZERO;
						data_packet.packet.damage.bit_3 = IR_ONE;
						break;
						}

		case Damage_25:{  //По протоколу 1001 = 25
						data_packet.packet.damage.bit_0 = IR_ONE;
						data_packet.packet.damage.bit_1 = IR_ZERO;
						data_packet.packet.damage.bit_2 = IR_ZERO;
						data_packet.packet.damage.bit_3 = IR_ONE;
						break;
						}

		case Damage_30:{  //По протоколу 1010 = 30
						data_packet.packet.damage.bit_0 = IR_ZERO;
						data_packet.packet.damage.bit_1 = IR_ONE;
						data_packet.packet.damage.bit_2 = IR_ZERO;
						data_packet.packet.damage.bit_3 = IR_ONE;
						break;
						}
		case Damage_35:{  //По протоколу 1011 = 35
						data_packet.packet.damage.bit_0 = IR_ONE;
						data_packet.packet.damage.bit_1 = IR_ONE;
						data_packet.packet.damage.bit_2 = IR_ZERO;
						data_packet.packet.damage.bit_3 = IR_ONE;
						break;
						}

		case Damage_40:{  //По протоколу 1100 = 40
						data_packet.packet.damage.bit_0 = IR_ZERO;
						data_packet.packet.damage.bit_1 = IR_ZERO;
						data_packet.packet.damage.bit_2 = IR_ONE;
						data_packet.packet.damage.bit_3 = IR_ONE;
						break;
						}

		case Damage_50:{  //По протоколу 1101 = 50
						data_packet.packet.damage.bit_0 = IR_ONE;
						data_packet.packet.damage.bit_1 = IR_ZERO;
						data_packet.packet.damage.bit_2 = IR_ONE;
						data_packet.packet.damage.bit_3 = IR_ONE;
						break;
						}
		case Damage_75:{  //По протоколу 1110 = 75
						data_packet.packet.damage.bit_0 = IR_ZERO;
						data_packet.packet.damage.bit_1 = IR_ONE;
						data_packet.packet.damage.bit_2 = IR_ONE;
						data_packet.packet.damage.bit_3 = IR_ONE;
						break;
						}

		case Damage_100:{  //По протоколу 1111 = 100
						data_packet.packet.damage.bit_0 = IR_ONE;
						data_packet.packet.damage.bit_1 = IR_ONE;
						data_packet.packet.damage.bit_2 = IR_ONE;
						data_packet.packet.damage.bit_3 = IR_ONE;
						break;
						}



			}



}




void init_var(void){            //Задаём значения переменным
last_simple = 0xFFFF; //иначе будет звук выстрела при сбросе
snd_adress.curr_adress = (uint8_t*)0xFFFF; //иначе бубдет попытка воспроизвести звук
ir_transmitter_on=false; 	//Запретим пока передачу данных (поскольку данные ещё не сформированны)
set_player_id(1);	//Устанавливаем идентификатор игрока
set_team_color(team_id());	//Устанавливаем идентификатор (цвет) команды
set_gun_damage(gun_damage());		//Устанавливаем мощьность оружия (урон)
data_packet.packet.header = IR_START;		//Устанавливаем  заголовку (старт биту) необходимую длительность
data_packet.packet.end_of_data = 0;		//0 - это указание передатчику, что данных для передачи больше нет
cursor_position = 0; //Курсор - на начало блока данных
start_bit_received = false;//Старт бит ещё не принят
bit_in_rx_buff = 0;//Буфер приемника пуст
rx_event = NOT_EVENT;//Сбрасываем события приемника
//reset_clock(); //обнуляем часы
life = 8;//здоровье -100% 
key_pressing_duration.key_1    =0;//обнуляем счетчики 
						  //длительности
						  //непрерывного нажатия кнопок
key_pressing_duration.key_1_inc=1;//разрешаем отсчет длительности
bullets = bullets_limit();

}




bool get_buffer_bit(uint8_t index){		//Считываем значение бита в буфере ИК-приемника
uint8_t byte_index;
uint8_t bit_index;
byte_index = index/8; //Определяем, в каком байте нахадится нужный бит
bit_index = index - (byte_index*8);//Определяем номер бита в байте
if(rx_buffer[byte_index]&(1<<(7-bit_index))) return true;
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


tteam_color team_id()//Опреднляем цвет нашей команды 
{


tteam_color result;
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

return result;
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
tgun_damage result;
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


devider = IR_F0/fqr; 
if (count > 160) count = 160; //ограничим время воспроизведения 16 секундами
beep_counter = (fqr/10)*count; //количество циклов (периодов колебаний)

for (uint16_t i=0; i < beep_counter; i++)
	{
		OCR0 = value;
		timer1=0;
		while (timer1 < devider);
		OCR0 = 0;
		timer1=0;
		while (timer1 < devider);

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

snd_adress.start_adress = &pSnd_hit[0];

snd_adress.end_adress = &pSnd_hit[sizeof(pSnd_hit)-1];

snd_adress.curr_adress = &pSnd_hit[0];

play_hit_snd = true; //разрешаем воспроизведенме звука

while (play_hit_snd);// ждем окончания воспроизведения звука
}
