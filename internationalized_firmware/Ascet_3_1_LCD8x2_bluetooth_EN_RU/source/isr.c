#include "ltag_ascetic.h"
//#include "milshot_snd.h"
//#include "joystick_driver.h"
//#include "hitraw.h"

/**************************************************************************************
* Обработчик прерываний таймера
***************************************************************************************/


ISR(TIMER2_COMP_vect){
//timer1++;
static volatile uint8_t prt;


prt = TSOP_IN&TSOP_PIN; 
if (prt==0) //На ножке ИК-приемника низкий уровень сигнала (пойман сигнал несущей)
	{
	//	PORTA &= ~(1 << 0); //включаем вспомогательный светодиод
		low_level_counter++;//Увеличиваем счетчик длительности низкоуровнего сигнала на ножке ИК-приемника
//	//	if (chit_detected_counter < (IR_ZERO*1000)) chit_detected_counter++;
//	//	if (chit_detected_counter >= (IR_ZERO*1000)) chit_detected=true;

	}
else  //На ножке ИК-приемника высокий уровень сигнала (пойман сигнал несущей)
	{
	//	PORTA |=(1<<0);	//выключаем вспомогательный светодиод
// //		chit_detected_counter = 0;
// //		if (chit_detected) chit_detected=false;
		high_level_counter++;///Увеличиваем счетчик длительности высокоуровнего сигнала на ножке ИК-приемника
		if((start_bit_received)&&(high_level_counter > IR_ZERO*8)/*&&(bit_in_rx_buff>=13)*/)
		{//Фиксируем окончание приема по таймауту
			start_bit_received	= false; 	//отменяем прием

			switch(bit_in_rx_buff)//проверим, сколько бит принято
			{
				case 14:
				{
					rx_event = RX_COMPLETE;			//Генерим событие "принят пакет"
					break;	
				}
				case 24:
				{
					rx_event = 	RX_MESSAGE_COMPLITE;//принято сообщение;
					break;	
				}
				default:
				{
					rx_event = RX_ERROR;			//генерируем событие - "ошибка приёма"
				}
			}

//			if (bit_in_rx_buff>=13) rx_event = RX_COMPLETE;			//Генерим событие "принят пакет"
//			else rx_event = RX_ERROR;			//генерируем событие - "ошибка приёма"
			

			receiver_on = false;//выключаем приемник
			if (ir_transmitter_on==false) TIMSK &=~_BV(OCIE2); //если передача не ведётся - выключаем прерывания
		}
		if((high_level_counter > IR_ZERO*8)&&(ir_transmitter_on==false))
		{
			receiver_on = false;//выключаем приемник
			TIMSK &=~_BV(OCIE2);
		}
	
	}
if (ir_transmitter_on==true)
	{//Если передача разрешена

		if (ir_pulse_counter > 0)		//необходимая длительность пачки импульсов 
		{								//ещё не достигнута, "мигаем" дальше
			IR_LED_INVERT;  			//необходимая длительность пачки 
			ir_pulse_counter--;
		}
		else							//пачка импульсов была отправлена, 
		{
			IR_LED_OFF;			//тушим ИК-диод 
			if ( ir_space_counter > 0)	//проверим, выдержан ли промежуток между импульсами
			{							//нет, промежуток не выдержан
			
					IR_LED_OFF;	//тушим ИК-диод
					ir_space_counter--;	//уменьшаем обратный счетчик паузы на один "тик"		
			}
			else //Пакет импульсов и промежуток между битами переданы
			{	 //нужно формировать следующую пачку (передаваемый бит)
				
				
				if (ir_tx_buffer_cursor.bits_for_tx>0) //если указатель указывает не на пустую ячейку
				{
					if(ir_tx_buffer_cursor.bit_mask == 0)//все биты текущего байта уже переданы
					{
						ir_tx_buffer_cursor.byte_pos++;//переходим к следующему байту
						ir_tx_buffer_cursor.bit_mask = (1<<7); //старший бит уходит первым
						
					}
					if (tx_buffer[ir_tx_buffer_cursor.byte_pos]&ir_tx_buffer_cursor.bit_mask)//если текущий бит равен "1"
					{
						ir_pulse_counter = IR_ONE;//отправим "1" (помигаем 1200 микросекунд)
					}
					else //текущий бит равен "0"
					{
						ir_pulse_counter = IR_ZERO;//отправим "0" (помигаем 600 микросекунд)
					}
					ir_space_counter = IR_SPACE;      //и про паузу не забудем					
					ir_tx_buffer_cursor.bit_mask = (ir_tx_buffer_cursor.bit_mask >> 1); //следующий бит
					ir_tx_buffer_cursor.bits_for_tx--; //уменьшаем количество оставшихся бит
//					ir_pulse_counter =data_packet.data[cursor_position++] ; //передадим импульс указанной длительностью

				}
				else //Все данные переданы (элемент, на который ссылается указатель, равен 0)
				{
					ir_transmitter_on=false; //выключаем передатчик
					FIRE_LED_OFF;
					display_bullets_update_now++;
				// если 	
					if (!receiver_on) //если нет приема пакета
					{
						TIMSK &=~_BV(OCIE2);          // Запрещаем прерывания по захвату/сравнению
						
					}				 
				}										
			}				 
		}
	}
else 	{//Если передача запрещена

		}


}



/**************************************************************************************
* Обработчик внещних прерываний вывода INT0
***************************************************************************************/


ISR(INT0_vect){
TIMSK |= _BV(OCIE2);          // Разрешаем прерывания по захвату/сравнению
receiver_on = true;
if(!(MCUCR&_BV(ISC00)))		 //если прерывание вызвано спадом 
	{
		MCUCR |=_BV(ISC00); //следующее прерывание будет сгенерировано фронтом
		if (start_bit_received)//Если старт-бит принят, то идет прием пакета
			{
				if((high_level_counter < (IR_SPACE + ERROR_TOLERANCE))&&(high_level_counter > (IR_SPACE - ERROR_TOLERANCE)))//Проверим длительность паузы между битами
				{
					//длительность паузы между импульсами корректна
				}
				else //Длительность паузы между приемом битов не корректна
				{//Фиксируем ошибку приёма
						start_bit_received	= false; 	//отменяем прием
						bit_in_rx_buff = 0;				//очищаем буфер
						rx_event = RX_ERROR;			//генерируем событие - "ошибка приёма"
//						TIMSK &= ~_BV(OCIE2);          // Запрещаем прерывания по захвату/сравнению
				}
			
			
			}		
		low_level_counter = 0;//Обнуляем счетчик длительности низкоуровнего сигнала на ножке ИК-приёмника
		high_level_counter = 0;//Обнуляем счетчик длительности высокоуровнего сигнала на ножке ИК-приёмника
	}
else 						//прерывание вызвано фронтом 
	{
		MCUCR &=~_BV(ISC00); //следующее прерывание будет сгенерировано спадом
		
		if (start_bit_received)//Если старт-бит принят, то идет прием пакета
			{
				if((low_level_counter < (IR_ZERO + ERROR_TOLERANCE))&&(low_level_counter > (IR_ZERO - ERROR_TOLERANCE)))//Проверим, соответствует ли длительность пакета нулевому биту
				{
					set_buffer_bit(bit_in_rx_buff++, false);//Длительность пачки соответствует биту со значением 0, заносим ноль в буфер приема
				
				}
				else //Нет, это не бит со значением 0
				{
					if((low_level_counter < (IR_ONE + ERROR_TOLERANCE))&&(low_level_counter > (IR_ONE - ERROR_TOLERANCE)))//, может это бит со значением 1?
					{
							set_buffer_bit(bit_in_rx_buff++, true);//Длительность пачки соответствует биту со значением 1, заносим еденицу в буфер приема		
					}
					else //Это ни единица, ни ноль - это помеха 
					{
						start_bit_received	= false; 	//отменяем прием
						bit_in_rx_buff = 0;				//очищаем буфер
						rx_event = RX_ERROR;			//генерируем событие - "ошибка приёма"
//						TIMSK &= ~_BV(OCIE2);          // Запрещаем прерывания по захвату/сравнению
					}
				}
			}
		else //Старт-бит ещё не принят
		{
			if ((low_level_counter < (IR_START + ERROR_TOLERANCE))&&(low_level_counter > (IR_START - ERROR_TOLERANCE))) //Может это старт-бит?	
			{//Это старт-бит
				bit_in_rx_buff = 0;				//очищаем буфер
				start_bit_received	= true; 	//разрешаем прием пакетов (бит)
				
			}
			else //это не старт-бит, это помеха 
			{
				//Игнорируем
			}
		}
		
		
		low_level_counter = 0;//Обнуляем счетчик длительности низкоуровнего сигнала на ножке ИК-приёмника
		high_level_counter = 0;//Обнуляем счетчик длительности высокоуровнего сигнала на ножке ИК-приёмника
	
	


	}

}


/**************************************************************************************
* Обработчик внещних прерываний вывода INT1
* наступает при поднесении ключа тачмемори к считывателю
***************************************************************************************/


ISR(INT1_vect){
tm_connect = true;
volatile uint8_t tmp;
tmp = 0;
tmp++;
tm_read_serial_number();
}



/**************************************************************************************
* Считываем очередной сеймпл звука
* если eeprom не открыта, открываем
* если сеймпл последний - закрываем eeprom
***************************************************************************************/


uint8_t read_seimpl(){//считываем очередной сеймпл из буфера
uint8_t result;

//result = usartRxBuf[curr_pos_in_sound_buff];

if (curr_sound.simples_in_queue==1)//если это последний сеймпл в буфере
		{
			close_eeprom(&result);//считываем боследний буйт и закрываем eeprom
			eeprom_is_open = false;
			if(curr_sound.role = hit_sound) WOUND_LED_OFF;//если это был звук ранения, выключим светодиод повязки
			return result;
		}
else //это не последний сеймпл
	{
		if (!eeprom_is_open) //если eeprom закрыта
		{
		//eeprom_is_open = open_eeprom(eeprom_read_byte(&sound_1_adress));//открываем eeprom
			eeprom_is_open = open_eeprom(curr_sound.adress);//открываем eeprom
		};
		if(eeprom_is_open)
		{
			if ((curr_sound.simples_in_queue==cut_off_sound)&&(curr_sound.role==shot_sound))//(eeprom_read_word(&sound_1_size)/100)*(100-CUT_OFF_SOUNT))
			{

				if (fire_mode()==queues)
				{
			
					if ((get_keyboard_status()==key_pressed)&&(life>0)&&(bullets>0)) //курок нажат, то отсекаем звук
					{
							bullets--;//уменьшаем на 1 количество патронов
							send_ir_package();	//Производим "выстрел"
							
//							last_simple=0;		//воспроизводим звук сначала
							close_eeprom(&result);//считываем боследний буйт и закрываем eeprom
							eeprom_is_open = false;
			//eeprom_is_open = open_eeprom(eeprom_read_byte(&sound_1_adress));//открываем eeprom
			//				simples_in_queue=eeprom_read_word(&sound_1_size);
							curr_sound.simples_in_queue=curr_sound.size;
				//			display_bullets_update();
							return result;
					};
				};
			
			};
			if (curr_sound.simples_in_queue>0)//это не последний сеймпл в буфере
			{
				read_eeprom_byte(&result); //считываем очередной байт
				return result;
			};
		};
		
	};




return result;
}






/**************************************************************************************
* Обработчик внещних прерываний timer1A
***************************************************************************************/

ISR(TIMER1_COMPA_vect){
TIMSK &= ~_BV(OCIE1A);  //запрещаем прерывания timer1, чтобы не было рекурсии
sei(); 
if (curr_sound.role == shot_sound )
{	
	if ((bullets >0))//если патроны не кончились
	{
		if (curr_sound.simples_in_queue>0) //если не все сеймплы ещё воспроизведены
		{ 
			OCR0 =	read_seimpl();
			if (curr_sound.simples_in_queue==1) OCR0=0;
			curr_sound.simples_in_queue--;
		}
	}

	if (bullets <= 0) //патроны кончились
	{
		BULLETS_OUT_LED_ON; // включаем светодиод "Патроны кончились"
		if (curr_sound.simples_in_queue>0) //если не все сеймплы ещё воспроизведены
		{ 
			OCR0 =	read_seimpl();
			if (curr_sound.simples_in_queue==1) OCR0=0;
			curr_sound.simples_in_queue--;
		}
	}
}
else //это не выстрел
{
	if (curr_sound.simples_in_queue>0) //если не все сеймплы ещё воспроизведены
	{
		
		
		OCR0 =	read_seimpl();
		if (curr_sound.simples_in_queue==1) OCR0=0;
		curr_sound.simples_in_queue--;


	}


}

if(cyclic_deley_counter==1)//заканчивается задержка между выстрелами
{
	if (fire_mode()==queues)//если режим стрельбы очередями
	{
		if ((get_keyboard_status()==key_pressed)&&(life>0)&&(bullets>0)) //курок нажат, то отсекаем звук
		{
			bullets--;//уменьшаем на 1 количество патронов
			send_ir_package();	//Производим "выстрел"
			playback_sound(shot_sound);
		}
	}
}


if(cyclic_deley_counter) cyclic_deley_counter--;//уменьшаем значение обратного счетчика задержки между выcтрелами
static volatile uint16_t tmp_cntr=0;

	if ((tmp_cntr - (tmp_cntr/100)*100)==0)
	{
		
		uart_timer++;
                if(ir_error_ignore) ir_error_ignore--;
				if (safe_counter) safe_counter--;
		switch(keyboard_event) 
			{
		  	case no_key_pressing: 
		  		{
					keyboard_event=test_keyboard(); 
					break;
				}
		  	default:;         
			}	

		switch(reload_key_event)
			{
		  	case no_key_pressing: 
		  		{
					reload_key_event=test_reload_key(); 
					break;
				}
		  	default:;         
			}	

		switch(joystick_event)
			{
		  	case no_pressing: 
		  		{
					joystick_event=test_joystick(); 
					break;
				}
		  	default:;         
			}	
	}
if (++tmp_cntr > 1000) //пора обновить индикацию
	{
		tmp_cntr = 0;
		if(reload_countdown > 0) //обратный отсчет длительности перезарядки не окончен
		{
			reload_countdown--;
		}
		else //пора закончить перезаряд 
		{
			if(reload_state==waiting_countdown)reload_state=reload_now;
		} 
		static volatile uint8_t bit_mask = 0b00000001;
	
		if ((life_leds_status[0]&bit_mask)==0) 
			{
				LIFE_LED1_OFF;
			}
		else 
			{
				LIFE_LED1_ON;
			};
		if ((life_leds_status[1]&bit_mask)==0) 
			{
				LIFE_LED2_OFF;
			}
		else 
			{
				LIFE_LED2_ON;
			};
		if ((life_leds_status[2]&bit_mask)==0) 
			{
				LIFE_LED3_OFF;
			}
		else 
			{
				LIFE_LED3_ON;
			};

		if ((life_leds_status[3]&bit_mask)==0) 
			{
				LIFE_LED4_OFF;
			}
		else 
			{
				LIFE_LED4_ON;
			};

		bit_mask = (bit_mask<<1);
		if (bit_mask == 0)  bit_mask = 0b00000001;
		
		if(BT_STATE_IN&BT_STATE_PIN) {BT_STATE_LED_ON;}
		else {BT_STATE_LED_OFF;};
	
	}


/*
if (play_hit_snd) // если нужно воспроизвести звук ранения
	{
		if (snd_adress.end_adress >= snd_adress.curr_adress) //звук до конца не воспроизведён
		{
						
			OCR0 = pgm_read_byte(snd_adress.curr_adress++);
		}

		if (snd_adress.end_adress < snd_adress.curr_adress)//звук воспроизведён до конца
		{
		
			snd_adress.curr_adress = (uint8_t*)0xFFFF;
			play_hit_snd = false;
		
		}
	}

*/

if (fcurr_simple_prepared) 
	{
		OCR0 = curr_simple;
		fcurr_simple_prepared = false;
	}

if((!(TSOP_IN&TSOP_PIN))&&(!(BT_STATE_IN&BT_STATE_PIN)))//если на входе INT0 низкий уровень и нет блютус соединения
	{
		if (chit_detected_counter < (4000)) chit_detected_counter++;
		if (chit_detected_counter >= (4000)) chit_detected=true;

	}
else { // если высокий
		chit_detected_counter = 0;
		if (chit_detected) chit_detected=false;
}

timer2++;

cli();

TIMSK |= _BV(OCIE1A);  //теперь вновь разрешаем прерывания timer1


}





inline  TKEYBOARD_STATUS get_keyboard_status(void) {

volatile	TKEYBOARD_STATUS s_ret;
	
	switch (FIRE_KEY_IN&FIRE_KEY_PIN) //проверяем, нажат ли курок
		{
			case FIRE_KEY_PIN: s_ret=no_key_pressed  ; break;
			default: s_ret=key_pressed ;	
		}



		return s_ret;
}



inline  TKEYBOARD_STATUS get_reload_key_status(void) {

volatile	TKEYBOARD_STATUS s_ret;
	
	switch (RELOAD_KEY_IN&RELOAD_KEY_PIN) //проверяем, нажат ли кнопка "перезарядить"
		{
			case RELOAD_KEY_PIN: s_ret=no_key_pressed  ; break;
			default: s_ret=key_pressed ;	
		}



		return s_ret;
}



inline  TKEYBOARD_EVENT test_keyboard(void){
	TKEYBOARD_STATUS key_status;
	TKEYBOARD_EVENT ret_ev;
	key_status=get_keyboard_status();
	switch(key_status)  //проверяем, что нажато
	{
		case no_key_pressed: 
		{
			if (key_pressing_duration.key_1>= SHORT_DURATION)
			{
				ret_ev=key_pressing;
                key_pressing_duration.key_1    =0;
                key_pressing_duration.key_1_inc=0;
				return ret_ev;
			
			}

			else 
			{
				ret_ev=no_key_pressing;
                key_pressing_duration.key_1    =0;
                key_pressing_duration.key_1_inc=1;
			
			}
		
		} 
		break;
	 	case key_pressed  : //нажата кнопка 1
		{ 
			if(key_pressing_duration.key_1>= SHORT_DURATION)
			{
				ret_ev=key_pressing;
             	key_pressing_duration.key_1    =0;
                key_pressing_duration.key_1_inc=0;
			} 
			else 
			{
				key_pressing_duration.key_1 += key_pressing_duration.key_1_inc; 
                ret_ev=keyboard_event;

			}
		}
		break;
		default: ret_ev=keyboard_event;
	
	}

return   ret_ev;
}




inline  TKEYBOARD_EVENT test_reload_key(void){
	TKEYBOARD_STATUS key_status;
	TKEYBOARD_EVENT ret_ev;
	key_status=get_reload_key_status();
	switch(key_status)  //проверяем, что нажато
	{
		case no_key_pressed: 
		{
			if (key_pressing_duration.key_2>= SHORT_DURATION)
			{
				ret_ev=key_pressing;
                key_pressing_duration.key_2    =0;
                key_pressing_duration.key_2_inc=0;
				return ret_ev;
			
			}

			else 
			{
				ret_ev=no_key_pressing;
                key_pressing_duration.key_2    =0;
                key_pressing_duration.key_2_inc=1;
			
			}
		
		} 
		break;
	 	case key_pressed  : //нажата кнопка "Перезарядить"
		{ 
			if(key_pressing_duration.key_2>= SHORT_DURATION)
			{
				ret_ev=key_pressing;
             	key_pressing_duration.key_2    =0;
                key_pressing_duration.key_2_inc=0;
			} 
			else 
			{
				key_pressing_duration.key_2 += key_pressing_duration.key_2_inc; 
                ret_ev=reload_key_event;

			}
		}
		break;
		default: ret_ev=reload_key_event;
	
	}

return   ret_ev;
}





