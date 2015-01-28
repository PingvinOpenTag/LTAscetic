#include "ltag_ascetic.h"
//#include "milshot_snd.h"
//#include "joystick_driver.h"
//#include "hitraw.h"

/**************************************************************************************
* ���������� ���������� �������
***************************************************************************************/


ISR(TIMER2_COMP_vect){
//timer1++;
static volatile uint8_t prt;


prt = TSOP_IN&TSOP_PIN; 
if (prt==0) //�� ����� ��-��������� ������ ������� ������� (������ ������ �������)
	{
	//	PORTA &= ~(1 << 0); //�������� ��������������� ���������
		low_level_counter++;//����������� ������� ������������ ������������� ������� �� ����� ��-���������
//	//	if (chit_detected_counter < (IR_ZERO*1000)) chit_detected_counter++;
//	//	if (chit_detected_counter >= (IR_ZERO*1000)) chit_detected=true;

	}
else  //�� ����� ��-��������� ������� ������� ������� (������ ������ �������)
	{
	//	PORTA |=(1<<0);	//��������� ��������������� ���������
// //		chit_detected_counter = 0;
// //		if (chit_detected) chit_detected=false;
		high_level_counter++;///����������� ������� ������������ �������������� ������� �� ����� ��-���������
		if((start_bit_received)&&(high_level_counter > IR_ZERO*8)/*&&(bit_in_rx_buff>=13)*/)
		{//��������� ��������� ������ �� ��������
			start_bit_received	= false; 	//�������� �����

			switch(bit_in_rx_buff)//��������, ������� ��� �������
			{
				case 14:
				{
					rx_event = RX_COMPLETE;			//������� ������� "������ �����"
					break;	
				}
				case 24:
				{
					rx_event = 	RX_MESSAGE_COMPLITE;//������� ���������;
					break;	
				}
				default:
				{
					rx_event = RX_ERROR;			//���������� ������� - "������ �����"
				}
			}

//			if (bit_in_rx_buff>=13) rx_event = RX_COMPLETE;			//������� ������� "������ �����"
//			else rx_event = RX_ERROR;			//���������� ������� - "������ �����"
			

			receiver_on = false;//��������� ��������
			if (ir_transmitter_on==false) TIMSK &=~_BV(OCIE2); //���� �������� �� ������ - ��������� ����������
		}
		if((high_level_counter > IR_ZERO*8)&&(ir_transmitter_on==false))
		{
			receiver_on = false;//��������� ��������
			TIMSK &=~_BV(OCIE2);
		}
	
	}
if (ir_transmitter_on==true)
	{//���� �������� ���������

		if (ir_pulse_counter > 0)		//����������� ������������ ����� ��������� 
		{								//��� �� ����������, "������" ������
			IR_LED_INVERT;  			//����������� ������������ ����� 
			ir_pulse_counter--;
		}
		else							//����� ��������� ���� ����������, 
		{
			IR_LED_OFF;			//����� ��-���� 
			if ( ir_space_counter > 0)	//��������, �������� �� ���������� ����� ����������
			{							//���, ���������� �� ��������
			
					IR_LED_OFF;	//����� ��-����
					ir_space_counter--;	//��������� �������� ������� ����� �� ���� "���"		
			}
			else //����� ��������� � ���������� ����� ������ ��������
			{	 //����� ����������� ��������� ����� (������������ ���)
				
				
				if (ir_tx_buffer_cursor.bits_for_tx>0) //���� ��������� ��������� �� �� ������ ������
				{
					if(ir_tx_buffer_cursor.bit_mask == 0)//��� ���� �������� ����� ��� ��������
					{
						ir_tx_buffer_cursor.byte_pos++;//��������� � ���������� �����
						ir_tx_buffer_cursor.bit_mask = (1<<7); //������� ��� ������ ������
						
					}
					if (tx_buffer[ir_tx_buffer_cursor.byte_pos]&ir_tx_buffer_cursor.bit_mask)//���� ������� ��� ����� "1"
					{
						ir_pulse_counter = IR_ONE;//�������� "1" (�������� 1200 �����������)
					}
					else //������� ��� ����� "0"
					{
						ir_pulse_counter = IR_ZERO;//�������� "0" (�������� 600 �����������)
					}
					ir_space_counter = IR_SPACE;      //� ��� ����� �� �������					
					ir_tx_buffer_cursor.bit_mask = (ir_tx_buffer_cursor.bit_mask >> 1); //��������� ���
					ir_tx_buffer_cursor.bits_for_tx--; //��������� ���������� ���������� ���
//					ir_pulse_counter =data_packet.data[cursor_position++] ; //��������� ������� ��������� �������������

				}
				else //��� ������ �������� (�������, �� ������� ��������� ���������, ����� 0)
				{
					ir_transmitter_on=false; //��������� ����������
					FIRE_LED_OFF;
					display_bullets_update_now++;
				// ���� 	
					if (!receiver_on) //���� ��� ������ ������
					{
						TIMSK &=~_BV(OCIE2);          // ��������� ���������� �� �������/���������
						
					}				 
				}										
			}				 
		}
	}
else 	{//���� �������� ���������

		}


}



/**************************************************************************************
* ���������� ������� ���������� ������ INT0
***************************************************************************************/


ISR(INT0_vect){
TIMSK |= _BV(OCIE2);          // ��������� ���������� �� �������/���������
receiver_on = true;
if(!(MCUCR&_BV(ISC00)))		 //���� ���������� ������� ������ 
	{
		MCUCR |=_BV(ISC00); //��������� ���������� ����� ������������� �������
		if (start_bit_received)//���� �����-��� ������, �� ���� ����� ������
			{
				if((high_level_counter < (IR_SPACE + ERROR_TOLERANCE))&&(high_level_counter > (IR_SPACE - ERROR_TOLERANCE)))//�������� ������������ ����� ����� ������
				{
					//������������ ����� ����� ���������� ���������
				}
				else //������������ ����� ����� ������� ����� �� ���������
				{//��������� ������ �����
						start_bit_received	= false; 	//�������� �����
						bit_in_rx_buff = 0;				//������� �����
						rx_event = RX_ERROR;			//���������� ������� - "������ �����"
//						TIMSK &= ~_BV(OCIE2);          // ��������� ���������� �� �������/���������
				}
			
			
			}		
		low_level_counter = 0;//�������� ������� ������������ ������������� ������� �� ����� ��-��������
		high_level_counter = 0;//�������� ������� ������������ �������������� ������� �� ����� ��-��������
	}
else 						//���������� ������� ������� 
	{
		MCUCR &=~_BV(ISC00); //��������� ���������� ����� ������������� ������
		
		if (start_bit_received)//���� �����-��� ������, �� ���� ����� ������
			{
				if((low_level_counter < (IR_ZERO + ERROR_TOLERANCE))&&(low_level_counter > (IR_ZERO - ERROR_TOLERANCE)))//��������, ������������� �� ������������ ������ �������� ����
				{
					set_buffer_bit(bit_in_rx_buff++, false);//������������ ����� ������������� ���� �� ��������� 0, ������� ���� � ����� ������
				
				}
				else //���, ��� �� ��� �� ��������� 0
				{
					if((low_level_counter < (IR_ONE + ERROR_TOLERANCE))&&(low_level_counter > (IR_ONE - ERROR_TOLERANCE)))//, ����� ��� ��� �� ��������� 1?
					{
							set_buffer_bit(bit_in_rx_buff++, true);//������������ ����� ������������� ���� �� ��������� 1, ������� ������� � ����� ������		
					}
					else //��� �� �������, �� ���� - ��� ������ 
					{
						start_bit_received	= false; 	//�������� �����
						bit_in_rx_buff = 0;				//������� �����
						rx_event = RX_ERROR;			//���������� ������� - "������ �����"
//						TIMSK &= ~_BV(OCIE2);          // ��������� ���������� �� �������/���������
					}
				}
			}
		else //�����-��� ��� �� ������
		{
			if ((low_level_counter < (IR_START + ERROR_TOLERANCE))&&(low_level_counter > (IR_START - ERROR_TOLERANCE))) //����� ��� �����-���?	
			{//��� �����-���
				bit_in_rx_buff = 0;				//������� �����
				start_bit_received	= true; 	//��������� ����� ������� (���)
				
			}
			else //��� �� �����-���, ��� ������ 
			{
				//����������
			}
		}
		
		
		low_level_counter = 0;//�������� ������� ������������ ������������� ������� �� ����� ��-��������
		high_level_counter = 0;//�������� ������� ������������ �������������� ������� �� ����� ��-��������
	
	


	}

}


/**************************************************************************************
* ���������� ������� ���������� ������ INT1
* ��������� ��� ���������� ����� ��������� � �����������
***************************************************************************************/


ISR(INT1_vect){
tm_connect = true;
volatile uint8_t tmp;
tmp = 0;
tmp++;
tm_read_serial_number();
}



/**************************************************************************************
* ��������� ��������� ������ �����
* ���� eeprom �� �������, ���������
* ���� ������ ��������� - ��������� eeprom
***************************************************************************************/


uint8_t read_seimpl(){//��������� ��������� ������ �� ������
uint8_t result;

//result = usartRxBuf[curr_pos_in_sound_buff];

if (curr_sound.simples_in_queue==1)//���� ��� ��������� ������ � ������
		{
			close_eeprom(&result);//��������� ��������� ���� � ��������� eeprom
			eeprom_is_open = false;
			if(curr_sound.role = hit_sound) WOUND_LED_OFF;//���� ��� ��� ���� �������, �������� ��������� �������
			return result;
		}
else //��� �� ��������� ������
	{
		if (!eeprom_is_open) //���� eeprom �������
		{
		//eeprom_is_open = open_eeprom(eeprom_read_byte(&sound_1_adress));//��������� eeprom
			eeprom_is_open = open_eeprom(curr_sound.adress);//��������� eeprom
		};
		if(eeprom_is_open)
		{
			if ((curr_sound.simples_in_queue==cut_off_sound)&&(curr_sound.role==shot_sound))//(eeprom_read_word(&sound_1_size)/100)*(100-CUT_OFF_SOUNT))
			{

				if (fire_mode()==queues)
				{
			
					if ((get_keyboard_status()==key_pressed)&&(life>0)&&(bullets>0)) //����� �����, �� �������� ����
					{
							bullets--;//��������� �� 1 ���������� ��������
							send_ir_package();	//���������� "�������"
							
//							last_simple=0;		//������������� ���� �������
							close_eeprom(&result);//��������� ��������� ���� � ��������� eeprom
							eeprom_is_open = false;
			//eeprom_is_open = open_eeprom(eeprom_read_byte(&sound_1_adress));//��������� eeprom
			//				simples_in_queue=eeprom_read_word(&sound_1_size);
							curr_sound.simples_in_queue=curr_sound.size;
				//			display_bullets_update();
							return result;
					};
				};
			
			};
			if (curr_sound.simples_in_queue>0)//��� �� ��������� ������ � ������
			{
				read_eeprom_byte(&result); //��������� ��������� ����
				return result;
			};
		};
		
	};




return result;
}






/**************************************************************************************
* ���������� ������� ���������� timer1A
***************************************************************************************/

ISR(TIMER1_COMPA_vect){
TIMSK &= ~_BV(OCIE1A);  //��������� ���������� timer1, ����� �� ���� ��������
sei(); 
if (curr_sound.role == shot_sound )
{	
	if ((bullets >0))//���� ������� �� ���������
	{
		if (curr_sound.simples_in_queue>0) //���� �� ��� ������� ��� ��������������
		{ 
			OCR0 =	read_seimpl();
			if (curr_sound.simples_in_queue==1) OCR0=0;
			curr_sound.simples_in_queue--;
		}
	}

	if (bullets <= 0) //������� ���������
	{
		BULLETS_OUT_LED_ON; // �������� ��������� "������� ���������"
		if (curr_sound.simples_in_queue>0) //���� �� ��� ������� ��� ��������������
		{ 
			OCR0 =	read_seimpl();
			if (curr_sound.simples_in_queue==1) OCR0=0;
			curr_sound.simples_in_queue--;
		}
	}
}
else //��� �� �������
{
	if (curr_sound.simples_in_queue>0) //���� �� ��� ������� ��� ��������������
	{
		
		
		OCR0 =	read_seimpl();
		if (curr_sound.simples_in_queue==1) OCR0=0;
		curr_sound.simples_in_queue--;


	}


}

if(cyclic_deley_counter==1)//������������� �������� ����� ����������
{
	if (fire_mode()==queues)//���� ����� �������� ���������
	{
		if ((get_keyboard_status()==key_pressed)&&(life>0)&&(bullets>0)) //����� �����, �� �������� ����
		{
			bullets--;//��������� �� 1 ���������� ��������
			send_ir_package();	//���������� "�������"
			playback_sound(shot_sound);
		}
	}
}


if(cyclic_deley_counter) cyclic_deley_counter--;//��������� �������� ��������� �������� �������� ����� ��c�������
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
if (++tmp_cntr > 1000) //���� �������� ���������
	{
		tmp_cntr = 0;
		if(reload_countdown > 0) //�������� ������ ������������ ����������� �� �������
		{
			reload_countdown--;
		}
		else //���� ��������� ��������� 
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
if (play_hit_snd) // ���� ����� ������������� ���� �������
	{
		if (snd_adress.end_adress >= snd_adress.curr_adress) //���� �� ����� �� ������������
		{
						
			OCR0 = pgm_read_byte(snd_adress.curr_adress++);
		}

		if (snd_adress.end_adress < snd_adress.curr_adress)//���� ������������ �� �����
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

if((!(TSOP_IN&TSOP_PIN))&&(!(BT_STATE_IN&BT_STATE_PIN)))//���� �� ����� INT0 ������ ������� � ��� ������ ����������
	{
		if (chit_detected_counter < (4000)) chit_detected_counter++;
		if (chit_detected_counter >= (4000)) chit_detected=true;

	}
else { // ���� �������
		chit_detected_counter = 0;
		if (chit_detected) chit_detected=false;
}

timer2++;

cli();

TIMSK |= _BV(OCIE1A);  //������ ����� ��������� ���������� timer1


}





inline  TKEYBOARD_STATUS get_keyboard_status(void) {

volatile	TKEYBOARD_STATUS s_ret;
	
	switch (FIRE_KEY_IN&FIRE_KEY_PIN) //���������, ����� �� �����
		{
			case FIRE_KEY_PIN: s_ret=no_key_pressed  ; break;
			default: s_ret=key_pressed ;	
		}



		return s_ret;
}



inline  TKEYBOARD_STATUS get_reload_key_status(void) {

volatile	TKEYBOARD_STATUS s_ret;
	
	switch (RELOAD_KEY_IN&RELOAD_KEY_PIN) //���������, ����� �� ������ "������������"
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
	switch(key_status)  //���������, ��� ������
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
	 	case key_pressed  : //������ ������ 1
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
	switch(key_status)  //���������, ��� ������
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
	 	case key_pressed  : //������ ������ "������������"
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





