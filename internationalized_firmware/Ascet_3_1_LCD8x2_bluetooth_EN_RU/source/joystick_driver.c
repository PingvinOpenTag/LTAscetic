#include "ltag_ascetic.h"
//#include "joystick_driver_types.h"

void init_joystick(){
UP_KEY_DDR&=~UP_KEY_PIN;
UP_KEY_PORT|=UP_KEY_PIN;


RIGHT_KEY_DDR&=~RIGHT_KEY_PIN;
RIGHT_KEY_PORT|=RIGHT_KEY_PIN;

DOWN_KEY_DDR&=~DOWN_KEY_PIN;
DOWN_KEY_PORT|=DOWN_KEY_PIN;

LEFT_KEY_DDR&=~LEFT_KEY_PIN;
LEFT_KEY_PORT|=LEFT_KEY_PIN;

CENTRAL_KEY_DDR&=~CENTRAL_KEY_PIN;
CENTRAL_KEY_PORT|=CENTRAL_KEY_PIN;

joystick_key_pressing_duration.key_up=0;
joystick_key_pressing_duration.key_up_inc=1;
joystick_key_pressing_duration.key_right=0;
joystick_key_pressing_duration.key_right_inc=1;
joystick_key_pressing_duration.key_down=0;
joystick_key_pressing_duration.key_down_inc=1;
joystick_key_pressing_duration.key_left=0;
joystick_key_pressing_duration.key_left_inc=1;
joystick_key_pressing_duration.key_central=0;
joystick_key_pressing_duration.key_central_inc=1;

}




TJOYSTICK_EVENT test_joystick(){

TJOYSTICK_EVENT result;
result = no_pressing;

uint8_t j_status;
j_status = get_joystick_status();

switch(j_status)  //проверяем, что нажато
	{
	
		case UP_KEY_MASK: 
			{
				if(joystick_key_pressing_duration.key_up>= SHORT_DURATION)
				{
					result=key_up_pressing;
             		joystick_key_pressing_duration.key_up    =0;
                	joystick_key_pressing_duration.key_up_inc=0;
				} 
				else
				{
					joystick_key_pressing_duration.key_up += joystick_key_pressing_duration.key_up_inc;
					result = joystick_event;
				}
				
				
			}
		break;
		
		case RIGHT_KEY_MASK: 
			{
			
				if(joystick_key_pressing_duration.key_right>= SHORT_DURATION)
				{
					result=key_right_pressing;
             		joystick_key_pressing_duration.key_right    =0;
                	joystick_key_pressing_duration.key_right_inc=0;
				} 
				else
				{
					joystick_key_pressing_duration.key_right += joystick_key_pressing_duration.key_right_inc;
					result = joystick_event;
				}			
			
			}
		break;
	
		case DOWN_KEY_MASK: 
			{
				if(joystick_key_pressing_duration.key_down>= SHORT_DURATION)
				{
					result=key_down_pressing;
             		joystick_key_pressing_duration.key_down    =0;
                	joystick_key_pressing_duration.key_down_inc=0;
				} 
				else
				{
					joystick_key_pressing_duration.key_down += joystick_key_pressing_duration.key_down_inc;
					result = joystick_event;
				}						


			}
		break;

		case LEFT_KEY_MASK: 
			{
				if(joystick_key_pressing_duration.key_left>= SHORT_DURATION)
				{
					result=key_left_pressing;
             		joystick_key_pressing_duration.key_left    =0;
                	joystick_key_pressing_duration.key_left_inc=0;
				} 
				else
				{
					joystick_key_pressing_duration.key_left += joystick_key_pressing_duration.key_left_inc;
					result = joystick_event;
				}						
			
			}
		break;
		case CENTRAL_KEY_MASK: 
			{
				if(joystick_key_pressing_duration.key_central>= SHORT_DURATION)
				{
					result=key_central_pressing;
             		joystick_key_pressing_duration.key_central    =0;
                	joystick_key_pressing_duration.key_central_inc=0;
				} 
				else
				{
					joystick_key_pressing_duration.key_central += joystick_key_pressing_duration.key_central_inc;
					result = joystick_event;
				}						
			
			}
		break;

		case 0:
		{
			init_joystick();
		}
		break;
		default:
		{
			joystick_key_pressing_duration.key_up=0;
			joystick_key_pressing_duration.key_up_inc=0;
			joystick_key_pressing_duration.key_right=0;
			joystick_key_pressing_duration.key_right_inc=0;
			joystick_key_pressing_duration.key_down=0;
			joystick_key_pressing_duration.key_down_inc=0;
			joystick_key_pressing_duration.key_left=0;
			joystick_key_pressing_duration.key_left_inc=0;
			joystick_key_pressing_duration.key_central=0;
			joystick_key_pressing_duration.key_central_inc=0;
		} 
	
	}

return result;
} 


uint8_t get_joystick_status(){
uint8_t result;
result = 0;
if ((UP_KEY_IN & UP_KEY_PIN) CONDITION_FOR_THE_BUTTON_PRESSED) result |=UP_KEY_MASK;
if ((RIGHT_KEY_IN & RIGHT_KEY_PIN) CONDITION_FOR_THE_BUTTON_PRESSED) result |=RIGHT_KEY_MASK;
if ((DOWN_KEY_IN & DOWN_KEY_PIN) CONDITION_FOR_THE_BUTTON_PRESSED) result |=DOWN_KEY_MASK;
if ((LEFT_KEY_IN & LEFT_KEY_PIN) CONDITION_FOR_THE_BUTTON_PRESSED) result |=LEFT_KEY_MASK;
if ((CENTRAL_KEY_IN & CENTRAL_KEY_PIN) CONDITION_FOR_THE_BUTTON_PRESSED) result |=CENTRAL_KEY_MASK;
return result;


}
