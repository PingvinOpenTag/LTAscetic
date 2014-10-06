 #ifndef MENU_SLOTS_H_
 #define MENU_SLOTS_H_

 
 //Обработчики событий при выборе пунктов меню, для которых поределены действия
 void menu_slot_1(void){
 if (get_pult_default_settings()) pult_reset_to_defaul();
 }

 void menu_slot_2(void){
	get_ir_power_settings();
	curr_ir_pin=eeprom_read_byte(&eeprom_curr_ir_pin);
 }

#endif
