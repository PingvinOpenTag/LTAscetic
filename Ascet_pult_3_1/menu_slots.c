 //Обработчики событий при выборе пунктов меню, для которых поределены действия
 void menu_slot_1(void){
 if (get_pult_default_settings()) pult_reset_to_defaul();
 }

 void menu_slot_2(void){
	extern volatile EEMEM uint8_t eeprom_curr_ir_pin; 
	get_ir_power_settings();
	curr_ir_pin=eeprom_read_byte(&eeprom_curr_ir_pin);
 }
