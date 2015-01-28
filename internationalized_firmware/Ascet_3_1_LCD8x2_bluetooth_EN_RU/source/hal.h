#ifndef F_CPU						//Ётот параметр нужно задавать в свойствах проекта, если там не задан, задаем здесь
#define F_CPU 16000000  			// “актова€ частота в √ц
#endif



#define LIFE_LED1_ON LIFE_LED1_PORT|=LIFE_LED1_PIN
#define LIFE_LED1_OFF LIFE_LED1_PORT&=~LIFE_LED1_PIN
#define LIFE_LED2_ON LIFE_LED2_PORT|=LIFE_LED2_PIN
#define LIFE_LED2_OFF LIFE_LED2_PORT&=~LIFE_LED2_PIN
#define LIFE_LED3_ON LIFE_LED3_PORT|=LIFE_LED3_PIN
#define LIFE_LED3_OFF LIFE_LED3_PORT&=~LIFE_LED3_PIN
#define LIFE_LED4_ON LIFE_LED4_PORT|=LIFE_LED4_PIN
#define LIFE_LED4_OFF LIFE_LED4_PORT&=~LIFE_LED4_PIN
#define FIRE_LED_ON FIRE_LED_PORT|=FIRE_LED_PIN
#define FIRE_LED_OFF FIRE_LED_PORT&=~FIRE_LED_PIN
#define BT_STATE_LED_ON BT_STATE_LED_PORT|=BT_STATE_LED_PIN
#define BT_STATE_LED_OFF BT_STATE_LED_PORT&=~BT_STATE_LED_PIN
#define BULLETS_OUT_LED_ON BULLETS_OUT_LED_PORT|=BULLETS_OUT_LED_PIN
#define BULLETS_OUT_LED_OFF BULLETS_OUT_LED_PORT&=~BULLETS_OUT_LED_PIN
#define	WOUND_LED_ON WOUND_LED_PORT|=WOUND_LED_PIN
#define	WOUND_LED_OFF WOUND_LED_PORT&=~WOUND_LED_PIN
#define	WOUND_LED_INVERT WOUND_LED_PORT^=WOUND_LED_PIN
#define SOUND_ON SOUND_CONTROL_PORT|=SOUND_CONTROL_PIN
#define SOUND_OFF SOUND_CONTROL_PORT&=~SOUND_CONTROL_PIN

//#define IR_LED_INVERT IR_LED_PORT^=IR_LED_PIN
#define IR_LED_INVERT IR_LED_PORT^=curr_ir_pin



#define IR_LED_HIGH_POWER_OFF IR_LED_PORT&=~IR_LED_HIGH_POWER_PIN

#define IR_LED_LOW_POWER_OFF IR_LED_PORT&=~IR_LED_LOW_POWER_PIN

#define IR_LED_OFF IR_LED_HIGH_POWER_OFF;IR_LED_LOW_POWER_OFF


#define LIFE_LED1_INVERT LIFE_LED1_PORT^=LIFE_LED1_PIN
#define FIRE_LED_INVERT FIRE_LED_PORT^=FIRE_LED_PIN



#define OFF 0b00000000
#define FQR_1HZ 0b11110000
#define FQR_2HZ 0b11001100
#define FQR_4HZ 0b10101010
#define ON 0b11111111

#define SHORT_DURATION 4 	//минимальна€ длительность (в "тиках") непрерывного нажати€ курка, 
							//необходима€ дл€ фиксации событи€ "курок нажат"


#define CUT_OFF_SOUNT 10 //ƒлительность звука в режиме "очередь" в процентах от длительности звука в режиме "одиночный"


#define CYCLIC_RATE eeprom_read_word(&eeprom_cyclic_rate)//250

#define DEFAULT_BATT_FULL_VOLTAGE 6800 //напр€жение полностью зар€женной батареи в милливольтах

#define DEFAULT_BATT_LOW_VOLTAGE 5500 //напр€жение полностью разр€женной батареи в милливольтах

#define DEFAULT_IR_ERROR_IGNORE 80 //игнорируем ошибки приема в течение 1 с после выстрела