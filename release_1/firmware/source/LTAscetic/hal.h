#ifndef F_CPU						//Этот параметр нужно задавать в свойствах проекта, если там не задан, задаем здесь
#define F_CPU 16000000  			// Тактовая частота в Гц
#endif



#define LIFE_LED1_ON LIFE_LEDS_PORT|=LIFE_LED1_PIN
#define LIFE_LED1_OFF LIFE_LEDS_PORT&=~LIFE_LED1_PIN
#define LIFE_LED2_ON LIFE_LEDS_PORT|=LIFE_LED2_PIN
#define LIFE_LED2_OFF LIFE_LEDS_PORT&=~LIFE_LED2_PIN
#define LIFE_LED3_ON LIFE_LEDS_PORT|=LIFE_LED3_PIN
#define LIFE_LED3_OFF LIFE_LEDS_PORT&=~LIFE_LED3_PIN
#define LIFE_LED4_ON LIFE_LEDS_PORT|=LIFE_LED4_PIN
#define LIFE_LED4_OFF LIFE_LEDS_PORT&=~LIFE_LED4_PIN
#define FIRE_LED_ON FIRE_LED_PORT|=FIRE_LED_PIN
#define FIRE_LED_OFF FIRE_LED_PORT&=~FIRE_LED_PIN
#define BULLETS_OUT_LED_ON BULLETS_OUT_LED_PORT|=BULLETS_OUT_LED_PIN
#define BULLETS_OUT_LED_OFF BULLETS_OUT_LED_PORT&=~BULLETS_OUT_LED_PIN
#define	WOUND_LED_ON WOUND_LED_PORT|=WOUND_LED_PIN
#define	WOUND_LED_OFF WOUND_LED_PORT&=~WOUND_LED_PIN
#define	WOUND_LED_INVERT WOUND_LED_PORT^=WOUND_LED_PIN
#define IR_LED_INVERT IR_LED_PORT^=IR_LED_PIN
#define IR_LED_OFF IR_LED_PORT&=~IR_LED_PIN
#define LIFE_LED1_INVERT LIFE_LEDS_PORT^=LIFE_LED1_PIN
#define FIRE_LED_INVERT FIRE_LED_PORT^=FIRE_LED_PIN



#define OFF 0b00000000
#define FQR_1HZ 0b11110000
#define FQR_2HZ 0b11001100
#define FQR_4HZ 0b10101010
#define ON 0b11111111

#define SHORT_DURATION 4 	//минимальная длительность (в "тиках") непрерывного нажатия курка, 
							//необходимая для фиксации события "курок нажат"


#define CUT_OFF_SOUNT 10 //Длительность звука в режиме "очередь" в процентах от длительности звука в режиме "одиночный"



