#define UP_KEY_PORT PORTA //Порт, к которому подключен переключатель "DAMAGE" (урон)
#define UP_KEY_DDR DDRA 
#define UP_KEY_IN PINA
#define UP_KEY_PIN (1<<3)
#define UP_KEY_MASK (1<<0)



#define RIGHT_KEY_PORT PORTA //Порт, к которому подключен переключатель "DAMAGE" (урон)
#define RIGHT_KEY_DDR DDRA 
#define RIGHT_KEY_IN PINA
#define RIGHT_KEY_PIN (1<<2)
#define RIGHT_KEY_MASK (1<<1)


#define DOWN_KEY_PORT PORTA //Порт, к которому подключен переключатель "DAMAGE" (урон)
#define DOWN_KEY_DDR DDRA 
#define DOWN_KEY_IN PINA
#define DOWN_KEY_PIN (1<<5)
#define DOWN_KEY_MASK (1<<2)

#define LEFT_KEY_PORT PORTA //Порт, к которому подключен переключатель "DAMAGE" (урон)
#define LEFT_KEY_DDR DDRA 
#define LEFT_KEY_IN PINA
#define LEFT_KEY_PIN (1<<6)
#define LEFT_KEY_MASK (1<<3)



#define CENTRAL_KEY_PORT PORTA //Порт, к которому подключен переключатель "DAMAGE" (урон)
#define CENTRAL_KEY_DDR DDRA 
#define CENTRAL_KEY_IN PINA
#define CENTRAL_KEY_PIN (1<<4)
#define CENTRAL_KEY_MASK (1<<4)


//tck
#define CONDITION_FOR_THE_BUTTON_PRESSED == 0 

//#define CONDITION_FOR_THE_BUTTON_PRESSED != 0 


enum typjoystick_event  {
			no_pressing,
			key_up_pressing,
			key_right_pressing,
			key_down_pressing,
			key_left_pressing,
			key_central_pressing
	        };
typedef enum typjoystick_event TJOYSTICK_EVENT;



enum typjoystick_status {
			no_pressed,
			key_up_pressed,
			key_right_pressed,
			key_down_pressed,
			key_left_pressed,
			key_central_pressed
			};
typedef enum typjoystick_status TJOYSTICK_STATUS;


struct joystick_pressing_duration
                 { uint16_t key_up      ; // 
                                             //ia?aoey eiiiee 1
                   unsigned int key_up_inc:1; //aeo, ?ac?aoa?uee 
                                             //eee cai?aua?uee ion?ao
                                             //aeeoaeuiinoe ia?aoey eiiiee 1  
                   uint16_t key_right    ; //aeeoaeuiinou iai?a?uaiiai 
                                             //ia?aoey eiiiee 2
                   unsigned int key_right_inc:1; //aeo, ?ac?aoa?uee 
                                             //eee cai?aua?uee ion?ao
                                             //aeeoaeuiinoe ia?aoey eiiiee 2 
                   uint16_t key_down  ;
                   unsigned int key_down_inc:1;
				   
 				   uint16_t key_left  ;
				   unsigned int key_left_inc:1;

				   uint16_t key_central  ;
				   unsigned int key_central_inc:1;



				   unsigned int no_key   ;
                 };



void init_joystick(void);
TJOYSTICK_EVENT test_joystick(void);
uint8_t get_joystick_status(void);
