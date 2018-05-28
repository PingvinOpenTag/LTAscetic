#define IR_START_BIT_DURATION 2400	// Длительность Старт-Бита (в микросекундах)
#define IR_ONE_BIT_DURATION 1200	// Длительность Бита, соотретствующего единичке (в микросекундах)
#define IR_ZERO_BIT_DURATION 600	// Длительность Бита, соотретствующего нулю (в микросекундах)
#define IR_SPACE_DURATION 600		// Длительность Бита, соотретствующего интервалу между битами (в микросекундах)
//#define IR_F0 miles_protocol.carrier_frequency /*56000*/					// Несущая частота ИК-приемника (f0)
#define IR_F0 36000
#define ERROR_TOLERANCE miles_protocol.err_tolerance //14//22 //Допустимая погрешность длительности принимаемых бит (в "тиках" таймера)


#define ERROR_TOLERANCE_FOR_36KHZ 12//14
#define ERROR_TOLERANCE_FOR_56KHZ 19//22


#define DEFAULT_IR_F0 36000					// Несущая частота ИК-приемника (f0)
#define DEFAULT_ERROR_TOLERANCE 12//14 //Допустимая погрешность длительности принимаемых бит (в "тиках" таймера)


#define IR_ZERO miles_protocol.ir_zero//(IR_ZERO_BIT_DURATION*2*IR_F0)/1000000 //Длительность импулся, соответствующего биту = 0
													  //выраженная в "тиках" таймера 


#define RX_IR_ZERO (IR_ZERO_BIT_DURATION*8)/1000 //Длительность импулся, соответствующего биту = 0
													  //выраженная в "тиках" таймера 



#define IR_ONE miles_protocol.ir_one//(IR_ONE_BIT_DURATION*2*IR_F0)/1000000 //Длительность импулся, соответствующего биту = 1
													  //выраженная в "тиках" таймера 

#define RX_IR_ONE (IR_ONE_BIT_DURATION*8)/1000 //Длительность импулся, соответствующего биту = 1
													  //выраженная в "тиках" таймера 



#define IR_START miles_protocol.ir_start//(IR_START_BIT_DURATION*2*IR_F0)/1000000 //Длительность заголовка 
														//выраженная в "тиках" таймера 


#define RX_IR_START (IR_START_BIT_DURATION*8)/1000 //Длительность заголовка 
														//выраженная в "тиках" таймера 


#define IR_SPACE  miles_protocol.ir_space//(IR_SPACE_DURATION*2*IR_F0)/1000000 //Длительность интервала между передоваемыми битами
														//выраженная в "тиках" таймера 


#define RX_IR_SPACE  (IR_SPACE_DURATION*8)/1000 //Длительность интервала между передоваемыми битами
														//выраженная в "тиках" таймера 


#define  SAFE_DURATION eeprom_read_byte(&eeprom_safe_duration)//5      //время (в десятых секунды) неуязвимости после ранения




//#define ERROR_TOLERANCE 14

//#define ERROR_TOLERANCE 2


//---------------------------------------------------------------------//

enum Team_Color {	//Определим перечисляемый тип для работы с цветом команды
	Red, 	//00 = Red
	Blue, 	//01 = Blue		
	Yellow, //10 = Yellow
	Green}; //11 = Green

typedef enum Team_Color tteam_color;			

//---------------------------------------------------------------------//





//Определим перечисляемый тип
//для работы с "уроном"
enum GunDamage {	
	
	Damage_1,  //0000 = 1
	Damage_2,  //0001 = 2
	Damage_4,  //0010 = 4
	Damage_5,  //0011 = 5
	Damage_7,  //0100 = 7
	Damage_10, //0101 = 10
	Damage_15, //0110 = 15
	Damage_17, //0111 = 17
	Damage_20, //1000 = 20
	Damage_25, //1001 = 25
	Damage_30, //1010 = 30
	Damage_35, //1011 = 35
	Damage_40, //1100 = 40
	Damage_50, //1101 = 50
	Damage_75, //1110 = 75
	Damage_100 //1111 = 100
	};
	
	    								
typedef enum GunDamage tgun_damage; 	




//---------------------------------------------------------------------//


/*
union player_id_union {
tplayer_id bits;
uint8_t data [8];
};
*/

#define Add_Health 0x80 //сообщение добавить жизни
#define Add_Rounds 0x81 //сообщение добавить патронов
#define Change_color 0xA9//сообщение сменить цвет команды 
#define Command 0x83 //сообщение-команда
#define Valid_value 0xE8 //корректное значение контрольного байта сообщения
#define Set_life 0xA3 //установить хх жизней
#define Set_damage 0xA8 //установить хх урон
