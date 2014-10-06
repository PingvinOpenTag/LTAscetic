#define IR_START_BIT_DURATION 2400	// Длительность Старт-Бита (в микросекундах)
#define IR_ONE_BIT_DURATION 1200	// Длительность Бита, соотретствующего единичке (в микросекундах)
#define IR_ZERO_BIT_DURATION 600	// Длительность Бита, соотретствующего нулю (в микросекундах)
#define IR_SPACE_DURATION 600		// Длительность Бита, соотретствующего интервалу между битами (в микросекундах)
//#define IR_F0 56000					// Несущая частота ИК-приемника (f0)
#define IR_F0 36000

#define IR_ZERO (IR_ZERO_BIT_DURATION*2*IR_F0)/1000000 //Длительность импулся, соответствующего биту = 0
													  //выраженная в "тиках" таймера 


#define RX_IR_ZERO (IR_ZERO_BIT_DURATION*8)/1000 //Длительность импулся, соответствующего биту = 0
													  //выраженная в "тиках" таймера 



#define IR_ONE (IR_ONE_BIT_DURATION*2*IR_F0)/1000000 //Длительность импулся, соответствующего биту = 1
													  //выраженная в "тиках" таймера 

#define RX_IR_ONE (IR_ONE_BIT_DURATION*8)/1000 //Длительность импулся, соответствующего биту = 1
													  //выраженная в "тиках" таймера 



#define IR_START (IR_START_BIT_DURATION*2*IR_F0)/1000000 //Длительность заголовка 
														//выраженная в "тиках" таймера 


#define RX_IR_START (IR_START_BIT_DURATION*8)/1000 //Длительность заголовка 
														//выраженная в "тиках" таймера 


#define IR_SPACE  (IR_SPACE_DURATION*2*IR_F0)/1000000 //Длительность интервала между передоваемыми битами
														//выраженная в "тиках" таймера 


#define RX_IR_SPACE  (IR_SPACE_DURATION*8)/1000 //Длительность интервала между передоваемыми битами
														//выраженная в "тиках" таймера 





//#define ERROR_TOLERANCE 22//14 //Допустимая погрешность длительности принимаемых бит (в "тиках" таймера)

#define ERROR_TOLERANCE 14

//#define ERROR_TOLERANCE 22


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

typedef struct message { //структура для передаваемого сообщения
uint8_t DATA;
uint8_t ID;
} tmessage;


typedef union command_union {
uint16_t raw;
tmessage message;

} tcommand_union;

//miles протокол идентификаторы сообщения
#define ID_Add_Health 			0x80 //сообщение добавить жизни
#define ID_Add_Rounds 			0x81 //сообщение добавить патронов
#define ID_Command 				0x83 //сообщение-команда
#define ID_Clips_Pickup 		0x8A //сообщение 
#define ID_Health_Pickup 		0x8B 
#define ID_Flag_Pickup			0x8C

//LW дополнительные идентификаторы сообщения
#define ID_Radiation 			0xA0
#define ID_Anomaly 				0xA1
#define ID_Presetting 			0xA8
#define ID_Change_Color			0xA9
#define ID_Play_Sound			0xAA


//miles команды 
#define Command_ImmediateNewGame 	0x05 //команда "новая игра"
#define Command_Admin_Kill 			0x00 //команда "выключения" игрока
#define Command_Full_Health			0x0D //жизни - 100%
#define Command_Full_Ammo			0x06 //полные патроны

//LW дополнительные команды

#define Command_IR_Power_Change		0x0E
#define Command_Health_Double 		0x10

//контрольный байт
#define Control_Byte 				0xE8 //корректное значение контрольного байта сообщения

