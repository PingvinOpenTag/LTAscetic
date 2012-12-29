#include "joystick_driver_types.h"

#define RX_BUFFER_SIZE   8 //Размер буффера приемника
#define TX_BUFFER_SIZE   8 //Размер буффера приемника

#ifndef bool

                        #define bool unsigned char

                        #define true 1

                        #define false 0

#endif



//---------------------------------------------------------------------//


//Определим перечисляемый тип для событий ИК-приемника
enum Rx_Event {	
				NOT_EVENT, 		//нет событий
				RX_COMPLETE, 	//принят пакет
				RX_ERROR		//ошибка приема пакета
				}; 
typedef enum Rx_Event trx_event;














//---------------------------------------------------------------------//

//Определим структуру для хранения идентификатора игрока
//в ней будем хранить длительность импулсов  в "тиках" таймера (IR_ZERO или IR_ONE)
typedef struct PlayerID {
uint8_t bit_7;//последний бит (всегда должен быть равен IR_ZERO)
uint8_t bit_6; 
uint8_t bit_5; 
uint8_t bit_4;
uint8_t bit_3;
uint8_t bit_2;
uint8_t bit_1;
uint8_t bit_0;//Первый бит
} tplayer_id;

/*
union player_id_union {
tplayer_id bits;
uint8_t data [8];
};
*/


//Определим структуру для хранения идентификатора игрока
//в ней будем хранить длительность импулсов в "тиках" таймера (IR_ZERO или IR_ONE) 
typedef struct TeamID{
uint8_t bit_1;
uint8_t bit_0;
} tteam_id;
 


typedef struct Damage{
uint8_t bit_3; 
uint8_t bit_2; 
uint8_t bit_1; 
uint8_t bit_0;

} tdamage;




//Теперь опишем структуру пакета данных
typedef struct DataPacket {
uint8_t header; //заголовок, всегда должен быть равен IR_START (1 Байт)
tplayer_id player_id; //после заголовка идет идентификатор игрока (8 Байт)
tteam_id team_id;	  //Затем идентификатор команды (2 Байта)
tdamage damage;		  //Ну и последним стоит "урон" (4 Байта)
uint8_t end_of_data;  //Метка, указывающая передатчику, что данных для отправки больше нет (всегда должна быть равна 0) (1 Байт)
} tdata_packet;

//---------------------------------------------------------------------//

union data_packet_union{
tdata_packet packet;
uint8_t data[16];
};





//Опишем структуру, в которой будем хранить данные, считанные из полученного пакета
typedef struct RX_Packet {
uint8_t player_id;	//идентификатор попавшего в нас игрока
uint8_t team_id;	//идентификатор (цвет) его команды
uint8_t damage;		//приченённый нам урон
} trx_packet;


//Опишем структуру, в которой будем хранить адрес воспроизводимого дополнительного звука
typedef struct SoundAddres {
uint8_t *start_adress; //адрес первого сеймпла 
uint8_t *end_adress; //адрес последнего сеймпла
uint8_t *curr_adress; //адрес очередного сеймпла, который нужно воспроизвести 

} tsnd_adress;


enum typkeyboard_status {
			no_key_pressed,
			key_pressed
			};
typedef enum typkeyboard_status TKEYBOARD_STATUS;

enum typkeyboard_event  {
			no_key_pressing,
			key_pressing,
	        };
typedef enum typkeyboard_event TKEYBOARD_EVENT;


struct pressing_duration
		{ uint16_t key_1      ; //длительность непрерывного 
					    //нажатия кнопки 1 ("Клавиша огня")
		  unsigned int key_1_inc:1; //бит, разрешающий 
					    //или запрещающий отсчет
					    //длительности нажатия кнопки 1  
		  uint16_t key_2      ; //длительность непрерывного 
					    //нажатия кнопки 2 ("Перезарядить")
		  unsigned int key_2_inc:1; //бит, разрешающий 
					    //или запрещающий отсчет
					    //длительности нажатия кнопки 2  
		  
		  
		  
		  unsigned int no_key   ;

		};



enum tfire_mode_status {
single,
queues
};
typedef enum tfire_mode_status TFIRE_MODE_STATUS;





enum tdisplay_batt_mode {
icon,
digit
};
typedef enum tdisplay_batt_mode TDISPLAY_BATT_MODE;




//Определим структуру для хранения серийного номеры админского
//ключа Touch Memory
typedef struct TM_serial_num {
unsigned char device_code;//код устройства
unsigned char serial[6]; //серийный номер ключа
} ttm_serial_num;

/*

enum sound_buffer{
sound_buffer_1,
sound_buffer_2
};

typedef enum sound_buffer TSOUND_BUFFER;
*/



