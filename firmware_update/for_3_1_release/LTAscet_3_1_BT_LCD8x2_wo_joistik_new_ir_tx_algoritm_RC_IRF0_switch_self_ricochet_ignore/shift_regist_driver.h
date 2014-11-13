
#define SHIFT_REGISTER_DS_PORT PORTB //порт, к которуму подключен вход DS (данные) сдвигового регистра
#define SHIFT_REGISTER_SH_CP_PORT PORTB //порт, к которуму подключен вход SH_CP (тактовые импульсы) сдвигового регистра
#define SHIFT_REGISTER_ST_CP_PORT PORTB //порт, к которуму подключен вход ST_CP ("защлка" данных) сдвигового регистра

#define SHIFT_REGISTER_DS_DDR DDRB //управл€ющий регистр, к которуму подключен вход DS (данные) сдвигового регистра
#define SHIFT_REGISTER_SH_CP_DDR DDRB //управл€ющий регистр, к которуму подключен вход SH_CP (тактовые импульсы) сдвигового регистра
#define SHIFT_REGISTER_ST_CP_DDR DDRB //управл€ющий регистр, к которуму подключен вход ST_CP ("защлка" данных) сдвигового регистра

#define SHIFT_REGISTER_DS_PIN (1<<5) //вывод, к которуму подключен вход DS (данные) сдвигового регистра
#define SHIFT_REGISTER_SH_CP_PIN (1<<7) //вывод, к которуму подключен вход SH_CP (тактовые импульсы) сдвигового регистра
#define SHIFT_REGISTER_ST_CP_PIN (1<<6) //вывод, к которуму подключен вход ST_CP ("защлка" данных) сдвигового регистра




//все что ниже, при переопределении портов и пинов не трогаем!

#define SIHFT_REGISTER_DS_ON SHIFT_REGISTER_DS_PORT|=SHIFT_REGISTER_DS_PIN //выставл€ем в "1" сигнал DS  
#define SIHFT_REGISTER_DS_OFF SHIFT_REGISTER_DS_PORT&=~SHIFT_REGISTER_DS_PIN //выставл€ем в "0" сигнал DS  

#define SHIFT_REGISTER_SH_CP_ON SHIFT_REGISTER_SH_CP_PORT|=SHIFT_REGISTER_SH_CP_PIN //выставл€ем в "1" сигнал SH_CP
#define SHIFT_REGISTER_SH_CP_OFF SHIFT_REGISTER_SH_CP_PORT&=~SHIFT_REGISTER_SH_CP_PIN //выставл€ем в "0" сигнал SH_CP

#define SHIFT_REGISTER_ST_CP_ON SHIFT_REGISTER_ST_CP_PORT|=SHIFT_REGISTER_ST_CP_PIN ////выставл€ем в "1" сигнал ST_CP
#define SHIFT_REGISTER_ST_CP_OFF SHIFT_REGISTER_ST_CP_PORT&=~SHIFT_REGISTER_ST_CP_PIN ////выставл€ем в "0" сигнал ST_CP 


//настраиваем все управл€ющие сдвиговым регистром выводы как "выходы"
void init_shift_register(void){ //
SHIFT_REGISTER_DS_DDR|=SHIFT_REGISTER_DS_PIN;
SHIFT_REGISTER_SH_CP_DDR|= SHIFT_REGISTER_SH_CP_PIN;
SHIFT_REGISTER_ST_CP_DDR|= SHIFT_REGISTER_ST_CP_PIN;

SIHFT_REGISTER_DS_OFF;
SHIFT_REGISTER_SH_CP_OFF;
SHIFT_REGISTER_ST_CP_OFF;
};

void shift_register_set_data(volatile uint8_t shift_data){
volatile uint8_t mask;
mask = 0b10000000;
for (int i=0; i<8; i++)	
	{
		if ((mask&shift_data) ==0) 
		{SIHFT_REGISTER_DS_OFF;}	
//		if ((mask&shift_data)!=0) 
//		{SIHFT_REGISTER_DS_ON;}	
		
		else 
		{SIHFT_REGISTER_DS_ON;}
		SHIFT_REGISTER_SH_CP_ON;
//--//		asm("nop");
		SHIFT_REGISTER_SH_CP_OFF;
		mask = (mask>>1);
	}
	
	SHIFT_REGISTER_ST_CP_OFF;
//--//	asm("nop");
	SHIFT_REGISTER_ST_CP_ON;
}

void shift_register_clean(void){
shift_register_set_data(0);
}

