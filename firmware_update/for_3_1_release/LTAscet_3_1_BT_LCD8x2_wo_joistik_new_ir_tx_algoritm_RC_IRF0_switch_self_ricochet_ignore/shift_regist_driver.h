
#define SHIFT_REGISTER_DS_PORT PORTB //����, � �������� ��������� ���� DS (������) ���������� ��������
#define SHIFT_REGISTER_SH_CP_PORT PORTB //����, � �������� ��������� ���� SH_CP (�������� ��������) ���������� ��������
#define SHIFT_REGISTER_ST_CP_PORT PORTB //����, � �������� ��������� ���� ST_CP ("������" ������) ���������� ��������

#define SHIFT_REGISTER_DS_DDR DDRB //����������� �������, � �������� ��������� ���� DS (������) ���������� ��������
#define SHIFT_REGISTER_SH_CP_DDR DDRB //����������� �������, � �������� ��������� ���� SH_CP (�������� ��������) ���������� ��������
#define SHIFT_REGISTER_ST_CP_DDR DDRB //����������� �������, � �������� ��������� ���� ST_CP ("������" ������) ���������� ��������

#define SHIFT_REGISTER_DS_PIN (1<<5) //�����, � �������� ��������� ���� DS (������) ���������� ��������
#define SHIFT_REGISTER_SH_CP_PIN (1<<7) //�����, � �������� ��������� ���� SH_CP (�������� ��������) ���������� ��������
#define SHIFT_REGISTER_ST_CP_PIN (1<<6) //�����, � �������� ��������� ���� ST_CP ("������" ������) ���������� ��������




//��� ��� ����, ��� ��������������� ������ � ����� �� �������!

#define SIHFT_REGISTER_DS_ON SHIFT_REGISTER_DS_PORT|=SHIFT_REGISTER_DS_PIN //���������� � "1" ������ DS  
#define SIHFT_REGISTER_DS_OFF SHIFT_REGISTER_DS_PORT&=~SHIFT_REGISTER_DS_PIN //���������� � "0" ������ DS  

#define SHIFT_REGISTER_SH_CP_ON SHIFT_REGISTER_SH_CP_PORT|=SHIFT_REGISTER_SH_CP_PIN //���������� � "1" ������ SH_CP
#define SHIFT_REGISTER_SH_CP_OFF SHIFT_REGISTER_SH_CP_PORT&=~SHIFT_REGISTER_SH_CP_PIN //���������� � "0" ������ SH_CP

#define SHIFT_REGISTER_ST_CP_ON SHIFT_REGISTER_ST_CP_PORT|=SHIFT_REGISTER_ST_CP_PIN ////���������� � "1" ������ ST_CP
#define SHIFT_REGISTER_ST_CP_OFF SHIFT_REGISTER_ST_CP_PORT&=~SHIFT_REGISTER_ST_CP_PIN ////���������� � "0" ������ ST_CP 


//����������� ��� ����������� ��������� ��������� ������ ��� "������"
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

