#include "menu.h"




menuItem	Null_Menu = {(void*)0, (void*)0, (void*)0, (void*)0, 0, {0x00}};
volatile menuItem* selectedMenuItem; // ������� ����� ����
 



char strNULL[] PROGMEM = "";


//                 NEXT,      PREVIOUS     PARENT,     CHILD
MAKE_MENU(m_s1i1,  m_s1i2,    NULL_ENTRY,  NULL_ENTRY, m_s2i1,       0, "�������");
MAKE_MENU(m_s1i2,  m_s1i3,    m_s1i1,      NULL_ENTRY, m_s3i1,       0, "�����.");
MAKE_MENU(m_s1i3,  NULL_ENTRY,m_s1i2,      NULL_ENTRY, NULL_ENTRY,   0, "�����");
 
// ������� ������
MAKE_MENU(m_s2i1,  m_s2i2,    NULL_ENTRY,  m_s1i1,     NULL_ENTRY,   MENU_MODE1, "�����.�.");
MAKE_MENU(m_s2i2,  m_s2i3,    m_s2i1,      m_s1i1,     NULL_ENTRY,   MENU_MODE2, "�����.�.");
MAKE_MENU(m_s2i3,  m_s2i4,    m_s2i2,      m_s1i1,     NULL_ENTRY,   MENU_MODE3, "��������");
MAKE_MENU(m_s2i4,  NULL_ENTRY,m_s2i3,      m_s1i1,     NULL_ENTRY,   MENU_MODE3, "��������");
 
// ������� ���������
MAKE_MENU(m_s3i1,  m_s3i2,    NULL_ENTRY,  m_s1i2,     NULL_ENTRY,   m_slot_2, "����. ��");
MAKE_MENU(m_s3i2,  m_s3i3,    m_s3i1,      m_s1i2,     m_s5i1,       0,        "COM-����");
MAKE_MENU(m_s3i3,  NULL_ENTRY,m_s3i2,      m_s1i2,     NULL_ENTRY,   m_slot_1, "��������"); 
// ������� ��������
MAKE_MENU(m_s4i1,  m_s4i2,    NULL_ENTRY,  m_s3i1,     NULL_ENTRY,   MENU_SENS1, "������ 1");
MAKE_MENU(m_s4i2,  NULL_ENTRY,m_s4i1,      m_s3i1,     NULL_ENTRY,   MENU_SENS2, "������ 2");

// ������� �����
MAKE_MENU(m_s5i1,  m_s5i2,    NULL_ENTRY,  m_s3i2,     NULL_ENTRY,   MENU_WARM, "��������");
MAKE_MENU(m_s5i2,  NULL_ENTRY,m_s5i1,      m_s3i2,     NULL_ENTRY,   MENU_PROCESS, "�������");






extern void lcd_clrscr(void);
extern void lcd_gotoxy(uint8_t, uint8_t);
extern void USART_SendStr(char * data);



void menuChange(menuItem* NewMenu)
{
	if ((void*)NewMenu == (void*)&NULL_ENTRY)
	  return;
 
	selectedMenuItem = NewMenu;
}


char* menuText(int8_t menuShift)
{
	int8_t i;
	menuItem* tempMenu;
 
	if ((void*)selectedMenuItem == (void*)&NULL_ENTRY)
	  	return strNULL;
 	//	return ((char*)"");	
	i = menuShift;
	tempMenu = selectedMenuItem;
	if (i>0) {
		while( i!=0 ) {
			if ((void*)tempMenu != (void*)&NULL_ENTRY) {
				tempMenu = (menuItem*)pgm_read_word(&tempMenu->Next);
			}
			i--;
		}
	} else {
		while( i!=0 ) {
			if ((void*)tempMenu != (void*)&NULL_ENTRY) {
				tempMenu = (menuItem*)pgm_read_word(&tempMenu->Previous);
			}
			i++;
		}
	}
 
	if ((void*)tempMenu == (void*)&NULL_ENTRY) {
		return strNULL;
	//	return ((char*)"");	
	} else {
		return ((char *)tempMenu->Text);
	}
}


unsigned char dispMenu(void) {
	menuItem* tempMenu;

	lcd_clrscr();
	
	
	// ������ ������ - ���������. ��� ����� ���� �������� ������
//	lcd_gotoxy(0,0);
	lcd_home();
	tempMenu = (menuItem*)pgm_read_word(&selectedMenuItem->Parent);
	if ((void*)tempMenu == (void*)&NULL_ENTRY) { // �� �� ������� ������
		//lcd_puts("MENU:");
		lcd_puts_p(PSTR("����:"));
	} else {
		//lcd_puts_p((char *)tempMenu->Text);
		lcd_puts_p((char *)tempMenu->Text);
	}

	//lcd_clrscr(2);
	// ������ ������ - ������� ����� ����
	lcd_gotoxy(0,1);
	lcd_puts_p((char *)selectedMenuItem->Text);

//	USART_SendStrP((char *)selectedMenuItem->Text);
	return (1);
}



void startMenu(void) {
	selectedMenuItem = (menuItem*)&m_s1i1;
 
	dispMenu();
}






