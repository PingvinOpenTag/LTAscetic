#ifndef MENU_H_
#define MENU_H_

#include <avr/io.h>        // Определяет имена для портов ввода-ывода
#include <avr/pgmspace.h>  //Будем хранить константы в памяти программ


// для начала — пустой элемент. Который NULL на рисунке
#define NULL_ENTRY Null_Menu

typedef struct PROGMEM{
	void       *Next;
	void       *Previous;
	void       *Parent;
	void       *Child;
	uint8_t     Select;
	const char  Text[];
} menuItem;


#define MAKE_MENU(Name, Next, Previous, Parent, Child, Select, Text) \
	extern menuItem Next;     \
	extern menuItem Previous; \
	extern menuItem Parent;   \
	extern menuItem Child;  \
	menuItem Name = {(void*)&Next, (void*)&Previous, (void*)&Parent, (void*)&Child, (uint8_t)Select, { Text }}





enum {
    m_slot_1=1,
    m_slot_2,
    MENU_MODE1,
    MENU_MODE2,
    MENU_MODE3,
    MENU_SENS1,
    MENU_SENS2,
	MENU_WARM,
	MENU_PROCESS,
};





#define PREVIOUS   ((menuItem*)pgm_read_word(&selectedMenuItem->Previous))
#define NEXT       ((menuItem*)pgm_read_word(&selectedMenuItem->Next))
#define PARENT     ((menuItem*)pgm_read_word(&selectedMenuItem->Parent))
#define CHILD      ((menuItem*)pgm_read_word(&selectedMenuItem->Child))
#define SELECT	    (pgm_read_byte(&selectedMenuItem->Select))


extern void menuChange(menuItem*);
extern char* menuText(int8_t);
extern unsigned char dispMenu(void);
extern void startMenu(void); 
extern volatile menuItem* selectedMenuItem; // текущий пункт меню
extern menuItem	Null_Menu;

//extern void menu_slot_1(void);
//extern void menu_slot_2(void);

#endif

