#include "shift_regist_driver.h"

//#include "hal.h"
//#include <inttypes.h>
//#include <avr/pgmspace.h>
#include <util/delay.h>    // Дает возможность формирования задержки

#define Q0 (1<<0)
#define Q1 (1<<1)
#define Q2 (1<<2)
#define Q3 (1<<3)
#define Q4 (1<<4)
#define Q5 (1<<5)
#define Q6 (1<<6)
#define Q7 (1<<7)

//здесь определяем, на каких выводах сдвигового регистра сидять выводы LCD-дисплея

/*

#define RS Q2
#define E Q3
#define D4 Q4
#define D5 Q5
#define D6 Q6
#define D7 Q7
#define BL Q1
*/
#define RS Q1
#define E Q2
#define D4 Q3
#define D5 Q4
#define D6 Q5
#define D7 Q6
#define BL Q7

#define lcd_update() shift_register_set_data(lcd_buffer)


#define bl_high lcd_buffer|=BL //выставляем бит BL (подсветка) в буфере
#define bl_low lcd_buffer&=~BL //сбрасываем бит BL в буфере

#define lcd_bl_high() bl_high;lcd_update()  //выставляем бит BL непосредственно на сдвиговом регистре
#define lcd_bl_low() bl_low;lcd_update() //сбрасываем бит BL непосредственно на сдвиговом регистре

#define rs_high lcd_buffer|=RS //выставляем бит RS в буфере
#define rs_low lcd_buffer&=~RS //сбрасываем бит RS в буфере

#define lcd_rs_high() rs_high;lcd_update() //выставляем бит RS непосредственно на сдвиговом регистре
#define lcd_rs_low() rs_low;lcd_update() //сбрасываем бит RS непосредственно на сдвиговом регистре


#define e_high lcd_buffer|=E
#define e_low lcd_buffer&=~E

#define lcd_e_high() e_high ; lcd_update()
#define lcd_e_low() e_low ; lcd_update()



#define d4_high lcd_buffer|=D7
#define d4_low lcd_buffer&=~D7

#define lcd_d4_high() d4_high ; lcd_update();
#define lcd_d4_low() d4_low ; lcd_update()


#define data_0_high lcd_buffer|=D4
#define data_0_low lcd_buffer&=~D4

#define lcd_data_0_high() data_0_high ; lcd_update()
#define lcd_data_0_low() data_0_low ; lcd_update()




#define d5_high lcd_buffer|=D5
#define d5_low lcd_buffer&=~D5

#define lcd_d5_high() d5_high ; lcd_update()
#define lcd_d5_low() d5_low ; lcd_update()

#define data_1_high lcd_buffer|=D5
#define data_1_low lcd_buffer&=~D5

#define lcd_data_1_high() data_1_high ; lcd_update()
#define lcd_data_1_low() data_1_low ; lcd_update()




#define d6_high lcd_buffer|=D6
#define d6_low lcd_buffer&=~D6

#define lcd_d6_high() d6_high ; lcd_update()
#define lcd_d6_low() d6_low ; lcd_update()

#define data_2_high lcd_buffer|=D6
#define data_2_low lcd_buffer&=~D6

#define lcd_data_2_high() data_2_high ; lcd_update()
#define lcd_data_2_low() data_2_low ; lcd_update()


#define d7_high lcd_buffer|=D7
#define d7_low lcd_buffer&=~D7

#define lcd_d7_high() d7_high ; lcd_update()
#define lcd_d7_low() d7_low ; lcd_update()

#define data_3_high lcd_buffer|=D7
#define data_3_low lcd_buffer&=~D7

#define lcd_data_3_high() data_3_high ; lcd_update()
#define lcd_data_3_low() data_3_low ; lcd_update()

#define LCD_DATA0_PIN    D4            /**< pin for 4bit data bit 0  */
#define LCD_DATA1_PIN    D5           /**< pin for 4bit data bit 1  */
#define LCD_DATA2_PIN    D6           /**< pin for 4bit data bit 2  */
#define LCD_DATA3_PIN    D7           /**< pin for 4bit data bit 3  */

#define LCD_FUNCTION     D5           /* DB5: function set                   */
#define LCD_FUNCTION_8BIT     D4      /*   DB4: set 8BIT mode (0->4BIT mode) */
#define LCD_FUNCTION_4BIT     0       /*   DB4: set 8BIT mode (0->4BIT mode) */
#define LCD_CMD_FUNCTION     (1<<5)   /* DB5: function set  */   
#define LCD_CLR              (1<<0)   /* DB0: clear display                  */
#define LCD_ENTRY_MODE       (1<<2)   /* DB2: set entry mode                 */
#define LCD_ENTRY_INC        (1<<1)      /*   DB1: 1=increment, 0=decrement     */

#define LCD_CURSOR_HOME		 (1<<1)

#define LCD_HOME             (1<<1)      /* DB1: return to home position        */
/* instruction register bit positions, see HD44780U data sheet */



#define LCD_ENTRY_SHIFT       0      /*   DB2: 1=display shift on           */
#define LCD_ON                3      /* DB3: turn lcd/cursor on             */
#define LCD_ON_DISPLAY        2      /*   DB2: turn display on              */
#define LCD_ON_CURSOR         1      /*   DB1: turn cursor on               */
#define LCD_ON_BLINK          0      /*     DB0: blinking cursor ?          */
#define LCD_MOVE              4      /* DB4: move cursor/display            */
#define LCD_MOVE_DISP         3      /*   DB3: move display (0-> cursor) ?  */
#define LCD_MOVE_RIGHT        2      /*   DB2: move right (0-> left) ?      */


#define LCD_FUNCTION_2LINES   3      /*   DB3: two lines (0->one line)      */
#define LCD_FUNCTION_10DOTS   2      /*   DB2: 5x10 font (0->5x7 font)      */
#define LCD_CGRAM             6      /* DB6: set CG RAM address             */
#define LCD_DDRAM             7      /* DB7: set DD RAM address             */
#define LCD_BUSY              7      /* DB7: LCD is busy                    */

/* set entry mode: display shift on/off, dec/inc cursor move direction */
#define LCD_ENTRY_DEC            0x04   /* display shift off, dec cursor move dir */
#define LCD_ENTRY_DEC_SHIFT      0x05   /* display shift on,  dec cursor move dir */
#define LCD_ENTRY_INC_           0x06   /* display shift off, inc cursor move dir */
#define LCD_ENTRY_INC_SHIFT      0x07   /* display shift on,  inc cursor move dir */

/* display on/off, cursor on/off, blinking char at cursor position */
#define LCD_DISP_OFF             0x08   /* display off                            */
#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */
#define LCD_DISP_ON_BLINK        0x0D   /* display on, cursor off, blink char     */
#define LCD_DISP_ON_CURSOR       0x0E   /* display on, cursor on                  */
#define LCD_DISP_ON_CURSOR_BLINK 0x0F   /* display on, cursor on, blink char      */

/* move cursor/shift display */
#define LCD_MOVE_CURSOR_LEFT     0x10   /* move cursor left  (decrement)          */
#define LCD_MOVE_CURSOR_RIGHT    0x14   /* move cursor right (increment)          */
#define LCD_MOVE_DISP_LEFT       0x18   /* shift display left                     */
#define LCD_MOVE_DISP_RIGHT      0x1C   /* shift display right                    */

/* function set: set interface data length and number of display lines */
#define LCD_FUNCTION_4BIT_1LINE  0x20   /* 4-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_4BIT_2LINES 0x28   /* 4-bit interface, dual line,   5x7 dots */
#define LCD_FUNCTION_8BIT_1LINE  0x30   /* 8-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_8BIT_2LINES 0x38   /* 8-bit interface, dual line,   5x7 dots */


#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 
#define LCD_MODE_DEFAULT     ((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) )



#define LCD_LINES           2     /**< number of visible lines of the display */
#define LCD_DISP_LENGTH    16     /**< visibles characters per line of the display */
#define LCD_LINE_LENGTH  0x40     /**< internal line length of the display    */
#define LCD_START_LINE1  0x00     /**< DDRAM address of first char of line 1 */
#define LCD_START_LINE2  0x40     /**< DDRAM address of first char of line 2 */
#define LCD_START_LINE3  0x14     /**< DDRAM address of first char of line 3 */
#define LCD_START_LINE4  0x54     /**< DDRAM address of first char of line 4 */
#define LCD_WRAP_LINES      0     /**< 0: no wrap, 1: wrap at end of visibile line */


uint8_t lcd_buffer;
static volatile uint8_t pos; //текущая позиция курсора
const unsigned char Decode2Rus[255-192+1] PROGMEM = {
0x41,0xA0,0x42,0xA1,0xE0,0x45,0xA3,0xA4, 
0xA5,0xA6,0x4B,0xA7,0x4D,0x48,0x4F,0xA8, 
0x50,0x43,0x54,0xA9,0xAA,0x58,0xE1,0xAB, 
0xAC,0xE2,0xAD,0xAE,0xAD,0xAF,0xB0,0xB1, 
0x61,0xB2,0xB3,0xB4,0xE3,0x65,0xB6,0xB7, 
0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0x6F,0xBE, 
0x70,0x63,0xBF,0x79,0xE4,0x78,0xE5,0xC0, 
0xC1,0xE6,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7 };



#define lcd_e_toggle()  toggle_e()

#define lcd_e_delay()   __asm__ __volatile__( "rjmp 1f\n 1:" );   //#define lcd_e_delay() __asm__ __volatile__( "rjmp 1f\n 1: rjmp 2f\n 2:" );



/*************************************************************************
 delay loop for small accurate delays: 16-bit counter, 4 cycles/loop
*************************************************************************/
//static inline void _delayFourCycles(unsigned int __count)
//{
//    if ( __count == 0 )    
//        __asm__ __volatile__( "rjmp 1f\n 1:" );    // 2 cycles
//    else
//        __asm__ __volatile__ (
//    	    "1: sbiw %0,1" "\n\t"                  
//    	    "brne 1b"                              // 4 cycles/loop
//    	    : "=w" (__count)
//    	    : "0" (__count)
//    	   );
//}


/************************************************************************* 
delay for a minimum of <us> microseconds
the number of loops is calculated at compile-time from MCU clock frequency
*************************************************************************/
//#define delay(us)  _delayFourCycles( ( ( 1*(F_CPU/4000) )*us)/1000 )


/* toggle Enable Pin to initiate write */
static void toggle_e(void)
{
    lcd_e_high();
  //  lcd_e_delay();
    lcd_e_low();
}




/*************************************************************************
Initialize display and select type of cursor 
Input:    dispAttr LCD_DISP_OFF            display off
                   LCD_DISP_ON             display on, cursor off
                   LCD_DISP_ON_CURSOR      display on, cursor on
                   LCD_DISP_CURSOR_BLINK   display on, cursor on flashing
Returns:  none
*************************************************************************/
void init_lcd(){
init_shift_register(); //настраеваем порт для работы со сдвиговым
shift_register_clean();
/*
lcd_d4_high();
lcd_d5_high();
lcd_d6_high();
lcd_d7_high();

lcd_d4_low();
lcd_d5_low();
lcd_d6_low();
lcd_d7_low();
//shift_register_set_data(D4|D5|D6|D7);
*/
lcd_buffer = 0;
pos = 0;
_delay_ms(16+3);        /* wait 16ms or more after power-on       */
init_shift_register();
_delay_ms(15+3); 

lcd_function_set(LCD_FUNCTION_8BIT);

lcd_e_toggle();

lcd_e_toggle();

lcd_function_set(LCD_FUNCTION_4BIT);

lcd_command(LCD_CMD_FUNCTION|LCD_FUNCTION_DEFAULT);

//выключаем дисплей

lcd_command(1<<LCD_ON);

//очищаем дисплей
lcd_command(LCD_CLR);

//Enrty mode set
lcd_command(LCD_ENTRY_MODE);

//включаем дисплей 
//lcd_command(LCD_ON|LCD_ON_DISPLAY);

//--//lcd_command(LCD_ON|LCD_DISP_ON_BLINK);


lcd_command(LCD_DISP_ON);//включаем дисплей, курсор выключен

//ставим курсор на начало
lcd_command(LCD_ENTRY_MODE|LCD_ENTRY_INC);

lcd_generate_batt_symbols();
}


/*************************************************************************
Clear display and set cursor to home position
*************************************************************************/
void lcd_clrscr(void)
{
    lcd_command(LCD_CLR);
}



static void lcd_write(uint8_t data,uint8_t rs) 
{
    unsigned char dataBits ;

//	lcd_buffer=0; //очищаем буффер
    if (rs) {   /* write data        (RS=1, RW=0) */
       rs_high;
    } else {    /* write instruction (RS=0, RW=0) */
       rs_low;
    }
   // lcd_rw_low();

    
	//сначала выведем старший полубайт
	if(data & 0x80) data_3_high;
    else data_3_low;
	if(data & 0x40) data_2_high;
	else data_2_low;
    if(data & 0x20) data_1_high;
	else data_1_low;
    if(data & 0x10) data_0_high;
 	else data_0_low;
	lcd_update(); //выдвигаем данные из буфера в регистр   
    lcd_e_toggle();//говорим дисплею, что данные готовы
	_delay_ms(2);//-----// 
	
//	lcd_buffer=0; //очищаем буффер
	//теперь выведем младший полубайт

	if(data & 0x08) data_3_high;
	else data_3_low;
    if(data & 0x04) data_2_high;
	else data_2_low;
    if(data & 0x02) data_1_high;
	else data_1_low;
    if(data & 0x01) data_0_high;
    else data_0_low;
	lcd_update(); //выдвигаем данные из буфера в регистр   
	lcd_e_toggle(); //говорим дисплею, что данные готовы       
	_delay_ms(2);//---// 
	

	 /* all data pins high (inactive) */
//	data_3_high;
//    data_2_high;
//    data_1_high;
//    data_0_high;
//	lcd_update(); //выдвигаем данные из буфера в регистр   
	
	
}


/*************************************************************************
Send LCD controller instruction command
Input:   instruction to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_command(uint8_t cmd)
{
   // lcd_waitbusy();
    lcd_write(cmd,0);
}


/*************************************************************************
Send data byte to LCD controller 
Input:   data to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void lcd_data(uint8_t data)
{
//    lcd_waitbusy();
    lcd_write(data,1);
}




void lcd_function_set(uint8_t Attr){
lcd_buffer = LCD_FUNCTION|Attr;
lcd_e_toggle();
//shift_register_set_data(LCD_FUNCTION|Attr);
//shift_register_set_data(LCD_FUNCTION|Attr|E);
//_delay_ms(15); 
//shift_register_set_data(LCD_FUNCTION|Attr);
//_delay_ms(15); //-//
_delay_ms(3);
}




/*************************************************************************
Display character at current cursor position 
Input:    character to be displayed                                       
Returns:  none
*************************************************************************/
void lcd_putc(char c)
{
    
    //pos = lcd_waitbusy();   // read busy-flag and address counter
    if (c=='\n')
    {
        lcd_newline(pos);
    }
    else
    {
#if LCD_WRAP_LINES==1
#if LCD_LINES==1
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
        }
#elif LCD_LINES==2
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
        }else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ){
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
        }
#elif LCD_LINES==4
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
        }else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ) {
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE3,0);
        }else if ( pos == LCD_START_LINE3+LCD_DISP_LENGTH ) {
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE4,0);
        }else if ( pos == LCD_START_LINE4+LCD_DISP_LENGTH ) {
            lcd_write((1<<LCD_DDRAM)+LCD_START_LINE1,0);
        }
#endif
//        lcd_waitbusy();
#endif
        
		if(c>=192) c = pgm_read_byte(&(Decode2Rus[c-192]));
		lcd_write(c, 1);
		pos++;
    }

}/* lcd_putc */


/*************************************************************************
Display string without auto linefeed 
Input:    string to be displayed
Returns:  none
*************************************************************************/
void lcd_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        lcd_putc(c);
    }

}/* lcd_puts */





/*************************************************************************
Move cursor to the start of next line or to the first line if the cursor 
is already on the last line.
*************************************************************************/
inline void lcd_newline(uint8_t pos)
{
 volatile register uint8_t addressCounter;
//	uint8_t addressCounter;

#if LCD_LINES==1
    addressCounter = 0;
#endif
#if LCD_LINES==2
    if ( pos < (LCD_START_LINE2) )
        addressCounter = LCD_START_LINE2;
    else
        addressCounter = LCD_START_LINE1;
#endif
#if LCD_LINES==4
#if KS0073_4LINES_MODE
    if ( pos < LCD_START_LINE2 )
        addressCounter = LCD_START_LINE2;
    else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE3) )
        addressCounter = LCD_START_LINE3;
    else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE4) )
        addressCounter = LCD_START_LINE4;
    else 
        addressCounter = LCD_START_LINE1;
#else
    if ( pos < LCD_START_LINE3 )
        addressCounter = LCD_START_LINE2;
    else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE4) )
        addressCounter = LCD_START_LINE3;
    else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE2) )
        addressCounter = LCD_START_LINE4;
    else 
        addressCounter = LCD_START_LINE1;
#endif
#endif
    lcd_command((1<<LCD_DDRAM)+addressCounter);
	pos=addressCounter;
}/* lcd_newline */


/*
** PUBLIC FUNCTIONS 
*/




inline void lcd_backspace(){
 volatile register uint8_t addressCounter;
#if LCD_LINES==2
   // if ( pos < (LCD_START_LINE2) )
     //   pos = LCD_START_LINE1;
   // else
 addressCounter = pos-1;

#endif
lcd_command((1<<LCD_DDRAM)+addressCounter);
pos = addressCounter;

}


/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/
void lcd_gotoxy(uint8_t x, uint8_t y)
{
#if LCD_LINES==1
    lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
#endif
#if LCD_LINES==2
    if ( y==0 ) 
	{
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
		pos = LCD_START_LINE1+x;
    }
	else
    {
	    lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
		pos=LCD_START_LINE2+x;
	}
#endif
#if LCD_LINES==4
    if ( y==0 )
	{
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE1+x);
		pos = LCD_START_LINE1+x;
	}
    else if ( y==1)
	{
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE2+x);
    	pos = LCD_START_LINE2+x;
	}
	else if ( y==2)
	{
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE3+x);
    	pos = LCD_START_LINE3+x;
	}
	else /* y==3 */
	{
        lcd_command((1<<LCD_DDRAM)+LCD_START_LINE4+x);
		pos = LCD_START_LINE4+x;
	}
#endif

}/* lcd_gotoxy */


/*************************************************************************
Set cursor to home position
*************************************************************************/
void lcd_home(void)
{
    lcd_command(LCD_HOME);
	pos = LCD_HOME;
}


/*************************************************************************
Display string from program memory without auto linefeed 
Input:     string from program memory be be displayed                                        
Returns:   none
*************************************************************************/
void lcd_puts_p(const char *progmem_s)
/* print string from program memory on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = pgm_read_byte(progmem_s++)) ) {
        lcd_putc(c);
    }

}/* lcd_puts_p */



void lcd_bl_on(void)
{
lcd_bl_high();
}


void lcd_bl_off(void)
{
lcd_bl_low();
}



/*****************************************
*создаем в памяти ЖКИ символы батарейки
*****************************************/

void lcd_generate_batt_symbols(void)
{
lcd_command(1<<LCD_CGRAM|0); //устанавливаем адрес CGRAM=0 
lcd_data(0b00001110);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00011111);
lcd_data(0b00000000);

lcd_data(0b00001110);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00000000);

lcd_data(0b00001110);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00000000);

lcd_data(0b00001110);
lcd_data(0b00010001);
lcd_data(0b00010001);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00000000);


lcd_data(0b00001110);
lcd_data(0b00010001);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00000000);

lcd_data(0b00001110);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00000000);
lcd_command(1<<LCD_DDRAM|0); //устанавливаем адрес DDRAM=0 

}



void lcd_generate_additional_symbols(void)
{
lcd_command(1<<LCD_CGRAM|0); //устанавливаем адрес CGRAM=0 

lcd_data(0b00001110);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00000000);

lcd_data(0b00000000);
lcd_data(0b00000000);
lcd_data(0b00001010);
lcd_data(0b00011111);
lcd_data(0b00001110);
lcd_data(0b00000100);
lcd_data(0b00000000);
lcd_data(0b00000000);

lcd_data(0b00000110);
lcd_data(0b00001111);
lcd_data(0b00001111);
lcd_data(0b00001111);
lcd_data(0b00001111);
lcd_data(0b00000110);
lcd_data(0b00001111);
lcd_data(0b00000000);


lcd_data(0b00001110);
lcd_data(0b00011010);
lcd_data(0b00011110);
lcd_data(0b00011010);
lcd_data(0b00011110);
lcd_data(0b00011010);
lcd_data(0b00001111);
lcd_data(0b00000000);


lcd_data(0b00000110);
lcd_data(0b00001001);
lcd_data(0b00000110);
lcd_data(0b00001111);
lcd_data(0b01001111);
lcd_data(0b00001111);
lcd_data(0b00000110);
lcd_data(0b00000110);



lcd_data(0b00010000);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00011111);
lcd_data(0b00010000);
lcd_data(0b00010000);
lcd_data(0b00010000);
lcd_data(0b00010000);


lcd_command(1<<LCD_DDRAM|0); //устанавливаем адрес DDRAM=0 

}



void lcd_generate_bat_level_symbols(uint8_t bt_level)
{
lcd_command(1<<LCD_CGRAM|0); //устанавливаем адрес CGRAM=0 

switch(bt_level)
{
	case 0:
	{
		lcd_data(0b00001110);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00011111);
		lcd_data(0b00000000);
	} 
	break;
	case 1:
	{
		lcd_data(0b00001110);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00000000);
	} 
	break;
	case 2:
	{
		lcd_data(0b00001110);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00000000);
	}
	break;
	case 3:
	{
		lcd_data(0b00001110);
		lcd_data(0b00010001);
		lcd_data(0b00010001);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00000000);
	}
	break;

	case 4:
	{
		lcd_data(0b00001110);
		lcd_data(0b00010001);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00000000);

	}
	break;
	case 5:
	{	lcd_data(0b00001110);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00011111);
		lcd_data(0b00000000);


	}
	break;
	default: break;
}

lcd_command(1<<LCD_DDRAM|0); //устанавливаем адрес DDRAM=0 

}

