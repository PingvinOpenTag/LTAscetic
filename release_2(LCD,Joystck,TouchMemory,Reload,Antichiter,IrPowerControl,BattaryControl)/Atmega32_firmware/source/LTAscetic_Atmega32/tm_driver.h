//#include <util/delay.h>    // Дает возможность формирования задержки

#define TM_TX_PULLDOWN()   DDRD|=(1<<3);
#define TM_TX_RELEASE()   DDRD &=~(1<<3); 	

enum typtm_event {no_tm_event, tm_crc_ok, tm_crc_error};
typedef enum typtm_event TTM_EVENT;






//#define lcd_e_delay()   __asm__ __volatile__( "rjmp 1f\n 1:" );   //#define lcd_e_delay() __asm__ __volatile__( "rjmp 1f\n 1: rjmp 2f\n 2:" );



/*************************************************************************
 delay loop for small accurate delays: 16-bit counter, 4 cycles/loop
*************************************************************************/
static inline void _delayFourCycles(unsigned int __count)
{
    if ( __count == 0 )    
        __asm__ __volatile__( "rjmp 1f\n 1:" );    // 2 cycles
    else
        __asm__ __volatile__ (
    	    "1: sbiw %0,1" "\n\t"                  
    	    "brne 1b"                              // 4 cycles/loop
    	    : "=w" (__count)
    	    : "0" (__count)
    	   );
}


/************************************************************************* 
delay for a minimum of <us> microseconds
the number of loops is calculated at compile-time from MCU clock frequency
*************************************************************************/
#define delay(us)  _delayFourCycles( ( ( 1*(F_CPU/4000) )*us)/1000 )



volatile  unsigned char ch;
volatile unsigned char crc8;
volatile unsigned char code;
volatile unsigned char in;
volatile unsigned char device_code; //код устройства
volatile unsigned char tm_code[6]; //считанный код ключа Touch_memory 
volatile TTM_EVENT tm_event; //события считывателя TouchMemory





/**************************************************************************************
* Функция выполняет настройку внешних прерываний вывода INT1
***************************************************************************************/
void init_tm(void){
tm_connect = false;
DDRD &=~(1<<3); 				//Настраиваем вывод INT1 как вход
PORTD&=~(1<<3);					//Выключаем подтягивающий резистор 
MCUCR |=_BV(ISC11);				//Прерывания будут генерироваться 
MCUCR &=~_BV(ISC10);			//по спаду импульса
GICR |=_BV(INT1); 				//Разрешаем внешние прерывания на INT1
tm_event = no_tm_event;
}



/*****************************************************************************
 * tm_send
 *
 * Отправка байта.
 */
void tm_send( unsigned char byte )
{
    unsigned char i,b;
    b=byte;
    for( i = 0; i < 8; i++ )
    {
        volatile unsigned char pulldown_time, recovery_time;

        if( b & 1 )
        {
			/* write-one */
			TM_TX_PULLDOWN();
	        TM_TX_RELEASE();
			delay( 80 );	
        }
        if ((b&1)==0)
        {
          /* write-zero */
         	TM_TX_PULLDOWN();
			delay( 70 );
	        TM_TX_RELEASE();
   		    delay( 20 );	 		   
        }

        b >>= 1;
    }
}




/*****************************************************************************
 * tm_read
 *
 * Приём байта.
 */
unsigned char tm_read()
{
    unsigned char i, ret = 0;

    for( i = 0; i < 8; i++ )
    {
 //       unsigned char in;

 //       dint();
        TM_TX_PULLDOWN();
 ////       delay( 1 );
        TM_TX_RELEASE();
        delay( 5);
        in =  (PIND) & (1<<3);
//        eint();
        ret >>= 1;
        if(in)
            ret |= (1<<7);
        /* release & recovery */
        delay( 200);
    }
    return ret;
}


/*****************************************************************************
 * new_crc8
 *
 * Вычисление нового значения CRC8.
 */
unsigned char new_crc8( unsigned char crc8, unsigned char byte )
{
    unsigned char i;

    for( i = 0; i < 8; i++ )
    {
        if( (crc8 ^ byte) & 1 )
            crc8 = ((crc8 ^ 0x18) >> 1) | 0x80;
        else
            crc8 >>= 1;
        byte >>= 1;
    }
    return crc8;
}




void tm_read_serial_number()
{       
//GICR &=~_BV(INT1); 				//Запрешаем внешние прерывания на INT1

uint8_t read_attempts = 1;
while( read_attempts-- )
if (tm_event==no_tm_event)  {



	{	delay(512); 




		/* reset */
		TM_TX_PULLDOWN();
		delay(480);  /* tRSTL > 480us */
		/* завершаем импульс сброса, проверяем уровень, должен быть высокий */
//		cli(); //запрещаем общие прерывания
		TM_TX_RELEASE();
//		GIFR &=~_BV(INTF1);

		delay(7);   /* 15us minimum tPDH */
//		GIFR |=_BV(INTF1);
 		if( (PIND&(1<<3)) == 0 )
        {
            /* вероятно, вход просто закорочен */
//            sei();
            continue;
        }
		 	/* контролируем presence pulse */
        
		delay(80);  /* 60us minimum tPDL */
//		sei(); //разрешаем общие прерывания

		if( PIND&(1<<3) )
            /* что-то не так, уровень должен оставаться низким */
            continue;

		/* ожидаем завершения presence pulse */

        delay(240); /* tPDL */
        if( !(PIND&(1<<3)) )
            /* таймаут */
            continue;

		/* выдерживаем высокий уровень tRSTH */
        delay( 480 );
		
		  /* посылаем команду Read ROM */
        tm_send( 0x33 );

 /* читаем тип устройства */
      
		device_code = tm_read();
        if( 0 == device_code)
        /* тип устройства не может быть нулевым */
        continue;

		

		crc8 = new_crc8( 0, device_code);


 /* читаем код */
        for( ch = 0; ch < 6; ch++ )
        {
            code = tm_read();
            tm_code[ ch ] = code;
            crc8 = new_crc8( crc8, code );
        }
 /* читаем crc8 и проверяем */
        ch = tm_read();
		if( ch != crc8 )
           {
           tm_event=tm_crc_error;           
			continue;
	
           }
        else { 
			tm_event=tm_crc_ok;  
		return;}

	}
///	sei(); //разрешаем общие прерывания
	}
GIFR |=_BV(INTF1);
//GICR |=_BV(INT1); 				//Разрешаем внешние прерывания на INT1

} 	

/******************************************
* сравниваем считанный сериный номер ключа 
* с номером, записанным в eeprom
*******************************************/

uint8_t tm_verification(void)
{
if (eeprom_read_byte(&eeprom_tm_serial_num.device_code)!=device_code) return 0;
for (int i = 0; i<6; i++ )
						{
							if (eeprom_read_byte(&eeprom_tm_serial_num.serial[i])!=tm_code[i])  return 0;

						}
return 1;
}
