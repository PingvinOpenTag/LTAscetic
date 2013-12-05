//***************************************************************************
//
//  Author(s)...: Pashgan    http://ChipEnable.Ru   
//
//  Target(s)...: ATMega8535
//
//  Compiler....: WINAVR
//
//  Description.: ������� USART/UART � ��������� �������
//
//  Data........: 11.01.10 
//
//***************************************************************************
#include "usart.h"
#include "ltag_ascetic.h"


//���������� �����
volatile unsigned char usartTxBuf[SIZE_BUF];
volatile unsigned char txBufTail = 0;
volatile unsigned char txBufHead = 0;
volatile unsigned char txCount = 0;

//�������� �����
volatile unsigned char usartRxBuf[SIZE_BUF];
volatile unsigned char rxBufTail = 0;
volatile unsigned char rxBufHead = 0;
volatile unsigned char rxCount = 0;

//������������� usart`a
void USART_Init(void)
{
  UBRRH = UBRRH_VALUE;
  UBRRL = UBRRL_VALUE;
  #if USE_2X
   UCSRA |= (1 << U2X);
  #else
   UCSRA &= (~(1 << U2X));
  #endif
  UCSRB = (1<<RXCIE)/*|(1<<TXCIE)*/|(1<<RXEN)|(1<<TXEN); //����. ������ ��� ������ � ��������, ���� ������, ���� ��������.
  UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); //������ ����� 8 ��������
}

//______________________________________________________________________________
//���������� ����������� �������� ����������� ������
unsigned char USART_GetTxCount(void)
{
  return txCount;  
}

//"�������" ���������� �����
void USART_FlushTxBuf(void)
{
  txBufTail = 0;
  txBufHead = 0;
  txCount = 0;
}

//�������� ������ � �����, ���������� ������ ��������
void USART_PutChar(unsigned char sym)
{
 
 while ((UCSRA & (1<<UDRE)) == 0){};
 UDR = sym;
 /*
  //	if(sym>=192) sym = pgm_read_byte(&(Decode2Rus[sym-192]));
  //���� ������ usart �������� � ��� ������ ������
  //����� ��� ����� � ������� UDR
  if(((UCSRA & (1<<UDRE)) != 0) && (txCount == 0)) 
  {
  	UDR = sym;
  }

 else {
	//if (!(((UCSRA & (1<<UDRE)) != 0) && (txCount == 0))){ 
    if (txCount < SIZE_BUF){    //���� � ������ ��� ���� �����
      usartTxBuf[txBufTail] = sym; //�������� � ���� ������
      txCount++;                   //�������������� ������� ��������
      txBufTail++;                 //� ������ ������ ������
      if (txBufTail == SIZE_BUF) txBufTail = 0;
    }
  }
*/
}

//������� ���������� ������ �� usart`�
void USART_SendStr(char * data)
{
  unsigned char sym;
//  while(*data)
  while((sym = *data++)){
    
    USART_PutChar(sym);
  }
}

//������� ���������� ������ �� ������ �������� �� usart`�

void USART_SendStrP(char * data)
{
  unsigned char sym;
//  while(*data)
  while((sym = pgm_read_byte(data++))){
    
    USART_PutChar(sym);
  }
}
//���������� ���������� �� ���������� �������� 
ISR(USART_TXC_vect)
{
  if (txCount > 0){              //���� ����� �� ������
    UDR = usartTxBuf[txBufHead]; //���������� � UDR ������ �� ������
    txCount--;                   //��������� ������� ��������
    txBufHead++;                 //�������������� ������ ������ ������
    if (txBufHead == SIZE_BUF) txBufHead = 0;
  } 
} 

//______________________________________________________________________________
//���������� ����������� �������� ����������� � �������� ������
unsigned char USART_GetRxCount(void)
{
  return rxCount;  
}

//"�������" �������� �����
void USART_FlushRxBuf(void)
{
  DisableRxInt(); //��������� ���������� �� ���������� ������
  rxBufTail = 0;
  rxBufHead = 0;
  rxCount = 0;
  EnableRxInt();
}

//������ ������
unsigned char USART_GetChar(void)
{
  unsigned char sym;
  if (rxCount > 0){                     //���� �������� ����� �� ������  
    sym = usartRxBuf[rxBufHead];        //��������� �� ���� ������    
    rxCount--;                          //��������� ������� ��������
    rxBufHead++;                        //���������������� ������ ������ ������  
    if (rxBufHead == SIZE_BUF) rxBufHead = 0;
    return sym;                         //������� ����������� ������
  }
  return 0;
}


//���������� �� ���������� ������
ISR(USART_RXC_vect) 
{
  
  if (rxCount < SIZE_BUF){                //���� � ������ ��� ���� �����                     	
      usartRxBuf[rxBufTail] = UDR;        //������� ������ �� UDR � �����
     /*
	  if (usartRxBuf[rxBufTail]=='\r')
	  { 
  		cr_received=true;
	  }
	 */
	 switch(usartRxBuf[rxBufTail])
	 {
	 	case '\r': 	cr_received=true;
		break;
		case '1': 	worn_led_on_timer+=300;
		break;


	 }
	  rxBufTail++;                             //��������� ������ ������ ��������� ������ 
      if (rxBufTail == SIZE_BUF) rxBufTail = 0;  
      rxCount++;                                 //��������� ������� �������� ��������
    }
} 

