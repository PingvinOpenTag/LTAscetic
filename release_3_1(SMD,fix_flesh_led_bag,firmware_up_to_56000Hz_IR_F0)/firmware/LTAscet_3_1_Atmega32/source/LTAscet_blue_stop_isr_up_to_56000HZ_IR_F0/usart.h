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
#ifndef USART_H
#define USART_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>




#ifndef F_CPU
	#define F_CPU 16000000     //������ ������� ������
#endif
#define BAUD 9600         //��������� �������� ������
#include <util/setbaud.h> //����� ����� ������� ��� �������

//������� ��� ���������� � ���������� ���������� usart`a
#define EnableRxInt()   UCSRB |= (1<<RXCIE);
#define DisableRxInt()  UCSRB &= (~(1<<RXCIE));
#define EnableTxInt()   UCSRB |= (1<<TXCIE);
#define DisableTxInt()  UCSRB &= (~(1<<TXCIE));


#define SIZE_BUF 256       //� ������ ��������� ������� - <255




void USART_Init(void); //������������� usart`a
unsigned char USART_GetTxCount(void); //����� ����� �������� ����������� ������
void USART_FlushTxBuf(void); //�������� ���������� �����
void USART_PutChar(unsigned char sym); //�������� ������ � �����
void USART_SendStr(char * data); //������� ������ �� usart`�
void USART_SendStrP(char * data); //������� ������ �� ������ �������� �� usart`�
unsigned char USART_GetRxCount(void); //����� ����� �������� � �������� ������
void USART_FlushRxBuf(void); //�������� �������� �����
unsigned char USART_GetChar(void); //��������� �������� ����� usart`a 
#endif //USART_H
