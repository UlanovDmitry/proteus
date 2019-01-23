//*** ������ ������ � USART ����������� ����������������� AVR ***

#include <avr/io.h>
#include <avr/interrupt.h>

#define BAUDRATE 9600 // �������� ������ �������
#define F_CPU 8000000UL // ������� ������� �����������

unsigned char NUM = 0;  
unsigned char count = 0;
unsigned char byte_receive = 0;
unsigned char i = 1;

// ������� �������� � ���
void _delay_us(unsigned char time_us)
{ register unsigned char i;

for(i = 0; i < time_us; i++)
{
asm volatile(" PUSH  R0 ");
asm volatile(" POP   R0 ");
}
}

// ������� �������� � ��
void _delay_ms(unsigned int time_ms)
{ register unsigned int i;

for(i = 0; i < time_ms; i++)
{ 
_delay_us(250);
_delay_us(250);
_delay_us(250);
_delay_us(250);
}
}

#define RS PD2 
#define EN PD3

// ������� �������� �������
void lcd_com(unsigned char p)
{
PORTD &= ~(1 << RS); // RS = 0 (������ ������)
PORTD |= (1 << EN); // EN = 1 (������ ������ ������� � LCD)
PORTD &= 0x0F; PORTD |= (p & 0xF0); // ������� ����
_delay_us(100);
PORTD &= ~(1 << EN); // EN = 0 (����� ������ ������� � LCD)
_delay_us(100);
PORTD |= (1 << EN); // EN = 1 (������ ������ ������� � LCD)
PORTD &= 0x0F; PORTD |= (p << 4); // ������� ����
_delay_us(100);
PORTD &= ~(1 << EN); // EN = 0 (����� ������ ������� � LCD)
_delay_us(100);
}

// ������� �������� ������
void lcd_data(unsigned char p)
{
PORTD |= (1 << RS)|(1 << EN); // RS = 1 (������ ������), EN - 1 (������ ������ ������� � LCD)
PORTD &= 0x0F; PORTD |= (p & 0xF0); // ������� ����
_delay_us(100);
PORTD &= ~(1 << EN); // EN = 0 (����� ������ ������� � LCD)
_delay_us(100);
PORTD |= (1 << EN); // EN = 1 (������ ������ ������� � LCD)
PORTD &= 0x0F; PORTD |= (p << 4); // ������� ����
_delay_us(100);
PORTD &= ~(1 << EN); // EN = 0 (����� ������ ������� � LCD)
_delay_us(100);
}

// ������� ������������� LCD
void lcd_init(void)
{
_delay_ms(50); // �������� ���������� ��-������

// ���������������� ����������������� ������
PORTD |= (1 << PD5);
PORTD &= ~(1 << PD4);

// ����������� ����������������� ������
PORTD |= (1 << EN);
PORTD &= ~(1 << EN);
_delay_ms(5); 

lcd_com(0x28); // ���� 4 ���, LCD - 2 ������
lcd_com(0x08); // ������ ���������� �������
lcd_com(0x01); // ������� �������
_delay_us(100);
lcd_com(0x06); // ����� ������� ������
lcd_com(0x0C); // ��������� �������, ������ �� �����
}

// ������� ������ ������ �� LCD
void lcd_string(unsigned char command, char *string)
{
lcd_com(0x0C);
lcd_com(command);
while(*string != '\0')
{
lcd_data(*string);
string++;
}
}

// ������� �������� ������ �� USART
void uart_send(char data)
{
while(!( UCSRA & (1 << UDRE)));	// ������� ����� ��������� ����� ��������
UDR = data; // �������� ������ � �����, �������� ��������
}

// ������� �������� ������ �� USART
void str_uart_send(char *string)
{
while(*string != '\0')
{
uart_send(*string);
string++;
}
}

// ������� ������ ������ �� USART
int uart_receive(void)
{
while(!(UCSRA & (1 << RXC))); // �������, ����� ������ ����� ��������
return UDR; // ������ ������ �� ������ � ���������� �� ��� ������ �� ������������
}

// ������� ������������� USART
void uart_init(void)
{
// ��������� ����������: 8 ��� ������, 1 �������� ���, ��� �������� ��������
// USART ��������: �������
// USART ����������: �������
// USART �����: �����������
// USART �������� ������: 9600

UBRRL = (F_CPU/BAUDRATE/16-1); // ��������� �������� ������ �������  
UBRRH = (F_CPU/BAUDRATE/16-1) >> 8;

UCSRB |= (1 << RXCIE)| // ��������� ���������� �� ���������� ������ ������
          (1 << RXEN)|(1 << TXEN); // �������� �������� � ����������
                                                
UCSRC |= (1 << URSEL)| // ��� ������� � �������� UCSRC ���������� ��� URSEL
         (1 << UCSZ1)|(1 << UCSZ0); // ������ ������� � ����� 8 ���
}

// ���������� �� ��������� ������ ������ �� USART
ISR(USART_RXC_vect)
{
NUM = UDR; // ��������� ������ �� USART
byte_receive = 1;
uart_send(NUM); // �������� ������ �� USART
	
if(NUM == 'a') // ���� ������ ������ "a", �������� ���������
PORTB |= (1 << PB0);
if(NUM == 'b') // ���� ������ ������ "b", ��������� ���������
PORTB &= ~(1 << PB0);
}

// ������� �������
int main(void)
{
DDRB |= (1 << PB0); // ���������
PORTB = 0x00;        			

DDRC  = 0x00;	
PORTC = 0xFF;
	
DDRD  = 0b11111110;
PORTD = 0x00;
	
lcd_init(); // ������������� LCD
uart_init(); // ������������� USART
	
sei(); // ��������� ��������� ����������

str_uart_send("Initialization system\r\n"); // �������� ������ �� USART
lcd_string(0x80, " AVR USART TEST "); // ������� ������ �� LCD
_delay_ms(2500);
lcd_com(0x01); // ������� LCD
	
while(1)
{

if((PINC & (1 << PC0)) == 0) // ���� ������ ������ 1
{
while((PINC & (1 << PC0)) == 0){} // ���� ���������� ������ 1
str_uart_send("Button 1 TEST\r\n"); // �������� ������ �� USART
lcd_string(0x80, "Button 1 TEST"); // ������� ������ �� LCD
_delay_ms(1000);
lcd_com(0x01); // ������� LCD
}

if((PINC & (1 << PC1)) == 0) // ���� ������ ������ 2
{
while((PINC & (1 << PC1)) == 0){} // ���� ���������� ������ 2
str_uart_send("Button 2 TEST\r\n"); // �������� ������ �� USART
lcd_string(0x80, "Button 2 TEST"); // ������� ������ �� LCD
_delay_ms(1000);
lcd_com(0x01); // ������� LCD
}

if((PINC & (1 << PC2)) == 0) // ���� ������ ������ 3
{
while((PINC & (1 << PC2)) == 0){} // ���� ���������� ������ 3
str_uart_send("Button 3 TEST\r\n"); // �������� ������ �� USART
lcd_string(0x80, "Button 3 TEST"); // ������� ������ �� LCD 
_delay_ms(1000);
lcd_com(0x01); // ������� LCD
}		

if(byte_receive)
{
byte_receive = 0;
count++;
lcd_data(NUM); // ������� ������ �� LCD
if(count > 16) // ���� ������ ���������
{
count = 0;
lcd_com(0x01); // ������� LCD
}
}
}
}