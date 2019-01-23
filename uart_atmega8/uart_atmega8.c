//*** Пример работы с USART интерфейсом микроконтроллеров AVR ***

#include <avr/io.h>
#include <avr/interrupt.h>

#define BAUDRATE 9600 // Скорость обмена данными
#define F_CPU 8000000UL // Рабочая частота контроллера

unsigned char NUM = 0;  
unsigned char count = 0;
unsigned char byte_receive = 0;
unsigned char i = 1;

// Функция задержки в мкс
void _delay_us(unsigned char time_us)
{ register unsigned char i;

for(i = 0; i < time_us; i++)
{
asm volatile(" PUSH  R0 ");
asm volatile(" POP   R0 ");
}
}

// Функция задержки в мс
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

// Функция передачи команды
void lcd_com(unsigned char p)
{
PORTD &= ~(1 << RS); // RS = 0 (запись команд)
PORTD |= (1 << EN); // EN = 1 (начало записи команды в LCD)
PORTD &= 0x0F; PORTD |= (p & 0xF0); // старший нибл
_delay_us(100);
PORTD &= ~(1 << EN); // EN = 0 (конец записи команды в LCD)
_delay_us(100);
PORTD |= (1 << EN); // EN = 1 (начало записи команды в LCD)
PORTD &= 0x0F; PORTD |= (p << 4); // младший нибл
_delay_us(100);
PORTD &= ~(1 << EN); // EN = 0 (конец записи команды в LCD)
_delay_us(100);
}

// Функция передачи данных
void lcd_data(unsigned char p)
{
PORTD |= (1 << RS)|(1 << EN); // RS = 1 (запись данных), EN - 1 (начало записи команды в LCD)
PORTD &= 0x0F; PORTD |= (p & 0xF0); // старший нибл
_delay_us(100);
PORTD &= ~(1 << EN); // EN = 0 (конец записи команды в LCD)
_delay_us(100);
PORTD |= (1 << EN); // EN = 1 (начало записи команды в LCD)
PORTD &= 0x0F; PORTD |= (p << 4); // младший нибл
_delay_us(100);
PORTD &= ~(1 << EN); // EN = 0 (конец записи команды в LCD)
_delay_us(100);
}

// Функция инициализации LCD
void lcd_init(void)
{
_delay_ms(50); // Ожидание готовности ЖК-модуля

// Конфигурирование четырехразрядного режима
PORTD |= (1 << PD5);
PORTD &= ~(1 << PD4);

// Активизация четырехразрядного режима
PORTD |= (1 << EN);
PORTD &= ~(1 << EN);
_delay_ms(5); 

lcd_com(0x28); // шина 4 бит, LCD - 2 строки
lcd_com(0x08); // полное выключение дисплея
lcd_com(0x01); // очистка дисплея
_delay_us(100);
lcd_com(0x06); // сдвиг курсора вправо
lcd_com(0x0C); // включение дисплея, курсор не видим
}

// Функция вывода строки на LCD
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

// Функция передачи данных по USART
void uart_send(char data)
{
while(!( UCSRA & (1 << UDRE)));	// Ожидаем когда очистится буфер передачи
UDR = data; // Помещаем данные в буфер, начинаем передачу
}

// Функция передачи строки по USART
void str_uart_send(char *string)
{
while(*string != '\0')
{
uart_send(*string);
string++;
}
}

// Функция приема данных по USART
int uart_receive(void)
{
while(!(UCSRA & (1 << RXC))); // Ожидаем, когда данные будут получены
return UDR; // Читаем данные из буфера и возвращаем их при выходе из подпрограммы
}

// Функция инициализации USART
void uart_init(void)
{
// Параметры соединения: 8 бит данные, 1 стоповый бит, нет контроля четности
// USART Приемник: Включен
// USART Передатчик: Включен
// USART Режим: Асинхронный
// USART Скорость обмена: 9600

UBRRL = (F_CPU/BAUDRATE/16-1); // Вычисляем скорость обмена данными  
UBRRH = (F_CPU/BAUDRATE/16-1) >> 8;

UCSRB |= (1 << RXCIE)| // Разрешаем прерывание по завершению приема данных
          (1 << RXEN)|(1 << TXEN); // Включаем приемник и передатчик
                                                
UCSRC |= (1 << URSEL)| // Для доступа к регистру UCSRC выставляем бит URSEL
         (1 << UCSZ1)|(1 << UCSZ0); // Размер посылки в кадре 8 бит
}

// Прерывание по окончанию приема данных по USART
ISR(USART_RXC_vect)
{
NUM = UDR; // Принимаем символ по USART
byte_receive = 1;
uart_send(NUM); // Посылаем символ по USART
	
if(NUM == 'a') // Если принят символ "a", включаем светодиод
PORTB |= (1 << PB0);
if(NUM == 'b') // Если принят символ "b", выключаем светодиод
PORTB &= ~(1 << PB0);
}

// Главная функция
int main(void)
{
DDRB |= (1 << PB0); // Светодиод
PORTB = 0x00;        			

DDRC  = 0x00;	
PORTC = 0xFF;
	
DDRD  = 0b11111110;
PORTD = 0x00;
	
lcd_init(); // Инициализация LCD
uart_init(); // Инициализация USART
	
sei(); // Глобально разрешаем прерывания

str_uart_send("Initialization system\r\n"); // Передаем строку по USART
lcd_string(0x80, " AVR USART TEST "); // Выводим строку на LCD
_delay_ms(2500);
lcd_com(0x01); // Очищаем LCD
	
while(1)
{

if((PINC & (1 << PC0)) == 0) // Если нажата кнопка 1
{
while((PINC & (1 << PC0)) == 0){} // Ждем отпускания кнопки 1
str_uart_send("Button 1 TEST\r\n"); // Передаем строку по USART
lcd_string(0x80, "Button 1 TEST"); // Выводим строку на LCD
_delay_ms(1000);
lcd_com(0x01); // Очищаем LCD
}

if((PINC & (1 << PC1)) == 0) // Если нажата кнопка 2
{
while((PINC & (1 << PC1)) == 0){} // Ждем отпускания кнопки 2
str_uart_send("Button 2 TEST\r\n"); // Передаем строку по USART
lcd_string(0x80, "Button 2 TEST"); // Выводим строку на LCD
_delay_ms(1000);
lcd_com(0x01); // Очищаем LCD
}

if((PINC & (1 << PC2)) == 0) // Если нажата кнопка 3
{
while((PINC & (1 << PC2)) == 0){} // Ждем отпускания кнопки 3
str_uart_send("Button 3 TEST\r\n"); // Передаем строку по USART
lcd_string(0x80, "Button 3 TEST"); // Выводим строку на LCD 
_delay_ms(1000);
lcd_com(0x01); // Очищаем LCD
}		

if(byte_receive)
{
byte_receive = 0;
count++;
lcd_data(NUM); // Выводим символ на LCD
if(count > 16) // Если строка заполнена
{
count = 0;
lcd_com(0x01); // Очищаем LCD
}
}
}
}