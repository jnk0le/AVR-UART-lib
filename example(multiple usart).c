#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"


int main(void)
{
	uart_init(0, BAUD_CALC(19200)); // 8n1 transmission is set as default 
	uart_init(1, BAUD_CALC(115200));
	sei(); // enable interrupts, library wouldn't work without this
	
	uart_puts(0, "hello from usart 0\r\n"); // write const string to usart buffer 
	// if you do not have enough SRAM memory space to keep all strings, try to use puts_P instead
	uart_puts_P(1, "hello from flashed, usart 1\r\n"); // write string to usart buffer from flash memory
	
	char buffer[13];
	
	while(1)
	{
		uart_puts(0, "bytes waiting in receiver buffer : ");
		uart_putint(0, uart_AvailableBytes(0)); // ask for bytes waiting in receiver buffer
		uart_getsl(0, buffer, 13); // read 13 bytes or one line from usart buffer // uart_gets(0, buffer) is deprecated
		uart_puts(0, "/r/n");
		
		uart_putfloat(1, 0.1337f);
		uart_puts(1, "/r/n");
		uart_putstr(1, buffer); // write dynamic string to usart buffer // C++ restriction, in C its the same as uart_puts()
		_delay_ms(5000);
	}
}