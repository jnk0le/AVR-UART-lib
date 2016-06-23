#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "usart.h"

const char foo_string[] PROGMEM = "Unluckily gcc string polling doesn't work for PROGMEM/PSTR() strings";

void main(void)
{
	//uart_set_FrameFormat(0, USART_8BIT_DATA|USART_1STOP_BIT|USART_NO_PARITY|USART_ASYNC_MODE); // default settings
	uart_init(0, BAUD_CALC(19200)); // 8n1 transmission is set as default 
	uart_init(1, BAUD_CALC(115200));
	
	stdout = &uart0_io; // attach uart stream to stdout & stdin
	stdin = &uart0_io; // uart0_in if no TX and uart0_out if no RX (depends on config macros)
	
	sei(); // enable interrupts, library wouldn't work without this
	
	uart_puts(0, "hello from usart 0\r\n"); // write const string to usart buffer // C++ restriction, in C its the same as uart_putstr()
	// if you do not have enough SRAM memory space to keep all strings, try to use puts_P instead
	uart_puts_P(1, "hello from flashed, usart 1\r\n"); // write string to usart buffer from flash memory // string is parsed by PSTR() macro
	uart_puts_p(1, foo_string); 
	printf("hello from printf\n");
	
	char buffer[25];
	uart_gets(1, buffer, 25) // read at most 24 bytes from buffer (CR,LF will not be cut)
	
	int a;
	
	uart_puts(0, "gimmie a number: ");
	a = uart_getint(0);
	
	uart_puts(0, "numba a: ");
	uart_putint(0, a);
	uart_puts(0, "\r\n");
	
	while(1)
	{
		uart_puts(0, "bytes waiting in receiver buffer : ");
		uart_putint(0, uart_AvailableBytes(0)); // ask for bytes waiting in receiver buffer
		uart_getln(0, buffer, 25); // read 24 bytes or one line from usart buffer
		
		if !(strcmp(buffer, "people who annoy you"))
		{
			uart_putc(0, '>');
			_delay_ms(5000);
			uart_puts_P(0, " naggers");
		}
		uart_puts(0, "\r\n");
		
		uart_putfloat(1, 0.1337f);
		uart_puts(1, "\r\n");
		uart_putstr(1, buffer); // write array string to usart buffer 
		
		fprintf(uart1_io, "Say my name: ");
		fscanf(uart1_io, "%s", buffer);
		fprintf(uart1_io, "So it's %s, You are damn' right.\n", buffer);
		
		_delay_ms(5000);
	}
}