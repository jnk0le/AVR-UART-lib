#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"

const char foo_string[] PROGMEM = "Unluckily gcc string polling doesn't work for PROGMEM/PSTR() strings";

void main(void)
{
	//uart_set_FrameFormat(USART_8BIT_DATA|USART_1STOP_BIT|USART_NO_PARITY|USART_ASYNC_MODE); // default settings
	uart_init(BAUD_CALC(115200)); // 8n1 transmission is set as default
	
	stdout = &uart0_io; // attach uart stream to stdout & stdin
	stdin = &uart0_io; // uart0_in if no TX and uart0_out if no RX (depends on config macros)
	
	sei(); // enable interrupts, library wouldn't work without this
		
	uart_puts("hello from usart 0\r\n"); // write const string to usart buffer // C++ restriction, in C its the same as uart_putstr()
	// if you do not have enough SRAM memory space to keep all strings, try to use puts_P instead
	uart_puts_P("hello from flashed, usart\r\n"); // write string to usart buffer from flash memory // string is parsed by PSTR() macro
	uart_puts_p(foo_string);
	uart_puts_p(PSTR("we can also do like this\r\n"));
	
	printf("hello from printf\n");

	char buffer[25];
	uart_gets(buffer, 25); // read at most 24 bytes from buffer (CR,LF will not be cut)
	
	int a;
	
	uart_puts("gimmie a number: ");
	a = uart_getint();
	
	uart_puts("numba a: ");
	uart_putint(a);
	uart_puts("\r\n");
	
	while(1)
	{
		uart_puts("bytes waiting in receiver buffer : ");
		uart_putint(uart_AvailableBytes()); // ask for bytes waiting in receiver buffer
		uart_getln(buffer, 25); // read 24 bytes or one line from usart buffer 
		
		if (!strcmp(buffer, "people who annoy you"))
		{
			uart_putc('>');
			_delay_ms(5000);
			uart_puts_P(" Googles");
		}
		uart_puts("\r\n");
		
		uart_putfloat(0.1337f);
		uart_puts("\r\n");
		uart_putstr(buffer); // write array string to usart buffer 
		uart_puts("\r\n");
			
		printf("Say my name: ");
		scanf("%s", buffer);
		printf("So it's %s, You are damn' right.\n", buffer);
		
		_delay_ms(5000);
	}
}