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
	//uart0_set_FrameFormat(USART_8BIT_DATA|USART_1STOP_BIT|USART_NO_PARITY|USART_ASYNC_MODE); // default settings
	uart0_init(BAUD_CALC(19200)); // 8n1 transmission is set as default 
	uart1_init(BAUD_CALC(115200));
	
	stdout = &uart0_io; // attach uart stream to stdout & stdin
	stdin = &uart0_io; // uart0_in and uart0_out are only available if NO_USART_RX or NO_USART_TX is defined
	
	sei(); // enable interrupts, library wouldn't work without this
	
	uart0_puts("hello from usart 0\r\n"); // write const string to usart buffer // C++ restriction, in C its the same as uart_putstr()
	// if you do not have enough SRAM memory space to keep all strings, try to use puts_P instead
	uart1_puts_P("hello from flashed, usart 1\r\n"); // write string to usart buffer from flash memory // string is parsed by PSTR() macro
	uart1_puts_p(foo_string); 
	uart0_puts_p(PSTR("we can also do like this\r\n"));

	printf("hello from printf\n");
	
	char buffer[25];
	uart1_gets(buffer, 25); // read at most 24 bytes from buffer (CR,LF will not be cut)
	
	int a;
	
	uart0_puts("gimmie a number: ");
	a = uart0_getint();
	
	uart0_puts("numba a: ");
	uart0_putint(a);
	uart0_puts("\r\n");
	
	while(1)
	{
		uart0_puts("bytes waiting in receiver buffer : ");
		uart0_putint(uart0_AvailableBytes()); // ask for bytes waiting in receiver buffer
		uart0_getln(buffer, 25); // read 24 bytes or one line from usart buffer
		
		if (!strcmp(buffer, "people who annoy you"))
		{
			uart0_putc('>');
			_delay_ms(5000);
			uart0_puts_P(" Googles");
		}
		uart0_puts("\r\n");
		
		uart1_putfloat(0.1337f);
		uart1_puts("\r\n");
		uart1_putstr(buffer); // write array string to usart buffer 
		
		fprintf(uart1_io, "Say my name: ");
		fscanf(uart1_io, "%s", buffer);
		fprintf(uart1_io, "So it's %s, You are damn' right.\n", buffer);
		
		_delay_ms(5000);
	}
}