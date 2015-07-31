#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h> // printf()/scanf()

#include "usart.h"


void main(void)
{
	uart_init(BAUD_CALC(115200)); // 8n1 transmission is set as default
	
	stdout = &uart0_io; // attach uart stream to stdout & stdin
	stdin = &uart0_io; // uart0_in if no TX and uart0_out if no RX (depends on compilation macros)
	
	sei(); // enable interrupts, library wouldn't work without this
		
	uart_puts("hello from usart 0\r\n"); // write const string to usart buffer
	// if you do not have enough SRAM memory space to keep all strings, try to use puts_P instead
	uart_puts_P("hello from flashed, usart\r\n"); // write string to usart buffer from flash memory
		
	char buffer[13];
	uart_gets(buffer, 13) // read 13 bytes from buffer (CR,LF will not be cut)
	
	while(1)
	{
		uart_puts("bytes waiting in receiver buffer : ");
		uart_putint(uart_AvailableBytes()); // ask for bytes waiting in receiver buffer
		uart_getsln(buffer, 13); // read 12 bytes or one line from usart buffer 
		uart_puts("/r/n");
		
		uart_putfloat(0.1337f);
		uart_puts("/r/n");
		uart_putstr(buffer); // write array string to usart buffer // C++ restriction, in C its the same as uart_puts()
		uart_puts("/r/n");
			
		printf("Say my name: ");
		scanf("%s", buffer);
		printf("So it's %s, You are damn' right.\n", buffer);
		
		_delay_ms(5000);
	}
}