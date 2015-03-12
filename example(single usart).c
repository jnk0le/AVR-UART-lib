#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"


int main(void)
{
	uart_init(BAUD_CALC(115200)); // 8n1 transmission is set as default // same as uart0_init
	sei(); // enable interrupts, library wouldn't work without this
	
	uart_puts("hello from usart 0\r\n"); // write const string to usart buffer  // same as uart0_puts
	// if you do not have enough SRAM memory space to keep all strings, try to use puts_P instead
	uart_puts_P("hello from flashed, usart\r\n"); // write string to usart buffer from flash memory
	
	char buffer[13];
	
	while(1)
    {
		uart_puts("bytes waiting in receiver buffer : ");
		uart_putint(uart_AvailableBytes()); // ask for bytes waiting in receiver buffer
		uart_getsl(buffer, 13); // read 13 bytes or one line from usart buffer // same as uart0_gets
		uart_puts("/r/n");
		
		uart_putfloat(0.1337f);
		uart_puts("/r/n");
		uart_putstr(buffer); // write dynamic string to usart buffer
		uart_puts("/r/n");
		
		_delay_ms(5000);
    }
}