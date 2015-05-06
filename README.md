# Easy-AVR-USART-C-Library
An interrupt-driven USART (RS232) library for AVR microcontrollers, with support for multiple UARTs, using ring
buffers for receive/transmit. Designed to be easy to use, especially like arduino libraries (with less overload).

# Features
- easy to use
- support for up to 4 USART's
- allow for binary transmission
- optimized as much as possible to reduce code size
- and much more

int main(void)
{
	uart_init(BAUD_CALC(9600));
    sei();
    while(1)
    {
		uart_puts("hello world\r\n");
		_delay_ms(1000);
    }
}

gives

		   text	   data	    bss	    dec	    hex	filename
		    446	     14	     69	    529	    211	avrt.elf

Program Memory Usage 	:	460 bytes   1,4 % Full
Data Memory Usage 		:	83 bytes   4,1 % Full

defined  NO_USART_RX flag gives

		   text	   data	    bss	    dec	    hex	filename
		    368	     14	     35	    417	    1a1	avrt.elf

Program Memory Usage 	:	382 bytes   1,2 % Full
Data Memory Usage 		:	49 bytes   2,4 % Full