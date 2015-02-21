# Easy-AVR-USART-C-Library
easy to use C avr uart library


int main(void)
{
	uart0_init(BAUD_CALC(9600));
    sei();
    while(1)
    {
		uart_puts("hello world\r\n");
		_delay_ms(1000);
    }
}

gives