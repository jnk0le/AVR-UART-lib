#include <avr/io.h>
#include "usart.h"

int main(void)
{
	//set highest posiible speed
	uart_init(0);
	uart_set_U2X();

	//PD2 is used as an CTS input for USART0
	
	//configure any interrupt source capable of triggering at both edges
	//for single signal INTn isr can be used 
	PCICR = (1 << PCIE2);
	PCMSK2 = (1 << PCINT18);

	//if hardware flow control is available (eg. 32u4) it should be used instead of soft implementation
	//uart_hardware_flow_control_init(1,1); // cts,rts - pass non zero value to enable // hardware rts will only work if isr cannot keep up with incoming data

	sei();

	uint8_t a = 0xff;

	while (1)
	{
		uart_putc(++a);
	}

}

ISR(PCINT2_vect)
{
	cts0_isr_handler();
	//cts1_isr_handler();
	//...

	//add code here if required for other inputs (PCINT only)
}