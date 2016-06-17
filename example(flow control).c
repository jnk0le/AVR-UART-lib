#include <avr/io.h>
#include "usart.h"

int main(void)
{
	// set highest posiible speed
	uart_init(0);
	uart_set_U2X();

	// configure any interrupt source capable of triggering at both edges
	// for single signal INTn isr can be used
	PCICR = (1 << PCIE2);
	PCMSK2 = (1 << PCINT18); // PD2 is used as an CTS input for USART0

	// if hardware flow control is available (eg. 32u4) it should be used instead of soft implementation
	// hardware rts will work only when USART_EXTEND_RX_BUFFER is defined
	//uart_hardware_flow_control_init(1,1); // cts,rts - pass non zero value to enable

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

	// add code here if required for other inputs (PCINT only)
}