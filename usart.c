#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "usart.h"

#ifndef NO_TX0_INTERRUPT
	volatile uint8_t tx0_first_byte, tx0_last_byte, interrupt_semaphore0;
	char tx0_buffer[TX0_BUFFER_SIZE];
#endif

#ifndef NO_RX0_INTERRUPT
	volatile uint8_t rx0_first_byte, rx0_last_byte;
	char rx0_buffer[RX0_BUFFER_SIZE];
#endif

#ifdef USE_USART1

#ifndef NO_TX1_INTERRUPT
	volatile uint8_t tx1_first_byte, tx1_last_byte, interrupt_semaphore1;
	char tx1_buffer[TX1_BUFFER_SIZE];
#endif

#ifndef NO_RX1_INTERRUPT
	volatile uint8_t rx1_first_byte, rx1_last_byte;
	char rx1_buffer[RX1_BUFFER_SIZE];
#endif

#endif // USE_USART1

void uart0_init(uint16_t baudRate)
{
	UBRR0L_REGISTER = (uint8_t) baudRate;
	UBRR0H_REGISTER = (baudRate>>8);
		
#ifdef USART0_U2X_SPEED
	UCSR0A_REGISTER |= (1<<U2X0_BIT); // enable double speed
#endif
	UCSR0B_REGISTER = USART0_CONFIG_B;
	// (1<<TXEN0_BIT)|(1<<RXEN0_BIT)|(1<<TXCIE0_BIT)|(1<<RXCIE0_BIT);
	// 8n1 is set by default, setting UCSRC is not needed
		
#ifndef NO_TX0_INTERRUPT
	interrupt_semaphore0 = unlocked;
#endif
}

void uart0_init_C(uint8_t UCSRC_reg, uint16_t baudRate)
{
	UBRR0L_REGISTER = (uint8_t) baudRate;
	UBRR0H_REGISTER = (baudRate>>8);
	
#ifdef USART0_U2X_SPEED
	UCSR0A_REGISTER |= (1<<U2X0_BIT); // enable double speed
#endif
	UCSR0B_REGISTER = USART0_CONFIG_B;
	// (1<<TXEN0_BIT)|(1<<RXEN0_BIT)|(1<<TXCIE0_BIT)|(1<<RXCIE0_BIT);
	// 8n1 is set by default, setting UCSRC is not needed
	UCSR0C_REGISTER = UCSRC_reg;
	
#ifndef NO_TX0_INTERRUPT
	interrupt_semaphore0 = unlocked;
#endif
}

#ifdef USE_USART1
void uart1_init(uint16_t baudRate)
{
	UBRR1L_REGISTER = (uint8_t) baudRate;
	UBRR1H_REGISTER = (baudRate>>8);

#ifdef USART1_U2X_SPEED
	UCSR1A_REGISTER |= (1<<U2X1_BIT); // enable double speed
#endif
	UCSR1B_REGISTER = USART1_CONFIG_B;
	// (1<<TXCIE1_BIT)|(1<<RXCIE1_BIT)|(1<<TXEN1_BIT)|(1<<RXEN1_BIT);
	// 8n1 is set by default, setting UCSRC is not needed

#ifndef NO_TX1_INTERRUPT
	interrupt_semaphore1 = unlocked;
#endif
}

void uart1_init_C(uint8_t UCSRC_reg, uint16_t baudRate)
{
	UBRR1L_REGISTER = (uint8_t) baudRate;
	UBRR1H_REGISTER = (baudRate>>8);

#ifdef USART1_U2X_SPEED
	UCSR1A_REGISTER |= (1<<U2X1_BIT); // enable double speed
#endif
	UCSR1B_REGISTER = USART1_CONFIG_B;
	// (1<<TXCIE1_BIT)|(1<<RXCIE1_BIT)|(1<<TXEN1_BIT)|(1<<RXEN1_BIT);
	// 8n1 is set by default, setting UCSRC is not needed
	UCSR0C_REGISTER = UCSRC_reg;

#ifndef NO_TX1_INTERRUPT
	interrupt_semaphore1 = unlocked;
#endif
}
#endif // USE_USART1

#ifndef NO_TX0_INTERRUPT
void uart0_putc(char data)
{
	register uint8_t tmp_tx_last_byte = tx0_last_byte;
	register uint8_t tmp_tx_first_byte = tx0_first_byte;
	
	tx0_buffer[tmp_tx_last_byte] = data;
	tx0_last_byte = (tmp_tx_last_byte + 1) & TX0_BUFFER_MASK; // calculate new position of TX tail in buffer
	
	while(tmp_tx_first_byte == tx0_last_byte); // wait for free space in buffer
	
	if(interrupt_semaphore0 == unlocked) // if transmitter interrupt is disabled
	{
		interrupt_semaphore0 = locked;
		UDR0_REGISTER = tx0_buffer[tmp_tx_first_byte]; // enable transmitter interrupt
	}
}

void uart0_putstr(char *string)
{
	while(*string)
		uart0_putc(*string++);
}

void uart0_putstrl(char *string, uint8_t BytesToWrite)
{
	while(BytesToWrite--)
		uart0_putc(*string++);
}

void uart0_puts_p(const char *string)
{
	while(pgm_read_byte(string))
		uart0_putc(pgm_read_byte(string++));
}

void uart0_putint(int16_t data)
{
	char buffer[7]; // heading, 5 digit bytes, NULL
	itoa(data, buffer, 10);

	uart0_putstr(buffer);
}

void uart0_put_hex(int16_t data)
{
	char buffer[6]; // heading, 4 digit bytes, NULL
	itoa(data, buffer, 16);
	
	uart0_putstr(buffer);
}

void uart0_putlong(int32_t data)
{
	char buffer[17]; // heading, 15 digit bytes, NULL
	ltoa(data, buffer, 10);
	
	uart0_putstr(buffer);
}
#endif // NO_TX0_INTERRUPT

#ifndef NO_RX0_INTERRUPT
char uart0_getc(void)
{
	register char temp;
	
	register uint8_t tmp_rx_first_byte = rx0_first_byte; // saves 4 bytes
	register uint8_t tmp_rx_last_byte = rx0_last_byte; // saves 8 bytes // no one knows how
	
	temp = (tmp_rx_first_byte == tmp_rx_last_byte) ? 0:rx0_buffer[tmp_rx_first_byte];
	if(tmp_rx_first_byte != tmp_rx_last_byte)
	rx0_first_byte = (tmp_rx_first_byte+1) & RX0_BUFFER_MASK; // calculate new position of RX head in buffer
	
	if(temp == '\n')   temp = 0;
	return temp;
}

void uart0_gets(char *buffer)
{
	do *buffer = uart0_getc();
	while(*buffer++);
}
void uart0_getsl(char *buffer, uint8_t bufferlimit)
{
	while(--bufferlimit)
	{
		*buffer = uart0_getc();
		if(*buffer++ == 0)
		return;
	}
	*buffer = 0;
}

uint8_t uart0_getbin(uint8_t *data)
{
	register uint8_t tmp_rx_first_byte = rx0_first_byte;
	
	*data = rx0_buffer[tmp_rx_first_byte];
	if(tmp_rx_first_byte != rx0_last_byte) // if buffer is not empty
	{
		rx0_first_byte = (tmp_rx_first_byte+1) & RX0_BUFFER_MASK; // calculate new position of RX head in buffer
		return COMPLETED; // result = 0
	}
	else
		return BUFFER_EMPTY; // in this case data value is a trash // result = 1
}
#endif // NO_RX0_INTERRUPT

#ifdef USE_USART1

#ifndef NO_TX1_INTERRUPT
void uart1_putc(char data)
{
	register uint8_t tmp_tx_last_byte = tx1_last_byte;
	register uint8_t tmp_tx_first_byte = tx1_first_byte;
	
	tx1_buffer[tmp_tx_last_byte] = data;
	tx1_last_byte = (tmp_tx_last_byte + 1) & TX1_BUFFER_MASK; // calculate new position of TX tail in buffer
	
	while(tmp_tx_first_byte == tx1_last_byte); // wait for free space in buffer
	
	if(interrupt_semaphore1 == unlocked) // if transmitter interrupt is disabled
	{
		interrupt_semaphore1 = locked;
		UDR1_REGISTER = tx1_buffer[tmp_tx_first_byte]; // enable transmitter interrupt
	}
}

void uart1_putstr(char *string)
{
	while(*string)
		uart1_putc(*string++);
}

void uart1_putstrl(char *string, uint8_t BytesToWrite)
{
	while(BytesToWrite--)
		uart1_putc(*string++);
}

void uart1_puts_p(const char *string)
{
	while(pgm_read_byte(string))
		uart1_putc(pgm_read_byte(string++));
}

void uart1_putint(int16_t data)
{
	char buffer[7]; // heading, 5 digit bytes, NULL
	itoa(data, buffer, 10);

	uart1_putstr(buffer);
}

void uart1_put_hex(int16_t data)
{
	char buffer[6]; // heading, 4 digit bytes, NULL
	itoa(data, buffer, 16);
	
	uart1_putstr(buffer);
}

void uart1_putlong(int32_t data)
{
	char buffer[17]; // heading, 15 digit bytes, NULL
	ltoa(data, buffer, 10);
	
	uart1_putstr(buffer);
}
#endif // NO_TX1_INTERRUPT

#ifndef NO_RX1_INTERRUPT
char uart1_getc(void)
{
	register char temp;
	
	register uint8_t tmp_rx_first_byte = rx1_first_byte; // saves 4 bytes
	register uint8_t tmp_rx_last_byte = rx1_last_byte; // saves 8 bytes // no one knows how
	
	temp = (tmp_rx_first_byte == tmp_rx_last_byte) ? 0:rx1_buffer[tmp_rx_first_byte];
	if(tmp_rx_first_byte != tmp_rx_last_byte)
	rx1_first_byte = (tmp_rx_first_byte+1) & RX1_BUFFER_MASK; // calculate new position of RX head in buffer
	
	if(temp == '\n')   temp = 0;
	return temp;
}

void uart1_gets(char *buffer)
{
	do *buffer = uart1_getc();
	while(*buffer++);
}
void uart1_getsl(char *buffer, uint8_t bufferlimit)
{
	while(--bufferlimit)
	{
		*buffer = uart1_getc();
		if(*buffer++ == 0)
		return;
	}
	*buffer = 0;
}

uint8_t uart1_getbin(uint8_t *data)
{
	register uint8_t tmp_rx_first_byte = rx1_first_byte;
	
	*data = rx1_buffer[tmp_rx_first_byte];
	if(tmp_rx_first_byte != rx1_last_byte) // if buffer is not empty
	{
		rx1_first_byte = (tmp_rx_first_byte+1) & RX1_BUFFER_MASK; // calculate new position of RX head in buffer
		return COMPLETED; // result = 0
	}
	else
		return BUFFER_EMPTY; // in this case data value is a trash // result = 1
}
#endif // NO_RX1_INTERRUPT

#endif // USE_USART1

#ifndef NO_TX0_INTERRUPT
ISR(TX0_INTERRUPT)
{
	register uint8_t tmp_tx_first_byte = tx0_first_byte = (tx0_first_byte + 1) & TX0_BUFFER_MASK;
	// calculate new position of TX head in buffer, write back and use it as register variable // saved 4 bytes
	
	if(tmp_tx_first_byte != tx0_last_byte)
	{
		UDR0_REGISTER = tx0_buffer[tmp_tx_first_byte]; // transmit character from the buffer
	}
	else
	{
		interrupt_semaphore0 = unlocked;
	}
}
#endif // NO_TX0_INTERRUPT

#ifndef NO_RX0_INTERRUPT
ISR(RX0_INTERRUPT)
{
	register char tmp;
	tmp = UDR0_REGISTER; // save received character to temporary register
	
	register uint8_t tmp_rx_last_byte = rx0_last_byte + 1; // saves 20 bytes // working only in this way, in the other adds 18 bytes to stock size
	
	#ifdef RX0_BINARY_MODE // not optimal and no one knows why
	if(rx0_first_byte != (tmp_rx_last_byte)) // tmp_rx_last_byte + 1
	#else
	if(rx0_first_byte != (tmp_rx_last_byte) && (tmp != '\r')) // tmp_rx_last_byte + 1
	#endif
	{
		rx0_buffer[tmp_rx_last_byte-1] = tmp; // tmp_rx_last_byte
		rx0_last_byte = (tmp_rx_last_byte) & RX0_BUFFER_MASK; // calculate new position of RX tail in buffer // tmp_rx_last_byte + 1
	}
}
#endif // NO_RX0_INTERRUPT

#ifdef USE_USART1

#ifndef NO_TX1_INTERRUPT
ISR(TX1_INTERRUPT)
{
	register uint8_t tmp_tx_first_byte = tx1_first_byte = (tx1_first_byte + 1) & TX1_BUFFER_MASK;
	
	if(tmp_tx_first_byte != tx1_last_byte)
	{
		UDR1_REGISTER = tx1_buffer[tmp_tx_first_byte]; // transmit character from the buffer
	}
	else
	{
		interrupt_semaphore1 = unlocked;
	}
}
#endif // NO_TX1_INTERRUPT

#ifndef NO_RX1_INTERRUPT
ISR(RX1_INTERRUPT)
{
	register char tmp;
	tmp = UDR1_REGISTER; // save received character to temporary register
	
	register uint8_t tmp_rx_last_byte = rx1_last_byte + 1;
	
	#ifdef RX1_BINARY_MODE
	if(rx1_first_byte != (tmp_rx_last_byte)) // pUSART1 -> rx_last_byte+1
	#else
	if(rx1_first_byte != (tmp_rx_last_byte) && (tmp != '\r'))	// pUSART1 -> rx_last_byte+1
	#endif
	{
		rx1_buffer[tmp_rx_last_byte-1] = tmp;	// pUSART1 -> rx_last_byte
		rx1_last_byte = (tmp_rx_last_byte) & RX1_BUFFER_MASK;	// pUSART1 -> rx_last_byte+1
	}
}
#endif // NO_RX1_INTERRUPT

#endif // USE_USART1