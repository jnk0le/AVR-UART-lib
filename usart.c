#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "usart.h"

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)

	void uart_init(uint8_t usartct, uint16_t ubbr_value)
	{
		switch(usartct)
		{
		#ifdef USE_USART0
			case 0:
				UBRR0L_REGISTER = (uint8_t) ubbr_value;
				UBRR0H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART0_U2X_SPEED
				UCSR0A_REGISTER |= (1<<U2X0_BIT); // enable double speed
			#endif
			
				UCSR0B_REGISTER = USART0_CONFIG_B;
					// (1<<TXEN0_BIT)|(1<<RXEN0_BIT)|(1<<TXCIE0_BIT)|(1<<RXCIE0_BIT);
					// 8n1 is set by default, setting UCSRC is not needed
			
			#ifndef NO_TX0_INTERRUPT
				interrupt_semaphore0 = unlocked;
			#endif
			
			break;
		#endif // NO_USART0
		#ifdef USE_USART1
			case 1:
				UBRR1L_REGISTER = (uint8_t) ubbr_value;
				UBRR1H_REGISTER = (ubbr_value>>8);
				
			#ifdef USART1_U2X_SPEED
				UCSR1A_REGISTER |= (1<<U2X1_BIT); // enable double speed
			#endif
				
				UCSR1B_REGISTER = USART1_CONFIG_B;
					// (1<<TXEN1_BIT)|(1<<RXEN1_BIT)|(1<<TXCIE1_BIT)|(1<<RXCIE1_BIT);
					// 8n1 is set by default, setting UCSRC is not needed
				
			#ifndef NO_TX1_INTERRUPT
				interrupt_semaphore1 = unlocked;
			#endif
			
			break;
		#endif // USE_USART1
		#ifdef USE_USART2
			case 2:
				UBRR2L_REGISTER = (uint8_t) ubbr_value;
				UBRR2H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART2_U2X_SPEED
				UCSR2A_REGISTER |= (1<<U2X2_BIT); // enable double speed
			#endif
			
				UCSR2B_REGISTER = USART2_CONFIG_B;
					// (1<<TXEN2_BIT)|(1<<RXEN2_BIT)|(1<<TXCIE2_BIT)|(1<<RXCIE2_BIT);
					// 8n1 is set by default, setting UCSRC is not needed
			
			#ifndef NO_TX2_INTERRUPT
				interrupt_semaphore2 = unlocked;
			#endif
			
			break;
		#endif // USE_USART2
		#ifdef USE_USART3
			case 3:
				UBRR3L_REGISTER = (uint8_t) ubbr_value;
				UBRR3H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART3_U2X_SPEED
				UCSR3A_REGISTER |= (1<<U2X3_BIT); // enable double speed
			#endif
			
				UCSR3B_REGISTER = USART3_CONFIG_B;
					// (1<<TXEN3_BIT)|(1<<RXEN3_BIT)|(1<<TXCIE3_BIT)|(1<<RXCIE3_BIT);
					// 8n1 is set by default, setting UCSRC is not needed
			
			#ifndef NO_TX3_INTERRUPT
				interrupt_semaphore3 = unlocked;
			#endif
			//break;
		#endif // USE_USART3
		}
		
	}

	void uart_set_UCSRC(uint8_t usartct, uint8_t UCSRC_reg)
	{
		switch(usartct)
		{
		#ifdef USE_USART0
			case 0: UCSR0C_REGISTER |= UCSRC_reg; break;
		#endif // USE_USART0
		#ifdef USE_USART1
			case 1: UCSR1C_REGISTER |= UCSRC_reg; break;
		#endif // USE_USART1
		#ifdef USE_USART2
			case 2: UCSR2C_REGISTER |= UCSRC_reg; break;
		#endif // USE_USART2
		#ifdef USE_USART3
			case 3: UCSR3C_REGISTER |= UCSRC_reg; //break;
		#endif // USE_USART3
		}
		
	}
	
	void uart_set_U2X(uint8_t usartct)
	{
		switch(usartct)
		{
		#ifdef USE_USART0
			case 0: UCSR0A_REGISTER |= (1<<U2X0_BIT); break;
		#endif // USE_USART0
		#ifdef USE_USART1
			case 1: UCSR1A_REGISTER |= (1<<U2X1_BIT); break;
		#endif // USE_USART1
		#ifdef USE_USART2
			case 2: UCSR2A_REGISTER |= (1<<U2X2_BIT); break;
		#endif // USE_USART2
		#ifdef USE_USART3
			case 3: UCSR3A_REGISTER |= (1<<U2X3_BIT); //break;
		#endif // USE_USART3
		}
		
	}

#else // single USART mcu

//	//  these functions are now static inline in header file
// 	void uart_init(uint16_t ubbr_value)
// 	{
// 		UBRR0L_REGISTER = (uint8_t) ubbr_value;
//		UBRR0H_REGISTER = (ubbr_value>>8);
// 			
// 	#ifdef USART0_U2X_SPEED
// 		UCSR0A_REGISTER |= (1<<U2X0_BIT); // enable double speed
// 	#endif
// 			
// 		UCSR0B_REGISTER = USART0_CONFIG_B;
// 			// (1<<TXEN0_BIT)|(1<<RXEN0_BIT)|(1<<TXCIE0_BIT)|(1<<RXCIE0_BIT);
// 			// 8n1 is set by default, setting UCSRC is not needed
// 			
// 	#ifndef NO_TX0_INTERRUPT
// 		interrupt_semaphore0 = unlocked;
// 	#endif
// 	}

// 	void uart_set_UCSRC(uint8_t UCSRC_reg)
// 	{
// 		UCSR0C_REGISTER |= UCSRC_reg; 
// 	}
// 	
// 	void uart_set_U2X(void)
// 	{
// 		UCSR0A_REGISTER |= (1<<U2X0_BIT); 
// 	}

#endif // single/multi USART
	
#ifndef NO_USART_TX
	
#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)
	
	void uart_putc(uint8_t usartct, char data)
	{
		register uint8_t tmp_tx_last_byte;
		
		switch(usartct)
		{
		#ifndef NO_TX0_INTERRUPT 
			case 0:
				tmp_tx_last_byte = tx0_last_byte;
			
				tx0_buffer[tmp_tx_last_byte] = data;
				tmp_tx_last_byte = tx0_last_byte = (tmp_tx_last_byte + 1) & TX0_BUFFER_MASK; // calculate new position of TX tail in buffer
			
				while(tx0_first_byte == tmp_tx_last_byte); // wait for free space in buffer
			
				if(interrupt_semaphore0 == unlocked) // if transmitter interrupt is disabled
				{
					interrupt_semaphore0 = locked;
					UDR0_REGISTER = tx0_buffer[tx0_first_byte]; // enable transmitter interrupt
				}
			break; 
		#endif // NO_TX0_INTERRUPT
		#ifndef NO_TX1_INTERRUPT 
			case 1:
				tmp_tx_last_byte = tx1_last_byte;
			
				tx1_buffer[tmp_tx_last_byte] = data;
				tmp_tx_last_byte = tx1_last_byte = (tmp_tx_last_byte + 1) & TX1_BUFFER_MASK; // calculate new position of TX tail in buffer
			
				while(tx1_first_byte == tmp_tx_last_byte); // wait for free space in buffer
			
				if(interrupt_semaphore1 == unlocked) // if transmitter interrupt is disabled
				{
					interrupt_semaphore1 = locked;
					UDR1_REGISTER = tx1_buffer[tx1_first_byte]; // enable transmitter interrupt
				}
			break;
		#endif // NO_TX1_INTERRUPT
		#ifndef NO_TX2_INTERRUPT 
			case 2:
				tmp_tx_last_byte = tx2_last_byte;
			
				tx2_buffer[tmp_tx_last_byte] = data;
				tmp_tx_last_byte = tx2_last_byte = (tmp_tx_last_byte + 1) & TX2_BUFFER_MASK; // calculate new position of TX tail in buffer
			
				while(tx2_first_byte == tmp_tx_last_byte); // wait for free space in buffer
			
				if(interrupt_semaphore2 == unlocked) // if transmitter interrupt is disabled
				{
					interrupt_semaphore2 = locked;
					UDR2_REGISTER = tx2_buffer[tx2_first_byte]; // enable transmitter interrupt
				}
			break;
		#endif // NO_TX2_INTERRUPT
		#ifndef NO_TX3_INTERRUPT 
			case 3:
				tmp_tx_last_byte = tx3_last_byte;
			
				tx3_buffer[tmp_tx_last_byte] = data;
				tmp_tx_last_byte = tx3_last_byte = (tmp_tx_last_byte + 1) & TX3_BUFFER_MASK; // calculate new position of TX tail in buffer
			
				while(tx3_first_byte == tmp_tx_last_byte); // wait for free space in buffer
			
				if(interrupt_semaphore3 == unlocked) // if transmitter interrupt is disabled
				{
					interrupt_semaphore3 = locked;
					UDR3_REGISTER = tx3_buffer[tx3_first_byte]; // enable transmitter interrupt
				}
				//break;
			#endif // NO_TX3_INTERRUPT
			}
		
	}

	void uart_putstr(uint8_t usartct, char *string)
	{
		while(*string)
			uart_putc(usartct, *string++);
	}

	void uart_putstrl(uint8_t usartct, char *string, uint8_t BytesToWrite)
	{
		while(BytesToWrite--)
			uart_putc(usartct, *string++);
	}

	void uart_puts_p(uint8_t usartct, const char *string)
	{
		while(pgm_read_byte(string))
			uart_putc(usartct, pgm_read_byte(string++));
	}

	void uart_putint(uint8_t usartct, int16_t data)
	{
		char buffer[7]; // heading, 5 digit bytes, NULL
		itoa(data, buffer, 10);

		uart_putstr(usartct, buffer);
	}

	void uart_put_hex(uint8_t usartct, int16_t data)
	{
		char buffer[6]; // heading, 4 digit bytes, NULL
		itoa(data, buffer, 16);
	
		uart_putstr(usartct, buffer);
	}

	void uart_putlong(uint8_t usartct, int32_t data)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		ltoa(data, buffer, 10);
	
		uart_putstr(usartct, buffer);
	}

	void uart_putfloat(uint8_t usartct, float data)
	{
		char buffer[16];
		dtostrf(data, 15, 6, buffer);
	
		char *p = buffer;
		while(*p == ' ') // remove all unwanted spaces
			p++;
	
		uart_putstr(usartct, p);
	}

	void uart_fputfloat(uint8_t usartct, float data, uint8_t size, uint8_t precision)
	{
		char buffer[size+1];
		dtostrf(data, size, precision, buffer);
	
		char *p = buffer;
		while(*p == ' ') // remove all unwanted spaces
			p++;
	
		uart_putstr(usartct, p);
	}
	
#else // single USART mcu

	void uart_putc(char data)
	{
		register uint8_t tmp_tx_last_byte = tx0_last_byte;
		
		tx0_buffer[tmp_tx_last_byte] = data;
		tmp_tx_last_byte = tx0_last_byte = (tmp_tx_last_byte + 1) & TX0_BUFFER_MASK; // calculate new position of TX tail in buffer
			
		while(tx0_first_byte == tmp_tx_last_byte); // wait for free space in buffer
		
		if(interrupt_semaphore0 == unlocked) // if transmitter interrupt is disabled
		{
			interrupt_semaphore0 = locked;
			UDR0_REGISTER = tx0_buffer[tx0_first_byte]; // enable transmitter interrupt
		}
		
	}

	void uart_putstr(char *string)
	{
		while(*string)
			uart_putc(*string++);
	}

	void uart_putstrl(char *string, uint8_t BytesToWrite)
	{
		while(BytesToWrite--)
			uart_putc(*string++);
	}

	void uart_puts_p(const char *string)
	{
		while(pgm_read_byte(string))
			uart_putc(pgm_read_byte(string++));
	}

	void uart_putint(int16_t data)
	{
		char buffer[7]; // heading, 5 digit bytes, NULL
		itoa(data, buffer, 10);

		uart_putstr(buffer);
	}

	void uart_put_hex(int16_t data)
	{
		char buffer[6]; // heading, 4 digit bytes, NULL
		itoa(data, buffer, 16);
		
		uart_putstr(buffer);
	}

	void uart_putlong(int32_t data)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		ltoa(data, buffer, 10);
		
		uart_putstr(buffer);
	}

	void uart_putfloat(float data)
	{
		char buffer[16];
		dtostrf(data, 15, 6, buffer);
		
		char *p = buffer;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart_putstr(p);
	}

	void uart_fputfloat(float data, uint8_t size, uint8_t precision)
	{
		char buffer[size+1];
		dtostrf(data, size, precision, buffer);
		
		char *p = buffer;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart_putstr(p);
	}

#endif // single/multi USART

#endif // NO_USART_TX

#ifndef NO_USART_RX

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)

	char uart_getc(uint8_t usartct)
	{
		register char temp;
	
		register uint8_t tmp_rx_first_byte;
		register uint8_t tmp_rx_last_byte; 
	
		switch(usartct)
		{
		#ifndef NO_RX0_INTERRUPT 
			default: // case 0: // avoid [-Wmaybe-uninitialized] warning

				tmp_rx_first_byte = rx0_first_byte;
				tmp_rx_last_byte = rx0_last_byte;
				
				temp = (tmp_rx_first_byte == tmp_rx_last_byte) ? 0:rx0_buffer[tmp_rx_first_byte];
				if(tmp_rx_first_byte != tmp_rx_last_byte)
					rx0_first_byte = (tmp_rx_first_byte+1) & RX0_BUFFER_MASK; // calculate new position of RX head in buffer
			
			break;
		#endif // NO_RX0_INTERRUPT
		#ifndef NO_RX1_INTERRUPT 
			case 1: 
				tmp_rx_first_byte = rx1_first_byte;
				tmp_rx_last_byte = rx1_last_byte;
				
				temp = (tmp_rx_first_byte == tmp_rx_last_byte) ? 0:rx1_buffer[tmp_rx_first_byte];
				if(tmp_rx_first_byte != tmp_rx_last_byte)
					rx1_first_byte = (tmp_rx_first_byte+1) & RX1_BUFFER_MASK; // calculate new position of RX head in buffer
				
			break;
		#endif // NO_RX1_INTERRUPT
		#ifndef NO_RX2_INTERRUPT
			case 2:
				tmp_rx_first_byte = rx2_first_byte;
				tmp_rx_last_byte = rx2_last_byte;
				
				temp = (tmp_rx_first_byte == tmp_rx_last_byte) ? 0:rx2_buffer[tmp_rx_first_byte];
				if(tmp_rx_first_byte != tmp_rx_last_byte)
					rx2_first_byte = (tmp_rx_first_byte+1) & RX2_BUFFER_MASK; // calculate new position of RX head in buffer
			
			break;
		#endif // NO_RX2_INTERRUPT
		#ifndef NO_RX3_INTERRUPT
			case 3:
				tmp_rx_first_byte = rx3_first_byte;
				tmp_rx_last_byte = rx3_last_byte;
				
				temp = (tmp_rx_first_byte == tmp_rx_last_byte) ? 0:rx3_buffer[tmp_rx_first_byte];
				if(tmp_rx_first_byte != tmp_rx_last_byte)
					rx3_first_byte = (tmp_rx_first_byte+1) & RX3_BUFFER_MASK; // calculate new position of RX head in buffer
		#endif // NO_RX3_INTERRUPT
		}
	
		if(temp == '\n')   temp = 0;
		return temp;
	}

	void uart_gets(uint8_t usartct, char *buffer)
	{
		do *buffer = uart_getc(usartct);
		while(*buffer++);
	}
	
	void uart_getsl(uint8_t usartct, char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			*buffer = uart_getc(usartct);
			if(*buffer++ == 0)
				return;
		}
		*buffer = 0;
	}

	uint8_t uart_getData(uint8_t usartct, uint8_t *data)
	{
		register uint8_t tmp_rx_first_byte;
		
		switch(usartct)
		{
		#ifndef NO_RX0_INTERRUPT
			case 0:
				tmp_rx_first_byte = rx0_first_byte;
			
				*data = rx0_buffer[tmp_rx_first_byte];
				if(tmp_rx_first_byte != rx0_last_byte) // if buffer is not empty
				{
					rx0_first_byte = (tmp_rx_first_byte+1) & RX0_BUFFER_MASK; // calculate new position of RX head in buffer
					return COMPLETED; // result = 0
				}
			break;
		#endif // NO_RX0_INTERRUPT
		#ifndef NO_RX1_INTERRUPT
			case 1:
				tmp_rx_first_byte = rx1_first_byte;
				
				*data = rx1_buffer[tmp_rx_first_byte];
				if(tmp_rx_first_byte != rx1_last_byte) // if buffer is not empty
				{
					rx1_first_byte = (tmp_rx_first_byte+1) & RX1_BUFFER_MASK; // calculate new position of RX head in buffer
					return COMPLETED; // result = 0
				}
			break;
		#endif // NO_RX1_INTERRUPT
		#ifndef NO_RX2_INTERRUPT
			case 2:
				tmp_rx_first_byte = rx2_first_byte;
				
				*data = rx2_buffer[tmp_rx_first_byte];
				if(tmp_rx_first_byte != rx2_last_byte) // if buffer is not empty
				{
					rx2_first_byte = (tmp_rx_first_byte+1) & RX2_BUFFER_MASK; // calculate new position of RX head in buffer
					return COMPLETED; // result = 0
				}
			break;
		#endif // NO_RX2_INTERRUPT
		#ifndef NO_RX3_INTERRUPT
			case 3:
				tmp_rx_first_byte = rx3_first_byte;
				
				*data = rx3_buffer[tmp_rx_first_byte];
				if(tmp_rx_first_byte != rx3_last_byte) // if buffer is not empty
				{
					rx3_first_byte = (tmp_rx_first_byte+1) & RX3_BUFFER_MASK; // calculate new position of RX head in buffer
					return COMPLETED; // result = 0
				}
		#endif // NO_RX3_INTERRUPT
		}
		return BUFFER_EMPTY; // in this case data value is a trash // result = 1
	}

	uint8_t uart_AvailableBytes(uint8_t usartct) 
	{
		switch(usartct)
		{
		#ifndef NO_RX0_INTERRUPT
			default: return (rx0_last_byte - rx0_first_byte) & RX0_BUFFER_MASK;
		#endif // NO_RX0_INTERRUPT
		#ifndef NO_RX1_INTERRUPT
			case 1: return (rx1_last_byte - rx1_first_byte) & RX1_BUFFER_MASK;
		#endif // NO_RX1_INTERRUPT
		#ifndef NO_RX2_INTERRUPT
			case 2: return (rx2_last_byte - rx2_first_byte) & RX2_BUFFER_MASK;
		#endif // NO_RX2_INTERRUPT
		#ifndef NO_RX3_INTERRUPT
			case 3: return (rx3_last_byte - rx3_first_byte) & RX3_BUFFER_MASK;
		#endif // NO_RX3_INTERRUPT
		}
		
	}

#else // single USART mcu

	char uart_getc(void)
	{
		register char temp;
		
		register uint8_t tmp_rx_first_byte = rx0_first_byte;
		register uint8_t tmp_rx_last_byte = rx0_last_byte;
		
		temp = (tmp_rx_first_byte == tmp_rx_last_byte) ? 0:rx0_buffer[tmp_rx_first_byte];
		if(tmp_rx_first_byte != tmp_rx_last_byte)
			rx0_first_byte = (tmp_rx_first_byte+1) & RX0_BUFFER_MASK; // calculate new position of RX head in buffer
			
		if(temp == '\n')   temp = 0;
		return temp;
	}

	void uart_gets(char *buffer)
	{
		do *buffer = uart_getc();
		while(*buffer++);
	}
	
	void uart_getsl(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			*buffer = uart_getc();
			if(*buffer++ == 0)
				return;
		}
		*buffer = 0;
	}

	uint8_t uart_getData(uint8_t *data)
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

	uint8_t uart_AvailableBytes(void)
	{
		return (rx0_last_byte - rx0_first_byte) & RX0_BUFFER_MASK;
	}

#endif // single/multi USART

#endif // NO_USART_RX

#ifndef NO_TX0_INTERRUPT
	ISR(TX0_INTERRUPT)
	{
		register uint8_t tmp_tx_first_byte = tx0_first_byte = (tx0_first_byte + 1) & TX0_BUFFER_MASK;
		// calculate new position of TX head in buffer, write back and use it as register variable 
		
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
	
		#ifdef RX0_BINARY_MODE 
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
			if(rx1_first_byte != (tmp_rx_last_byte)) // tmp_rx_last_byte + 1
		#else
			if(rx1_first_byte != (tmp_rx_last_byte) && (tmp != '\r'))	// tmp_rx_last_byte + 1
		#endif
		{
			rx1_buffer[tmp_rx_last_byte-1] = tmp;	// tmp_rx_last_byte
			rx1_last_byte = (tmp_rx_last_byte) & RX1_BUFFER_MASK; // tmp_rx_last_byte + 1
		}
		
	}
#endif // NO_RX1_INTERRUPT

#ifndef NO_TX2_INTERRUPT
	ISR(TX2_INTERRUPT)
	{
		register uint8_t tmp_tx_first_byte = tx2_first_byte = (tx2_first_byte + 1) & TX2_BUFFER_MASK;
	
		if(tmp_tx_first_byte != tx2_last_byte)
		{
			UDR2_REGISTER = tx2_buffer[tmp_tx_first_byte]; // transmit character from the buffer
		}
		else
		{
			interrupt_semaphore2 = unlocked;
		}
		
	}
#endif // NO_TX2_INTERRUPT

#ifndef NO_RX2_INTERRUPT
	ISR(RX2_INTERRUPT)
	{
		register char tmp;
		tmp = UDR2_REGISTER; // save received character to temporary register
	
		register uint8_t tmp_rx_last_byte = rx2_last_byte + 1;
	
		#ifdef RX2_BINARY_MODE
			if(rx2_first_byte != (tmp_rx_last_byte))
		#else
			if(rx2_first_byte != (tmp_rx_last_byte) && (tmp != '\r'))
		#endif
		{
			rx2_buffer[tmp_rx_last_byte-1] = tmp;
			rx2_last_byte = (tmp_rx_last_byte) & RX2_BUFFER_MASK;
		}
		
	}
#endif // NO_RX2_INTERRUPT

#ifndef NO_TX3_INTERRUPT
	ISR(TX3_INTERRUPT)
	{
		register uint8_t tmp_tx_first_byte = tx2_first_byte = (tx3_first_byte + 1) & TX3_BUFFER_MASK;
	
		if(tmp_tx_first_byte != tx3_last_byte)
		{
			UDR3_REGISTER = tx3_buffer[tmp_tx_first_byte]; // transmit character from the buffer
		}
		else
		{
			interrupt_semaphore3 = unlocked;
		}
		
	}
#endif // NO_TX3_INTERRUPT

#ifndef NO_RX3_INTERRUPT
	ISR(RX3_INTERRUPT)
	{
		register char tmp;
		tmp = UDR3_REGISTER; // save received character to temporary register
		
		register uint8_t tmp_rx_last_byte = rx3_last_byte + 1;
		
		#ifdef RX3_BINARY_MODE
			if(rx3_first_byte != (tmp_rx_last_byte))
		#else
			if(rx3_first_byte != (tmp_rx_last_byte) && (tmp != '\r'))
		#endif
		{
			rx3_buffer[tmp_rx_last_byte-1] = tmp;
			rx3_last_byte = (tmp_rx_last_byte) & RX3_BUFFER_MASK;
		}
		
	}
#endif // NO_RX3_INTERRUPT