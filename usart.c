//**************************************************************
// ****** FUNCTIONS FOR USART COMMUNICATION *******
//**************************************************************
//Compiler          : AVR-GCC
//Author            : jnk0le@hotmail.com
//                    https://github.com/jnk0le
//Date              : 24 June 2015
//License           : MIT
//**************************************************************

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "usart.h"

#ifndef NO_TX0_INTERRUPT
	volatile uint8_t tx0_first_byte, tx0_last_byte;
	char tx0_buffer[TX0_BUFFER_SIZE];
#endif

#ifndef NO_RX0_INTERRUPT
	volatile uint8_t rx0_first_byte, rx0_last_byte;
	char rx0_buffer[RX0_BUFFER_SIZE];
#endif

#ifndef NO_TX1_INTERRUPT
	volatile uint8_t tx1_first_byte, tx1_last_byte;
	char tx1_buffer[TX1_BUFFER_SIZE];
#endif

#ifndef NO_RX1_INTERRUPT
	volatile uint8_t rx1_first_byte, rx1_last_byte;
	char rx1_buffer[RX1_BUFFER_SIZE];
#endif

#ifndef NO_TX2_INTERRUPT
	volatile uint8_t tx2_first_byte, tx2_last_byte;
	char tx2_buffer[TX2_BUFFER_SIZE];
#endif

#ifndef NO_RX2_INTERRUPT
	volatile uint8_t rx2_first_byte, rx2_last_byte;
	char rx2_buffer[RX2_BUFFER_SIZE];
#endif

#ifndef NO_TX3_INTERRUPT
	volatile uint8_t tx3_first_byte, tx3_last_byte;
	char tx3_buffer[TX3_BUFFER_SIZE];
#endif

#ifndef NO_RX3_INTERRUPT
	volatile uint8_t rx3_first_byte, rx3_last_byte;
	char rx3_buffer[RX3_BUFFER_SIZE];
#endif

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)

//******************************************************************
//Function  : To reinitialize USART interface (runtime speed changing).
//Arguments : 1. Id of selected USART interface.
//          : 2. Calculated UBBR value to initialize equal speed.
//Return    :    none
//Note      : Use BAUD_CALC(speed) macro to calculate UBBR value.
//          : All data inside UDR shift register will be lost.
//          : U2X bit is cleared if USARTn_U2X_SPEED is not set.
//******************************************************************
	void uart_reinit(uint8_t usartct, uint16_t ubbr_value)
	{
		switch(usartct)
		{
			default:
		#ifdef USE_USART0
			case 0:
				UCSR0B_REGISTER = 0; // flush all buffers
				
			#ifdef USART0_RS485_MODE
				___DDR(RS485_CONTROL0_PORT) |= (1<<RS485_CONTROL0_PIN);
				___PORT(RS485_CONTROL0_PORT) &= ~(1<<RS485_CONTROL0_PIN); //set low
			#endif
			
				UBRR0L_REGISTER = (uint8_t) ubbr_value;
				UBRR0H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART0_U2X_SPEED
				UCSR0A_REGISTER |= (1<<U2X0_BIT); // enable double speed
			#else
				UCSR0A_REGISTER &= ~(1<<U2X0_BIT);
			#endif
			
				UCSR0B_REGISTER = USART0_CONFIG_B;
				// 8n1 is set by default, setting UCSRC is not needed
			
			break;
		#endif // NO_USART0
		#ifdef USE_USART1
			case 1:
				UCSR1B_REGISTER = 0; // flush all buffers
				
			#ifdef USART1_RS485_MODE
				___DDR(RS485_CONTROL1_PORT) |= (1<<RS485_CONTROL1_PIN);
				___PORT(RS485_CONTROL1_PORT) &= ~(1<<RS485_CONTROL1_PIN); //set low
			#endif
			
				UBRR1L_REGISTER = (uint8_t) ubbr_value;
				UBRR1H_REGISTER = (ubbr_value>>8);
				
			#ifdef USART1_U2X_SPEED
				UCSR1A_REGISTER |= (1<<U2X1_BIT); // enable double speed
			#else
				UCSR1A_REGISTER &= ~(1<<U2X1_BIT);
			#endif
				
				UCSR1B_REGISTER = USART1_CONFIG_B;
				// 8n1 is set by default, setting UCSRC is not needed
			
			break;
		#endif // USE_USART1
		#ifdef USE_USART2
			case 2:
				UCSR2B_REGISTER = 0; // flush all buffers
				
			#ifdef USART2_RS485_MODE
				___DDR(RS485_CONTROL2_PORT) |= (1<<RS485_CONTROL2_PIN);
				___PORT(RS485_CONTROL2_PORT) &= ~(1<<RS485_CONTROL2_PIN); //set low
			#endif
			
				UBRR2L_REGISTER = (uint8_t) ubbr_value;
				UBRR2H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART2_U2X_SPEED
				UCSR2A_REGISTER |= (1<<U2X2_BIT); // enable double speed
			#else
				UCSR2A_REGISTER &= ~(1<<U2X2_BIT);
			#endif
			
				UCSR2B_REGISTER = USART2_CONFIG_B;
				// 8n1 is set by default, setting UCSRC is not needed
			
			break;
		#endif // USE_USART2
		#ifdef USE_USART3
			case 3:
				UCSR3B_REGISTER = 0; // flush all buffers
				
			#ifdef USART3_RS485_MODE
				___DDR(RS485_CONTROL3_PORT) |= (1<<RS485_CONTROL3_PIN);
				___PORT(RS485_CONTROL3_PORT) &= ~(1<<RS485_CONTROL3_PIN); //set low
			#endif
			
				UBRR3L_REGISTER = (uint8_t) ubbr_value;
				UBRR3H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART3_U2X_SPEED
				UCSR3A_REGISTER |= (1<<U2X3_BIT); // enable double speed
			#else
				UCSR3A_REGISTER &= ~(1<<U2X3_BIT);
			#endif
			
				UCSR3B_REGISTER = USART3_CONFIG_B;
				// 8n1 is set by default, setting UCSRC is not needed
			
			//break;
		#endif // USE_USART3
		}
	}

#else // single USART mcu

//******************************************************************
//Function  : To reinitialize USART interface (runtime speed changing).
//Arguments : Calculated UBBR value to initialize equal speed.
//Return    : none
//Note      : Use BAUD_CALC(speed) macro to calculate UBBR value.
//          : All data inside UDR shift register will be lost.
//          : U2X bit is cleared if USARTn_U2X_SPEED is not set.
//******************************************************************
	void uart_reinit(uint16_t ubbr_value)
	{
		UCSR0B_REGISTER = 0; //flush all buffers
		
	#ifdef USART0_RS485_MODE
		___DDR(RS485_CONTROL0_PORT) |= (1<<RS485_CONTROL0_PIN);
		___PORT(RS485_CONTROL0_PORT) &= ~(1<<RS485_CONTROL0_PIN); //set low
	#endif
		
		UBRR0L_REGISTER = (uint8_t) ubbr_value;
		UBRR0H_REGISTER = (ubbr_value>>8);
			
	#ifdef USART0_U2X_SPEED
		UCSR0A_REGISTER |= (1<<U2X0_BIT); // enable double speed
	#else
		UCSR0A_REGISTER &= ~(1<<U2X0_BIT);
	#endif
			
		UCSR0B_REGISTER = USART0_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
	}
	
#endif // single/multi USART

#ifndef NO_USART_TX
	
#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)

//******************************************************************
//Function  : Send single character/byte.
//Arguments : 1. Id of selected USART interface.
//          : 2. character/byte to send.
//Return    :    none
//******************************************************************	
	void uart_putc(uint8_t usartct, char data)
	{
	#ifdef PUTC_CONVERT_LF_TO_CRLF
		if(data == '\n')
			uart_putc(usartct, '\r');
	#endif
		register uint8_t tmp_tx_last_byte;
		
		switch(usartct)
		{
			default: // first found case as default (byte saving)
		#ifndef NO_TX0_INTERRUPT 
			 case 0:
				tmp_tx_last_byte = (tx0_last_byte + 1) & TX0_BUFFER_MASK; // calculate new position of TX head in buffer
			
				while(tx0_first_byte == tmp_tx_last_byte); // wait for free space in buffer
						
				tx0_buffer[tmp_tx_last_byte] = data;
				tx0_last_byte = tmp_tx_last_byte;
			
			#ifdef USART0_RS485_MODE
				cli();
				___PORT(RS485_CONTROL0_PORT) |= (1<<RS485_CONTROL0_PIN); //set high
			#endif
				UCSR0B_REGISTER |= (1<<UDRIE0_BIT); // enable UDRE interrupt
				
			#ifdef USART0_RS485_MODE
				sei();
			#endif
				break; 
		#endif // NO_TX0_INTERRUPT
		#ifndef NO_TX1_INTERRUPT 
			case 1:
				tmp_tx_last_byte = (tx1_last_byte + 1) & TX1_BUFFER_MASK; // calculate new position of TX head in buffer
			
				while(tx1_first_byte == tmp_tx_last_byte); // wait for free space in buffer
				
				tx1_buffer[tmp_tx_last_byte] = data;
				tx1_last_byte = tmp_tx_last_byte;
				
			#ifdef USART1_RS485_MODE
				cli();
				___PORT(RS485_CONTROL1_PORT) |= (1<<RS485_CONTROL1_PIN); //set high
			#endif
				UCSR1B_REGISTER |= (1<<UDRIE1_BIT); // enable UDRE interrupt
				
			#ifdef USART1_RS485_MODE
				sei();
			#endif
				break;
		#endif // NO_TX1_INTERRUPT
		#ifndef NO_TX2_INTERRUPT 
			case 2:
				tmp_tx_last_byte = (tx2_last_byte + 1) & TX2_BUFFER_MASK; // calculate new position of TX head in buffer
			
				while(tx2_first_byte == tmp_tx_last_byte); // wait for free space in buffer
				
				tx2_buffer[tmp_tx_last_byte] = data;
				tx2_last_byte = tmp_tx_last_byte;
				
			#ifdef USART2_RS485_MODE
				cli();
				___PORT(RS485_CONTROL2_PORT) |= (1<<RS485_CONTROL2_PIN); //set high
			#endif
				UCSR2B_REGISTER |= (1<<UDRIE2_BIT); // enable UDRE interrupt
				
			#ifdef USART2_RS485_MODE
				sei();
			#endif
				break;
		#endif // NO_TX2_INTERRUPT
		#ifndef NO_TX3_INTERRUPT 
			case 3:
				tmp_tx_last_byte = (tx3_last_byte + 1) & TX3_BUFFER_MASK; // calculate new position of TX head in buffer
			
				while(tx3_first_byte == tmp_tx_last_byte); // wait for free space in buffer
				
				tx3_buffer[tmp_tx_last_byte] = data;
				tx3_last_byte = tmp_tx_last_byte;
				
			#ifdef USART3_RS485_MODE
				cli();
				___PORT(RS485_CONTROL3_PORT) |= (1<<RS485_CONTROL3_PIN); //set high
			#endif
				UCSR3B_REGISTER |= (1<<UDRIE3_BIT); // enable UDRE interrupt
				
			#ifdef USART3_RS485_MODE
				sei();
			#endif
				//break;
			#endif // NO_TX3_INTERRUPT
			}
		
	}
	
//******************************************************************
//Function  : Send single character/byte.
//Arguments : 1. Id of selected USART interface.
//          : 2. character/byte to send.
//Return    : Status value: 0 = BUFFER_FULL, 1 = COMPLETED.
//Note      : If character cannot be sent due to full transmit buffer, function will abort transmitting character
//******************************************************************
	uint8_t uart_putc_noblock(uint8_t usartct, char data)
	{
		register uint8_t tmp_tx_last_byte;
		
		switch(usartct)
		{
			default: // first found case as default (byte saving)
		#ifndef NO_TX0_INTERRUPT 
			 case 0:
				tmp_tx_last_byte = (tx0_last_byte + 1) & TX0_BUFFER_MASK; // calculate new position of TX head in buffer
			
				if(tx0_first_byte == tmp_tx_last_byte)
					return BUFFER_FULL;
						
				tx0_buffer[tmp_tx_last_byte] = data;
				tx0_last_byte = tmp_tx_last_byte;
						
			#ifdef USART0_RS485_MODE
				cli();
				___PORT(RS485_CONTROL0_PORT) |= (1<<RS485_CONTROL0_PIN); //set high
			#endif
				UCSR0B_REGISTER |= (1<<UDRIE0_BIT); // enable UDRE interrupt
				
			#ifdef USART0_RS485_MODE
				sei();
			#endif
				break; 
		#endif // NO_TX0_INTERRUPT
		#ifndef NO_TX1_INTERRUPT 
			case 1:
				tmp_tx_last_byte = (tx1_last_byte + 1) & TX1_BUFFER_MASK; // calculate new position of TX head in buffer
			
				if(tx1_first_byte == tmp_tx_last_byte)
					return BUFFER_FULL;
				
				tx1_buffer[tmp_tx_last_byte] = data;
				tx1_last_byte = tmp_tx_last_byte;
				
			#ifdef USART1_RS485_MODE
				cli();
				___PORT(RS485_CONTROL1_PORT) |= (1<<RS485_CONTROL1_PIN); //set high
			#endif
				UCSR1B_REGISTER |= (1<<UDRIE1_BIT); // enable UDRE interrupt
				
			#ifdef USART1_RS485_MODE
				sei();
			#endif
				break;
		#endif // NO_TX1_INTERRUPT
		#ifndef NO_TX2_INTERRUPT 
			case 2:
				tmp_tx_last_byte = (tx2_last_byte + 1) & TX2_BUFFER_MASK; // calculate new position of TX head in buffer
			
				if(tx2_first_byte == tmp_tx_last_byte)
					return BUFFER_FULL;
				
				tx2_buffer[tmp_tx_last_byte] = data;
				tx2_last_byte = tmp_tx_last_byte;
				
			#ifdef USART2_RS485_MODE
				cli();
				___PORT(RS485_CONTROL2_PORT) |= (1<<RS485_CONTROL2_PIN); //set high
			#endif
				UCSR2B_REGISTER |= (1<<UDRIE2_BIT); // enable UDRE interrupt
				
			#ifdef USART2_RS485_MODE
				sei();
			#endif
				break;
		#endif // NO_TX2_INTERRUPT
		#ifndef NO_TX3_INTERRUPT 
			case 3:
				tmp_tx_last_byte = (tx3_last_byte + 1) & TX3_BUFFER_MASK; // calculate new position of TX head in buffer
			
				if(tx3_first_byte == tmp_tx_last_byte)
					return BUFFER_FULL;
				
				tx3_buffer[tmp_tx_last_byte] = data;
				tx3_last_byte = tmp_tx_last_byte;
				
			#ifdef USART3_RS485_MODE
				cli();
				___PORT(RS485_CONTROL3_PORT) |= (1<<RS485_CONTROL3_PIN); //set high
			#endif
				UCSR3B_REGISTER |= (1<<UDRIE3_BIT); // enable UDRE interrupt
				
			#ifdef USART3_RS485_MODE
				sei();
			#endif
				//break;
		#endif // NO_TX3_INTERRUPT
		}
		
		return COMPLETED;
	}

//******************************************************************
//Function  : Send string array.
//Arguments : 1. Id of selected USART interface.
//          : 2. Pointer to string array terminated by NULL.
//Return    :    none
//******************************************************************
	void uart_putstr(uint8_t usartct, char *string)
	{
		while(*string)
			uart_putc(usartct, *string++);
	}

//******************************************************************
//Function  : Send string not terminated by NULL or part of the string array.
//Arguments : 1. Id of selected USART interface.
//          : 2. Pointer to string array.
//          : 3. Number of characters/bytes to send.
//Return    :    none
//******************************************************************
	void uart_putstrl(uint8_t usartct, char *string, uint8_t BytesToWrite)
	{
		while(BytesToWrite--)
			uart_putc(usartct, *string++);
	}

//******************************************************************
//Function  : Send string from flash memory.
//Arguments : 1. Id of selected USART interface.
//          : 2. Pointer to string placed in flash memory.
//Return    :    none
//******************************************************************
	void uart_puts_p(uint8_t usartct, const char *string)
	{
		register char c;
		while ((c = pgm_read_byte(string++)) ) uart_putc(usartct,c);
	}

//******************************************************************
//Function  : Send integer formated into ASCI string (base 10).
//Arguments : 1. Id of selected USART interface.
//          : 2. int16_t data value.
//Return    :    none
//******************************************************************
	void uart_putint(uint8_t usartct, int16_t data)
	{
		char buffer[7]; // heading, 5 digit bytes, NULL
		itoa(data, buffer, 10);

		uart_putstr(usartct, buffer);
	}
	
//******************************************************************
//Function  : Send integer formated into ASCI string.
//Arguments : 1. Id of selected USART interface.
//          : 2. int16_t data value.
//          : 3. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart_putintr(uint8_t usartct, int16_t data, uint8_t radix)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		itoa(data, buffer, radix);

		uart_putstr(usartct, buffer);
	}
	
//******************************************************************
//Function  : Send unsigned integer formated into ASCI string (base 10).
//Arguments : 1. Id of selected USART interface.
//          : 2. uint16_t data value.
//Return    :    none
//******************************************************************
	void uart_putuint(uint8_t usartct, uint16_t data)
	{
		char buffer[7]; // heading, 5 digit bytes, NULL
		utoa(data, buffer, 10);

		uart_putstr(usartct, buffer);
	}
	
//******************************************************************
//Function  : Send unsigned integer formated into ASCI string.
//Arguments : 1. Id of selected USART interface.
//          : 2. uint16_t data value.
//          : 3. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart_putuintr(uint8_t usartct, uint16_t data, uint8_t radix)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		utoa(data, buffer, radix);

		uart_putstr(usartct, buffer);
	}

//******************************************************************
//Function  : Send integer formated into ASCI string (base 16)
//Arguments : 1. Id of selected USART interface.
//          : 2. int16_t data value.
//Return    :    none
//******************************************************************
	void uart_put_hex(uint8_t usartct, uint16_t data)
	{
		char buffer[6]; // heading, 4 digit bytes, NULL
		utoa(data, buffer, 16);
	
		uart_putstr(usartct, buffer);
	}

//******************************************************************
//Function  : Send long integer formated into ASCI string (base 10).
//Arguments : 1. Id of selected USART interface.
//          : 2. int32_t data value.
//Return    :    none
//******************************************************************
	void uart_putlong(uint8_t usartct, int32_t data)
	{
		char buffer[12]; // heading, 10 digit bytes, NULL
		ltoa(data, buffer, 10);
	
		uart_putstr(usartct, buffer);
	}

//******************************************************************
//Function  : Send long integer formated into ASCI string.
//Arguments : 1. Id of selected USART interface.
//          : 2. int32_t data value.
//          : 3. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart_putlongr(uint8_t usartct, int32_t data, uint8_t radix)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		ltoa(data, buffer, radix);
		
		uart_putstr(usartct, buffer);
	}

//******************************************************************
//Function  : Send unsigned long integer formated into ASCI string (base 10).
//Arguments : 1. Id of selected USART interface.
//          : 2. uint32_t data value.
//Return    :    none
//******************************************************************
	void uart_putulong(uint8_t usartct, uint32_t data)
	{
		char buffer[12]; // heading, 10 digit bytes, NULL
		ultoa(data, buffer, 10);
	
		uart_putstr(usartct, buffer);
	}

//******************************************************************
//Function  : Send unsigned long integer formated into ASCI string.
//Arguments : 1. Id of selected USART interface.
//          : 2. uint32_t data value.
//          : 3. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart_putulongr(uint8_t usartct, uint32_t data, uint8_t radix)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		ltoa(data, buffer, radix);
		
		uart_putstr(usartct, buffer);
	}

//******************************************************************
//Function  : Send floating point value formated into ASCI string.
//Arguments : 1. Id of selected USART interface.
//          : 2. float data value.
//Return    :    none
//******************************************************************
	void uart_putfloat(uint8_t usartct, float data)
	{
		char buffer[16];
		dtostrf(data, 15, 6, buffer);
	
		char *p = buffer;
		while(*p == ' ') // remove all unwanted spaces
			p++;
	
		uart_putstr(usartct, p);
	}

//******************************************************************
//Function  : Send floating point value formated into ASCI string.
//Arguments : 1. Id of selected USART interface.
//          : 2. Float data value.
//          : 3. Number of displayed digits after the dot.
//Return    :    none
//******************************************************************
	void uart_fputfloat(uint8_t usartct, float data, uint8_t precision)
	{
		char buffer[16];
		dtostrf(data, 15, precision, buffer);
	
		char *p = buffer;
		while(*p == ' ') // remove all unwanted spaces
			p++;
	
		uart_putstr(usartct, p);
	}
	
#else // single USART mcu

//******************************************************************
//Function  : Send single character/byte.
//Arguments : Character/byte to send.
//Return    : none
//******************************************************************
	void uart_putc(char data)
	{
	#ifdef PUTC_CONVERT_LF_TO_CRLF
		if (data == '\n')
			uart_putc('\r');
	#endif
		register uint8_t tmp_tx_last_byte = (tx0_last_byte + 1) & TX0_BUFFER_MASK; // calculate new position of TX head in buffer
		
		while(tx0_first_byte == tmp_tx_last_byte); // wait for free space in buffer
		
		tx0_buffer[tmp_tx_last_byte] = data;
		tx0_last_byte = tmp_tx_last_byte;
		
	#ifdef USART0_RS485_MODE
		cli();
		___PORT(RS485_CONTROL0_PORT) |= (1<<RS485_CONTROL0_PIN); //set high
	#endif
		UCSR0B_REGISTER |= (1<<UDRIE0_BIT); // enable UDRE interrupt
	
	#ifdef USART0_RS485_MODE
		sei();
	#endif
	}
	
//******************************************************************
//Function  : Send single character/byte.
//Arguments : Character/byte to send.
//Return    : Status value: 0 = BUFFER_FULL, 1 = COMPLETED.
//Note      : If character cannot be sent due to full transmit buffer, function will abort transmitting character
//******************************************************************
	uint8_t uart_putc_noblock(char data)
	{
		register uint8_t tmp_tx_last_byte = (tx0_last_byte + 1) & TX0_BUFFER_MASK; // calculate new position of TX head in buffer
		
		if(tx0_first_byte == tmp_tx_last_byte)
			return BUFFER_FULL;
		
		tx0_buffer[tmp_tx_last_byte] = data;
		tx0_last_byte = tmp_tx_last_byte;
		
	#ifdef USART0_RS485_MODE
		cli();
		___PORT(RS485_CONTROL0_PORT) |= (1<<RS485_CONTROL0_PIN); //set high
	#endif
		UCSR0B_REGISTER |= (1<<UDRIE0_BIT); // enable UDRE interrupt
		
	#ifdef USART0_RS485_MODE
		sei();
	#endif
		return COMPLETED;
	}

//******************************************************************
//Function  : Send string array.
//Arguments : Pointer to string array terminated by NULL.
//Return    : none
//******************************************************************
	void uart_putstr(char *string)
	{
		while(*string)
			uart_putc(*string++);
	}

//******************************************************************
//Function  : Send string not terminated by NULL or part of the string array.
//Arguments : 1. Pointer to string array.
//          : 2. Number of characters/bytes to send.
//Return    :    none
//******************************************************************
	void uart_putstrl(char *string, uint8_t BytesToWrite)
	{
		while(BytesToWrite--)
			uart_putc(*string++);
	}

//******************************************************************
//Function  : Send string from flash memory.
//Arguments : Pointer to string placed in flash memory.
//Return    : none
//******************************************************************
	void uart_puts_p(const char *string)
	{
		register char c;
		while ( (c = pgm_read_byte(string++)) ) uart_putc(c);
	}

//******************************************************************
//Function  : Send integer formated into ASCI string (base 10).
//Arguments : int16_t data value.
//Return    : none
//******************************************************************
	void uart_putint(int16_t data)
	{
		char buffer[7]; // heading, 5 digit bytes, NULL
		itoa(data, buffer, 10);

		uart_putstr(buffer);
	}
	
//******************************************************************
//Function  : Send integer formated into ASCI string.
//Arguments : 1. uint16_t data value.
//          : 2. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart_putintr(int16_t data, uint8_t radix)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		itoa(data, buffer, radix);

		uart_putstr(buffer);
	}
	
//******************************************************************
//Function  : Send unsigned integer formated into ASCI string (base 10).
//Arguments : uint16_t data value.
//Return    : none
//******************************************************************
	void uart_putuint(uint16_t data)
	{
		char buffer[7]; // heading, 5 digit bytes, NULL
		utoa(data, buffer, 10);

		uart_putstr(buffer);
	}
	
//******************************************************************
//Function  : Send unsigned integer formated into ASCI string.
//Arguments : 1. uint16_t data value.
//          : 2. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart_putuintr(uint16_t data, uint8_t radix)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		utoa(data, buffer, radix);

		uart_putstr(buffer);
	}

//******************************************************************
//Function  : Send unsigned integer formated into ASCI string (base 16)
//Arguments : uint16_t data value.
//Return    : none
//******************************************************************
	void uart_put_hex(uint16_t data)
	{
		char buffer[6]; // heading, 4 digit bytes, NULL
		utoa(data, buffer, 16);
		
		uart_putstr(buffer);
	}

//******************************************************************
//Function  : Send long integer formated into ASCI string (base 10).
//Arguments : int32_t data value.
//Return    : none
//******************************************************************
	void uart_putlong(int32_t data)
	{
		char buffer[12]; // heading, 10 digit bytes, NULL
		ltoa(data, buffer, 10);
		
		uart_putstr(buffer);
	}
	
//******************************************************************
//Function  : Send long integer formated into ASCI string.
//Arguments : 1. int32_t data value.
//          : 2. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart_putlongr(int32_t data, uint8_t radix)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		ltoa(data, buffer, radix);
		
		uart_putstr(buffer);
	}
	
//******************************************************************
//Function  : Send unsigned long integer formated into ASCI string (base 10).
//Arguments : uint32_t data value.
//Return    : none
//******************************************************************
	void uart_putulong(uint32_t data)
	{
		char buffer[12]; // heading, 10 digit bytes, NULL
		ultoa(data, buffer, 10);
		
		uart_putstr(buffer);
	}
	
//******************************************************************
//Function  : Send unsigned long integer formated into ASCI string.
//Arguments : 1. uint32_t data value.
//          : 2. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart_putulongr(uint32_t data, uint8_t radix)
	{
		char buffer[17]; // heading, 15 digit bytes, NULL
		ultoa(data, buffer, radix);
		
		uart_putstr(buffer);
	}

//******************************************************************
//Function  : Send floating point value formated into ASCI string.
//Arguments : float data value.
//Return    : none
//******************************************************************
	void uart_putfloat(float data)
	{
		char buffer[16];
		dtostrf(data, 15, 6, buffer);
		
		char *p = buffer;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart_putstr(p);
	}

//******************************************************************
//Function  : Send floating point integer formated into ASCI string.
//Arguments : 1. Float data value.
//          : 2. Number of displayed digits after the dot.
//Return    :    none
//******************************************************************
	void uart_fputfloat(float data, uint8_t precision)
	{
		char buffer[16];
		dtostrf(data, 15, precision, buffer);
		
		char *p = buffer;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart_putstr(p);
	}

#endif // single/multi USART

#endif // NO_USART_TX

#ifndef NO_USART_RX

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)

//******************************************************************
//Function  : To receive single character/byte.
//Arguments : Id of selected USART interface.
//Return    : Received character or NULL if buffer is empty.
//******************************************************************
	char uart_getc(uint8_t usartct)
	{
		register uint8_t tmp_rx_first_byte, tmp;
	
		switch(usartct)
		{
			default: // first found case as default (byte saving)
		#ifndef NO_RX0_INTERRUPT
			case 0:
				tmp_rx_first_byte = rx0_first_byte;
				
				if(tmp_rx_first_byte == rx0_last_byte) return 0;
				rx0_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX0_BUFFER_MASK;
				
				tmp = rx0_buffer[tmp_rx_first_byte];
			
			#ifdef RX0_GETC_ECHO
			
			#ifdef RX_NEWLINE_MODE_N
				if(tmp == '\n') uart_putc(usartct,'\r');
			#endif
				
				uart_putc(usartct, tmp);
				
			#ifdef RX_NEWLINE_MODE_R
				if(tmp == '\r') uart_putc(usartct,'\n');
			#endif
				
			#endif // RX0_GETC_ECHO
				
			break;
		#endif // NO_RX0_INTERRUPT
		#ifndef NO_RX1_INTERRUPT 
			case 1: 
				tmp_rx_first_byte = rx1_first_byte;
				
				if(tmp_rx_first_byte == rx1_last_byte) return 0;
				rx1_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX1_BUFFER_MASK;
				
				tmp = rx1_buffer[tmp_rx_first_byte];
			
			#ifdef RX1_GETC_ECHO
			
			#ifdef RX_NEWLINE_MODE_N
				if(tmp == '\n') uart_putc(usartct,'\r');
			#endif
			
				uart_putc(usartct, tmp);
			
			#ifdef RX_NEWLINE_MODE_R
				if(tmp == '\r') uart_putc(usartct,'\n');
			#endif
			
			#endif // RX1_GETC_ECHO
				
			break;
		#endif // NO_RX1_INTERRUPT
		#ifndef NO_RX2_INTERRUPT
			case 2:
				tmp_rx_first_byte = rx2_first_byte;
				
				if(tmp_rx_first_byte == rx2_last_byte) return 0;
				rx2_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX2_BUFFER_MASK;
				
				tmp = rx2_buffer[tmp_rx_first_byte];
			
			#ifdef RX2_GETC_ECHO
			
			#ifdef RX_NEWLINE_MODE_N
				if(tmp == '\n') uart_putc(usartct,'\r');
			#endif
			
				uart_putc(usartct, tmp);
			
			#ifdef RX_NEWLINE_MODE_R
				if(tmp == '\r') uart_putc(usartct,'\n');
			#endif
			
			#endif // RX2_GETC_ECHO
			
			break;
		#endif // NO_RX2_INTERRUPT
		#ifndef NO_RX3_INTERRUPT
			case 3:
				tmp_rx_first_byte = rx3_first_byte;
				
				if(tmp_rx_first_byte == rx3_last_byte) return 0;
				rx3_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX3_BUFFER_MASK;
				
				tmp = rx3_buffer[tmp_rx_first_byte];
				
			#ifdef RX3_GETC_ECHO
			
			#ifdef RX_NEWLINE_MODE_N
				if(tmp == '\n') uart_putc(usartct,'\r');
			#endif
			
				uart_putc(usartct, tmp);
			
			#ifdef RX_NEWLINE_MODE_R
				if(tmp == '\r') uart_putc(usartct,'\n');
			#endif
			
			#endif // RX3_GETC_ECHO
				
		#endif // NO_RX3_INTERRUPT
		}
	
	return tmp;
	}

//******************************************************************
//Function  : Reads string from receiver buffer.
//Arguments : 1. Id of selected USART interface.
//          : 2. Pointer to array to fill with received string.
//Return    :    none
//Note      : Received string will be terminated by NULL.
//          : OBSOLETE - possibility of buffer overflows
//******************************************************************
//	void uart_getBuffer(uint8_t usartct, char *buffer)
//	{
//		do *buffer = uart_getc(usartct);
//		while(*buffer++);
//	}
	
//******************************************************************
//Function  : Reads string from receiver buffer
//Arguments : 1. Id of selected USART interface.
//          : 2. Pointer to array to fill with received string.
//          : 3. Limit for receiving string size (array size)
//Return    :    none
//Note      : Received string will be terminated by NULL positioned at bufferlimit-1
//          : or at the end of the string if it's shorter than bufferlimit-1
//          : terminators CR LF will not be cut
//******************************************************************
	void uart_gets(uint8_t usartct, char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			*buffer = uart_getc(usartct);
			if(*buffer++ == 0)
			return;
		}
	*buffer = 0;
	}

//******************************************************************
//Function  : Reads one line from the receiver buffer. (waits for EOL terminator)
//Arguments : 1. Id of selected USART interface.
//          : 2. Pointer to array to fill with received string.
//          : 3. Limit for receiving string size (array size)
//Return    :    none
//Note      : Received string will be terminated by NULL positioned at bufferlimit-1
//          : or at the end of the string if it's shorter than bufferlimit-1
//          : CR and LF terminators will be cut from the stream. 
//			: Function will return if bufferlimit is reached without waiting for newline terminator 
//******************************************************************
	void uart_getln(uint8_t usartct, char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			do{
				*buffer = uart_getc(usartct);
			}while(*buffer == 0);
		
		#ifdef RX_NEWLINE_MODE_N 
			if(*buffer == '\n')
		#else
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart_getc(usartct)) );
			#endif
				break;
			}
		
			buffer++;

		}
		*buffer = 0;
	}

//******************************************************************
//Function  : Reads burst of characters until first whitespace (waits for EOL terminator or first whitespace)
//Arguments : 1. Id of selected USART interface.
//          : 2. Pointer to array to fill with received string. 
//          : 3. Limit for receiving string size (array size)
//Return    :    none
//Note      : Received string will be terminated by NULL positioned at bufferlimit-1
//          : or at the end of the string if it's shorter than bufferlimit-1
//          : CR and LF terminators will be cut.
//          : Function will return if bufferlimit is reached without waiting for newline terminator
//          : Function will cut all whitespaces before first nonspace character
//******************************************************************
	void uart_getlnToFirstWhiteSpace(uint8_t usartct, char *buffer, uint8_t bufferlimit)
	{
		*buffer++ = uart_skipWhiteSpaces(usartct);
		bufferlimit--;
		
		while(--bufferlimit)
		{
			do{
				*buffer = uart_getc(usartct);
			} while(*buffer == 0);
				
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else //RX_NEWLINE_MODE_R
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart_getc(usartct)) );
			#endif
				break;
			}
			else if(*buffer <= 32)
				break; // string reading is done, we will exit
			
			buffer++;
		
		}
		*buffer = 0;
	}

//******************************************************************
//Function  : To skip all incoming whitespace characters until first nonspace character.
//Arguments : ID of selected usart interface.
//Return    : First received nonspace character.
//Note      : First nonspace character is cut from receiver buffer.
//******************************************************************
	char uart_skipWhiteSpaces(uint8_t usartct)
	{
		register char c;
		
		do{
			c = uart_getc(usartct);
		}while(c <= 32);
		
		return c;
	}

//******************************************************************
//Function  : Read 16bit integer value from the input stream.
//Arguments : ID of selected usart interface.
//Return    : Received 16bit integer value.
//******************************************************************
	int16_t uart_getint(uint8_t usartct)
	{
		char buff[32];
		uart_getlnToFirstWhiteSpace(usartct, buff, 32);
		
		return atoi(buff);
	}

//******************************************************************
//Function  : Read 32bit integer value from the input stream.
//Arguments : ID of selected usart interface.
//Return    : Received 32bit integer value.
//******************************************************************
	int32_t uart_getlong(uint8_t usartct)
	{
		char buff[32];
		uart_getlnToFirstWhiteSpace(usartct, buff, 32);
			
		return atol(buff);
	}

//******************************************************************
//Function  : Read floating point value from the input stream.
//Arguments : ID of selected usart interface.
//Return    : Received float value.
//******************************************************************
	float uart_getfloat(uint8_t usartct)
	{
		char buff[32];
		uart_getlnToFirstWhiteSpace(usartct, buff, 32);
		
		return atof(buff);
	}

//******************************************************************
//Function  : To receive single byte in binary transmission.
//Arguments : 1. Id of selected USART interface.
//          : 2. Pointer to byte which have to be filed by incoming data.
//Return    :    Status value: 0 = BUFFER_EMPTY, 1 = COMPLETED.
//Note      : This function doesn't cut CR, LF, NULL terminators
//          : If receiver buffer is empty return status = BUFFER_EMPTY instead of returning NULL (as in getc).
//******************************************************************
	uint8_t uart_getData(uint8_t usartct, uint8_t *data)
	{
		register uint8_t tmp_rx_first_byte;
		
		switch(usartct)
		{
			default: //case 0: // avoid compiler warnings if RX0 is not used
		#ifndef NO_RX0_INTERRUPT
			
				tmp_rx_first_byte = rx0_first_byte;
			
				if(tmp_rx_first_byte == rx0_last_byte) return BUFFER_EMPTY; // result = 0
						
				rx0_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX0_BUFFER_MASK;
				*data = rx0_buffer[tmp_rx_first_byte];
				
			break;
		#endif // NO_RX0_INTERRUPT
		#ifndef NO_RX1_INTERRUPT
			case 1:
				tmp_rx_first_byte = rx1_first_byte;
				
				if(tmp_rx_first_byte == rx1_last_byte) return BUFFER_EMPTY; // result = 0
				
				rx1_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX1_BUFFER_MASK;
				*data = rx1_buffer[tmp_rx_first_byte];
				
			break;
		#endif // NO_RX1_INTERRUPT
		#ifndef NO_RX2_INTERRUPT
			case 2:
				tmp_rx_first_byte = rx2_first_byte;
				
				if(tmp_rx_first_byte == rx2_last_byte) return BUFFER_EMPTY; // result = 0
				
				rx2_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX2_BUFFER_MASK;
				*data = rx2_buffer[tmp_rx_first_byte];
				
			break;
		#endif // NO_RX2_INTERRUPT
		#ifndef NO_RX3_INTERRUPT
			case 3:
				tmp_rx_first_byte = rx3_first_byte;
				
				if(tmp_rx_first_byte == rx3_last_byte) return BUFFER_EMPTY; // result = 0
				
				rx3_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX3_BUFFER_MASK;
				*data = rx3_buffer[tmp_rx_first_byte];
				
		#endif // NO_RX3_INTERRUPT
		}
		
		return COMPLETED; // result = 1
	}

//******************************************************************
//Function  : To check how many bytes are waiting in the receiver buffer.
//Arguments : Id of selected USART interface.
//Return    : Number of bytes waiting in receiver buffer.
//******************************************************************
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

//******************************************************************
//Function  : To receive single character/byte.
//Arguments : none
//Return    : Received character or NULL if buffer is empty.
//******************************************************************
	char uart_getc(void)
	{
		register uint8_t tmp_rx_first_byte = rx0_first_byte;
		
		if(tmp_rx_first_byte == rx0_last_byte) return 0;
		rx0_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX0_BUFFER_MASK;
		
		register char tmp = rx0_buffer[tmp_rx_first_byte];
	
	#ifdef RX0_GETC_ECHO
		
	#ifdef RX_NEWLINE_MODE_N
		if(tmp == '\n') uart_putc('\r');
	#endif
		
		uart_putc(tmp);
		
	#ifdef RX_NEWLINE_MODE_R
		if(tmp == '\r') uart_putc('\n');
	#endif

	#endif // RX0_GETC_ECHO
		
		return tmp;
	}


//******************************************************************
//Function  : Reads string from receiver buffer.
//Arguments : Pointer to array to fill with received string.
//Return    : none
//Note      : Received string will be terminated by NULL.
//          : OBSOLETE - possibility of buffer overflows
//******************************************************************
//	void uart_getBuffer(char *buffer)
//	{
//		do *buffer = uart_getc();
//		while(*buffer++);
//	}
	
//******************************************************************
//Function  : Reads string from receiver buffer
//Arguments : 1. Pointer to array to fill with received string.
//          : 2. Limit for receiving string size (array size)
//Return    :    none
//Note      : Received string will be terminated by NULL positioned at bufferlimit-1
//          : or at the end of the string if it's shorter than bufferlimit-1
//			: terminators CR LF will not be cut
//******************************************************************
	void uart_gets(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			*buffer = uart_getc();
			if(*buffer++ == 0)
			return;
		}
	*buffer = 0;
	}

//******************************************************************
//Function  : Reads one line from the receiver buffer. (waits for EOL terminator)
//Arguments : 1. Pointer to array to fill with received string.
//          : 2. Limit for receiving string size (array size)
//Return    :    none
//Note      : Received string will be terminated by NULL positioned at bufferlimit-1
//          : or at the end of the string if it's shorter than bufferlimit-1
//          : CR and LF terminators will be cut. 
//          : Function will return if bufferlimit is reached without waiting for newline terminator
//******************************************************************
	void uart_getln(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			do{
				*buffer = uart_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart_getc()) );
			#endif
				break;
			}
			buffer++;
		}
		*buffer = 0;
	}

//******************************************************************
//Function  : Reads burst of characters until first whitespace (waits for EOL terminator or first whitespace)
//Arguments : 1. Pointer to array to fill with received string.
//          : 2. Limit for receiving string size (array size)
//Return    :    none
//Note      : Received string will be terminated by NULL positioned at bufferlimit-1
//          : or at the end of the string if it's shorter than bufferlimit-1
//          : CR and LF terminators will be cut.
//          : Function will return if bufferlimit is reached without waiting for newline terminator
//          : Function will cut all whitespaces before first nonspace character
//******************************************************************
	void uart_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit)
	{
		*buffer++ = uart_skipWhiteSpaces();
		bufferlimit--;
		
		while(--bufferlimit)
		{
			do{
				*buffer = uart_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else //RX_NEWLINE_MODE_R
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart_getc()) );
			#endif
				break;
			}
			else if(*buffer <= 32)
				break; // string reading is done, we will exit

			buffer++;
		}
		*buffer = 0;
	}

//******************************************************************
//Function  : To skip all incoming whitespace characters until first nonspace character.
//Arguments : none
//Return    : First received nonspace character.
//Note      : First nonspace character is cut from receiver buffer.
//******************************************************************
	char uart_skipWhiteSpaces(void)
	{
		register char c;
		
		do{
			c = uart_getc();
		}while(c <= 32);
		
		return c;
	}

//******************************************************************
//Function  : Read 16bit integer value from the input stream.
//Arguments : none
//Return    : Received 16bit integer value.
//******************************************************************
	int16_t uart_getint(void)
	{
		char buff[32];
		uart_getlnToFirstWhiteSpace(buff, 32);
		
		return atoi(buff);
	}

//******************************************************************
//Function  : Read 32bit integer value from the input stream.
//Arguments : none
//Return    : Received 32bit integer value
//******************************************************************
	int32_t uart_getlong(void)
	{
		char buff[32];
		uart_getlnToFirstWhiteSpace(buff, 32);
		
		return atol(buff);
	}

//******************************************************************
//Function  : Read floating point value from the input stream.
//Arguments : none
//Return    : Received float value.
//******************************************************************
	float uart_getfloat(void)
	{
		char buff[32];
		uart_getlnToFirstWhiteSpace(buff, 32);
		
		return atof(buff);
	}

//******************************************************************
//Function  : To receive single byte in binary transmission.
//Arguments : Pointer to byte which have to be filed by incoming data.
//Return    : Status value: 0 = BUFFER_EMPTY, 1 = COMPLETED.
//Note      : This function doesn't cut CR, LF, NULL terminators
//          : If receiver buffer is empty return status = BUFFER_EMPTY instead of returning NULL (as in getc).
//******************************************************************
	uint8_t uart_getData(uint8_t *data)
	{
		register uint8_t tmp_rx_first_byte = rx0_first_byte;
		
		if(tmp_rx_first_byte == rx0_last_byte) return BUFFER_EMPTY; // result = 0
		
		rx0_first_byte = tmp_rx_first_byte = (tmp_rx_first_byte+1) & RX0_BUFFER_MASK;
		*data = rx0_buffer[tmp_rx_first_byte];
		
		return COMPLETED; // result = 1
	}

//******************************************************************
//Function  : To check how many bytes are waiting in the receiver buffer.
//Arguments : none
//Return    : Number of bytes waiting in receiver buffer.
//******************************************************************
	uint8_t uart_AvailableBytes(void)
	{
		return (rx0_last_byte - rx0_first_byte) & RX0_BUFFER_MASK;
	}

#endif // single/multi USART

#endif // NO_USART_RX


#ifdef UART_USE_STDIO

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)

	#ifndef NO_USART_TX
		
		void uart_putchar(char data, FILE *stream)
		{
			if ( data == '\n') uart_putc((uint16_t) stream -> udata, '\r');
			
			uart_putc((uint16_t) stream -> udata, data);
		}
		
	#endif // NO_USART_TX

	#ifndef NO_USART_RX
		
		char uart_getchar(FILE *stream)
		{
			uint8_t data;
			
			while( BUFFER_EMPTY == uart_getData((uint16_t) stream -> udata, &data) );
			
		#ifdef RX_STDIO_GETCHAR_ECHO
			uart_putc((uint16_t) stream -> udata, data);
		#endif
			
			return data;
		}
		
	#endif // NO_USART_RX

	
	#ifdef USE_USART0
	
		#if defined(NO_RX0_INTERRUPT)
			FILE uart0_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, 0);
	
		#elif defined(NO_TX0_INTERRUPT)
			FILE uart0_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, 0);
		#else
			FILE uart0_io = FDEV_SETUP_STREAM_U(uart_putchar, uart_getchar, _FDEV_SETUP_RW, 0);
		#endif
	
	#endif // USE_USART0
	
	#ifdef USE_USART1
	
		#if defined(NO_RX1_INTERRUPT)
			FILE uart1_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, 1);
	
		#elif defined(NO_TX1_INTERRUPT)
			FILE uart1_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, 1);
		#else
			FILE uart1_io = FDEV_SETUP_STREAM_U(uart_putchar, uart_getchar, _FDEV_SETUP_RW, 1);
		#endif
	
	#endif // USE_USART1
	
	#ifdef USE_USART2
	
		#if defined(NO_RX2_INTERRUPT)
			FILE uart2_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, 2);
	
		#elif defined(NO_TX2_INTERRUPT)
			FILE uart2_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, 2);
		#else
			FILE uart2_io = FDEV_SETUP_STREAM_U(uart_putchar, uart_getchar, _FDEV_SETUP_RW, 2);
		#endif
	
	#endif // USE_USART2
	
	#ifdef USE_USART3
	
		#if defined(NO_RX3_INTERRUPT)
			FILE uart3_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, 3);
	
		#elif defined(NO_TX3_INTERRUPT)
			FILE uart3_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, 3);
		#else
			FILE uart3_io = FDEV_SETUP_STREAM_U(uart_putchar, uart_getchar, _FDEV_SETUP_RW, 3);
		#endif
	
	#endif // USE_USART3

#else // single USART mcu

	#ifndef NO_TX0_INTERRUPT
		
		void uart_putchar(char data, FILE *stream)
		{
			if (data == '\n') uart_putc('\r');
		
			uart_putc(data);
		}
	#endif // NO_TX0_INTERRUPT

	#ifndef NO_RX0_INTERRUPT
		
		char uart_getchar(FILE *stream)
		{
			uint8_t data;
		
			while( BUFFER_EMPTY == uart_getData(&data) );
		
		#ifdef RX_STDIO_GETCHAR_ECHO
			uart_putc(data);
		#endif
		
			return data;
		}
	#endif //NO_RX0_INTERRUPT
	
	
	#if defined(NO_RX0_INTERRUPT)
		FILE uart0_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
		
	#elif defined(NO_TX0_INTERRUPT)
		FILE uart0_in = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
	#else
		FILE uart0_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	#endif

#endif // single/multi USART

#endif // UART_USE_STDIO

//******************************************************************
//ISR prototypes
//******************************************************************
/*
	ISR(TXn_INTERRUPT)
	{
		register uint8_t tmp_tx_first_byte = txn_first_byte;
		
		if(tmp_tx_first_byte != txn_last_byte)
		{
			tmp_tx_first_byte = (tmp_tx_first_byte + 1) & TXn_BUFFER_MASK; // calculate new position of TX head in buffer
			
			UDRn_REGISTER = txn_buffer[tmp_tx_first_byte]; // transmit character from the buffer
			txn_first_byte = tmp_tx_first_byte;
		}
		else
		{
			UCSRnB_REGISTER &= ~(1<<UDRIEn_BIT);
		}
		
	}

	ISR(RXn_INTERRUPT)
	{
		register uint8_t tmp_rx_last_byte = (rxn_last_byte + 1) & RXn_BUFFER_MASK;
		register uint8_t tmp = UDRn_REGISTER;

		if(rxn_first_byte != tmp_rx_last_byte)
		{
			rxn_buffer[tmp_rx_last_byte] = tmp;
			rxn_last_byte = tmp_rx_last_byte;
		}
		
	}
*/

#ifndef NO_TX0_INTERRUPT

	ISR(UDRE0_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"                      /* 4 ISR entry */
	
		"push  r0 \n\t"                          /* 2 */
		"in    r0, __SREG__ \n\t"                /* 1 */

		"push  r24 \n\t"                         /* 2 */ 
		"push  r25 \n\t"                         /* 2 */
		"push  r30 \n\t"                         /* 2 */
		"push  r31 \n\t"                         /* 2 */
		
		/* load globals */
		"lds   r24, (tx0_first_byte) \n\t"       /* 2 */
		"lds   r25, (tx0_last_byte) \n\t"        /* 2 */
		
		/* if(tmp_tx_first_byte != tx0_last_byte) */
		"cp    r24, r25 \n\t"                    /* 1 */
		"breq  USART0_DISABLE_UDRIE_%= \n\t"                        /* 1/2 */
		
		/* tmp_tx_first_byte = tmp_tx_first_byte + 1 */
		"subi  r24, 0xFF \n\t"                   /* 1 */
	
		/* tmp_tx_first_byte &= TX0_BUFFER_MASK */
	#if (TX0_BUFFER_MASK != 0xff)
		"andi  r24, %M[mask]\n\t"                /* 1 */
	#endif
		
		/* tx0_buffer[tmp_tx_first_byte] */
		"mov   r30, r24 \n\t"                    /* 1 */
		"ldi   r31, 0x00 \n\t"                   /* 1 */
		"subi  r30, lo8(-(tx0_buffer)) \n\t"     /* 1 */
		"sbci  r31, hi8(-(tx0_buffer)) \n\t"     /* 1 */
		"ld    r25, Z \n\t"                      /* 2 */
		
		/* UDR0_REGISTER = tx0_buffer[tmp_tx_first_byte] */
	#ifdef USART0_IN_IO_ADDRESS_SPACE
		"out   %M[UDR_reg_IO], r25 \n\t"          /* 1 */
	#else
		"sts   %M[UDR_reg], r25 \n\t"          /* 2 */
	#endif
		"sts   (tx0_first_byte), r24 \n\t"       /* 2 */
		
	"USART0_TX_EXIT_%=: "
		"pop   r31 \n\t"                         /* 2 */
		"pop   r30 \n\t"                         /* 2 */
		"pop   r25 \n\t"                         /* 2 */
		"pop   r24 \n\t"                         /* 2 */
		
		"out   __SREG__ , r0 \n\t"               /* 1 */
		"pop   r0 \n\t"                          /* 2 */

		"reti \n\t"                              /* 4 ISR return */ // 49 cycles total in worst case
		
		/* UCSR0B_REGISTER &= ~(1<<UDRIE0_BIT) */
	"USART0_DISABLE_UDRIE_%=: "
	#ifdef USART0_IN_IO_ADDRESS_SPACE
	
	#ifdef USART0_UCSRB_NOT_ACCESIBLE_FROM_CBI
		"in   r24, %M[control_reg] \n\t"        /* 1 */
		"andi  r24, 0xDF \n\t"                   /* 1 */
		"out   %M[control_reg], r24\n\t"         /* 1 */
	#else // cbi
		"cbi   %M[control_reg_IO], %M[udrie_bit] \n\t"    /* 2 */
	#endif // to cbi or not to cbi
	
	#else
		"lds   r24, %M[control_reg] \n\t"        /* 2 */
		"andi  r24, 0xDF \n\t"                   /* 1 */
		"sts   %M[control_reg], r24\n\t"         /* 2 */
	#endif
		"rjmp   USART0_TX_EXIT_%= \n\t"          /* 2 */
		
		: /* output operands */
		
		: /* input operands */
		[UDR_reg_IO]   "M"    (_SFR_IO_ADDR(UDR0_REGISTER)),
		[UDR_reg]   "M"    (_SFR_MEM_ADDR(UDR0_REGISTER)),
		[control_reg_IO] "M"    (_SFR_IO_ADDR(UCSR0B_REGISTER)),
		[control_reg] "M"    (_SFR_MEM_ADDR(UCSR0B_REGISTER)),
		//[udrie_mask_disable] "M" ( ~(1<<UDRIE0_BIT) ), //no way
		[udrie_bit] "M"		(UDRIE0_BIT),
		[mask]        "M"    (TX0_BUFFER_MASK)
		
		/* no clobbers */
		);
	}	
	
#ifdef USART0_RS485_MODE
	
	ISR(TXC0_INTERRUPT, ISR_NAKED) // ISR is compiled to only one cbi instruction - no need for large prologue/epilogue
	{
		___PORT(RS485_CONTROL0_PORT) &= ~(1<<RS485_CONTROL0_PIN); // set low after completed transaction
		reti();
	}
	
#endif // USART0_RS485_MODE
	
#endif // NO_TX0_INTERRUPT

#ifndef NO_RX0_INTERRUPT

	ISR(RX0_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"                      /* 4 ISR entry */
	
		"push  r0 \n\t"                          /* 2 */
		"in    r0, __SREG__ \n\t"                /* 1 */
		
		"push  r18 \n\t"                         /* 2 */
		"push  r24 \n\t"                         /* 2 */
		"push  r25 \n\t"                         /* 2 */
		"push  r30 \n\t"                         /* 2 */
		"push  r31 \n\t"                         /* 2 */
		
		/* read byte from UDR register */
	#ifdef USART0_IN_IO_ADDRESS_SPACE
		"in   r25, %M[UDR_reg_IO] \n\t"          /* 1 */
	#else
		"lds   r25, %M[UDR_reg] \n\t"            /* 2 */
	#endif

	#ifdef USART_UNSAFE_RX_INTERRUPT // enable interrupt after satisfying UDR register
		"sei \n\t"                               /* 1 */
	#endif
		/* load globals */
		"lds   r24, (rx0_last_byte) \n\t"        /* 2 */
		"lds   r18, (rx0_first_byte) \n\t"       /* 2 */
		
		/* tmp_rx_last_byte = rx0_last_byte + 1 */
		"subi  r24, 0xFF \n\t"                   /* 1 */
		
		/* tmp_rx_last_byte &= RX0_BUFFER_MASK */
	#if (RX0_BUFFER_MASK != 0xff)
		"andi  r24, %M[mask]\n\t"                /* 1 */
	#endif
		
		/* if(rx0_first_byte != tmp_rx_last_byte) */
		"cp    r18, r24 \n\t"                    /* 1 */
		"breq  USART0_RX_EXIT_%= \n\t"                        /* 1/2 */
		
		/* rx0_buffer[tmp_rx_last_byte] = tmp */
		"mov   r30, r24 \n\t"                    /* 1 */
		"ldi   r31, 0x00 \n\t"                   /* 1 */
		"subi  r30, lo8(-(rx0_buffer))\n\t"      /* 1 */
		"sbci  r31, hi8(-(rx0_buffer))\n\t"      /* 1 */
		"st    Z, r25 \n\t"                      /* 2 */
		
		/* rx0_last_byte = tmp_rx_last_byte */
		"sts   (rx0_last_byte), r24 \n\t"        /* 2 */

	"USART0_RX_EXIT_%=: "
	#ifdef USART_UNSAFE_RX_INTERRUPT
		"cli \n\t"                               /* 1 */
	#endif
		"pop   r31 \n\t"                         /* 2 */
		"pop   r30 \n\t"                         /* 2 */
		"pop   r25 \n\t"                         /* 2 */
		"pop   r24 \n\t"                         /* 2 */
		"pop   r18 \n\t"                         /* 2 */
		
		"out   __SREG__ , r0 \n\t"               /* 1 */
		"pop   r0 \n\t"                          /* 2 */

		"reti \n\t"                              /* 4 ISR return */ // 52 cycles total in worst case
		
		: /* output operands */
		
		: /* input operands */
		[UDR_reg_IO]   "M"    (_SFR_IO_ADDR(UDR0_REGISTER)),
		[UDR_reg]	"M"    (_SFR_MEM_ADDR(UDR0_REGISTER)),
		[mask]      "M"    (RX0_BUFFER_MASK)
		
		/* no clobbers */
		);
		
	}

#endif // NO_RX0_INTERRUPT

#ifndef NO_TX1_INTERRUPT

	ISR(UDRE1_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"                      /* 4 ISR entry */
		
		"push  r0 \n\t"                          /* 2 */
		"in    r0, __SREG__ \n\t"                /* 1 */

		"push  r24 \n\t"                         /* 2 */
		"push  r25 \n\t"                         /* 2 */
		"push  r30 \n\t"                         /* 2 */
		"push  r31 \n\t"                         /* 2 */
		
		/* load globals */
		"lds   r24, (tx1_first_byte) \n\t"       /* 2 */
		"lds   r25, (tx1_last_byte) \n\t"        /* 2 */
		
		/* if(tmp_tx_first_byte != tx1_last_byte) */
		"cp    r24, r25 \n\t"                    /* 1 */
		"breq  USART1_DISABLE_UDRIE_%= \n\t"                        /* 1/2 */
		
		/* tmp_tx_first_byte = tmp_tx_first_byte + 1 */
		"subi  r24, 0xFF \n\t"                   /* 1 */
				
		/* tmp_tx_first_byte &= TX1_BUFFER_MASK */
	#if (TX1_BUFFER_MASK != 0xff)
		"andi  r24, %M[mask]\n\t"                /* 1 */
	#endif
		
		/* tx1_buffer[tmp_tx_first_byte] */
		"mov   r30, r24 \n\t"                    /* 1 */
		"ldi   r31, 0x00 \n\t"                   /* 1 */
		"subi  r30, lo8(-(tx1_buffer)) \n\t"     /* 1 */
		"sbci  r31, hi8(-(tx1_buffer)) \n\t"     /* 1 */
		"ld    r25, Z \n\t"                      /* 2 */
		
		/* UDR1_REGISTER = tx1_buffer[tmp_tx_first_byte] */
		#ifdef USART0_IN_IO_ADDRESS_SPACE
		"out   %M[UDR_reg_IO], r25 \n\t"          /* 1 */
	#else
		"sts   %M[UDR_reg], r25 \n\t"          /* 2 */
	#endif
		"sts   (tx1_first_byte), r24 \n\t"       /* 2 */
		
	"USART1_TX_EXIT_%=: "
		"pop   r31 \n\t"                         /* 2 */
		"pop   r30 \n\t"                         /* 2 */
		"pop   r25 \n\t"                         /* 2 */
		"pop   r24 \n\t"                         /* 2 */
		
		"out   __SREG__ , r0 \n\t"               /* 1 */
		"pop   r0 \n\t"                          /* 2 */

		"reti \n\t"                              /* 4 ISR return */
		
		/* UCSR1B_REGISTER &= ~(1<<UDRIE1_BIT) */
	"USART1_DISABLE_UDRIE_%=: "
	#ifdef USART0_IN_IO_ADDRESS_SPACE
		"cbi   %M[control_reg_IO], %M[udrie_bit] \n\t"    /* 2 */
	#else
		"lds   r24, %M[control_reg] \n\t"        /* 2 */
		"andi  r24, 0xDF \n\t"                   /* 1 */
		"sts   %M[control_reg], r24\n\t"         /* 2 */
	#endif
		"rjmp   USART1_TX_EXIT_%= \n\t"          /* 2 */
		
		: /* output operands */
		
		: /* input operands */
		[UDR_reg_IO]   "M"    (_SFR_IO_ADDR(UDR1_REGISTER)),
		[UDR_reg]   "M"    (_SFR_MEM_ADDR(UDR1_REGISTER)),
		[control_reg_IO] "M"    (_SFR_IO_ADDR(UCSR1B_REGISTER)),
		[control_reg] "M"    (_SFR_MEM_ADDR(UCSR1B_REGISTER)),
		//[udrie_mask_disable] "M" ( ~(1<<UDRIE1_BIT) ), //no way
		[udrie_bit] "M"		(UDRIE1_BIT),
		[mask]        "M"    (TX1_BUFFER_MASK)
		
		/* no clobbers */
		);
		
	}

#ifdef USART1_RS485_MODE

	ISR(TXC1_INTERRUPT, ISR_NAKED) // ISR is compiled to only one cbi instruction - no need for large prologue/epilogue
	{
		___PORT(RS485_CONTROL1_PORT) &= ~(1<<RS485_CONTROL1_PIN); // set low after completed transaction
		reti();
	}

#endif // USART1_RS485_MODE

#endif // NO_TX1_INTERRUPT

#ifndef NO_RX1_INTERRUPT
	
	ISR(RX1_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"                      /* 4 ISR entry */
	
		"push  r0 \n\t"                         /* 2 */
		"in    r0, __SREG__ \n\t"               /* 1 */
	
		"push  r18 \n\t"                         /* 2 */
		"push  r24 \n\t"                         /* 2 */
		"push  r25 \n\t"                         /* 2 */
		"push  r30 \n\t"                         /* 2 */
		"push  r31 \n\t"                         /* 2 */

		/* read byte from UDR register */
	#ifdef USART1_IN_IO_ADDRESS_SPACE
		"in   r25, %M[UDR_reg_IO] \n\t"          /* 1 */
	#else
		"lds   r25, %M[UDR_reg] \n\t"            /* 2 */
	#endif
		
	#ifdef USART_UNSAFE_RX_INTERRUPT // enable interrupt after satisfying UDR register
		"sei \n\t"                               /* 1 */
	#endif
		/* load globals */
		"lds   r24, (rx1_last_byte) \n\t"        /* 2 */
		"lds   r18, (rx1_first_byte) \n\t"       /* 2 */
		
		/* tmp_rx_last_byte = rx1_last_byte + 1 */
		"subi  r24, 0xFF \n\t"                   /* 1 */
				
		/* tmp_rx_last_byte &= RX1_BUFFER_MASK */
	#if (RX1_BUFFER_MASK != 0xff)
		"andi  r24, %M[mask]\n\t"                /* 1 */
	#endif
		
		/* if(rx1_first_byte != tmp_rx_last_byte) */
		"cp    r18, r24 \n\t"                    /* 1 */
		"breq  USART1_RX_EXIT_%= \n\t"                        /* 1/2 */
		
		/* rx1_buffer[tmp_rx_last_byte] = tmp */
		"mov   r30, r24 \n\t"                    /* 1 */
		"ldi   r31, 0x00 \n\t"                   /* 1 */
		"subi  r30, lo8(-(rx1_buffer))\n\t"      /* 1 */
		"sbci  r31, hi8(-(rx1_buffer))\n\t"      /* 1 */
		"st    Z, r25 \n\t"                      /* 2 */
		
		/* rx1_last_byte = tmp_rx_last_byte */
		"sts   (rx1_last_byte), r24 \n\t"        /* 2 */

	"USART1_RX_EXIT_%=: "
	#ifdef USART_UNSAFE_RX_INTERRUPT
		"cli \n\t"                               /* 1 */
	#endif
		"pop   r31 \n\t"                         /* 2 */
		"pop   r30 \n\t"                         /* 2 */
		"pop   r25 \n\t"                         /* 2 */
		"pop   r24 \n\t"                         /* 2 */
		"pop   r18 \n\t"                         /* 2 */

		"out   __SREG__ , r0 \n\t"               /* 1 */
		"pop   r0 \n\t"                          /* 2 */

		"reti \n\t"                              /* 4 ISR return */
		
		: /* output operands */
		
		: /* input operands */
		[UDR_reg_IO]   "M"    (_SFR_IO_ADDR(UDR0_REGISTER)),
		[UDR_reg] "M"    (_SFR_MEM_ADDR(UDR1_REGISTER)),
		[mask]      "M"    (RX1_BUFFER_MASK)
		
		/* no clobbers */
		);
		
	}

#endif // NO_RX1_INTERRUPT

#ifndef NO_TX2_INTERRUPT

	ISR(UDRE2_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"                      /* 4 ISR entry */
	
		"push  r0 \n\t"                         /* 2 */
		"in    r0, __SREG__ \n\t"               /* 1 */
	
		"push  r24 \n\t"                         /* 2 */
		"push  r25 \n\t"                         /* 2 */
		"push  r30 \n\t"                         /* 2 */
		"push  r31 \n\t"                         /* 2 */
	
		/* load globals */
		"lds   r24, (tx2_first_byte) \n\t"       /* 2 */
		"lds   r25, (tx2_last_byte) \n\t"        /* 2 */
		
		/* if(tmp_tx_first_byte != tx2_last_byte) */
		"cp    r24, r25 \n\t"                    /* 1 */
		"breq  USART2_DISABLE_UDRIE_%= \n\t"                        /* 1/2 */
		
		/* tmp_tx_first_byte = tmp_tx_first_byte + 1 */
		"subi  r24, 0xFF \n\t"                   /* 1 */
		
		/* tmp_tx_first_byte &= TX2_BUFFER_MASK */
	#if (TX2_BUFFER_MASK != 0xff)
		"andi  r24, %M[mask]\n\t"                /* 1 */
	#endif
		
		/* tx2_buffer[tmp_tx_first_byte] */
		"mov   r30, r24 \n\t"                    /* 1 */
		"ldi   r31, 0x00 \n\t"                   /* 1 */
		"subi  r30, lo8(-(tx2_buffer)) \n\t"     /* 1 */
		"sbci  r31, hi8(-(tx2_buffer)) \n\t"     /* 1 */
		"ld    r25, Z \n\t"                      /* 2 */
		
		/* UDR2_REGISTER = tx2_buffer[tmp_tx_first_byte] */
		"sts   %M[UDR_reg], r25 \n\t"          /* 2 */
		"sts   (tx2_first_byte), r24 \n\t"       /* 2 */

	"USART2_TX_EXIT_%=: "
		"pop   r31 \n\t"                         /* 2 */
		"pop   r30 \n\t"                         /* 2 */
		"pop   r25 \n\t"                         /* 2 */
		"pop   r24 \n\t"                         /* 2 */
		
		"out   __SREG__ , r0 \n\t"               /* 1 */
		"pop   r0 \n\t"                          /* 2 */

		"reti \n\t"                              /* 4 ISR return */
		
		/* UCSR2B_REGISTER &= ~(1<<UDRIE2_BIT) */
	"USART2_DISABLE_UDRIE_%=: "
		
		"lds   r24, %M[control_reg] \n\t"        /* 2 */
		"andi  r24, 0xDF \n\t"                   /* 1 */
		"sts   %M[control_reg], r24\n\t"         /* 2 */
				
		"rjmp   USART2_TX_EXIT_%= \n\t"          /* 2 */
		
		: /* output operands */
		
		: /* input operands */
		[UDR_reg]   "M"    (_SFR_MEM_ADDR(UDR2_REGISTER)),
		[control_reg] "M"    (_SFR_MEM_ADDR(UCSR2B_REGISTER)),
		//[udrie_mask_disable] "M" ( ~(1<<UDRIE2_BIT) ), //no way
		[mask]        "M"    (TX2_BUFFER_MASK)
		
		/* no clobbers */
		);
		
	}

#ifdef USART2_RS485_MODE

	ISR(TXC2_INTERRUPT, ISR_NAKED) // ISR is compiled to only one cbi instruction - no need for large prologue/epilogue
	{
		___PORT(RS485_CONTROL2_PORT) &= ~(1<<RS485_CONTROL2_PIN); // set low after completed transaction
		reti();
	}

#endif // USART2_RS485_MODE

#endif // NO_TX2_INTERRUPT

#ifndef NO_RX2_INTERRUPT

	ISR(RX2_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"                      /* 4 ISR entry */
		
		"push  r0 \n\t"                         /* 2 */
		"in    r0, __SREG__ \n\t"               /* 1 */

		"push  r18 \n\t"                         /* 2 */
		"push  r24 \n\t"                         /* 2 */
		"push  r25 \n\t"                         /* 2 */
		"push  r30 \n\t"                         /* 2 */
		"push  r31 \n\t"                         /* 2 */

		/* read byte from UDR register */
		"lds   r25, %M[UDR_reg] \n\t"          /* 2 */
		
	#ifdef USART_UNSAFE_RX_INTERRUPT // enable interrupt after satisfying UDR register
		"sei \n\t"                               /* 1 */
	#endif
		/* load globals */
		"lds   r24, (rx2_last_byte) \n\t"        /* 2 */
		"lds   r18, (rx2_first_byte) \n\t"       /* 2 */
		
		/* tmp_rx_last_byte = rx2_last_byte + 1 */
		"subi  r24, 0xFF \n\t"                   /* 1 */
		
		/* tmp_rx_last_byte &= RX2_BUFFER_MASK */
	#if (RX2_BUFFER_MASK != 0xff)
		"andi  r24, %M[mask]\n\t"                /* 1 */
	#endif
		
		/* if(rx2_first_byte != tmp_rx_last_byte) */
		"cp    r18, r24 \n\t"                    /* 1 */
		"breq  USART2_RX_EXIT_%= \n\t"                        /* 1/2 */
		
		/* rx2_buffer[tmp_rx_last_byte] = tmp */
		"mov   r30, r24 \n\t"                    /* 1 */
		"ldi   r31, 0x00 \n\t"                   /* 1 */
		"subi  r30, lo8(-(rx2_buffer))\n\t"      /* 1 */
		"sbci  r31, hi8(-(rx2_buffer))\n\t"      /* 1 */
		"st    Z, r25 \n\t"                      /* 2 */
		
		/* rx2_last_byte = tmp_rx_last_byte */
		"sts   (rx2_last_byte), r24 \n\t"        /* 2 */

	"USART2_RX_EXIT_%=: "
	#ifdef USART_UNSAFE_RX_INTERRUPT
		"cli \n\t"                               /* 1 */
	#endif
		"pop   r31 \n\t"                         /* 2 */
		"pop   r30 \n\t"                         /* 2 */
		"pop   r25 \n\t"                         /* 2 */
		"pop   r24 \n\t"                         /* 2 */
		"pop   r18 \n\t"                         /* 2 */
	
		"out   __SREG__ , r0 \n\t"               /* 1 */
		"pop   r0 \n\t"                          /* 2 */

		"reti \n\t"                              /* 4 ISR return */
		
		: /* output operands */
		
		: /* input operands */
		[UDR_reg] "M"    (_SFR_MEM_ADDR(UDR2_REGISTER)),
		[mask]      "M"    (RX2_BUFFER_MASK)
		
		/* no clobbers */
		);
		
	}
	
#endif // NO_RX2_INTERRUPT

#ifndef NO_TX3_INTERRUPT

	ISR(UDRE3_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"                      /* 4 ISR entry */
	
		"push  r0 \n\t"                          /* 2 */
		"in    r0, __SREG__ \n\t"                /* 1 */
	
		"push  r24 \n\t"                         /* 2 */
		"push  r25 \n\t"                         /* 2 */
		"push  r30 \n\t"                         /* 2 */
		"push  r31 \n\t"                         /* 2 */
		
		/* load globals */
		"lds   r24, (tx3_first_byte) \n\t"       /* 2 */
		"lds   r25, (tx3_last_byte) \n\t"        /* 2 */
		
		/* if(tmp_tx_first_byte != tx3_last_byte) */
		"cp    r24, r25 \n\t"                    /* 1 */
		"breq  USART3_DISABLE_UDRIE_%= \n\t"                        /* 1/2 */
		
		/* tmp_tx_first_byte = tmp_tx_first_byte + 1 */
		"subi  r24, 0xFF \n\t"                   /* 1 */
		
		/* tmp_tx_first_byte &= TX3_BUFFER_MASK */
	#if (TX3_BUFFER_MASK != 0xff)
		"andi  r24, %M[mask]\n\t"                /* 1 */
	#endif
		
		/* tx3_buffer[tmp_tx_first_byte] */
		"mov   r30, r24 \n\t"                    /* 1 */
		"ldi   r31, 0x00 \n\t"                   /* 1 */
		"subi  r30, lo8(-(tx3_buffer)) \n\t"     /* 1 */
		"sbci  r31, hi8(-(tx3_buffer)) \n\t"     /* 1 */
		"ld    r25, Z \n\t"                      /* 2 */
		
		/* UDR3_REGISTER = tx3_buffer[tmp_tx_first_byte] */
		"sts   %M[UDR_reg], r25 \n\t"          /* 2 */
		"sts   (tx3_first_byte), r24 \n\t"       /* 2 */
		
	"USART3_TX_EXIT_%=: "
		"pop   r31 \n\t"                         /* 2 */
		"pop   r30 \n\t"                         /* 2 */
		"pop   r25 \n\t"                         /* 2 */
		"pop   r24 \n\t"                         /* 2 */
	
		"out   __SREG__ , r0 \n\t"               /* 1 */
		"pop   r0 \n\t"                          /* 2 */

		"reti \n\t"                              /* 4 ISR return */
		
		/* UCSR3B_REGISTER &= ~(1<<UDRIE3_BIT) */
	"USART3_DISABLE_UDRIE_%=: "
		
		"lds   r24, %M[control_reg] \n\t"        /* 2 */
		"andi  r24, 0xDF \n\t"                   /* 1 */
		"sts   %M[control_reg], r24\n\t"         /* 2 */
		
		"rjmp   USART3_TX_EXIT_%= \n\t"          /* 2 */
		
		: /* output operands */
		
		: /* input operands */
		[UDR_reg]   "M"    (_SFR_MEM_ADDR(UDR3_REGISTER)),
		[control_reg] "M"    (_SFR_MEM_ADDR(UCSR3B_REGISTER)),
		//[udrie_mask_disable] "M" ( ~(1<<UDRIE3_BIT) ), //no way
		[mask]        "M"    (TX3_BUFFER_MASK)
		
		/* no clobbers */
		);
		
	}
	
#ifdef USART3_RS485_MODE

	ISR(TXC3_INTERRUPT, ISR_NAKED) // ISR is compiled to only one cbi instruction - no need for large prologue/epilogue
	{
		___PORT(RS485_CONTROL3_PORT) &= ~(1<<RS485_CONTROL3_PIN); // set low after completed transaction
		reti();
	}

#endif // USART0_RS485_MODE	
	
#endif // NO_TX3_INTERRUPT

#ifndef NO_RX3_INTERRUPT

	ISR(RX3_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"                      /* 4 ISR entry */
		
		"push  r0 \n\t"                          /* 2 */
		"in    r0, __SREG__ \n\t"                /* 1 */
	
		"push  r18 \n\t"                         /* 2 */
		"push  r24 \n\t"                         /* 2 */
		"push  r25 \n\t"                         /* 2 */
		"push  r30 \n\t"                         /* 2 */
		"push  r31 \n\t"                         /* 2 */

		/* read byte from UDR register */
		"lds   r25, %M[UDR_reg] \n\t"          /* 2 */
		
	#ifdef USART_UNSAFE_RX_INTERRUPT // enable interrupt after satisfying UDR register
		"sei \n\t"                               /* 1 */
	#endif
		/* load globals */
		"lds   r24, (rx3_last_byte) \n\t"        /* 2 */
		"lds   r18, (rx3_first_byte) \n\t"       /* 2 */
		
		/* tmp_rx_last_byte = rx3_last_byte + 1 */
		"subi  r24, 0xFF \n\t"                   /* 1 */
		
		/* tmp_rx_last_byte &= RX3_BUFFER_MASK */
	#if (RX3_BUFFER_MASK != 0xff)
		"andi  r24, %M[mask]\n\t"                /* 1 */
	#endif
		
		/* if(rx3_first_byte != tmp_rx_last_byte) */
		"cp    r18, r24 \n\t"                    /* 1 */
		"breq  USART3_RX_EXIT_%= \n\t"                        /* 1/2 */
		
		/* rx3_buffer[tmp_rx_last_byte] = tmp */
		"mov   r30, r24 \n\t"                    /* 1 */
		"ldi   r31, 0x00 \n\t"                   /* 1 */
		"subi  r30, lo8(-(rx3_buffer))\n\t"      /* 1 */
		"sbci  r31, hi8(-(rx3_buffer))\n\t"      /* 1 */
		"st    Z, r25 \n\t"                      /* 2 */
		
		/* rx3_last_byte = tmp_rx_last_byte */
		"sts   (rx3_last_byte), r24 \n\t"        /* 2 */
		
	"USART3_RX_EXIT_%=: "
	#ifdef USART_UNSAFE_RX_INTERRUPT
		"cli \n\t"                               /* 1 */
	#endif
		"pop   r31 \n\t"                         /* 2 */
		"pop   r30 \n\t"                         /* 2 */
		"pop   r25 \n\t"                         /* 2 */
		"pop   r24 \n\t"                         /* 2 */
		"pop   r18 \n\t"                         /* 2 */
	
		"out   __SREG__ , r0 \n\t"               /* 1 */
		"pop   r0 \n\t"                          /* 2 */

		"reti \n\t"                              /* 4 ISR return */
		
		: /* output operands */
		
		: /* input operands */
		[UDR_reg] "M"    (_SFR_MEM_ADDR(UDR3_REGISTER)),
		[mask]      "M"    (RX3_BUFFER_MASK)
		
		/* no clobbers */
		);
		
	}

#endif // NO_RX3_INTERRUPT