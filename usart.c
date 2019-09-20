//**************************************************************
// ****** FUNCTIONS FOR AVR USART COMMUNICATION *******
//**************************************************************
//Compiler          : AVR-GCC (c99/gnu99)
//Author            : jnk0le@hotmail.com
//                    https://github.com/jnk0le
//License           : MIT
//**************************************************************

#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>

#include "usart.h"

#ifndef NO_TX0_INTERRUPT
	volatile uint8_t tx0_Tail, tx0_Head;
	char tx0_buffer[TX0_BUFFER_SIZE] __attribute__ ((used));
#endif

#ifndef NO_RX0_INTERRUPT
	volatile uint8_t rx0_Tail, rx0_Head;
	char rx0_buffer[RX0_BUFFER_SIZE] __attribute__ ((used));
#endif

#ifndef NO_TX1_INTERRUPT
	volatile uint8_t tx1_Tail, tx1_Head;
	char tx1_buffer[TX1_BUFFER_SIZE] __attribute__ ((used));
#endif

#ifndef NO_RX1_INTERRUPT
	volatile uint8_t rx1_Tail, rx1_Head;
	char rx1_buffer[RX1_BUFFER_SIZE] __attribute__ ((used));
#endif

#ifndef NO_TX2_INTERRUPT
	volatile uint8_t tx2_Tail, tx2_Head;
	char tx2_buffer[TX2_BUFFER_SIZE] __attribute__ ((used));
#endif

#ifndef NO_RX2_INTERRUPT
	volatile uint8_t rx2_Tail, rx2_Head;
	char rx2_buffer[RX2_BUFFER_SIZE] __attribute__ ((used));
#endif

#ifndef NO_TX3_INTERRUPT
	volatile uint8_t tx3_Tail, tx3_Head;
	char tx3_buffer[TX3_BUFFER_SIZE] __attribute__ ((used));
#endif

#ifndef NO_RX3_INTERRUPT
	volatile uint8_t rx3_Tail, rx3_Head;
	char rx3_buffer[RX3_BUFFER_SIZE] __attribute__ ((used));
#endif

#ifdef USART_NO_LOCAL_BUFFERS
	extern char u_tmp_buff[];
#endif

#ifdef USE_USART0
/*
	void uart_init(uint16_t ubrr_value)
	{
	#ifdef USART0_RS485_MODE
		RS485_CONTROL0_DDR |= (1<<RS485_CONTROL0_IONUM); // default pin state is low
	#endif
		
		UBRR0L_REGISTER = (uint8_t) ubrr_value;
		UBRR0H_REGISTER = (ubrr_value>>8);
		
	#ifdef USART0_U2X_SPEED
		#ifdef USART0_MPCM_MODE
			UCSR0A_REGISTER = (1<<U2X0_BIT)|(1<<MPCM0_BIT);
		#else
			UCSR0A_REGISTER = (1<<U2X0_BIT); // enable double speed
		#endif
	#elif defined(USART0_MPCM_MODE)
		UCSR0A_REGISTER |= (1<<MPCM0_BIT);
	#endif
		
		UCSR0B_REGISTER = USART0_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
		
	#ifdef USART0_USE_SOFT_RTS
		RTS0_DDR |= (1<<RTS0_IONUM);
	#endif
	}
*/

//******************************************************************
//Function  : To reinitialize USART interface (runtime speed changing).
//Arguments : Calculated UBRR value to initialize equal speed.
//Return    : none
//Note      : Use BAUD_CALC(speed) macro to calculate UBRR value.
//          : All data inside UDR shift register will be lost.
//          : U2X bit is cleared if USARTn_U2X_SPEED is not defined.
//******************************************************************
	void uart0_reinit(uint16_t ubrr_value)
	{
	#ifdef USART0_USE_SOFT_RTS
		RTS0_PORT |= (1<<RTS0_IONUM);
	#endif
		
	#ifdef USART0_RS485_MODE
		RS485_CONTROL0_PORT &= ~(1<<RS485_CONTROL0_IONUM); //set low
		RS485_CONTROL0_DDR |= (1<<RS485_CONTROL0_IONUM);
	#endif
		
		UCSR0B_REGISTER = 0; //flush all hardware buffers
		
		//(writing TXENn to zero) will not become effective until ongoing and pending transmissions are completed
		
		UBRR0L_REGISTER = (uint8_t) ubrr_value;
		UBRR0H_REGISTER = (ubrr_value>>8);

	#ifdef USART0_U2X_SPEED
		#ifdef USART0_MPCM_MODE
			UCSR0A_REGISTER = (1<<U2X0_BIT)|(1<<MPCM0_BIT);
		#else
			UCSR0A_REGISTER = (1<<U2X0_BIT); // enable double speed
		#endif
	#elif defined(USART0_MPCM_MODE)
		UCSR0A_REGISTER = (1<<MPCM0_BIT);
	#endif
	
		UCSR0B_REGISTER = USART0_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
		
	#ifdef USART0_USE_SOFT_RTS
		RTS0_DDR |= (1<<RTS0_IONUM);
		RTS0_PORT &= ~(1<<RTS0_IONUM);
	#endif
	}
#endif // USE_USART0

#ifdef USE_USART1
	void uart1_reinit(uint16_t ubrr_value)
	{
	#ifdef USART1_USE_SOFT_RTS
		RTS1_PORT |= (1<<RTS1_IONUM);
	#endif
	
	#ifdef USART1_RS485_MODE
		RS485_CONTROL1_PORT &= ~(1<<RS485_CONTROL1_IONUM); //set low
		RS485_CONTROL1_DDR |= (1<<RS485_CONTROL1_IONUM);
	#endif
		
		UCSR1B_REGISTER = 0; //flush all hardware buffers
		
		//(writing TXENn to zero) will not become effective until ongoing and pending transmissions are completed
		
		UBRR1L_REGISTER = (uint8_t) ubrr_value;
		UBRR1H_REGISTER = (ubrr_value>>8);

	#ifdef USART1_U2X_SPEED
		#ifdef USART1_MPCM_MODE
			UCSR1A_REGISTER = (1<<U2X1_BIT)|(1<<MPCM1_BIT);
		#else
			UCSR1A_REGISTER = (1<<U2X1_BIT); // enable double speed
		#endif
	#elif defined(USART1_MPCM_MODE)
		UCSR1A_REGISTER = (1<<MPCM1_BIT);
	#endif
	
		UCSR1B_REGISTER = USART1_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
		
	#ifdef USART1_USE_SOFT_RTS
		RTS1_DDR |= (1<<RTS1_IONUM);
		RTS1_PORT &= ~(1<<RTS1_IONUM);
	#endif
	}
#endif // USE_USART1

#ifdef USE_USART2
	void uart2_reinit(uint16_t ubrr_value)
	{
	#ifdef USART2_USE_SOFT_RTS
		RTS2_PORT |= (1<<RTS2_IONUM);
	#endif
	
	#ifdef USART2_RS485_MODE
		RS485_CONTROL2_PORT &= ~(1<<RS485_CONTROL2_IONUM); //set low
		RS485_CONTROL2_DDR |= (1<<RS485_CONTROL2_IONUM);
	#endif
		
		UCSR2B_REGISTER = 0; //flush all hardware buffers
		
		//(writing TXENn to zero) will not become effective until ongoing and pending transmissions are completed

		UBRR2L_REGISTER = (uint8_t) ubrr_value;
		UBRR2H_REGISTER = (ubrr_value>>8);

	#ifdef USART2_U2X_SPEED
		#ifdef USART2_MPCM_MODE
			UCSR2A_REGISTER = (1<<U2X2_BIT)|(1<<MPCM2_BIT);
		#else
			UCSR2A_REGISTER = (1<<U2X2_BIT); // enable double speed
		#endif
	#elif defined(USART2_MPCM_MODE)
		UCSR2A_REGISTER = (1<<MPCM2_BIT);
	#endif
	
		UCSR2B_REGISTER = USART2_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
		
	#ifdef USART2_USE_SOFT_RTS
		RTS2_DDR |= (1<<RTS2_IONUM);
		RTS2_PORT &= ~(1<<RTS2_IONUM);
	#endif
	}
#endif // USE_USART2

#ifdef USE_USART3
	void uart3_reinit(uint16_t ubrr_value)
	{
	#ifdef USART3_USE_SOFT_RTS
		RTS3_PORT |= (1<<RTS3_IONUM);
	#endif
	
	#ifdef USART3_RS485_MODE
		RS485_CONTROL3_PORT &= ~(1<<RS485_CONTROL3_IONUM); //set low
		RS485_CONTROL3_DDR |= (1<<RS485_CONTROL3_IONUM);
	#endif
		
		UCSR3B_REGISTER = 0; //flush all hardware buffers
		
		UBRR3L_REGISTER = (uint8_t) ubrr_value;
		UBRR3H_REGISTER = (ubrr_value>>8);

	#ifdef USART3_U2X_SPEED
		#ifdef USART3_MPCM_MODE
			UCSR3A_REGISTER = (1<<U2X3_BIT)|(1<<MPCM3_BIT);
		#else
			UCSR3A_REGISTER = (1<<U2X3_BIT); // enable double speed
		#endif
	#elif defined(USART3_MPCM_MODE)
		UCSR3A_REGISTER = (1<<MPCM3_BIT);
	#endif
	
		UCSR3B_REGISTER = USART3_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
		
	#ifdef USART3_USE_SOFT_RTS
		RTS3_DDR |= (1<<RTS3_IONUM);
		RTS3_PORT &= ~(1<<RTS3_IONUM);
	#endif
	}
#endif // USE_USART3

#ifndef NO_USART_TX

#ifndef NO_TX0_INTERRUPT
//******************************************************************
//Function  : Send single character/byte.
//Arguments : Character/byte to send.
//Return    : none
//******************************************************************
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart0_putc(char data)
	{
	#ifdef PUTC0_CONVERT_LF_TO_CRLF
		if (data == '\n')
			uart0_putc('\r');
	#endif
		
	#ifdef USART0_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx0_Head;
		register uint8_t tmp_tx_Tail = tx0_Tail;
		
	#ifdef USART0_USE_SOFT_CTS
		if(!(CTS0_PIN & (1<<CTS0_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR0A_REGISTER & UDRE0_BIT))
		{
			UDR0_REGISTER = data;
			return;
		}
		
		tmp_tx_Head = (tmp_tx_Head + 1) & TX0_BUFFER_MASK;
		
		while(tmp_tx_Tail == tmp_tx_Head) // wait for free space in buffer
		{
			tmp_tx_Tail = tx0_Tail; // for faster pass through, results in a little bigger code
		}
	#else
		register uint8_t tmp_tx_Head = (tx0_Head + 1) & TX0_BUFFER_MASK; // calculate new position of TX head in buffer
		
		while(tx0_Tail == tmp_tx_Head); // wait for free space in buffer
	#endif
		
		tx0_buffer[tmp_tx_Head] = data;
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx0_Head = tmp_tx_Head;
			
		#ifdef USART0_RS485_MODE
			RS485_CONTROL0_PORT |= (1<<RS485_CONTROL0_IONUM); //set high
		#endif
			
		#ifdef USART0_USE_SOFT_CTS
			if(!(CTS0_PIN & (1<<CTS0_IONUM)))
		#endif
			{
				UCSR0B_REGISTER |= (1<<UDRIE0_BIT); // enable UDRE interrupt
			}
		}
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart0_putc(char data)
	{
		register uint8_t tmp_tx_Head asm("r25");
		
	#ifdef PUTC0_CONVERT_LF_TO_CRLF
		asm volatile("\n\t"
			"cpi	%[dat], '\n' \n\t"
			"brne	skip_recursive_%=\n\t"
			"push	%[dat] \n\t"
			"ldi	%[dat], '\r' \n\t"
			"rcall	uart0_putc \n\t"
			"pop	%[dat] \n\t"
		"skip_recursive_%=:"
			: // outputs
			[dat]  "+r" (data) // will be used later, do not let the compiler to do anything weird
			: // inputs
			: // clobbers
		);
	#endif
		
	#ifdef USART0_PUTC_FAST_INSERTIONS
		
		asm volatile("\n\t"
			"lds	%[head], (tx0_Head) \n\t"
			
		#ifdef USART0_USE_SOFT_CTS
			"sbic	%M[cts_port], %M[cts_pin] \n\t"
			"rjmp	normal_insert_%= \n\t"  
		#endif
				
			"lds	r27, (tx0_Tail) \n\t"
			"cpse	r27, %[head] \n\t"
			"rjmp	normal_insert_%= \n\t"
				
		#ifdef USART0_IN_IO_ADDRESS_SPACE
			"sbis	%M[UCSRA_reg_IO], %M[udre_bit] \n\t"
		#elif defined(USART0_IN_UPPER_IO_ADDRESS_SPACE)
			"in 	r26, %M[UCSRA_reg_IO] \n\t"
			"sbrs	r26, %M[udre_bit] \n\t"
		#else
			"lds	r26, %M[UCSRA_reg] \n\t"
			"sbrs	r26, %M[udre_bit] \n\t"
		#endif
			"rjmp	normal_insert_%= \n\t"
				
		#ifdef USART0_IN_IO_ADDRESS_SPACE
			"out	%M[UDR_reg], %[dat] \n\t"
		#else
			"sts	%M[UDR_reg], %[dat] \n\t"
		#endif
			"ret	\n\t"
				
		"normal_insert_%=:"
			"inc	%[head] \n\t"
			
		#if (TX0_BUFFER_MASK != 0xff)
			"andi	%[head], %M[mask] \n\t"
		#endif
				
		"waitforspace_%=:"
			"lds	r27, (tx0_Tail) \n\t"
			"cp		r27, %[head] \n\t"
			"breq	waitforspace_%= \n\t"
				
			: // outputs
			[head] "=r" (tmp_tx_Head),
			[dat]  "+r" (data) // will be used later, do not let the compiler to do anything weird
			: // inputs
		#ifdef USART0_USE_SOFT_CTS
			[cts_port]      "M" (_SFR_IO_ADDR(CTS0_PORT)),
			[cts_pin]       "M" (CTS0_IONUM),
		#endif
			[mask]          "M" (TX0_BUFFER_MASK),
			[UCSRA_reg]     "n" (_SFR_MEM_ADDR(UCSR0A_REGISTER)),
			[UCSRA_reg_IO]  "M" (_SFR_IO_ADDR(UCSR0A_REGISTER)),
			[UDR_reg]	    "n" (_SFR_MEM_ADDR(UDR0_REGISTER)),
			[UDR_reg_IO]    "M" (_SFR_IO_ADDR(UDR0_REGISTER)),
			[udre_bit]      "M"	(UDRE0_BIT)
			: // clobbers
			"r26","r27"
		);
	#else
		asm volatile("\n\t"
			"lds	%[head], (tx0_Head) \n\t"
			"inc	%[head] \n\t"
			
		#if (TX0_BUFFER_MASK != 0xff)
			"andi	%[head], %M[mask] \n\t"
		#endif
				
		"waitforspace_%=:"
			"lds	r27, (tx0_Tail) \n\t"
			"cp		r27, %[head] \n\t"
			"breq	waitforspace_%= \n\t"
				
			: // outputs
			[head] "=r" (tmp_tx_Head)
			: // inputs
			[mask] "M" (TX0_BUFFER_MASK)
			: // clobbers
			"r27"
		);
	#endif
		
		asm volatile("\n\t"
			"mov	r26, %[index]  \n\t"
		
		#if !defined(__AVR_ATtiny2313__)&&!defined(__AVR_ATtiny2313A__) // on ATtiny2313 upper byte in pointer pair is ignored
			"ldi	r27, 0x00 \n\t"
		#endif
			"subi	r26, lo8(-(tx0_buffer)) \n\t"
		
		#ifndef USART_USE_TINY_MEMORY_MODEL
			"sbci	r27, hi8(-(tx0_buffer)) \n\t"
		#endif
			"st		X, %[dat] \n\t"
			
			: // outputs
			[index] "+r" (tmp_tx_Head), // will be used later, do not let the compiler to do anything weird
			[dat]   "+r" (data) // not modified, so reduce register moves if inlined
			: // inputs
			: // clobbers
			"r26","r27"
		);
	
		cli();
		{
			tx0_Head = tmp_tx_Head;
		
		#ifdef USART0_RS485_MODE
			RS485_CONTROL0_PORT |= (1<<RS485_CONTROL0_IONUM); // start transmitting
		#endif
		
		#ifdef USART0_USE_SOFT_CTS
			if(!(CTS0_PIN & (1<<CTS0_IONUM)))
		#endif
			{
			#ifdef USART0_IN_IO_ADDRESS_SPACE
				UCSR0B_REGISTER |= (1<<UDRIE0_BIT); // enable UDRE interrupt
			#else
				asm volatile("\n\t"
				#ifdef USART0_IN_UPPER_IO_ADDRESS_SPACE
					"in   r25, %M[control_reg_IO] \n\t"
					"ori  r25, (1<<%M[udrie_bit]) \n\t"
					"out   %M[control_reg_IO], r25\n\t"
				#else 
					"lds   r25, %M[control_reg] \n\t"
					"ori  r25, (1<<%M[udrie_bit]) \n\t"
					"sts   %M[control_reg], r25 \n\t"
				#endif
					: // outputs
					: // inputs
					[control_reg] "n" (_SFR_MEM_ADDR(UCSR0B_REGISTER)),
					[udrie_bit]   "M" (UDRIE0_BIT)
					: // clobbers
					"r25"
				);
			#endif
			}
		}
	
		reti();
		asm volatile("\n\t"::"r" (data):); // data was passed in r24 and will be returned in the same register, make sure it is not affected by the compiler 
	}
	
	char uart0_putc_(char data) __attribute__ ((alias ("uart0_putc"))); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
#endif // USART_NO_ABI_BREAKING_PREMATURES

//******************************************************************
//Function  : Send single character/byte.
//Arguments : Character/byte to send.
//Return    : Status value: 0 = BUFFER_FULL, 1 = COMPLETED.
//Note      : If character cannot be sent due to full transmit buffer, function will abort transmitting character
//******************************************************************
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	uint8_t uart0_putc_noblock(char data)
	{
	#ifdef USART0_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx0_Head;
		register uint8_t tmp_tx_Tail = tx0_Tail;
	
	#ifdef USART0_USE_SOFT_CTS
		if(!(CTS0_PIN & (1<<CTS0_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR0A_REGISTER & UDRE0_BIT))
		{
			UDR0_REGISTER = data;
			return COMPLETED;
		}
	
		tmp_tx_Head = (tmp_tx_Head + 1) & TX0_BUFFER_MASK;
	
		if(tmp_tx_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#else
		register uint8_t tmp_tx_Head = (tx0_Head + 1) & TX0_BUFFER_MASK; // calculate new position of TX head in buffer
	
		if(tx0_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#endif
	
		tx0_buffer[tmp_tx_Head] = data;
	
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx0_Head = tmp_tx_Head;
		
		#ifdef USART0_RS485_MODE
			RS485_CONTROL0_PORT |= (1<<RS485_CONTROL0_IONUM); //set high
		#endif
		
		#ifdef USART0_USE_SOFT_CTS
			if(!(CTS0_PIN & (1<<CTS0_IONUM)))
		#endif
			{
				UCSR0B_REGISTER |= (1<<UDRIE0_BIT); // enable UDRE interrupt
			}
		}
		return COMPLETED;
	}
#else //!USART_NO_ABI_BREAKING_PREMATURES
	uint8_t uart0_putc_noblock(char data)
	{
	#ifdef USART0_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx0_Head;
		register uint8_t tmp_tx_Tail = tx0_Tail;
		
	#ifdef USART0_USE_SOFT_CTS
		if(!(CTS0_PIN & (1<<CTS0_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR0A_REGISTER & UDRE0_BIT))
		{
			UDR0_REGISTER = data;
			return COMPLETED;
		}
		
		tmp_tx_Head = (tmp_tx_Head + 1) & TX0_BUFFER_MASK;
		
		if(tmp_tx_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#else
		register uint8_t tmp_tx_Head = (tx0_Head + 1) & TX0_BUFFER_MASK; // calculate new position of TX head in buffer
		
		if(tx0_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#endif
	
		asm volatile("\n\t"
			"mov	r26, %[index]  \n\t"
		
		#if !defined(__AVR_ATtiny2313__)&&!defined(__AVR_ATtiny2313A__) // on ATtiny2313 upper byte in pointer pair is ignored
			"ldi	r27, 0x00 \n\t"
		#endif
			"subi	r26, lo8(-(tx0_buffer)) \n\t"
		
		#ifndef USART_USE_TINY_MEMORY_MODEL
			"sbci	r27, hi8(-(tx0_buffer)) \n\t"
		#endif
			"st		X, %[dat] \n\t"
			
			: // outputs
			[index] "+r" (tmp_tx_Head), // will be used later, do not let the compiler to do anything weird
			[dat]   "+r" (data) // not modified, so reduce register moves if inlined
			: // inputs
			: // clobbers
		#if !defined(__AVR_ATtiny2313__)&&!defined(__AVR_ATtiny2313A__)
			"r27",
		#endif
			"r26"
		);
		
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx0_Head = tmp_tx_Head;
			
		#ifdef USART0_RS485_MODE
			RS485_CONTROL0_PORT |= (1<<RS485_CONTROL0_IONUM); // start transmitting
		#endif
			
		#ifdef USART0_USE_SOFT_CTS
			if(!(CTS0_PIN & (1<<CTS0_IONUM)))
		#endif
			{
				UCSR0B_REGISTER |= (1<<UDRIE0_BIT); // enable UDRE interrupt
			}
		}
		return COMPLETED;
	}
#endif

//******************************************************************
//Function  : Send string array.
//Arguments : Pointer to string array terminated by NULL.
//Return    : none
//******************************************************************
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart0_putstr(char *string)
	{
		char c;
		while ((c = *string++)) uart0_putc(c);
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart0_putstr(char *string)
	{
		asm volatile("\n\t"
		
		"load_loop_%=:"
			"ld 	r24, Z+ \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	skip_loop_%= \n\t"
			"rcall	uart0_putc \n\t" // Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
		
			: // outputs
			: // inputs
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

//******************************************************************
//Function  : Send string not terminated by NULL or part of the string array.
//Arguments : 1. Pointer to string array.
//          : 2. Number of characters/bytes to send.
//Return    :    none
//******************************************************************
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart0_putstrl(char *string, uint8_t BytesToWrite)
	{
		while(BytesToWrite--)
			uart0_putc(*string++);
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart0_putstrl(char *string, uint8_t BytesToWrite)
	{
		asm volatile("\n\t"
			"add	%[counter], r30 \n\t" // add ZL to a counter to compare against current pointer (8 bit length, doesn't care if overflow)
		"load_loop_%=:"
			"cp 	%[counter], r30\n\t"
			"breq	skip_loop_%= \n\t"
			"ld 	r24, Z+ \n\t"
			"rcall	uart0_putc \n\t" // counter and Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
		
			: // outputs
			: // inputs
			[counter] "r" (BytesToWrite),
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

//******************************************************************
//Function  : Send string from flash memory.
//Arguments : Pointer to string placed in flash memory.
//Return    : none
//******************************************************************
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart0_puts_p(const __flash char *string)
	{
	#ifndef USART0_NOT_ACCESIBLE_FROM_CBI // tiny 102/104
		register char c;
		while ( (c = *string++) ) uart0_putc(c); 
	#endif
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart0_puts_p(const __flash char *string)
	{
	#if !defined(__AVR_ATtiny102__)||!defined(__AVR_ATtiny104__)
		asm volatile("\n\t"
		
		"load_loop_%=:"
			"lpm 	r24, Z+ \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	skip_loop_%= \n\t"
			"rcall	uart0_putc \n\t" // Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
		
			: // outputs
			: // inputs
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	#endif
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

//******************************************************************
//Function  : Send integer formated into ASCI string (base 10).
//Arguments : int16_t data value.
//Return    : none
//******************************************************************
	void uart0_putint(int16_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		itoa(data, u_tmp_buff, 10);
		uart0_putstr(u_tmp_buff);
	}
	
//******************************************************************
//Function  : Send integer formated into ASCI string.
//Arguments : 1. uint16_t data value.
//          : 2. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart0_putintr(int16_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		itoa(data, u_tmp_buff, radix);
		uart0_putstr(u_tmp_buff);
	}
	
//******************************************************************
//Function  : Send unsigned integer formated into ASCI string (base 10).
//Arguments : uint16_t data value.
//Return    : none
//******************************************************************
	void uart0_putuint(uint16_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		utoa(data, u_tmp_buff, 10);
		uart0_putstr(u_tmp_buff);
	}
	
//******************************************************************
//Function  : Send unsigned integer formated into ASCI string.
//Arguments : 1. uint16_t data value.
//          : 2. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart0_putuintr(uint16_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		utoa(data, u_tmp_buff, radix);
		uart0_putstr(u_tmp_buff);
	}

//******************************************************************
//Function  : Send unsigned integer formated into ASCI string (base 16)
//Arguments : uint16_t data value.
//Return    : none
//******************************************************************
	void uart0_puthex(uint8_t data)
	{
		uint8_t tmp; 
		
		tmp = (data >> 4) & 0x0f;
	#ifdef USART_PUTHEX_IN_UPPERCASE
		uart0_putc( (tmp <= 9 ? '0' + tmp : 'A' - 10 + tmp));
	#else
		uart0_putc( (tmp <= 9 ? '0' + tmp : 'a' - 10 + tmp));
	#endif
		
		tmp = data & 0x0f;
	#ifdef USART_PUTHEX_IN_UPPERCASE
		uart0_putc( (tmp <= 9 ? '0' + tmp : 'A' - 10 + tmp));
	#else
		uart0_putc( (tmp <= 9 ? '0' + tmp : 'a' - 10 + tmp));
	#endif
	}

//******************************************************************
//Function  : Send long integer formated into ASCI string (base 10).
//Arguments : int32_t data value.
//Return    : none
//******************************************************************
	void uart0_putlong(int32_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
		
		ltoa(data, u_tmp_buff, 10);
		uart0_putstr(u_tmp_buff);
	}
	
//******************************************************************
//Function  : Send long integer formated into ASCI string.
//Arguments : 1. int32_t data value.
//          : 2. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart0_putlongr(int32_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		ltoa(data, u_tmp_buff, radix);
		uart0_putstr(u_tmp_buff);
	}
	
//******************************************************************
//Function  : Send unsigned long integer formated into ASCI string (base 10).
//Arguments : uint32_t data value.
//Return    : none
//******************************************************************
	void uart0_putulong(uint32_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
		
		ultoa(data, u_tmp_buff, 10);
		uart0_putstr(u_tmp_buff);
	}
	
//******************************************************************
//Function  : Send unsigned long integer formated into ASCI string.
//Arguments : 1. uint32_t data value.
//          : 2. Base value (DEC, HEX, OCT, BIN, etc.).
//Return    :    none
//******************************************************************
	void uart0_putulongr(uint32_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		ultoa(data, u_tmp_buff, radix);
		uart0_putstr(u_tmp_buff);
	}

//******************************************************************
//Function  : Send floating point value formated into ASCI string.
//Arguments : float data value.
//Return    : none
//******************************************************************
	void uart0_putfloat(float data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[16];
	#endif
		
		dtostrf(data, 15, 6, u_tmp_buff);
		
		char *p = u_tmp_buff;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart0_putstr(p);
	}

//******************************************************************
//Function  : Send floating point integer formated into ASCI string.
//Arguments : 1. Float data value.
//          : 2. Number of displayed digits after the dot.
//Return    :    none
//******************************************************************
	void uart0_fputfloat(float data, uint8_t precision)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[16];
	#endif
		
		dtostrf(data, 15, precision, u_tmp_buff);
		
		char *p = u_tmp_buff;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart0_putstr(p);
	}
	
//******************************************************************
//Function  : Wait until all data in TX buffer are flushed.
//Arguments : none
//Return    : none
//******************************************************************	
	void uart0_flush(void)
	{
	#ifdef USART0_RS485_MODE // flush UDR buffer
		while (RS485_CONTROL0_PORT & (1<<RS485_CONTROL0_IONUM));
	#else
		while(tx0_Tail != tx0_Head); // just flush the ring buffer
	#endif
	}
	
//******************************************************************
//Function  : To check how many bytes are waiting in the transmitter buffer.
//Arguments : none
//Return    : Number of bytes waiting in transmitter buffer.
//******************************************************************
//uint8_t uart0_BytesToSend(void)
//	{
//		return (tx0_Head - tx0_Tail) & TX0_BUFFER_MASK;
//	}

//******************************************************************
//Function  : Transmit address of selected slave in MPCM mode.
//Arguments : Address of selected slave.
//Return    : none
//******************************************************************
#ifdef USART0_MPCM_MODE
	void uart0_mpcm_transmit_addres_Frame(uint8_t dat)
	{
		while(tx0_Tail != tx0_Head);
		UCSR0B_REGISTER |= (1<<TXB80_BIT);
		uart_putc(dat);
		while(tx0_Tail != tx0_Head);
		UCSR0B_REGISTER &= ~(1<<TXB80_BIT); // not sure if necessary
	}
#endif
#endif // NO_TX0_INTERRUPT

#ifndef NO_TX1_INTERRUPT

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart1_putc(char data)
	{
	#ifdef PUTC1_CONVERT_LF_TO_CRLF
		if (data == '\n')
			uart1_putc('\r');
	#endif
		
	#ifdef USART1_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx1_Head;
		register uint8_t tmp_tx_Tail = tx1_Tail;
		
	#ifdef USART1_USE_SOFT_CTS
		if(!(CTS1_PIN & (1<<CTS1_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR1A_REGISTER & UDRE1_BIT))
		{
			UDR1_REGISTER = data;
			return;
		}
		
		tmp_tx_Head = (tmp_tx_Head + 1) & TX1_BUFFER_MASK;
		
		while(tmp_tx_Tail == tmp_tx_Head) // wait for free space in buffer
		{
			tmp_tx_Tail = tx1_Tail; // for faster pass through, results in a little bigger code
		}
	#else
		register uint8_t tmp_tx_Head = (tx1_Head + 1) & TX1_BUFFER_MASK; // calculate new position of TX head in buffer
		
		while(tx1_Tail == tmp_tx_Head); // wait for free space in buffer
	#endif
		
		tx1_buffer[tmp_tx_Head] = data;
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx1_Head = tmp_tx_Head;
			
		#ifdef USART1_RS485_MODE
			RS485_CONTROL1_PORT |= (1<<RS485_CONTROL1_IONUM); //set high
		#endif
			
		#ifdef USART1_USE_SOFT_CTS
			if(!(CTS1_PIN & (1<<CTS1_IONUM)))
		#endif
			{
				UCSR1B_REGISTER |= (1<<UDRIE1_BIT); // enable UDRE interrupt
			}
		}
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart1_putc(char data)
	{
		register uint8_t tmp_tx_Head asm("r25");
		
	#ifdef PUTC1_CONVERT_LF_TO_CRLF
		asm volatile("\n\t"
			"cpi	%[dat], '\n' \n\t"
			"brne	skip_recursive_%=\n\t"
			"push	%[dat] \n\t"
			"ldi	%[dat], '\r' \n\t"
			"rcall	uart1_putc \n\t"
			"pop	%[dat] \n\t"
		"skip_recursive_%=:"
			: // outputs
			[dat]  "+r" (data) // will be used later, do not let the compiler to do anything weird
			: // inputs
			: // clobbers
		);
	#endif
		
	#ifdef USART1_PUTC_FAST_INSERTIONS
		
		asm volatile("\n\t"
			"lds	%[head], (tx1_Head) \n\t"
			
		#ifdef USART1_USE_SOFT_CTS
			"sbic	%M[cts_port], %M[cts_pin] \n\t"
			"rjmp	normal_insert_%= \n\t"  
		#endif
				
			"lds	r27, (tx1_Tail) \n\t"
			"cpse	r27, %[head] \n\t"
			"rjmp	normal_insert_%= \n\t"
				
		#ifdef USART1_IN_IO_ADDRESS_SPACE
			"sbis	%M[UCSRA_reg_IO], %M[udre_bit] \n\t"
		#else
			"lds	r26, %M[UCSRA_reg] \n\t"
			"sbrs	r26, %M[udre_bit] \n\t"
		#endif
			"rjmp	normal_insert_%= \n\t"
				
		#ifdef USART1_IN_IO_ADDRESS_SPACE
			"out	%M[UDR_reg], %[dat] \n\t"
		#else
			"sts	%M[UDR_reg], %[dat] \n\t"
		#endif
			"ret	\n\t"
				
		"normal_insert_%=:"
			"inc	%[head] \n\t"
			
		#if (TX1_BUFFER_MASK != 0xff)
			"andi	%[head], %M[mask] \n\t"
		#endif
				
		"waitforspace_%=:"
			"lds	r27, (tx1_Tail) \n\t"
			"cp		r27, %[head] \n\t"
			"breq	waitforspace_%= \n\t"
				
			: // outputs
			[head] "=r" (tmp_tx_Head),
			[dat]  "+r" (data)
			: // inputs
		#ifdef USART1_USE_SOFT_CTS
			[cts_port]      "M" (_SFR_IO_ADDR(CTS1_PORT)),
			[cts_pin]       "M" (CTS1_IONUM),
		#endif
			[mask]          "M" (TX1_BUFFER_MASK),
			[UCSRA_reg]     "n" (_SFR_MEM_ADDR(UCSR1A_REGISTER)),
			[UCSRA_reg_IO]  "M" (_SFR_IO_ADDR(UCSR1A_REGISTER)),
			[UDR_reg]	    "n" (_SFR_MEM_ADDR(UDR1_REGISTER)),
			[UDR_reg_IO]    "M" (_SFR_IO_ADDR(UDR1_REGISTER)),
			[udre_bit]      "M"	(UDRE1_BIT)
			: // clobbers
			"r26","r27"
		);
	#else
		asm volatile("\n\t"
			"lds	%[head], (tx1_Head) \n\t"
			"inc	%[head] \n\t"
			
		#if (TX1_BUFFER_MASK != 0xff)
			"andi	%[head], %M[mask] \n\t"
		#endif
				
		"waitforspace_%=:"
			"lds	r27, (tx1_Tail) \n\t"
			"cp		r27, %[head] \n\t"
			"breq	waitforspace_%= \n\t"
				
			: // outputs
			[head] "=r" (tmp_tx_Head)
			: // inputs
			[mask] "M" (TX1_BUFFER_MASK)
			: // clobbers
			"r27"
		);
	#endif
		
		asm volatile("\n\t"
			"mov	r26, %[index]  \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(tx1_buffer)) \n\t"
			"sbci	r27, hi8(-(tx1_buffer)) \n\t"
			"st		X, %[dat] \n\t"
			
			: // outputs
			[index] "+r" (tmp_tx_Head),
			[dat]   "+r" (data)
			: // inputs
			: // clobbers
			"r26","r27"
		);
		
		cli();
		{
			tx1_Head = tmp_tx_Head;
		
		#ifdef USART1_RS485_MODE
			RS485_CONTROL1_PORT |= (1<<RS485_CONTROL1_IONUM); // start transmitting
		#endif
	
		#ifdef USART1_USE_SOFT_CTS
			if(!(CTS1_PIN & (1<<CTS1_IONUM)))
		#endif
			{
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				UCSR1B_REGISTER |= (1<<UDRIE1_BIT); // enable UDRE interrupt
			#else
				asm volatile("\n\t"
					"lds   r25, %M[control_reg] \n\t"
					"ori  r25, (1<<%M[udrie_bit]) \n\t"
					"sts   %M[control_reg], r25 \n\t"
					: // outputs
					: // inputs
					[control_reg] "n" (_SFR_MEM_ADDR(UCSR1B_REGISTER)),
					[udrie_bit]   "M" (UDRIE1_BIT)
					: // clobbers
					"r25"
				);
			#endif
			}
		}
	
		reti();
		
		asm volatile("\n\t"::"r" (data):); // data was passed in r24 and will be returned in the same register, make sure it is not affected by the compiler 
	}
	
	char uart1_putc_(char data) __attribute__ ((alias ("uart1_putc"))); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	uint8_t uart1_putc_noblock(char data)
	{
	#ifdef USART1_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx1_Head;
		register uint8_t tmp_tx_Tail = tx1_Tail;
	
	#ifdef USART1_USE_SOFT_CTS
		if(!(CTS1_PIN & (1<<CTS1_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR1A_REGISTER & UDRE1_BIT))
		{
			UDR1_REGISTER = data;
			return COMPLETED;
		}
	
		tmp_tx_Head = (tmp_tx_Head + 1) & TX1_BUFFER_MASK;
	
		if(tmp_tx_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#else
		register uint8_t tmp_tx_Head = (tx1_Head + 1) & TX1_BUFFER_MASK; // calculate new position of TX head in buffer
	
		if(tx1_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#endif
	
		tx1_buffer[tmp_tx_Head] = data;
	
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx1_Head = tmp_tx_Head;
		
		#ifdef USART1_RS485_MODE
			RS485_CONTROL1_PORT |= (1<<RS485_CONTROL1_IONUM); //set high
		#endif
		
		#ifdef USART1_USE_SOFT_CTS
			if(!(CTS1_PIN & (1<<CTS1_IONUM)))
		#endif
			{
				UCSR1B_REGISTER |= (1<<UDRIE1_BIT); // enable UDRE interrupt
			}
		}
		return COMPLETED;
	}
#else //!USART_NO_ABI_BREAKING_PREMATURES
	uint8_t uart1_putc_noblock(char data)
	{
	#ifdef USART1_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx1_Head;
		register uint8_t tmp_tx_Tail = tx1_Tail;
		
	#ifdef USART1_USE_SOFT_CTS
		if(!(CTS1_PIN & (1<<CTS1_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR1A_REGISTER & UDRE1_BIT))
		{
			UDR1_REGISTER = data;
			return COMPLETED;
		}
		
		tmp_tx_Head = (tmp_tx_Head + 1) & TX1_BUFFER_MASK;
		
		if(tmp_tx_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#else
		register uint8_t tmp_tx_Head = (tx1_Head + 1) & TX1_BUFFER_MASK; // calculate new position of TX head in buffer
		
		if(tx1_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#endif
	
		asm volatile("\n\t"
			"mov	r26, %[index]  \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(tx1_buffer)) \n\t"
			"sbci	r27, hi8(-(tx1_buffer)) \n\t"
			"st		X, %[dat] \n\t"
			
			: // outputs
			[index] "+r" (tmp_tx_Head),
			[dat]   "+r" (data) 
			: // inputs
			: // clobbers
			"r26","r27"
		);
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx1_Head = tmp_tx_Head;
			
		#ifdef USART1_RS485_MODE
			RS485_CONTROL1_PORT |= (1<<RS485_CONTROL1_IONUM); // start transmitting
		#endif
			
		#ifdef USART1_USE_SOFT_CTS
			if(!(CTS1_PIN & (1<<CTS1_IONUM)))
		#endif
			{
				UCSR1B_REGISTER |= (1<<UDRIE1_BIT); // enable UDRE interrupt
			}
		}
		return COMPLETED;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart1_putstr(char *string)
	{
		char c;
		while ((c = *string++)) uart1_putc(c);
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart1_putstr(char *string)
	{
		asm volatile("\n\t"
		
		"load_loop_%=:"
			"ld 	r24, Z+ \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	skip_loop_%= \n\t"
			"rcall	uart1_putc \n\t" // Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
		
			: // outputs
			: // inputs
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart1_putstrl(char *string, uint8_t BytesToWrite)
	{
		while(BytesToWrite--)
			uart1_putc(*string++);
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart1_putstrl(char *string, uint8_t BytesToWrite)
	{
		asm volatile("\n\t"
			"add	%[counter], r30 \n\t" // add ZL to a counter to compare against current pointer (8 bit length, doesn't care if overflow)
		"load_loop_%=:"
			"cp 	%[counter], r30\n\t"
			"breq	skip_loop_%= \n\t"
			"ld 	r24, Z+ \n\t"
			"rcall	uart1_putc \n\t" // counter and Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
		
			: // outputs
			: // inputs
			[counter] "r" (BytesToWrite),
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart1_puts_p(const __flash char *string)
	{
		register char c;
		while ( (c = *string++) ) uart1_putc(c); 
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart1_puts_p(const __flash char *string)
	{
		asm volatile("\n\t"
		
		"load_loop_%=:"
			"lpm 	r24, Z+ \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	skip_loop_%= \n\t"
			"rcall	uart1_putc \n\t" // Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
	
			: // outputs
			: // inputs
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

	void uart1_putint(int16_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		itoa(data, u_tmp_buff, 10);
		uart1_putstr(u_tmp_buff);
	}

	void uart1_putintr(int16_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		itoa(data, u_tmp_buff, radix);
		uart1_putstr(u_tmp_buff);
	}

	void uart1_putuint(uint16_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		utoa(data, u_tmp_buff, 10);
		uart1_putstr(u_tmp_buff);
	}
	
	void uart1_putuintr(uint16_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		utoa(data, u_tmp_buff, radix);
		uart1_putstr(u_tmp_buff);
	}

	void uart1_puthex(uint8_t data)
	{
		uint8_t tmp; 
		
		tmp = (data >> 4) & 0x0f;
	#ifdef USART_PUTHEX_IN_UPPERCASE
		uart1_putc( (tmp <= 9 ? '0' + tmp : 'A' - 10 + tmp));
	#else
		uart1_putc( (tmp <= 9 ? '0' + tmp : 'a' - 10 + tmp));
	#endif
		
		tmp = data & 0x0f;
	#ifdef USART_PUTHEX_IN_UPPERCASE
		uart1_putc( (tmp <= 9 ? '0' + tmp : 'A' - 10 + tmp));
	#else
		uart1_putc( (tmp <= 9 ? '0' + tmp : 'a' - 10 + tmp));
	#endif
	}

	void uart1_putlong(int32_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
		
		ltoa(data, u_tmp_buff, 10);
		uart1_putstr(u_tmp_buff);
	}
	
	void uart1_putlongr(int32_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		ltoa(data, u_tmp_buff, radix);
		uart1_putstr(u_tmp_buff);
	}
	
	void uart1_putulong(uint32_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
		
		ultoa(data, u_tmp_buff, 10);
		uart1_putstr(u_tmp_buff);
	}
	
	void uart1_putulongr(uint32_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		ultoa(data, u_tmp_buff, radix);
		uart1_putstr(u_tmp_buff);
	}

	void uart1_putfloat(float data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[16];
	#endif
		
		dtostrf(data, 15, 6, u_tmp_buff);
		
		char *p = u_tmp_buff;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart1_putstr(p);
	}

	void uart1_fputfloat(float data, uint8_t precision)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[16];
	#endif
		
		dtostrf(data, 15, precision, u_tmp_buff);
		
		char *p = u_tmp_buff;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart1_putstr(p);
	}
	
	void uart1_flush(void)
	{
	#ifdef USART1_RS485_MODE // flush UDR buffer
		while (RS485_CONTROL1_PORT & (1<<RS485_CONTROL1_IONUM));
	#else	
		while(tx1_Tail != tx1_Head); // just flush the ring buffer 
	#endif
	}

#ifdef USART1_MPCM_MODE
	void uart1_mpcm_transmit_addres_Frame(uint8_t dat)
	{
		while(tx1_Tail != tx1_Head);
		UCSR1B_REGISTER |= (1<<TXB81_BIT);
		uart1_putc(dat);
		while(tx1_Tail != tx1_Head);
		UCSR1B_REGISTER &= ~(1<<TXB81_BIT); // not sure if necessary
	}
#endif
#endif // NO_TX1_INTERRUPT

#ifndef NO_TX2_INTERRUPT

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart2_putc(char data)
	{
	#ifdef PUTC2_CONVERT_LF_TO_CRLF
		if (data == '\n')
			uart2_putc('\r');
	#endif
		
	#ifdef USART2_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx2_Head;
		register uint8_t tmp_tx_Tail = tx2_Tail;
		
	#ifdef USART2_USE_SOFT_CTS
		if(!(CTS2_PIN & (1<<CTS2_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR2A_REGISTER & UDRE2_BIT))
		{
			UDR2_REGISTER = data;
			return;
		}
		
		tmp_tx_Head = (tmp_tx_Head + 1) & TX2_BUFFER_MASK;
		
		while(tmp_tx_Tail == tmp_tx_Head) // wait for free space in buffer
		{
			tmp_tx_Tail = tx2_Tail; // for faster pass through, results in a little bigger code
		}
	#else
		register uint8_t tmp_tx_Head = (tx2_Head + 1) & TX2_BUFFER_MASK; // calculate new position of TX head in buffer
		
		while(tx2_Tail == tmp_tx_Head); // wait for free space in buffer
	#endif
		
		tx2_buffer[tmp_tx_Head] = data;
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx2_Head = tmp_tx_Head;
			
		#ifdef USART2_RS485_MODE
			RS485_CONTROL2_PORT |= (1<<RS485_CONTROL2_IONUM); //set high
		#endif
			
		#ifdef USART2_USE_SOFT_CTS
			if(!(CTS2_PIN & (1<<CTS2_IONUM)))
		#endif
			{
				UCSR2B_REGISTER |= (1<<UDRIE2_BIT); // enable UDRE interrupt
			}
		}
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart2_putc(char data)
	{
		register uint8_t tmp_tx_Head asm("r25");
		
	#ifdef PUTC2_CONVERT_LF_TO_CRLF
		asm volatile("\n\t"
			"cpi	%[dat], '\n' \n\t"
			"brne	skip_recursive_%=\n\t"
			"push	%[dat] \n\t"
			"ldi	%[dat], '\r' \n\t"
			"rcall	uart2_putc \n\t"
			"pop	%[dat] \n\t"
		"skip_recursive_%=:"
			: // outputs
			[dat]  "+r" (data) // will be used later, do not let the compiler to do anything weird
			: // inputs
			: // clobbers
		);
	#endif
		
	#ifdef USART2_PUTC_FAST_INSERTIONS
			
		asm volatile("\n\t"
			"lds	%[head], (tx2_Head) \n\t"
			
		#ifdef USART2_USE_SOFT_CTS
			"sbic	%M[cts_port], %M[cts_pin] \n\t"
			"rjmp	normal_insert_%= \n\t"  
		#endif
				
			"lds	r27, (tx2_Tail) \n\t"
			"cpse	r27, %[head] \n\t"
			"rjmp	normal_insert_%= \n\t"
			
			"lds	r26, %M[UCSRA_reg] \n\t"
			"sbrs	r26, %M[udre_bit] \n\t"
			"rjmp	normal_insert_%= \n\t"
				
			"sts	%M[UDR_reg], %[dat] \n\t"
			"ret	\n\t"
				
		"normal_insert_%=:"
			"inc	%[head] \n\t"
			
		#if (TX2_BUFFER_MASK != 0xff)
			"andi	%[head], %M[mask] \n\t"
		#endif
				
		"waitforspace_%=:"
			"lds	r27, (tx2_Tail) \n\t"
			"cp		r27, %[head] \n\t"
			"breq	waitforspace_%= \n\t"
				
			: // outputs
			[head] "=r" (tmp_tx_Head),
			[dat]  "+r" (data)
			: // inputs
		#ifdef USART2_USE_SOFT_CTS
			[cts_port]      "M"    (_SFR_IO_ADDR(CTS2_PORT)),
			[cts_pin]       "M"    (CTS2_IONUM),
		#endif
			[mask]          "M" (TX2_BUFFER_MASK),
			[UCSRA_reg]     "n" (_SFR_MEM_ADDR(UCSR2A_REGISTER)),
			[UCSRA_reg_IO]  "M" (_SFR_IO_ADDR(UCSR2A_REGISTER)),
			[UDR_reg]	    "n" (_SFR_MEM_ADDR(UDR2_REGISTER)),
			[UDR_reg_IO]    "M" (_SFR_IO_ADDR(UDR2_REGISTER)),
			[udre_bit]      "M"	(UDRE2_BIT)
			: // clobbers
			"r26","r27"
		);
	#else
		asm volatile("\n\t"
			"lds	%[head], (tx2_Head) \n\t"
			"inc	%[head] \n\t"
			
		#if (TX2_BUFFER_MASK != 0xff)
			"andi	%[head], %M[mask] \n\t"
		#endif
				
		"waitforspace_%=:"
			"lds	r27, (tx2_Tail) \n\t"
			"cp		r27, %[head] \n\t"
			"breq	waitforspace_%= \n\t"
				
			: // outputs
			[head] "=r" (tmp_tx_Head)
			: // inputs
			[mask] "M" (TX2_BUFFER_MASK)
			: // clobbers
			"r27"
		);
	#endif
		
		asm volatile("\n\t"
			"mov	r26, %[index]  \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(tx2_buffer)) \n\t"
			"sbci	r27, hi8(-(tx2_buffer)) \n\t"
			"st		X, %[dat] \n\t"
			
			: // outputs
			[index] "+r" (tmp_tx_Head),
			[dat]   "+r" (data)
			: // inputs
			: // clobbers
			"r26","r27"
		);
		
		cli();
		{
			tx2_Head = tmp_tx_Head;
		
		#ifdef USART2_RS485_MODE
			RS485_CONTROL2_PORT |= (1<<RS485_CONTROL2_IONUM); // start transmitting
		#endif
		
		#ifdef USART2_USE_SOFT_CTS
			if(!(CTS2_PIN & (1<<CTS2_IONUM)))
		#endif
			{
				asm volatile("\n\t"
					"lds   r25, %M[control_reg] \n\t"
					"ori  r25, (1<<%M[udrie_bit]) \n\t"
					"sts   %M[control_reg], r25 \n\t"
					: // outputs
					: // inputs
					[control_reg] "n" (_SFR_MEM_ADDR(UCSR2B_REGISTER)),
					[udrie_bit]   "M" (UDRIE2_BIT)
					: // clobbers
					"r25"
				);
			}
		}
	
		reti();
		asm volatile("\n\t"::"r" (data):); // data was passed in r24 and will be returned in the same register, make sure it is not affected by the compiler 
	}
	
	char uart2_putc_(char data) __attribute__ ((alias ("uart2_putc"))); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	uint8_t uart2_putc_noblock(char data)
	{
	#ifdef USART2_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx2_Head;
		register uint8_t tmp_tx_Tail = tx2_Tail;
	
	#ifdef USART2_USE_SOFT_CTS
		if(!(CTS2_PIN & (1<<CTS2_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR2A_REGISTER & UDRE2_BIT))
		{
			UDR2_REGISTER = data;
			return COMPLETED;
		}
	
		tmp_tx_Head = (tmp_tx_Head + 1) & TX2_BUFFER_MASK;
	
		if(tmp_tx_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#else
		register uint8_t tmp_tx_Head = (tx2_Head + 1) & TX2_BUFFER_MASK; // calculate new position of TX head in buffer
	
		if(tx2_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#endif
	
		tx2_buffer[tmp_tx_Head] = data;
	
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx2_Head = tmp_tx_Head;
		
		#ifdef USART2_RS485_MODE
			RS485_CONTROL2_PORT |= (1<<RS485_CONTROL2_IONUM); //set high
		#endif
		
		#ifdef USART2_USE_SOFT_CTS
			if(!(CTS2_PIN & (1<<CTS2_IONUM)))
		#endif
			{
				UCSR2B_REGISTER |= (1<<UDRIE2_BIT); // enable UDRE interrupt
			}
		}
		return COMPLETED;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	uint8_t uart2_putc_noblock(char data)
	{
	#ifdef USART2_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx2_Head;
		register uint8_t tmp_tx_Tail = tx2_Tail;
		
	#ifdef USART2_USE_SOFT_CTS
		if(!(CTS2_PIN & (1<<CTS2_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR2A_REGISTER & UDRE2_BIT))
		{
			UDR2_REGISTER = data;
			return COMPLETED;
		}
		
		tmp_tx_Head = (tmp_tx_Head + 1) & TX2_BUFFER_MASK;
		
		if(tmp_tx_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#else
		register uint8_t tmp_tx_Head = (tx2_Head + 1) & TX2_BUFFER_MASK; // calculate new position of TX head in buffer
		
		if(tx2_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#endif
	
		asm volatile("\n\t"
			"mov	r26, %[index]  \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(tx2_buffer)) \n\t"
			"sbci	r27, hi8(-(tx2_buffer)) \n\t"
			"st		X, %[dat] \n\t"
			
			: // outputs
			[index] "+r" (tmp_tx_Head),
			[dat]   "+r" (data)
			: // inputs
			: // clobbers
			"r26","r27"
		);
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx2_Head = tmp_tx_Head;
			
		#ifdef USART2_RS485_MODE
			RS485_CONTROL2_PORT |= (1<<RS485_CONTROL2_IONUM); // start transmitting
		#endif
			
		#ifdef USART2_USE_SOFT_CTS
			if(!(CTS2_PIN & (1<<CTS2_IONUM)))
		#endif
			{
				UCSR2B_REGISTER |= (1<<UDRIE2_BIT); // enable UDRE interrupt
			}
		}
		return COMPLETED;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES
	
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart2_putstr(char *string)
	{
		char c;
		while ((c = *string++)) uart2_putc(c);
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart2_putstr(char *string)
	{
		asm volatile("\n\t"
			
		"load_loop_%=:"
			"ld 	r24, Z+ \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	skip_loop_%= \n\t"
			"rcall	uart2_putc \n\t" // Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
		
			: // outputs
			: // inputs
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart2_putstrl(char *string, uint8_t BytesToWrite)
	{
		while(BytesToWrite--)
			uart2_putc(*string++);
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart2_putstrl(char *string, uint8_t BytesToWrite)
	{
		asm volatile("\n\t"
			"add	%[counter], r30 \n\t" // add ZL to a counter to compare against current pointer (8 bit length, doesn't care if overflow)
		"load_loop_%=:"
			"cp 	%[counter], r30\n\t"
			"breq	skip_loop_%= \n\t"
			"ld 	r24, Z+ \n\t"
			"rcall	uart2_putc \n\t" // counter and Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
		
			: // outputs
			: // inputs
			[counter] "r" (BytesToWrite),
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart2_puts_p(const __flash char *string)
	{
		register char c;
		while ( (c = *string++) ) uart2_putc(c);
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart2_puts_p(const __flash char *string)
	{
		asm volatile("\n\t"
		
		"load_loop_%=:"
			"lpm 	r24, Z+ \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	skip_loop_%= \n\t"
			"rcall	uart2_putc \n\t" // Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
	
			: // outputs
			: // inputs
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

	void uart2_putint(int16_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		itoa(data, u_tmp_buff, 10);
		uart2_putstr(u_tmp_buff);
	}

	void uart2_putintr(int16_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		itoa(data, u_tmp_buff, radix);
		uart2_putstr(u_tmp_buff);
	}

	void uart2_putuint(uint16_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		utoa(data, u_tmp_buff, 10);
		uart2_putstr(u_tmp_buff);
	}
	
	void uart2_putuintr(uint16_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		utoa(data, u_tmp_buff, radix);
		uart2_putstr(u_tmp_buff);
	}

	void uart2_puthex(uint8_t data)
	{
		uint8_t tmp; 
		
		tmp = (data >> 4) & 0x0f;
	#ifdef USART_PUTHEX_IN_UPPERCASE
		uart2_putc( (tmp <= 9 ? '0' + tmp : 'A' - 10 + tmp));
	#else
		uart2_putc( (tmp <= 9 ? '0' + tmp : 'a' - 10 + tmp));
	#endif
		
		tmp = data & 0x0f;
	#ifdef USART_PUTHEX_IN_UPPERCASE
		uart2_putc( (tmp <= 9 ? '0' + tmp : 'A' - 10 + tmp));
	#else
		uart2_putc( (tmp <= 9 ? '0' + tmp : 'a' - 10 + tmp));
	#endif
	}

	void uart2_putlong(int32_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
		
		ltoa(data, u_tmp_buff, 10);
		uart2_putstr(u_tmp_buff);
	}
	
	void uart2_putlongr(int32_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		ltoa(data, u_tmp_buff, radix);
		uart2_putstr(u_tmp_buff);
	}
	
	void uart2_putulong(uint32_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
		
		ultoa(data, u_tmp_buff, 10);
		uart2_putstr(u_tmp_buff);
	}
	
	void uart2_putulongr(uint32_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		ultoa(data, u_tmp_buff, radix);
		uart2_putstr(u_tmp_buff);
	}

	void uart2_putfloat(float data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[16];
	#endif
		
		dtostrf(data, 15, 6, u_tmp_buff);
		
		char *p = u_tmp_buff;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart2_putstr(p);
	}

	void uart2_fputfloat(float data, uint8_t precision)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[16];
	#endif
		
		dtostrf(data, 15, precision, u_tmp_buff);
		
		char *p = u_tmp_buff;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart2_putstr(p);
	}
	
	void uart2_flush(void)
	{
	#ifdef USART2_RS485_MODE // flush UDR buffer
		while (RS485_CONTROL2_PORT & (1<<RS485_CONTROL2_IONUM));
	#else	
		while(tx2_Tail != tx2_Head); // just flush the ring buffer 
	#endif
	}

#ifdef USART2_MPCM_MODE
	void uart2_mpcm_transmit_addres_Frame(uint8_t dat)
	{
		while(tx2_Tail != tx2_Head);
		UCSR2B_REGISTER |= (1<<TXB82_BIT);
		uart2_putc(dat);
		while(tx2_Tail != tx2_Head);
		UCSR2B_REGISTER &= ~(1<<TXB82_BIT); // not sure if necessary
	}
#endif
#endif // NO_TX2_INTERRUPT

#ifndef NO_TX3_INTERRUPT

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart3_putc(char data)
	{
	#ifdef PUTC3_CONVERT_LF_TO_CRLF
		if (data == '\n')
			uart3_putc('\r');
	#endif
		
	#ifdef USART3_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx3_Head;
		register uint8_t tmp_tx_Tail = tx3_Tail;
		
	#ifdef USART3_USE_SOFT_CTS
		if(!(CTS3_PIN & (1<<CTS3_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR3A_REGISTER & UDRE3_BIT))
		{
			UDR3_REGISTER = data;
			return;
		}
		
		tmp_tx_Head = (tmp_tx_Head + 1) & TX3_BUFFER_MASK;
		
		while(tmp_tx_Tail == tmp_tx_Head) // wait for free space in buffer
		{
			tmp_tx_Tail = tx3_Tail; // for faster pass through, results in a little bigger code
		}
	#else
		register uint8_t tmp_tx_Head = (tx3_Head + 1) & TX3_BUFFER_MASK; // calculate new position of TX head in buffer
		
		while(tx3_Tail == tmp_tx_Head); // wait for free space in buffer
	#endif
		
		tx3_buffer[tmp_tx_Head] = data;
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx3_Head = tmp_tx_Head;
			
		#ifdef USART3_RS485_MODE
			RS485_CONTROL3_PORT |= (1<<RS485_CONTROL3_IONUM); //set high
		#endif
			
		#ifdef USART3_USE_SOFT_CTS
			if(!(CTS3_PIN & (1<<CTS3_IONUM)))
		#endif
			{
				UCSR3B_REGISTER |= (1<<UDRIE3_BIT); // enable UDRE interrupt
			}
		}
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart3_putc(char data)
	{
		register uint8_t tmp_tx_Head asm("r25");
		
	#ifdef PUTC3_CONVERT_LF_TO_CRLF
		asm volatile("\n\t"
			"cpi	%[dat], '\n' \n\t"
			"brne	skip_recursive_%=\n\t"
			"push	%[dat] \n\t"
			"ldi	%[dat], '\r' \n\t"
			"rcall	uart3_putc \n\t"
			"pop	%[dat] \n\t"
		"skip_recursive_%=:"
			: // outputs
			[dat]  "+r" (data) // will be used later, do not let the compiler to do anything weird
			: // inputs
			: // clobbers
		);
	#endif
		
	#ifdef USART3_PUTC_FAST_INSERTIONS
		
		asm volatile("\n\t"
			"lds	%[head], (tx3_Head) \n\t"
			
		#ifdef USART3_USE_SOFT_CTS
			"sbic	%M[cts_port], %M[cts_pin] \n\t"
			"rjmp	normal_insert_%= \n\t"  
		#endif
				
			"lds	r27, (tx3_Tail) \n\t"
			"cpse	r27, %[head] \n\t"
			"rjmp	normal_insert_%= \n\t"
			
			"lds	r26, %M[UCSRA_reg] \n\t"
			"sbrs	r26, %M[udre_bit] \n\t"
			"rjmp	normal_insert_%= \n\t"
				
			"sts	%M[UDR_reg], %[dat] \n\t"
			"ret	\n\t"
				
		"normal_insert_%=:"
			"inc	%[head] \n\t"
			
		#if (TX3_BUFFER_MASK != 0xff)
			"andi	%[head], %M[mask] \n\t"
		#endif
				
		"waitforspace_%=:"
			"lds	r27, (tx3_Tail) \n\t"
			"cp		r27, %[head] \n\t"
			"breq	waitforspace_%= \n\t"
				
			: // outputs
			[head] "=r" (tmp_tx_Head),
			[dat]  "+r" (data)
			: // inputs
		#ifdef USART3_USE_SOFT_CTS
			[cts_port]      "M"    (_SFR_IO_ADDR(CTS3_PORT)),
			[cts_pin]       "M"    (CTS3_IONUM),
		#endif
			[mask]          "M" (TX2_BUFFER_MASK),
			[UCSRA_reg]     "n" (_SFR_MEM_ADDR(UCSR3A_REGISTER)),
			[UCSRA_reg_IO]  "M" (_SFR_IO_ADDR(UCSR3A_REGISTER)),
			[UDR_reg]	    "n" (_SFR_MEM_ADDR(UDR3_REGISTER)),
			[UDR_reg_IO]    "M" (_SFR_IO_ADDR(UDR3_REGISTER)),
			[udre_bit]      "M"	(UDRE3_BIT)
			: // clobbers
			"r26","r27"
		);
	#else
			asm volatile("\n\t"
				"lds	%[head], (tx3_Head) \n\t"
				"inc	%[head] \n\t"
			
			#if (TX3_BUFFER_MASK != 0xff)
				"andi	%[head], %M[mask] \n\t"
			#endif
				
			"waitforspace_%=:"
				"lds	r27, (tx3_Tail) \n\t"
				"cp		r27, %[head] \n\t"
				"breq	waitforspace_%= \n\t"
				
				: // outputs
				[head] "=r" (tmp_tx_Head)
				: // inputs
				[mask] "M" (TX3_BUFFER_MASK)
				: // clobbers
				"r27"
			);
	#endif
		
		asm volatile("\n\t"
			"mov	r26, %[index]  \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(tx3_buffer)) \n\t"
			"sbci	r27, hi8(-(tx3_buffer)) \n\t"
			"st		X, %[dat] \n\t"
			
			: // outputs
			[index] "+r" (tmp_tx_Head),
			[dat]   "+r" (data)
			: // inputs
			: // clobbers
			"r26","r27"
		);
		
		cli();
		{
			tx3_Head = tmp_tx_Head;
		
		#ifdef USART3_RS485_MODE
			RS485_CONTROL3_PORT |= (1<<RS485_CONTROL3_IONUM); // start transmitting
		#endif
		
		#ifdef USART3_USE_SOFT_CTS
			if(!(CTS3_PIN & (1<<CTS3_IONUM)))
		#endif
			{
				asm volatile("\n\t"
					"lds   r25, %M[control_reg] \n\t"
					"ori  r25, (1<<%M[udrie_bit]) \n\t"
					"sts   %M[control_reg], r25 \n\t"
					: // outputs
					: // inputs
					[control_reg] "n" (_SFR_MEM_ADDR(UCSR3B_REGISTER)),
					[udrie_bit]   "M" (UDRIE3_BIT)
					: // clobbers
					"r25"
				);
			}
		}
	
		reti();
		
		asm volatile("\n\t"::"r" (data):); // data was passed in r24 and will be returned in the same register, make sure it is not affected by the compiler 
	}
	
	char uart3_putc_(char data) __attribute__ ((alias ("uart3_putc"))); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	uint8_t uart3_putc_noblock(char data)
	{
	#ifdef USART3_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx3_Head;
		register uint8_t tmp_tx_Tail = tx3_Tail;
	
	#ifdef USART3_USE_SOFT_CTS
		if(!(CTS3_PIN & (1<<CTS3_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR3A_REGISTER & UDRE3_BIT))
		{
			UDR3_REGISTER = data;
			return COMPLETED;
		}
	
		tmp_tx_Head = (tmp_tx_Head + 1) & TX3_BUFFER_MASK;
	
		if(tmp_tx_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#else
		register uint8_t tmp_tx_Head = (tx3_Head + 1) & TX3_BUFFER_MASK; // calculate new position of TX head in buffer
	
		if(tx3_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#endif
	
		tx3_buffer[tmp_tx_Head] = data;
	
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx3_Head = tmp_tx_Head;
		
		#ifdef USART3_RS485_MODE
			RS485_CONTROL3_PORT |= (1<<RS485_CONTROL3_IONUM); //set high
		#endif
		
		#ifdef USART3_USE_SOFT_CTS
			if(!(CTS3_PIN & (1<<CTS3_IONUM)))
		#endif
			{
				UCSR3B_REGISTER |= (1<<UDRIE3_BIT); // enable UDRE interrupt
			}
		}
		return COMPLETED;
	}
#else //!USART_NO_ABI_BREAKING_PREMATURES
	uint8_t uart3_putc_noblock(char data)
	{
	#ifdef USART3_PUTC_FAST_INSERTIONS
		register uint8_t tmp_tx_Head = tx3_Head;
		register uint8_t tmp_tx_Tail = tx3_Tail;
		
	#ifdef USART3_USE_SOFT_CTS
		if(!(CTS3_PIN & (1<<CTS3_IONUM)))
	#endif
		if(tmp_tx_Tail == tmp_tx_Head && (UCSR3A_REGISTER & UDRE3_BIT))
		{
			UDR3_REGISTER = data;
			return COMPLETED;
		}
		
		tmp_tx_Head = (tmp_tx_Head + 1) & TX3_BUFFER_MASK;
		
		if(tmp_tx_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#else
		register uint8_t tmp_tx_Head = (tx3_Head + 1) & TX3_BUFFER_MASK; // calculate new position of TX head in buffer
		
		if(tx3_Tail == tmp_tx_Head)
			return BUFFER_FULL;
	#endif
	
		asm volatile("\n\t"
			"mov	r26, %[index]  \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(tx3_buffer)) \n\t"
			"sbci	r27, hi8(-(tx3_buffer)) \n\t"
			"st		X, %[dat] \n\t"
			
			: // outputs
			[index] "+r" (tmp_tx_Head),
			[dat]   "+r" (data)
			: // inputs
			: // clobbers
			"r26","r27"
		);
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tx3_Head = tmp_tx_Head;
			
		#ifdef USART3_RS485_MODE
			RS485_CONTROL3_PORT |= (1<<RS485_CONTROL3_IONUM); // start transmitting
		#endif
			
		#ifdef USART3_USE_SOFT_CTS
			if(!(CTS3_PIN & (1<<CTS3_IONUM)))
		#endif
			{
				UCSR3B_REGISTER |= (1<<UDRIE3_BIT); // enable UDRE interrupt
			}
		}
		return COMPLETED;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart3_putstr(char *string)
	{
		char c;
		while ((c = *string++)) uart3_putc(c);
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart3_putstr(char *string)
	{
		asm volatile("\n\t"
		
		"load_loop_%=:"
			"ld 	r24, Z+ \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	skip_loop_%= \n\t"
			"rcall	uart3_putc \n\t" // Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
		
			: // outputs
			: // inputs
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart3_putstrl(char *string, uint8_t BytesToWrite)
	{
		while(BytesToWrite--)
			uart3_putc(*string++);
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart3_putstrl(char *string, uint8_t BytesToWrite)
	{
		asm volatile("\n\t"
			"add	%[counter], r30 \n\t" // add ZL to a counter to compare against current pointer (8 bit length, doesn't care if overflow)
		"load_loop_%=:"
			"cp 	%[counter], r30\n\t"
			"breq	skip_loop_%= \n\t"
			"ld 	r24, Z+ \n\t"
			"rcall	uart3_putc \n\t" // counter and Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
		
			: // outputs
			: // inputs
			[counter] "r" (BytesToWrite),
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart3_puts_p(const __flash char *string)
	{
		register char c;
		while ( (c = *string++) ) uart3_putc(c); 
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart3_puts_p(const __flash char *string)
	{
		asm volatile("\n\t"
			
		"load_loop_%=:"
			"lpm 	r24, Z+ \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	skip_loop_%= \n\t"
			"rcall	uart3_putc \n\t" // Z pointer will not be affected in uart_putc()
			"rjmp	load_loop_%= \n\t"
		"skip_loop_%=:"
	
			: // outputs
			: // inputs
			"z" (string)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_putc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

	void uart3_putint(int16_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		itoa(data, u_tmp_buff, 10);
		uart3_putstr(u_tmp_buff);
	}

	void uart3_putintr(int16_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		itoa(data, u_tmp_buff, radix);
		uart3_putstr(u_tmp_buff);
	}

	void uart3_putuint(uint16_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		utoa(data, u_tmp_buff, 10);
		uart3_putstr(u_tmp_buff);
	}
	
	void uart3_putuintr(uint16_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		utoa(data, u_tmp_buff, radix);
		uart3_putstr(u_tmp_buff);
	}

	void uart3_puthex(uint8_t data)
	{
		uint8_t tmp; 
		
		tmp = (data >> 4) & 0x0f;
	#ifdef USART_PUTHEX_IN_UPPERCASE
		uart3_putc( (tmp <= 9 ? '0' + tmp : 'A' - 10 + tmp));
	#else
		uart3_putc( (tmp <= 9 ? '0' + tmp : 'a' - 10 + tmp));
	#endif
		
		tmp = data & 0x0f;
	#ifdef USART_PUTHEX_IN_UPPERCASE
		uart3_putc( (tmp <= 9 ? '0' + tmp : 'A' - 10 + tmp));
	#else
		uart3_putc( (tmp <= 9 ? '0' + tmp : 'a' - 10 + tmp));
	#endif
	}

	void uart3_putlong(int32_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
		
		ltoa(data, u_tmp_buff, 10);
		uart3_putstr(u_tmp_buff);
	}
	
	void uart3_putlongr(int32_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		ltoa(data, u_tmp_buff, radix);
		uart3_putstr(u_tmp_buff);
	}
	
	void uart3_putulong(uint32_t data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
		
		ultoa(data, u_tmp_buff, 10);
		uart3_putstr(u_tmp_buff);
	}
	
	void uart3_putulongr(uint32_t data, uint8_t radix)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[17]; // heading, 15 digit bytes, NULL
	#endif
		
		ultoa(data, u_tmp_buff, radix);
		uart3_putstr(u_tmp_buff);
	}

	void uart3_putfloat(float data)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[16];
	#endif
		
		dtostrf(data, 15, 6, u_tmp_buff);
		
		char *p = u_tmp_buff;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart3_putstr(p);
	}

	void uart3_fputfloat(float data, uint8_t precision)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[16];
	#endif
		
		dtostrf(data, 15, precision, u_tmp_buff);
		
		char *p = u_tmp_buff;
		while(*p == ' ') // remove all unwanted spaces
			p++;
		
		uart3_putstr(p);
	}
	
	void uart3_flush(void)
	{
	#ifdef USART3_RS485_MODE // flush UDR buffer
		while (RS485_CONTROL3_PORT & (1<<RS485_CONTROL3_IONUM));
	#else	
		while(tx3_Tail != tx3_Head); // just flush the ring buffer 
	#endif
	}

#ifdef USART3_MPCM_MODE
	void uart3_mpcm_transmit_addres_Frame(uint8_t dat)
	{
		while(tx3_Tail != tx3_Head);
		UCSR3B_REGISTER |= (1<<TXB83_BIT);
		uart3_putc(dat);
		while(tx3_Tail != tx3_Head);
		UCSR3B_REGISTER &= ~(1<<TXB83_BIT); // not sure if necessary
	}
#endif
#endif // NO_TX3_INTERRUPT

#endif // NO_USART_TX

#ifndef NO_USART_RX

#ifndef NO_RX0_INTERRUPT
//******************************************************************
//Function  : To receive single character/byte.
//Arguments : none
//Return    : Received character or NULL if buffer is empty.
//******************************************************************
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	char uart0_getc(void)
	{
		register uint8_t tmp_rx_Tail = rx0_Tail;
		char tmp;
		
		if(tmp_rx_Tail == rx0_Head) 
			return 0;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX0_BUFFER_MASK;
		tmp = rx0_buffer[tmp_rx_Tail];
		
		rx0_Tail = tmp_rx_Tail;
		
	#ifdef USART0_EXTEND_RX_BUFFER
		UCSR0B_REGISTER |= (1<<RXCIE0_BIT);
	#endif
		
	#ifdef USART0_USE_SOFT_RTS
		if (RTS0_PORT & (1<<RTS0_IONUM))
			if (!(UCSR0A_REGISTER & (1<<RXC0_BIT))) // isr has fired so check if there is no unread data in UDR (if missed then next read will release RTS line)
				RTS0_PORT &= ~(1<<RTS0_IONUM);
	#endif
		
	#ifdef RX0_GETC_ECHO
		#ifdef RX_NEWLINE_MODE_N
			if(tmp == '\n')
				uart0_putc('\r');
		#endif
		
		tmp = uart0_putc_(tmp);
		
		#ifdef RX_NEWLINE_MODE_R
			if(tmp == '\r')
				uart0_putc('\n');
		#endif
	#endif // RX0_GETC_ECHO
		
		return tmp;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	char uart0_getc(void)
	{
		register uint8_t tmp_rx_Tail asm("r25");
		char tmp;
		
		tmp_rx_Tail = rx0_Tail;
		
		if(tmp_rx_Tail == rx0_Head) 
			return 0;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX0_BUFFER_MASK;
	
		asm volatile("\n\t"
			"mov	r26, %[index] \n\t"
		
		#if !defined(__AVR_ATtiny2313__)&&!defined(__AVR_ATtiny2313A__) // on ATtiny2313 upper byte in pointer pair is ignored
			"ldi	r27, 0x00 \n\t"
		#endif
			"subi	r26, lo8(-(rx0_buffer)) \n\t"
		
		#ifndef USART_USE_TINY_MEMORY_MODEL
			"sbci	r27, hi8(-(rx0_buffer)) \n\t"
		#endif
			"ld 	%[temp], X \n\t"
			
			: // outputs
			[index] "+r" (tmp_rx_Tail),
			[temp]  "=r" (tmp)
			: // inputs
			: // clobbers
		#if !defined(__AVR_ATtiny2313__)&&!defined(__AVR_ATtiny2313A__)
			"r27",
		#endif
			"r26"
		);
	
		rx0_Tail = tmp_rx_Tail;
	
	#ifdef USART0_EXTEND_RX_BUFFER
		UCSR0B_REGISTER |= (1<<RXCIE0_BIT);
	#endif
	
	#ifdef USART0_USE_SOFT_RTS
		if (RTS0_PORT & (1<<RTS0_IONUM))
			if (!(UCSR0A_REGISTER & (1<<RXC0_BIT))) // isr has fired so check if there is no unread data in UDR (if missed then next read will release RTS line)		
				RTS0_PORT &= ~(1<<RTS0_IONUM);
	#endif
	
	#ifdef RX0_GETC_ECHO
		#ifdef RX_NEWLINE_MODE_N
			if(tmp == '\n')
				uart0_putc('\r');
		#endif
		
		tmp = uart0_putc_(tmp);
		
		#ifdef RX_NEWLINE_MODE_R
			if(tmp == '\r')
				uart0_putc('\n');
		#endif
	#endif // RX0_GETC_ECHO
		
		return tmp;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

//******************************************************************
//Function  : Reads actual string from receiver buffer.
//Arguments : Pointer to array to fill with received string.
//Return    : none
//Note      : Received string will be terminated by NULL.
//          : OBSOLETE - possibility of buffer overflows
//******************************************************************
//	void uart0_getBuffer(char *buffer)
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
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart0_gets(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			*buffer = uart0_getc();
			if(*buffer++ == 0)
				break;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart0_gets(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
		
		"loop_%=:"
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
			"rcall	uart0_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"st 	Z+, r24 \n\t"
			"cpse	r24, __zero_reg__ \n\t"
			"rjmp	loop_%= \n\t"
		"store_NULL_%=:"
			"st 	Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

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
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart0_getln(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			do{
				*buffer = uart0_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart0_getc()) );
			#endif
				break;
			}
			buffer++;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart0_getln(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
		
		"loop_%=:"
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
		"wait_loop_%=:"
			"rcall	uart0_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"and	r24, r24 \n\t" // test for NULL
			"breq	wait_loop_%= \n\t"
		#ifdef RX_NEWLINE_MODE_N
			"cpi	r24, '\n' \n\t"
		#else
			"cpi	r24, '\r' \n\t"
		#endif

		#ifdef RX_NEWLINE_MODE_RN
			"breq	wait_loop2_%= \n\t"
		#else
			"breq	store_NULL_%= \n\t"
		#endif
			
			"st		Z+, r24 \n\t"
			"rjmp	loop_%= \n\t"
		
		#ifdef RX_NEWLINE_MODE_RN
		"wait_loop2_%=:"
			"rcall	uart0_getc \n\t"
			"and	r24, r24 \n\t"
			"breq	wait_loop2_%= \n\t"
		#endif
		
		"store_NULL_%=:"
			"st		Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24"
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

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
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart0_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit)
	{
		do{
			*buffer = uart0_getc();
		}while(*buffer <= 32);
		
		buffer++;
		bufferlimit--;
		
		while(--bufferlimit)
		{
			do{
				*buffer = uart0_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else 
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart0_getc()) );
			#endif
				break;
			}
			else if(*buffer <= 32)
				break; // string reading is done, we will exit

			buffer++;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart0_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
		
		"skip_whitespaces_loop_%=:"
			"rcall	uart0_getc \n\t" // counter and Z pointer will not be affected in uart0_getc()
			"cpi	r24, 0x21\n\t" // if(tmp <= 32)
			"brcs	skip_whitespaces_loop_%= \n\t" // skip all received whitespaces
			"st		Z+, r24 \n\t"
			"dec	%[limit] \n\t"
		
		"loop_%=:"	
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
		"wait_loop_%=:"
			"rcall	uart0_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"and	r24, r24 \n\t" // test for NULL
			"breq	wait_loop_%= \n\t"
		
		#ifdef RX_NEWLINE_MODE_N
			"cpi	r24, '\n' \n\t"
		#else
			"cpi	r24, '\r' \n\t"
		#endif
		
		#ifdef RX_NEWLINE_MODE_RN
			"breq	exit_wait_loop_%= \n\t"
		#else
			"breq	store_NULL_%= \n\t"
		#endif
			
			"cpi	r24, 0x21 \n\t" // if(tmp <= 32)
			"brcs	store_NULL_%= \n\t" // whitespace means end of this function, quit loop
			
			"st		Z+, r24 \n\t"
			"rjmp	loop_%=\n\t"
			
		#ifdef RX_NEWLINE_MODE_RN
		"exit_wait_loop_%=:"
			"rcall	uart0_getc \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	exit_wait_loop_%= \n\t"
		#endif
		
		"store_NULL_%=:"
			"st		Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

//******************************************************************
//Function  : To skip all incoming whitespace characters until first nonspace character.
//Arguments : none
//Return    : First received nonspace character.
//Note      : First nonspace character is cut from receiver buffer.
//******************************************************************
// 	char uart0_skipWhiteSpaces(void)
// 	{
// 		register char c;
// 		
// 		do{
//			c = uart0_getc();
//		}while(c <= 32);
// 		
// 		return c;
// 	}

//******************************************************************
//Function  : Read 16bit integer value from the input stream.
//Arguments : none
//Return    : Received 16bit integer value.
//******************************************************************
	int16_t uart0_getint(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		uart0_getlnToFirstWhiteSpace(u_tmp_buff, 7);
		
		return atoi(u_tmp_buff);
	}

//******************************************************************
//Function  : Read 32bit integer value from the input stream.
//Arguments : none
//Return    : Received 32bit integer value
//******************************************************************
	int32_t uart0_getlong(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
	
		uart0_getlnToFirstWhiteSpace(u_tmp_buff, 12);
		
		return atol(u_tmp_buff);
	}

//******************************************************************
//Function  : Read floating point value from the input stream.
//Arguments : none
//Return    : Received float value.
//******************************************************************
	float uart0_getfloat(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[32];
	#endif
	
		uart0_getlnToFirstWhiteSpace(u_tmp_buff, 32);
		
		return atof(u_tmp_buff);
	}

//******************************************************************
//Function  : To receive single byte in binary transmission.
//Arguments : none
//Return    : Signed 16 bit integer containing data in lower 8 bits 
//Note      : This function doesn't cut CR, LF, NULL terminators
//          : If receiver buffer is empty, return value is negative 
//          : so only sign bit have to be checked (x < 0 // x >= 0)
//******************************************************************
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	int16_t uart0_getData(void)
	{
		register uint8_t tmp_rx_Tail = rx0_Tail;
		uint8_t tmp;
		
		if(tmp_rx_Tail == rx0_Head) 
			return -1;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX0_BUFFER_MASK;
		tmp = rx0_buffer[tmp_rx_Tail];
		rx0_Tail = tmp_rx_Tail;
		
	#ifdef USART0_EXTEND_RX_BUFFER
		UCSR0B_REGISTER |= (1<<RXCIE0_BIT);
	#endif
	
	#ifdef USART0_USE_SOFT_RTS
		if (RTS0_PORT & (1<<RTS0_IONUM))
			if (!(UCSR0A_REGISTER & (1<<RXC0_BIT)))
				RTS0_PORT &= ~(1<<RTS0_IONUM);
	#endif
		
		return tmp;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	int16_t uart0_getData(void)
	{
		register uint8_t tmp_rx_Tail = rx0_Tail;
		uint8_t tmp;
		
		if(tmp_rx_Tail == rx0_Head) 
			return -1;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX0_BUFFER_MASK;
	
		asm volatile("\n\t"
			"mov	r26, %[index] \n\t"
		
		#if !defined(__AVR_ATtiny2313__)&&!defined(__AVR_ATtiny2313A__) // on ATtiny2313 upper byte in pointer pair is ignored
			"ldi	r27, 0x00 \n\t"
		#endif
			"subi	r26, lo8(-(rx0_buffer)) \n\t"
		
		#ifndef USART_USE_TINY_MEMORY_MODEL
			"sbci	r27, hi8(-(rx0_buffer)) \n\t"
		#endif
			"ld 	%[temp], X \n\t"
			
			: // outputs
			[index] "+r" (tmp_rx_Tail),
			[temp]  "=r" (tmp)
			: // inputs
			: // clobbers
		#if !defined(__AVR_ATtiny2313__)&&!defined(__AVR_ATtiny2313A__)	
			"r27",
		#endif
			"r26"
		);
		
		rx0_Tail = tmp_rx_Tail;
		
	#ifdef USART0_EXTEND_RX_BUFFER
		UCSR0B_REGISTER |= (1<<RXCIE0_BIT);
	#endif
	
	#ifdef USART0_USE_SOFT_RTS
		if (RTS0_PORT & (1<<RTS0_IONUM))
			if (!(UCSR0A_REGISTER & (1<<RXC0_BIT)))
				RTS0_PORT &= ~(1<<RTS0_IONUM);
	#endif
		
		return tmp;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

//******************************************************************
//Function  : To receive single byte in binary transmission.
//Arguments : Pointer to byte which have to be filed by incoming data.
//Return    : Status value: 0 = BUFFER_EMPTY, 1 = COMPLETED.
//Note      : This function doesn't cut CR, LF, NULL terminators
//          : If receiver buffer is empty return status = BUFFER_EMPTY instead of returning NULL (as in getc).
//******************************************************************
	uint8_t uart0_LoadData(uint8_t *data)
	{
		register uint8_t tmp_rx_Tail = rx0_Tail;
		
		if(tmp_rx_Tail == rx0_Head) return BUFFER_EMPTY; // result = 0
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX0_BUFFER_MASK;
		*data = rx0_buffer[tmp_rx_Tail];
		
		rx0_Tail = tmp_rx_Tail;
		
	#ifdef USART0_EXTEND_RX_BUFFER
		UCSR0B_REGISTER |= (1<<RXCIE0_BIT);
	#endif
	
	#ifdef USART0_USE_SOFT_RTS
		if (RTS0_PORT & (1<<RTS0_IONUM))
			if (!(UCSR0A_REGISTER & (1<<RXC0_BIT)))	
				RTS0_PORT &= ~(1<<RTS0_IONUM);
	#endif
		
		return COMPLETED; // result = 1
	}

//******************************************************************
//Function  : To check how many bytes are waiting in the receiver buffer.
//Arguments : none
//Return    : Number of bytes waiting in receiver buffer.
//******************************************************************
//	uint8_t uart0_AvailableBytes(void)
//	{
//		return (rx0_Head - rx0_Tail) & RX0_BUFFER_MASK;
//	}
	
//******************************************************************
//Function  : Peek at the next byte in buffer.
//Arguments : none
//Return    : Next byte in buffer.
//******************************************************************
	uint8_t uart0_peek(void)
	{
		return rx0_buffer[(rx0_Tail+1) & RX0_BUFFER_MASK];
	}
#endif // NO_RX0_INTERRUPT

#ifndef NO_RX1_INTERRUPT

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	char uart1_getc(void)
	{
		register uint8_t tmp_rx_Tail = rx1_Tail;
		char tmp;
		
		if(tmp_rx_Tail == rx1_Head) 
			return 0;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX1_BUFFER_MASK;
		tmp = rx1_buffer[tmp_rx_Tail];
		
		rx1_Tail = tmp_rx_Tail;
		
	#ifdef USART1_EXTEND_RX_BUFFER
		UCSR1B_REGISTER |= (1<<RXCIE1_BIT);
	#endif
		
	#ifdef USART1_USE_SOFT_RTS
		if (RTS1_PORT & (1<<RTS1_IONUM))
			if (!(UCSR1A_REGISTER & (1<<RXC1_BIT))) // isr has fired so check if there is no unread data in UDR (if missed then next read will release RTS line)
				RTS1_PORT &= ~(1<<RTS1_IONUM);
	#endif
		
	#ifdef RX1_GETC_ECHO
		#ifdef RX_NEWLINE_MODE_N
			if(tmp == '\n')
				uart1_putc('\r');
		#endif
		
		tmp = uart1_putc_(tmp);
		
		#ifdef RX_NEWLINE_MODE_R
			if(tmp == '\r')
				uart1_putc('\n');
		#endif
	#endif // RX1_GETC_ECHO
		
		return tmp;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	char uart1_getc(void)
	{
		register uint8_t tmp_rx_Tail asm("r25");
		char tmp;
		
		tmp_rx_Tail = rx1_Tail;
		
		if(tmp_rx_Tail == rx1_Head) 
			return 0;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX1_BUFFER_MASK;
	
		asm volatile("\n\t"
			"mov	r26, %[index] \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(rx1_buffer)) \n\t"
			"sbci	r27, hi8(-(rx1_buffer)) \n\t"
			"ld 	%[temp], X \n\t"
			
			: // outputs
			[index] "+r" (tmp_rx_Tail),
			[temp] "=r" (tmp)
			: // inputs
			: // clobbers
			"r26","r27"
		);
	
		rx1_Tail = tmp_rx_Tail;
	
	#ifdef USART1_EXTEND_RX_BUFFER
		UCSR1B_REGISTER |= (1<<RXCIE1_BIT);
	#endif
	
	#ifdef USART1_USE_SOFT_RTS
		if (RTS1_PORT & (1<<RTS1_IONUM))
			if (!(UCSR1A_REGISTER & (1<<RXC1_BIT))) // isr has fired so check if there is no unread data in UDR (if missed then next read will release RTS line)		
				RTS1_PORT &= ~(1<<RTS1_IONUM);
	#endif
	
	#ifdef RX1_GETC_ECHO
		#ifdef RX_NEWLINE_MODE_N
			if(tmp == '\n')
				uart1_putc('\r');
		#endif
		
		tmp = uart1_putc_(tmp);
		
		#ifdef RX_NEWLINE_MODE_R
			if(tmp == '\r') 
				uart1_putc('\n');
		#endif
	#endif // RX1_GETC_ECHO
		
		return tmp;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart1_gets(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			*buffer = uart1_getc();
			if(*buffer++ == 0)
				break;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart1_gets(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
		
		"loop_%=:"
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
			"rcall	uart1_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"st 	Z+, r24 \n\t"
			"cpse	r24, __zero_reg__ \n\t"
			"rjmp	loop_%= \n\t"
		"store_NULL_%=:"
			"st 	Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES
	
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart1_getln(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			do{
				*buffer = uart1_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart1_getc()) );
			#endif
				break;
			}
			buffer++;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart1_getln(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
		
		"loop_%=:"
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
		"wait_loop_%=:"
			"rcall	uart1_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"and	r24, r24 \n\t" // test for NULL
			"breq	wait_loop_%= \n\t"
		#ifdef RX_NEWLINE_MODE_N
			"cpi	r24, '\n' \n\t"
		#else
			"cpi	r24, '\r' \n\t"
		#endif

		#ifdef RX_NEWLINE_MODE_RN
			"breq	wait_loop2_%= \n\t"
		#else
			"breq	store_NULL_%= \n\t"
		#endif
			
			"st		Z+, r24 \n\t"
			"rjmp	loop_%= \n\t"
		
		#ifdef RX_NEWLINE_MODE_RN
		"wait_loop2_%=:"
			"rcall	uart1_getc \n\t"
			"and	r24, r24 \n\t"
			"breq	wait_loop2_%= \n\t"
		#endif
		
		"store_NULL_%=:"
			"st		Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart1_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit)
	{
		do{
			*buffer = uart1_getc();
		}while(*buffer <= 32);
		
		buffer++;
		bufferlimit--;
		
		while(--bufferlimit)
		{
			do{
				*buffer = uart1_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else //RX_NEWLINE_MODE_R
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart1_getc()) );
			#endif
				break;
			}
			else if(*buffer <= 32)
				break; // string reading is done, we will exit

			buffer++;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart1_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
			
		"skip_whitespaces_loop_%=:"
			"rcall	uart1_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"cpi	r24, 0x21\n\t" // if(tmp <= 32)
			"brcs	skip_whitespaces_loop_%= \n\t" // skip all received whitespaces
			"st		Z+, r24 \n\t"
			"dec	%[limit] \n\t"
		
		"loop_%=:"	
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
		"wait_loop_%=:"
			"rcall	uart1_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"and	r24, r24 \n\t" // test for NULL
			"breq	wait_loop_%= \n\t"
		
		#ifdef RX_NEWLINE_MODE_N
			"cpi	r24, '\n' \n\t"
		#else
			"cpi	r24, '\r' \n\t"
		#endif
		
		#ifdef RX_NEWLINE_MODE_RN
			"breq	exit_wait_loop_%= \n\t"
		#else
			"breq	store_NULL_%= \n\t"
		#endif
			
			"cpi	r24, 0x21 \n\t" // if(tmp <= 32)
			"brcs	store_NULL_%= \n\t" // whitespace means end of this function, quit loop
			
			"st		Z+, r24 \n\t"
			"rjmp	loop_%=\n\t"
			
		#ifdef RX_NEWLINE_MODE_RN
		"exit_wait_loop_%=:"
			"rcall	uart1_getc \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	exit_wait_loop_%= \n\t"
		#endif
		
		"store_NULL_%=:"
			"st		Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

	int16_t uart1_getint(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		uart1_getlnToFirstWhiteSpace(u_tmp_buff, 7);
		
		return atoi(u_tmp_buff);
	}

	int32_t uart1_getlong(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
	
		uart1_getlnToFirstWhiteSpace(u_tmp_buff, 12);
		
		return atol(u_tmp_buff);
	}

	float uart1_getfloat(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[32];
	#endif
	
		uart1_getlnToFirstWhiteSpace(u_tmp_buff, 32);
		
		return atof(u_tmp_buff);
	}

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	int16_t uart1_getData(void)
	{
		register uint8_t tmp_rx_Tail = rx1_Tail;
		uint8_t tmp;
		
		if(tmp_rx_Tail == rx1_Head) 
			return -1;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX1_BUFFER_MASK;
		tmp = rx1_buffer[tmp_rx_Tail];
		rx1_Tail = tmp_rx_Tail;
		
	#ifdef USART1_EXTEND_RX_BUFFER
		UCSR1B_REGISTER |= (1<<RXCIE1_BIT);
	#endif
	
	#ifdef USART1_USE_SOFT_RTS
		if (RTS1_PORT & (1<<RTS1_IONUM))
			if (!(UCSR1A_REGISTER & (1<<RXC1_BIT)))
				RTS1_PORT &= ~(1<<RTS1_IONUM);
	#endif
		
		return tmp;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	int16_t uart1_getData(void)
	{
		register uint8_t tmp_rx_Tail = rx1_Tail;
		uint8_t tmp;
		
		if(tmp_rx_Tail == rx1_Head) 
			return -1;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX1_BUFFER_MASK;
	
		asm volatile("\n\t"
			"mov	r26, %[index] \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(rx1_buffer)) \n\t"
			"sbci	r27, hi8(-(rx1_buffer)) \n\t"
			"ld 	%[temp], X \n\t"
			
			: // outputs
			[index] "+r" (tmp_rx_Tail),
			[temp]  "=r" (tmp)
			: // inputs
			: // clobbers
			"r26","r27"
		);
		
		rx1_Tail = tmp_rx_Tail;
		
	#ifdef USART1_EXTEND_RX_BUFFER
		UCSR1B_REGISTER |= (1<<RXCIE1_BIT);
	#endif
	
	#ifdef USART1_USE_SOFT_RTS
		if (RTS1_PORT & (1<<RTS1_IONUM))
			if (!(UCSR1A_REGISTER & (1<<RXC1_BIT)))
				RTS1_PORT &= ~(1<<RTS1_IONUM);
	#endif
		
		return tmp;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

	uint8_t uart1_LoadData(uint8_t *data)
	{
		register uint8_t tmp_rx_Tail = rx1_Tail;
		
		if(tmp_rx_Tail == rx1_Head) return BUFFER_EMPTY; // result = 0
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX1_BUFFER_MASK;
		*data = rx1_buffer[tmp_rx_Tail];
		
		rx1_Tail = tmp_rx_Tail;
		
	#ifdef USART1_EXTEND_RX_BUFFER
		UCSR1B_REGISTER |= (1<<RXCIE1_BIT);
	#endif
	
	#ifdef USART1_USE_SOFT_RTS
		if (RTS1_PORT & (1<<RTS1_IONUM))
			if (!(UCSR1A_REGISTER & (1<<RXC1_BIT)))
				RTS1_PORT &= ~(1<<RTS1_IONUM);
	#endif
		
		return COMPLETED; // result = 1
	}

	//uint8_t uart1_AvailableBytes(void)
	//{
	//	return (rx1_Head - rx1_Tail) & RX1_BUFFER_MASK;
	//}
	
	uint8_t uart1_peek(void)
	{
		return rx1_buffer[(rx1_Tail+1) & RX1_BUFFER_MASK];
	}
#endif // NO_RX1_INTERRUPT

#ifndef NO_RX2_INTERRUPT
	
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	char uart2_getc(void)
	{
		register uint8_t tmp_rx_Tail = rx2_Tail;
		char tmp;
		
		if(tmp_rx_Tail == rx2_Head)
			return 0;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX2_BUFFER_MASK;
		tmp = rx2_buffer[tmp_rx_Tail];
		
		rx2_Tail = tmp_rx_Tail;
		
	#ifdef USART2_EXTEND_RX_BUFFER
		UCSR2B_REGISTER |= (1<<RXCIE2_BIT);
	#endif
		
	#ifdef USART2_USE_SOFT_RTS
		if (RTS2_PORT & (1<<RTS2_IONUM))
			if (!(UCSR2A_REGISTER & (1<<RXC2_BIT))) // isr has fired so check if there is no unread data in UDR (if missed then next read will release RTS line)
				RTS2_PORT &= ~(1<<RTS2_IONUM);
	#endif
		
	#ifdef RX2_GETC_ECHO
		#ifdef RX_NEWLINE_MODE_N
			if(tmp == '\n')
				uart2_putc('\r');
		#endif
		
		tmp = uart2_putc_(tmp);
		
		#ifdef RX_NEWLINE_MODE_R
			if(tmp == '\r')
				uart2_putc('\n');
		#endif
	#endif // RX2_GETC_ECHO
		
		return tmp;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	char uart2_getc(void)
	{
		register uint8_t tmp_rx_Tail asm("r25");
		char tmp;
		
		tmp_rx_Tail = rx2_Tail;
		
		if(tmp_rx_Tail == rx2_Head) 
			return 0;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX2_BUFFER_MASK;
	
		asm volatile("\n\t"
			"mov	r26, %[index] \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(rx2_buffer)) \n\t"
			"sbci	r27, hi8(-(rx2_buffer)) \n\t"
			"ld 	%[temp], X \n\t"
			
			: // outputs
			[index] "+r" (tmp_rx_Tail),
			[temp]  "=r" (tmp)
			: // inputs
			: // clobbers
			"r26","r27"
		);
	
		rx2_Tail = tmp_rx_Tail;
	
	#ifdef USART2_EXTEND_RX_BUFFER
		UCSR2B_REGISTER |= (1<<RXCIE2_BIT);
	#endif
	
	#ifdef USART2_USE_SOFT_RTS
		if (RTS2_PORT & (1<<RTS2_IONUM))
			if (!(UCSR2A_REGISTER & (1<<RXC2_BIT))) // isr has fired so check if there is no unread data in UDR (if missed then next read will release RTS line)		
				RTS2_PORT &= ~(1<<RTS2_IONUM);
	#endif
	
	#ifdef RX2_GETC_ECHO
		#ifdef RX_NEWLINE_MODE_N
			if(tmp == '\n')
				uart2_putc('\r');
		#endif
		
		tmp = uart2_putc_(tmp);
		
		#ifdef RX_NEWLINE_MODE_R
			if(tmp == '\r') 
				uart2_putc('\n');
		#endif
	#endif // RX2_GETC_ECHO
		
		return tmp;
	}
#endif

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart2_gets(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			*buffer = uart2_getc();
			if(*buffer++ == 0)
				break;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart2_gets(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
		
		"loop_%=:"
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
			"rcall	uart2_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"st 	Z+, r24 \n\t"
			"cpse	r24, __zero_reg__ \n\t"
			"rjmp	loop_%= \n\t"
		"store_NULL_%=:"
			"st 	Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES
	
#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart2_getln(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			do{
				*buffer = uart2_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart2_getc()) );
			#endif
				break;
			}
			buffer++;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart2_getln(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
			
		"loop_%=:"
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
		"wait_loop_%=:"
			"rcall	uart2_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"and	r24, r24 \n\t" // test for NULL
			"breq	wait_loop_%= \n\t"
		#ifdef RX_NEWLINE_MODE_N
			"cpi	r24, '\n' \n\t"
		#else
			"cpi	r24, '\r' \n\t"
		#endif

		#ifdef RX_NEWLINE_MODE_RN
			"breq	wait_loop2_%= \n\t"
		#else
			"breq	store_NULL_%= \n\t"
		#endif
			
			"st		Z+, r24 \n\t"
			"rjmp	loop_%= \n\t"
		
		#ifdef RX_NEWLINE_MODE_RN
		"wait_loop2_%=:"
			"rcall	uart2_getc \n\t"
			"and	r24, r24 \n\t"
			"breq	wait_loop2_%= \n\t"
		#endif
		
		"store_NULL_%=:"
			"st		Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart2_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit)
	{
		do{
			*buffer = uart2_getc();
		}while(*buffer <= 32);
		
		buffer++;
		bufferlimit--;
		
		while(--bufferlimit)
		{
			do{
				*buffer = uart2_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else //RX_NEWLINE_MODE_R
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart2_getc()) );
			#endif
				break;
			}
			else if(*buffer <= 32)
				break; // string reading is done, we will exit

			buffer++;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart2_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
			
		"skip_whitespaces_loop_%=:"
			"rcall	uart2_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"cpi	r24, 0x21\n\t" // if(tmp <= 32)
			"brcs	skip_whitespaces_loop_%= \n\t" // skip all received whitespaces
			"st		Z+, r24 \n\t"
			"dec	%[limit] \n\t"
		
		"loop_%=:"	
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
		"wait_loop_%=:"
			"rcall	uart2_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"and	r24, r24 \n\t" // test for NULL
			"breq	wait_loop_%= \n\t"
		
		#ifdef RX_NEWLINE_MODE_N
			"cpi	r24, '\n' \n\t"
		#else
			"cpi	r24, '\r' \n\t"
		#endif
		
		#ifdef RX_NEWLINE_MODE_RN
			"breq	exit_wait_loop_%= \n\t"
		#else
			"breq	store_NULL_%= \n\t"
		#endif
			
			"cpi	r24, 0x21 \n\t" // if(tmp <= 32)
			"brcs	store_NULL_%= \n\t" // whitespace means end of this function, quit loop
			
			"st		Z+, r24 \n\t"
			"rjmp	loop_%=\n\t"
			
		#ifdef RX_NEWLINE_MODE_RN
		"exit_wait_loop_%=:"
			"rcall	uart2_getc \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	exit_wait_loop_%= \n\t"
		#endif
		
		"store_NULL_%=:"
			"st		Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

	int16_t uart2_getint(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		uart2_getlnToFirstWhiteSpace(u_tmp_buff, 7);
		
		return atoi(u_tmp_buff);
	}

	int32_t uart2_getlong(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
	
		uart2_getlnToFirstWhiteSpace(u_tmp_buff, 12);
		
		return atol(u_tmp_buff);
	}

	float uart2_getfloat(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[32];
	#endif
	
		uart2_getlnToFirstWhiteSpace(u_tmp_buff, 32);
		
		return atof(u_tmp_buff);
	}

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	int16_t uart2_getData(void)
	{
		register uint8_t tmp_rx_Tail = rx2_Tail;
		uint8_t tmp;
		
		if(tmp_rx_Tail == rx2_Head) 
			return -1;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX2_BUFFER_MASK;
		tmp = rx2_buffer[tmp_rx_Tail];
		rx2_Tail = tmp_rx_Tail;
		
	#ifdef USART2_EXTEND_RX_BUFFER
		UCSR2B_REGISTER |= (1<<RXCIE2_BIT);
	#endif
	
	#ifdef USART2_USE_SOFT_RTS
		if (RTS2_PORT & (1<<RTS2_IONUM))
			if (!(UCSR2A_REGISTER & (1<<RXC2_BIT)))
				RTS2_PORT &= ~(1<<RTS2_IONUM);
	#endif
		
		return tmp;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES

	int16_t uart2_getData(void)
	{
		register uint8_t tmp_rx_Tail = rx2_Tail;
		uint8_t tmp;
		
		if(tmp_rx_Tail == rx2_Head)
			return -1;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX2_BUFFER_MASK;
	
		asm volatile("\n\t"
			"mov	r26, %[index] \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(rx2_buffer)) \n\t"
			"sbci	r27, hi8(-(rx2_buffer)) \n\t"
			"ld 	%[temp], X \n\t"
			
			: // outputs
			[index] "+r" (tmp_rx_Tail),
			[temp] "=r" (tmp)
			: // inputs
			: // clobbers
			"r26","r27"
		);
		
		rx2_Tail = tmp_rx_Tail;
		
	#ifdef USART2_EXTEND_RX_BUFFER
		UCSR2B_REGISTER |= (1<<RXCIE2_BIT);
	#endif
	
	#ifdef USART2_USE_SOFT_RTS
		if (RTS2_PORT & (1<<RTS2_IONUM))
			if (!(UCSR2A_REGISTER & (1<<RXC2_BIT)))
				RTS2_PORT &= ~(1<<RTS2_IONUM);
	#endif
		
		return tmp;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

	uint8_t uart2_LoadData(uint8_t *data)
	{
		register uint8_t tmp_rx_Tail = rx2_Tail;
		
		if(tmp_rx_Tail == rx2_Head) return BUFFER_EMPTY; // result = 0
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX2_BUFFER_MASK;
		*data = rx2_buffer[tmp_rx_Tail];
		
		rx2_Tail = tmp_rx_Tail;
		
	#ifdef USART2_EXTEND_RX_BUFFER
		UCSR2B_REGISTER |= (1<<RXCIE2_BIT);
	#endif
	
	#ifdef USART2_USE_SOFT_RTS
		if (RTS2_PORT & (1<<RTS2_IONUM))
			if (!(UCSR2A_REGISTER & (1<<RXC2_BIT)))
				RTS2_PORT &= ~(1<<RTS2_IONUM);
	#endif
		
		return COMPLETED; // result = 1
	}

	//uint8_t uart2_AvailableBytes(void)
	//{
	//	return (rx2_Head - rx2_Tail) & RX2_BUFFER_MASK;
	//}
	
	uint8_t uart2_peek(void)
	{
		return rx2_buffer[(rx2_Tail+1) & RX2_BUFFER_MASK];
	}
#endif // NO_RX2_INTERRUPT

#ifndef NO_RX3_INTERRUPT

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	char uart3_getc(void)
	{
		register uint8_t tmp_rx_Tail = rx3_Tail;
		char tmp;
		
		if(tmp_rx_Tail == rx3_Head) 
			return 0;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX3_BUFFER_MASK;
		tmp = rx3_buffer[tmp_rx_Tail];
		
		rx3_Tail = tmp_rx_Tail;
		
	#ifdef USART3_EXTEND_RX_BUFFER
		UCSR3B_REGISTER |= (1<<RXCIE3_BIT);
	#endif
		
	#ifdef USART3_USE_SOFT_RTS
		if (RTS3_PORT & (1<<RTS3_IONUM))
			if (!(UCSR3A_REGISTER & (1<<RXC3_BIT))) // isr has fired so check if there is no unread data in UDR (if missed then next read will release RTS line)
				RTS3_PORT &= ~(1<<RTS3_IONUM);
	#endif
		
	#ifdef RX3_GETC_ECHO
		#ifdef RX_NEWLINE_MODE_N
			if(tmp == '\n')
				uart3_putc('\r');
		#endif
		
		tmp = uart3_putc_(tmp);
		
		#ifdef RX_NEWLINE_MODE_R
			if(tmp == '\r')
				uart3_putc('\n');
		#endif
	#endif // RX3_GETC_ECHO
		
		return tmp;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	char uart3_getc(void)
	{
		register uint8_t tmp_rx_Tail asm("r25");
		char tmp;
		
		tmp_rx_Tail = rx3_Tail;
		
		if(tmp_rx_Tail == rx3_Head) 
			return 0;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX3_BUFFER_MASK;
	
		asm volatile("\n\t"
			"mov	r26, %[index] \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(rx3_buffer)) \n\t"
			"sbci	r27, hi8(-(rx3_buffer)) \n\t"
			"ld 	%[temp], X \n\t"
			
			: // outputs
			[index] "+r" (tmp_rx_Tail),
			[temp]  "=r" (tmp)
			: // inputs
			: // clobbers
			"r26","r27"
		);
	
		rx3_Tail = tmp_rx_Tail;
	
	#ifdef USART3_EXTEND_RX_BUFFER
		UCSR3B_REGISTER |= (1<<RXCIE3_BIT);
	#endif
	
	#ifdef USART3_USE_SOFT_RTS
		if (RTS3_PORT & (1<<RTS3_IONUM))
			if (!(UCSR3A_REGISTER & (1<<RXC3_BIT))) // isr has fired so check if there is no unread data in UDR (if missed then next read will release RTS line)		
				RTS3_PORT &= ~(1<<RTS3_IONUM);
	#endif
	
	#ifdef RX3_GETC_ECHO
		#ifdef RX_NEWLINE_MODE_N
			if(tmp == '\n')
				uart3_putc('\r');
		#endif
		
		tmp = uart3_putc_(tmp);
		
		#ifdef RX_NEWLINE_MODE_R
			if(tmp == '\r') 
				uart3_putc('\n');
		#endif
	#endif // RX3_GETC_ECHO
		
		return tmp;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart3_gets(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			*buffer = uart3_getc();
			if(*buffer++ == 0)
				break;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart3_gets(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
		
		"loop_%=:"
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
			"rcall	uart3_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"st 	Z+, r24 \n\t"
			"cpse	r24, __zero_reg__ \n\t"
			"rjmp	loop_%= \n\t"
		"store_NULL_%=:"
			"st 	Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart3_getln(char *buffer, uint8_t bufferlimit)
	{
		while(--bufferlimit)
		{
			do{
				*buffer = uart3_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart3_getc()) );
			#endif
				break;
			}
			buffer++;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart3_getln(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
			
		"loop_%=:"
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
		"wait_loop_%=:"
			"rcall	uart3_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"and	r24, r24 \n\t" // test for NULL
			"breq	wait_loop_%= \n\t"
		#ifdef RX_NEWLINE_MODE_N
			"cpi	r24, '\n' \n\t"
		#else
			"cpi	r24, '\r' \n\t"
		#endif

		#ifdef RX_NEWLINE_MODE_RN
			"breq	wait_loop2_%= \n\t"
		#else
			"breq	store_NULL_%= \n\t"
		#endif
			
			"st		Z+, r24 \n\t"
			"rjmp	loop_%= \n\t"
		
		#ifdef RX_NEWLINE_MODE_RN
		"wait_loop2_%=:"
			"rcall	uart3_getc \n\t"
			"and	r24, r24 \n\t"
			"breq	wait_loop2_%= \n\t"
		#endif
		
		"store_NULL_%=:"
			"st		Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	void uart3_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit)
	{
		do{
			*buffer = uart3_getc();
		}while(*buffer <= 32);
		
		buffer++;
		bufferlimit--;
		
		while(--bufferlimit)
		{
			do{
				*buffer = uart3_getc();
			}while(*buffer == 0);
			
		#ifdef RX_NEWLINE_MODE_N
			if(*buffer == '\n')
		#else //RX_NEWLINE_MODE_R
			if(*buffer == '\r')
		#endif
			{
			#ifdef RX_NEWLINE_MODE_RN
				while( !(uart3_getc()) );
			#endif
				break;
			}
			else if(*buffer <= 32)
				break; // string reading is done, we will exit

			buffer++;
		}
		*buffer = 0;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	void uart3_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit)
	{
		asm volatile("\n\t"
			
		"skip_whitespaces_loop_%=:"
			"rcall	uart3_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"cpi	r24, 0x21\n\t" // if(tmp <= 32)
			"brcs	skip_whitespaces_loop_%= \n\t" // skip all received whitespaces
			"st		Z+, r24 \n\t"
			"dec	%[limit] \n\t"
		
		"loop_%=:"	
			"dec	%[limit] \n\t"
			"breq	store_NULL_%= \n\t" // buffer limit hit, quit loop
		"wait_loop_%=:"
			"rcall	uart3_getc \n\t" // counter and Z pointer will not be affected in uart_getc()
			"and	r24, r24 \n\t" // test for NULL
			"breq	wait_loop_%= \n\t"
		
		#ifdef RX_NEWLINE_MODE_N
			"cpi	r24, '\n' \n\t"
		#else
			"cpi	r24, '\r' \n\t"
		#endif
		
		#ifdef RX_NEWLINE_MODE_RN
			"breq	exit_wait_loop_%= \n\t"
		#else
			"breq	store_NULL_%= \n\t"
		#endif
			
			"cpi	r24, 0x21 \n\t" // if(tmp <= 32)
			"brcs	store_NULL_%= \n\t" // whitespace means end of this function, quit loop
			
			"st		Z+, r24 \n\t"
			"rjmp	loop_%=\n\t"
			
		#ifdef RX_NEWLINE_MODE_RN
		"exit_wait_loop_%=:"
			"rcall	uart3_getc \n\t"
			"and	r24, r24 \n\t" // test for NULL
			"breq	exit_wait_loop_%= \n\t"
		#endif
		
		"store_NULL_%=:"
			"st		Z, __zero_reg__ \n\t"
		
			: // outputs
			: // inputs
			"z" (buffer),
			[limit] "r" (bufferlimit)
			: // clobbers
			"r24",
			"r25","r26","r27" // uart_getc()
		);
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

	int16_t uart3_getint(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[7]; // heading, 5 digit bytes, NULL
	#endif
	
		uart3_getlnToFirstWhiteSpace(u_tmp_buff, 7);
		
		return atoi(u_tmp_buff);
	}

	int32_t uart3_getlong(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[12]; // heading, 10 digit bytes, NULL
	#endif
	
		uart3_getlnToFirstWhiteSpace(u_tmp_buff, 12);
		
		return atol(u_tmp_buff);
	}

	float uart3_getfloat(void)
	{
	#ifndef USART_NO_LOCAL_BUFFERS
		char u_tmp_buff[32];
	#endif
	
		uart3_getlnToFirstWhiteSpace(u_tmp_buff, 32);
		
		return atof(u_tmp_buff);
	}

#ifdef USART_NO_ABI_BREAKING_PREMATURES
	int16_t uart3_getData(void)
	{
		register uint8_t tmp_rx_Tail = rx3_Tail;
		uint8_t tmp;
		
		if(tmp_rx_Tail == rx3_Head) 
			return -1;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX3_BUFFER_MASK;
		tmp = rx3_buffer[tmp_rx_Tail];
		rx3_Tail = tmp_rx_Tail;
		
	#ifdef USART3_EXTEND_RX_BUFFER
		UCSR3B_REGISTER |= (1<<RXCIE3_BIT);
	#endif
	
	#ifdef USART3_USE_SOFT_RTS
		if (RTS3_PORT & (1<<RTS3_IONUM))
			if (!(UCSR3A_REGISTER & (1<<RXC3_BIT)))
				RTS3_PORT &= ~(1<<RTS3_IONUM);
	#endif
		
		return tmp;
	}
#else // !USART_NO_ABI_BREAKING_PREMATURES
	int16_t uart3_getData(void)
	{
		register uint8_t tmp_rx_Tail = rx3_Tail;
		uint8_t tmp;
		
		if(tmp_rx_Tail == rx3_Head) 
			return -1;
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX3_BUFFER_MASK;
	
		asm volatile("\n\t"
			"mov	r26, %[index] \n\t"
			"ldi	r27, 0x00 \n\t"
			"subi	r26, lo8(-(rx3_buffer)) \n\t"
			"sbci	r27, hi8(-(rx3_buffer)) \n\t"
			"ld 	%[temp], X \n\t"
			
			: // outputs
			[index] "+r" (tmp_rx_Tail),
			[temp] "=r" (tmp)
			: // inputs
			: // clobbers
			"r26","r27"
		);
		
		rx3_Tail = tmp_rx_Tail;
		
	#ifdef USART3_EXTEND_RX_BUFFER
		UCSR3B_REGISTER |= (1<<RXCIE3_BIT);
	#endif
	
	#ifdef USART3_USE_SOFT_RTS
		if (RTS3_PORT & (1<<RTS3_IONUM))
			if (!(UCSR3A_REGISTER & (1<<RXC3_BIT)))
				RTS3_PORT &= ~(1<<RTS3_IONUM);
	#endif
		
		return tmp;
	}
#endif // USART_NO_ABI_BREAKING_PREMATURES

	uint8_t uart3_LoadData(uint8_t *data)
	{
		register uint8_t tmp_rx_Tail = rx3_Tail;
		
		if(tmp_rx_Tail == rx3_Head) return BUFFER_EMPTY; // result = 0
		
		tmp_rx_Tail = (tmp_rx_Tail+1) & RX3_BUFFER_MASK;
		*data = rx3_buffer[tmp_rx_Tail];
		
		rx3_Tail = tmp_rx_Tail;
		
	#ifdef USART3_EXTEND_RX_BUFFER
		UCSR3B_REGISTER |= (1<<RXCIE3_BIT);
	#endif
	
	#ifdef USART3_USE_SOFT_RTS
		if (RTS3_PORT & (1<<RTS3_IONUM))
			if (!(UCSR3A_REGISTER & (1<<RXC3_BIT)))
				RTS3_PORT &= ~(1<<RTS3_IONUM);
	#endif
		
		return COMPLETED; // result = 1
	}

	//uint8_t uart3_AvailableBytes(void)
	//{
	//	return (rx3_Head - rx3_Tail) & RX3_BUFFER_MASK;
	//}
	
	uint8_t uart3_peek(void)
	{
		return rx3_buffer[(rx3_Tail+1) & RX3_BUFFER_MASK];
	}
#endif // NO_RX3_INTERRUPT

#endif // NO_USART_RX

/************************************************************************************
 *                           stdio.h stuff                                          *
 ************************************************************************************/

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)

	#ifndef NO_USART_TX
		
		int uart_putchar(char data, FILE *stream)
		{
			switch((uint16_t) stream -> udata)
			{
				default:
			#ifndef NO_TX0_INTERRUPT
				case 0: 
					if (data == '\n') 
						uart0_putc('\r'); 
					uart0_putc(data); break;
			#endif
			#ifndef NO_TX1_INTERRUPT
				case 1: 
					if (data == '\n')
						uart1_putc('\r');
					uart1_putc(data); break;
			#endif
			#ifndef NO_TX2_INTERRUPT
				case 2: 
					if (data == '\n')
						uart2_putc('\r');
					uart2_putc(data); break;
			#endif
			#ifndef NO_TX3_INTERRUPT
				case 3: 
					if (data == '\n')
						uart3_putc('\r');
					uart3_putc(data); break;
			#endif
			}
			return 0;
		}
		
	#endif // NO_USART_TX

	#ifndef NO_USART_RX
		
		int uart_getchar(FILE *stream)
		{
			int16_t tmp;
			
			switch((uint16_t) stream -> udata)
			{
				default:
			#ifndef NO_RX0_INTERRUPT
				case 0: 
					while ( (tmp = uart0_getData()) < 0 );
						
				#ifdef RX_STDIO_GETCHAR_ECHO
					tmp = uart0_putc_((uint8_t)tmp);
				#endif
					break;
			#endif
			#ifndef NO_RX1_INTERRUPT
				case 1: 
					while ( (tmp = uart1_getData()) < 0 );
						
				#ifdef RX_STDIO_GETCHAR_ECHO
					tmp = uart1_putc_((uint8_t)tmp);
				#endif
					break;
			#endif
			#ifndef NO_RX2_INTERRUPT
				case 2: 
					while ( (tmp = uart2_getData()) < 0 );
						
				#ifdef RX_STDIO_GETCHAR_ECHO
					tmp = uart2_putc_((uint8_t)tmp);
				#endif
					break;
			#endif
			#ifndef NO_RX3_INTERRUPT
				case 3: 
					while ( (tmp = uart3_getData()) < 0 );
						
				#ifdef RX_STDIO_GETCHAR_ECHO
					tmp = uart3_putc_((uint8_t)tmp);
				#endif
					break;
			#endif
			}
			
			return (uint8_t)tmp;
		}
		
	#endif // NO_USART_RX

	#ifdef USE_USART0
	
		#if defined(NO_RX0_INTERRUPT)
			FILE uart0_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, (void*)0);
	
		#elif defined(NO_TX0_INTERRUPT)
			FILE uart0_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, (void*)0);
		#else
			FILE uart0_io = FDEV_SETUP_STREAM_U(uart_putchar, uart_getchar, _FDEV_SETUP_RW, (void*)0);
			FILE uart0_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, (void*)0);
			FILE uart0_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, (void*)0);
		#endif
	
	#endif // USE_USART0
	
	#ifdef USE_USART1
	
		#if defined(NO_RX1_INTERRUPT)
			FILE uart1_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, (void*)1);
	
		#elif defined(NO_TX1_INTERRUPT)
			FILE uart1_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, (void*)1);
		#else
			FILE uart1_io = FDEV_SETUP_STREAM_U(uart_putchar, uart_getchar, _FDEV_SETUP_RW, (void*)1);
			FILE uart1_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, (void*)1);
			FILE uart1_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, (void*)1);
		#endif
	
	#endif // USE_USART1
	
	#ifdef USE_USART2
	
		#if defined(NO_RX2_INTERRUPT)
			FILE uart2_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, (void*)2);
	
		#elif defined(NO_TX2_INTERRUPT)
			FILE uart2_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, (void*)2);
		#else
			FILE uart2_io = FDEV_SETUP_STREAM_U(uart_putchar, uart_getchar, _FDEV_SETUP_RW, (void*)2);
			FILE uart2_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, (void*)2);
			FILE uart2_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, (void*)2);
		#endif
	
	#endif // USE_USART2
	
	#ifdef USE_USART3
	
		#if defined(NO_RX3_INTERRUPT)
			FILE uart3_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, (void*)3);
	
		#elif defined(NO_TX3_INTERRUPT)
			FILE uart3_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, (void*)3);
		#else
			FILE uart3_io = FDEV_SETUP_STREAM_U(uart_putchar, uart_getchar, _FDEV_SETUP_RW, (void*)3);
			FILE uart3_in = FDEV_SETUP_STREAM_U(NULL, uart_getchar, _FDEV_SETUP_READ, (void*)3);
			FILE uart3_out = FDEV_SETUP_STREAM_U(uart_putchar, NULL, _FDEV_SETUP_WRITE, (void*)3);
		#endif
	
	#endif // USE_USART3

#else // single USART mcu

	#ifndef NO_TX0_INTERRUPT
		
		int uart_putchar(char data, FILE *stream)
		{
			if (data == '\n') uart0_putc('\r');
		
			uart_putc(data);
			return 0;
		}
	#endif // NO_TX0_INTERRUPT

	#ifndef NO_RX0_INTERRUPT
		
		int uart_getchar(FILE *stream)
		{
			int16_t tmp;
			
			while ( (tmp = uart0_getData()) < 0 );
		
		#ifdef RX_STDIO_GETCHAR_ECHO
			tmp = uart0_putc_((uint8_t)tmp);
		#endif
		
			return (uint8_t)tmp;
		}
	#endif //NO_RX0_INTERRUPT
	
	#if defined(NO_RX0_INTERRUPT)
		FILE uart0_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
		
	#elif defined(NO_TX0_INTERRUPT)
		FILE uart0_in = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
	#else
		FILE uart0_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
		FILE uart0_in = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
		FILE uart0_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
	#endif

#endif // single/multi USART

//******************************************************************
//ISR prototypes
//******************************************************************
/*
	ISR(TXn_INTERRUPT) // do it in a little weird way
	{
		register uint8_t tmp_tx_Tail = (txn_Tail + 1) & TXn_BUFFER_MASK;
		
		if(tmp_tx_Tail == txn_Head)
			UCSRnB_REGISTER &= ~(1<<UDRIEn_BIT); // may be racing with putc insertions
			
		txn_Tail = tmp_tx_Tail; // it would create race condition if not used in isr
		UDRn_REGISTER = txn_buffer[tmp_tx_Tail]; // transmit character from the buffer
	}
	
	ISR(TXn_INTERRUPT) // non racing one // not used anymore
	{
		register uint8_t tmp_tx_Tail = txn_Tail;
		
		if(tmp_tx_Tail != txn_Head)
		{
			tmp_tx_Tail = (tmp_tx_Tail + 1) & TXn_BUFFER_MASK;
			txn_Tail = tmp_tx_Tail; // it would create race condition if not used in isr
			UDRn_REGISTER = txn_buffer[tmp_tx_Tail]; // transmit character from the buffer
		}
		else
			UCSRnB_REGISTER &= ~(1<<UDRIEn_BIT);
	}

	ISR(RXn_INTERRUPT)
	{
		register uint8_t tmp_rx_Head = (rxn_Head + 1) & RXn_BUFFER_MASK;
		register uint8_t tmp = UDRn_REGISTER;
		
	#if defined(USART0_MPCM_MODE)&&!defined(MPCM0_MASTER_ONLY)
		if(UCSRnA & (1<<MPCMn))
		{
			if(tmp == MPCMn_ADDRESS || tmp == MPCMn_GCALL_ADDRESS)
				UCSRnA &= ~(1<<MPCMn);
			else
				return;
		}
	#endif

		if(rxn_Tail != tmp_rx_Head)
		{
			rxn_Head = tmp_rx_Head; // it would create race condition if not used in isr
			rxn_buffer[tmp_rx_Head] = tmp;
		}
		
	}
*/

#ifndef NO_TX0_INTERRUPT

	ISR(UDRE0_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"push	r16 \n\t"
			"in		r16, __SREG__ \n\t"
		#else
			"in		%[sreg_save], __SREG__ \n\t"
		#endif
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"push	r30 \n\t"
			"push	r31 \n\t"
		#else
			#ifdef __AVR_HAVE_MOVW__
				"movw	%[z_save], r30 \n\t"
			#else // in this case only 4 cycles are prematured out
				"mov	%A[z_save], r30 \n\t"
				"mov	%B[z_save], r31 \n\t"
			#endif
		#endif
		
		#ifdef USART_UNSAFE_TX_INTERRUPT
			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"cbi	%M[control_reg_IO], %M[udrie_bit] \n\t"
			#elif defined(USART0_IN_UPPER_IO_ADDRESS_SPACE)
				"in		r31, %M[control_reg_IO] \n\t"
				"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
				"out	%M[control_reg_IO], r31\n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
			
			"sei \n\t"
		#endif
		
			TX0_EVERYCAL_EVENT
		
			"lds	r30, (tx0_Tail) \n\t"
			"lds	r31, (tx0_Head) \n\t"
		
		#ifdef USART_UNSAFE_TX_INTERRUPT
			"cp		r30, r31 \n\t"
			"breq	USART0_TX_EXIT \n\t"
		#endif
		
			"inc	r30 \n\t"
	
		#if (TX0_BUFFER_MASK != 0xff)
			"andi	r30, %M[mask]\n\t"
		#endif
		
		#ifndef USART_UNSAFE_TX_INTERRUPT
			"cpse	r30, r31 \n\t"
			"rjmp	USART0_TX_CONTINUE \n\t"
			
			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"cbi	%M[control_reg_IO], %M[udrie_bit] \n\t"
			#elif defined(USART0_IN_UPPER_IO_ADDRESS_SPACE)
				"in		r31, %M[control_reg_IO] \n\t"
				"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
				"out	%M[control_reg_IO], r31\n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
			
		"USART0_TX_CONTINUE: "
		#endif
			
			"sts	(tx0_Tail), r30 \n\t"
	
		#if !defined(__AVR_ATtiny2313__)&&!defined(__AVR_ATtiny2313A__) // on ATtiny2313 upper byte in pointer pair is ignored
			"ldi	r31, 0x00 \n\t"
		#endif
			"subi	r30, lo8(-(tx0_buffer)) \n\t"
		
		#ifndef USART_USE_TINY_MEMORY_MODEL
			"sbci	r31, hi8(-(tx0_buffer)) \n\t"
		#endif
			"ld		r30, Z \n\t"
		
		#ifdef USART0_IN_IO_ADDRESS_SPACE
			"out	%M[UDR_reg_IO], r30 \n\t"
		#else
			"sts	%M[UDR_reg], r30 \n\t"
		#endif
			
			TX0_TRANSMIT_EVENT
			
		#ifdef USART_UNSAFE_TX_INTERRUPT
			"cli \n\t"
			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"sbi	%M[control_reg_IO], %M[udrie_bit] \n\t"
			#elif defined(USART0_IN_UPPER_IO_ADDRESS_SPACE)	
				"in		r31, %M[control_reg_IO] \n\t"
				"ori	r31, (1<<%M[udrie_bit]) \n\t"
				"out	%M[control_reg_IO], r31\n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"ori	r31, (1<<%M[udrie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
		#endif
			
		"USART0_TX_EXIT: "
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r31 \n\t"
			"pop	r30 \n\t"
		#else
			#ifdef __AVR_HAVE_MOVW__
				"movw	r30, %[z_save] \n\t"
			#else
				"mov	r31, %B[z_save] \n\t"
				"mov	r30, %A[z_save] \n\t"
			#endif
		#endif
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#else
			"out	__SREG__, %[sreg_save] \n\t"
		#endif
		
			"reti \n\t"
			: // output operands
			USART_REG_SAVE_LIST
			
			: // input operands
			TX0_INPUT_OPERAND_LIST
			[UDR_reg_IO]     "M" (_SFR_IO_ADDR(UDR0_REGISTER)),
			[UDR_reg]        "n" (_SFR_MEM_ADDR(UDR0_REGISTER)),
			[control_reg_IO] "M" (_SFR_IO_ADDR(UCSR0B_REGISTER)),
			[control_reg]    "n" (_SFR_MEM_ADDR(UCSR0B_REGISTER)),
			[udrie_bit]      "M" (UDRIE0_BIT),
			[mask]           "M" (TX0_BUFFER_MASK)
		
			// no clobbers
		);
	}	
	
#if defined(USART0_USE_TXC_INTERRUPT)
	
#ifdef USATR0_NO_NAKED_TXC_INTERRUPT
	ISR(TXC0_INTERRUPT)
#else
	ISR(TXC0_INTERRUPT, ISR_NAKED) // ISR is compiled to only one cbi instruction - no need for large prologue/epilogue
#endif	
	{
	#ifdef USART0_RS485_MODE
		RS485_CONTROL0_PORT &= ~(1<<RS485_CONTROL0_IONUM); // set low after completed transaction
	#endif
	
		TXC0_interrupt_event();
	
	#ifndef USATR0_NO_NAKED_TXC_INTERRUPT
		reti();
	#endif
	}
	
#endif
	
#endif // NO_TX0_INTERRUPT

#ifndef NO_RX0_INTERRUPT

	ISR(RX0_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"

		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"push	r16 \n\t"
			"in		r16, __SREG__ \n\t"
		#else
			"in		%[sreg_save], __SREG__ \n\t"
		#endif
		
			"push	r25 \n\t"
			
		#ifdef USART0_PUSH_BEFORE_RX
			#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
				"push	r30 \n\t"
				"push	r31 \n\t"
			#else
				#ifdef __AVR_HAVE_MOVW__
					"movw	%[z_save], r30 \n\t"
				#else // in this case only 4 cycles are prematured out
					"mov	%A[z_save], r30\n\t"
					"mov	%B[z_save], r31\n\t"
				#endif
			#endif
		#endif
			
		#ifndef USART0_EXTEND_RX_BUFFER
			RX0_FRAMING_EVENT
			
			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"in		r25, %M[UDR_reg_IO] \n\t"
			#else
				"lds	r25, %M[UDR_reg] \n\t"
			#endif
		#endif
			
		#ifndef USART0_PUSH_BEFORE_RX
			#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
				"push	r30 \n\t"
				"push	r31 \n\t"
			#else
				#ifdef __AVR_HAVE_MOVW__
					"movw	%[z_save], r30 \n\t"
				#else // in this case only 4 cycles are prematured out
					"mov	%A[z_save], r30\n\t"
					"mov	%B[z_save], r31\n\t"
				#endif
			#endif
		#endif
		
		#ifdef USART_UNSAFE_RX_INTERRUPT
			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"cbi	%M[control_reg_IO], %M[rxcie_bit] \n\t"
			#elif defined(USART0_IN_UPPER_IO_ADDRESS_SPACE)
				"in		r31, %M[control_reg_IO] \n\t"
				"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
				"out	%M[control_reg_IO], r31\n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
			
			"sei \n\t"
		#endif
	
			RX0_EVERYCALL_EVENT
	
			"lds	r30, (rx0_Head) \n\t"
			"lds	r31, (rx0_Tail) \n\t"
		
			"inc	r30 \n\t"
		
		#if (RX0_BUFFER_MASK != 0xff)
			"andi	r30, %M[mask]\n\t"
		#endif
		
			"cp		r31, r30 \n\t"
		#if defined(USART0_USE_SOFT_RTS)||(defined(USART0_EXTEND_RX_BUFFER)&&!defined(USART_UNSAFE_RX_INTERRUPT))
			"breq	USART0_DISABLE_RXCIE \n\t"
		#elif defined(USART0_EXTEND_RX_BUFFER)&&defined(USART_UNSAFE_RX_INTERRUPT)
			"breq	USART0_RX_EXIT_SKIP \n\t"
		#else
			"breq	USART0_RX_EXIT \n\t"
		#endif
		
		#ifdef USART0_EXTEND_RX_BUFFER
			RX0_FRAMING_EVENT
			
			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"in		r25, %M[UDR_reg_IO] \n\t"
			#else
				"lds	r25, %M[UDR_reg] \n\t"
			#endif
		#endif
		
		RX0_EARLY_RECEIVE_EVENT
		
		#if defined(USART0_MPCM_MODE)&&!defined(MPCM0_MASTER_ONLY)

			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"in 	r31, %M[UCSRA_reg_IO] \n\t"
			#else
				"lds	r31, %M[UCSRA_reg] \n\t"
			#endif

				"sbrs	r31, %M[mpcm_bit] \n\t"
				"rjmp	USART0_RX_CONTINUE \n\t"
				"cpi	r25, %M[mpcm_address] \n\t"
			#ifdef MPCM0_GCALL_ADDRESS
				"breq	p_%= \n\t"
				"cpi	r25, %M[mpcm_gcall_address] \n\t"
			#endif
				"brne	USART0_RX_EXIT \n\t"
			"p_%=: "
				"andi	r31, ~(1<<%M[mpcm_bit]) \n\t"

			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"out	%M[UCSRA_reg_IO], r31 \n\t"
			#else
				"sts	%M[UCSRA_reg], r31 \n\t"
			#endif

		"USART0_RX_CONTINUE: "
		#endif
			
			"sts	(rx0_Head), r30 \n\t"
		
		#if !defined(__AVR_ATtiny2313__)&&!defined(__AVR_ATtiny2313A__)	// on ATtiny2313 upper byte in pointer pair is ignored
			"ldi	r31, 0x00 \n\t"
		#endif
			"subi	r30, lo8(-(rx0_buffer))\n\t"
		
		#ifndef USART_USE_TINY_MEMORY_MODEL
			"sbci	r31, hi8(-(rx0_buffer))\n\t"
		#endif	
			"st		Z, r25 \n\t"
		
			RX0_LATE_RECEIVE_EVENT
			
		"USART0_RX_EXIT: "
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"cli \n\t"
			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"sbi	%M[control_reg_IO], %M[rxcie_bit] \n\t"
			#elif defined(USART0_IN_UPPER_IO_ADDRESS_SPACE)
				"in		r31, %M[control_reg_IO] \n\t"
				"ori	r31, (1<<%M[rxcie_bit]) \n\t"
				"out	%M[control_reg_IO], r31\n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"ori	r31, (1<<%M[rxcie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
			
		"USART0_RX_EXIT_SKIP: "
		#endif
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r31 \n\t"
			"pop	r30 \n\t"
		#else
			#ifdef __AVR_HAVE_MOVW__
				"movw	r30, %[z_save] \n\t"
			#else
				"mov	r31, %B[z_save] \n\t"
				"mov	r30, %A[z_save] \n\t"
			#endif
		#endif
		
			"pop	r25 \n\t"
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#else
			"out	__SREG__, %[sreg_save] \n\t"
		#endif

			"reti \n\t"
		
		#if defined(USART0_USE_SOFT_RTS)||(defined(USART0_EXTEND_RX_BUFFER)&&!defined(USART_UNSAFE_RX_INTERRUPT))
		"USART0_DISABLE_RXCIE: "
		
		#ifdef USART0_USE_SOFT_RTS
			"sbi	%M[rts_port], %M[rts_pin] \n\t"
		#endif
		
		#ifndef USART_UNSAFE_RX_INTERRUPT
			#ifdef USART0_IN_IO_ADDRESS_SPACE
				"cbi	%M[control_reg_IO], %M[rxcie_bit] \n\t"
			#elif defined(USART0_IN_UPPER_IO_ADDRESS_SPACE)
				"in		r31, %M[control_reg_IO] \n\t"
				"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
				"out	%M[control_reg_IO], r31\n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
		#endif
			
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"rjmp	USART0_RX_EXIT_SKIP \n\t"
		#else
			"rjmp	USART0_RX_EXIT \n\t"
		#endif
		#endif
			: // output operands
			USART_REG_SAVE_LIST
			
			: // input operands
			RX0_INPUT_OPERAND_LIST
			[UDR_reg_IO]         "M" (_SFR_IO_ADDR(UDR0_REGISTER)),
			[UDR_reg]            "n" (_SFR_MEM_ADDR(UDR0_REGISTER)),
			[mask]               "M" (RX0_BUFFER_MASK),
			[mpcm_address]       "M" (MPCM0_ADDRESS),
		#ifdef MPCM0_GCALL_ADDRESS
			[mpcm_gcall_address] "M" (MPCM0_GCALL_ADDRESS),
		#endif
			[mpcm_bit]           "M" (MPCM0_BIT),
		#ifdef USART0_USE_SOFT_RTS
			[rts_port]           "M" (_SFR_IO_ADDR(RTS0_PORT)),
			[rts_pin]            "M" (RTS0_IONUM),
		#endif
			[UCSRA_reg]          "n" (_SFR_MEM_ADDR(UCSR0A_REGISTER)),
			[UCSRA_reg_IO]       "M" (_SFR_IO_ADDR(UCSR0A_REGISTER)),
			[control_reg_IO]     "M" (_SFR_IO_ADDR(UCSR0B_REGISTER)),
			[control_reg]        "n" (_SFR_MEM_ADDR(UCSR0B_REGISTER)),
			[rxcie_bit]          "M" (RXCIE0_BIT)
		
			// no clobbers
		);
		
	}

#endif // NO_RX0_INTERRUPT

#ifndef NO_TX1_INTERRUPT

	ISR(UDRE1_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"push	r16 \n\t"
			"in		r16, __SREG__ \n\t"
		#else
			"in		%[sreg_save], __SREG__ \n\t"
		#endif

		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"push	r30 \n\t"
			"push	r31 \n\t"
		#else
			"movw	%[z_save], r30 \n\t"
		#endif
		
		#ifdef USART_UNSAFE_TX_INTERRUPT
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"cbi	%M[control_reg_IO], %M[udrie_bit] \n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
		
			"sei \n\t"
		#endif
			
			TX1_EVERYCAL_EVENT
			
			"lds	r30, (tx1_Tail) \n\t"
			"lds	r31, (tx1_Head) \n\t"
		
		#ifdef USART_UNSAFE_TX_INTERRUPT
			"cp		r30, r31 \n\t"
			"breq	USART1_TX_EXIT \n\t"
		#endif
		
			"inc	r30 \n\t"
		
		#if (TX1_BUFFER_MASK != 0xff)
			"andi	r30, %M[mask]\n\t"
		#endif
		
		#ifndef USART_UNSAFE_TX_INTERRUPT
			"cpse	r30, r31 \n\t"
			"rjmp	USART1_TX_CONTINUE \n\t"
			
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"cbi	%M[control_reg_IO], %M[udrie_bit] \n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
			
		"USART1_TX_CONTINUE: "
		#endif
		
			"sts	(tx1_Tail), r30 \n\t"
		
			"ldi	r31, 0x00 \n\t"
			"subi	r30, lo8(-(tx1_buffer)) \n\t"
			"sbci	r31, hi8(-(tx1_buffer)) \n\t"
			
			"ld		r30, Z \n\t"
		
		#ifdef USART1_IN_IO_ADDRESS_SPACE
			"out	%M[UDR_reg_IO], r30 \n\t"
		#else
			"sts	%M[UDR_reg], r30 \n\t"
		#endif
		
			TX1_TRANSMIT_EVENT
		
		#ifdef USART_UNSAFE_TX_INTERRUPT
			"cli \n\t"
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"sbi	%M[control_reg_IO], %M[udrie_bit] \n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"ori	r31, (1<<%M[udrie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
		#endif
		
		"USART1_TX_EXIT: "
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r31 \n\t"
			"pop	r30 \n\t"
		#else
			"movw	r30, %[z_save] \n\t"
		#endif
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#else
			"out	__SREG__, %[sreg_save] \n\t"
		#endif

			"reti \n\t"
			: // output operands
			USART_REG_SAVE_LIST
			
			: // input operands
			TX1_INPUT_OPERAND_LIST
			[UDR_reg_IO]     "M" (_SFR_IO_ADDR(UDR1_REGISTER)),
			[UDR_reg]        "n" (_SFR_MEM_ADDR(UDR1_REGISTER)),
			[control_reg_IO] "M" (_SFR_IO_ADDR(UCSR1B_REGISTER)),
			[control_reg]    "n" (_SFR_MEM_ADDR(UCSR1B_REGISTER)),
			[udrie_bit]      "M" (UDRIE1_BIT),
			[mask]           "M" (TX1_BUFFER_MASK)
		
			// no clobbers
		);
		
	}

#if defined(USART1_USE_TXC_INTERRUPT)

#ifdef USATR1_NO_NAKED_TXC_INTERRUPT
	ISR(TXC1_INTERRUPT)
#else
	ISR(TXC1_INTERRUPT, ISR_NAKED) // ISR is compiled to only one cbi instruction - no need for large prologue/epilogue
#endif
	{
	#ifdef USART1_RS485_MODE
		RS485_CONTROL1_PORT &= ~(1<<RS485_CONTROL1_IONUM); // set low after completed transaction
	#endif
	
		TXC1_interrupt_event();
	
	#ifndef USATR1_NO_NAKED_TXC_INTERRUPT
		reti();
	#endif
	}

#endif // USART1_RS485_MODE

#endif // NO_TX1_INTERRUPT

#ifndef NO_RX1_INTERRUPT
	
	ISR(RX1_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"
	
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"push	r16 \n\t"
			"in		r16, __SREG__ \n\t"
		#else
			"in		%[sreg_save], __SREG__ \n\t"
		#endif
	
			"push	r25 \n\t"
		
		#ifdef USART1_PUSH_BEFORE_RX
			#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
				"push	r30 \n\t"
				"push	r31 \n\t"
			#else
				"movw	%[z_save], r30 \n\t"
			#endif
		#endif
		
		#ifndef USART1_EXTEND_RX_BUFFER
			RX1_FRAMING_EVENT
		
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"in		r25, %M[UDR_reg_IO] \n\t"
			#else
				"lds	r25, %M[UDR_reg] \n\t"
			#endif
		#endif
			
		#ifndef USART1_PUSH_BEFORE_RX
			#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
				"push	r30 \n\t"
				"push	r31 \n\t"
			#else
				"movw	%[z_save], r30 \n\t"
			#endif
		#endif
		
		#ifdef USART_UNSAFE_RX_INTERRUPT
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"cbi	%M[control_reg_IO], %M[rxcie_bit] \n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
		
			"sei \n\t"
		#endif
			
			RX1_EVERYCALL_EVENT
			
			"lds	r30, (rx1_Head) \n\t"
			"lds	r31, (rx1_Tail) \n\t"
		
			"inc	r30 \n\t"
		
		#if (RX1_BUFFER_MASK != 0xff)
			"andi	r30, %M[mask]\n\t"
		#endif
		
			"cp		r31, r30 \n\t"
		#if defined(USART1_USE_SOFT_RTS)||(defined(USART1_EXTEND_RX_BUFFER)&&!defined(USART_UNSAFE_RX_INTERRUPT))
			"breq	USART1_DISABLE_RXCIE \n\t"           
		#elif defined(USART1_EXTEND_RX_BUFFER)&&defined(USART_UNSAFE_RX_INTERRUPT)
			"breq	USART1_RX_EXIT_SKIP \n\t"          
		#else
			"breq	USART1_RX_EXIT \n\t"           
		#endif
			
		#ifdef USART1_EXTEND_RX_BUFFER
			RX1_FRAMING_EVENT
			
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"in		r25, %M[UDR_reg_IO] \n\t"
			#else
				"lds	r25, %M[UDR_reg] \n\t"
			#endif
		#endif
			
		RX1_EARLY_RECEIVE_EVENT
			
		#if defined(USART1_MPCM_MODE)&&!defined(MPCM1_MASTER_ONLY)
		
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"in		r31, %M[UCSRA_reg_IO] \n\t"
			#else
				"lds	r31, %M[UCSRA_reg] \n\t"
			#endif
		
				"sbrs	r31, %M[mpcm_bit] \n\t"
				"rjmp	USART1_RX_CONTINUE \n\t"
				"cpi	r25, %M[mpcm_address] \n\t"
			#ifdef MPCM1_GCALL_ADDRESS
				"breq	p_%= \n\t"
				"cpi	r25, %M[mpcm_gcall_address] \n\t"
			#endif
				"brne	USART1_RX_EXIT \n\t"
			"p_%=: "
				"andi	r31, ~(1<<%M[mpcm_bit]) \n\t"
		
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"out	%M[UCSRA_reg_IO], r31 \n\t"
			#else
				"sts	%M[UCSRA_reg], r31 \n\t"
			#endif
		
		"USART1_RX_CONTINUE: "
		#endif
			
			"sts	(rx1_Head), r30 \n\t"
		
			"ldi	r31, 0x00 \n\t"
			"subi	r30, lo8(-(rx1_buffer))\n\t"
			"sbci	r31, hi8(-(rx1_buffer))\n\t"
			"st		Z, r25 \n\t"
		
			RX1_LATE_RECEIVE_EVENT
		
		"USART1_RX_EXIT: "
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"cli \n\t"
			
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"sbi	%M[control_reg_IO], %M[rxcie_bit] \n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"ori	r31, (1<<%M[rxcie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
			
		"USART1_RX_EXIT_SKIP: "
		#endif
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r31 \n\t"
			"pop	r30 \n\t"
		#else
			"movw	r30, %[z_save] \n\t"
		#endif
		
			"pop	r25 \n\t"

		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#else
			"out	__SREG__, %[sreg_save] \n\t"
		#endif

			"reti \n\t"
		
		#if defined(USART1_USE_SOFT_RTS)||(defined(USART1_EXTEND_RX_BUFFER)&&!defined(USART_UNSAFE_RX_INTERRUPT))
		"USART1_DISABLE_RXCIE: "
		
		#ifdef USART1_USE_SOFT_RTS
			"sbi	%M[rts_port], %M[rts_pin] \n\t"
		#endif
		
		#ifndef USART_UNSAFE_RX_INTERRUPT
			#ifdef USART1_IN_IO_ADDRESS_SPACE
				"cbi	%M[control_reg_IO], %M[rxcie_bit] \n\t"
			#else
				"lds	r31, %M[control_reg] \n\t"
				"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
				"sts	%M[control_reg], r31 \n\t"
			#endif
		#endif
			
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"rjmp	USART1_RX_EXIT_SKIP \n\t"
		#else
			"rjmp	USART1_RX_EXIT \n\t"
		#endif
		#endif
			: // output operands
			USART_REG_SAVE_LIST
			
			: // input operands
			RX1_INPUT_OPERAND_LIST
			[UDR_reg_IO]         "M" (_SFR_IO_ADDR(UDR1_REGISTER)),
			[UDR_reg]            "n" (_SFR_MEM_ADDR(UDR1_REGISTER)),
			[mask]               "M" (RX1_BUFFER_MASK),
			[mpcm_address]       "M" (MPCM1_ADDRESS),
		#ifdef MPCM1_GCALL_ADDRESS
			[mpcm_gcall_address] "M" (MPCM1_GCALL_ADDRESS),
		#endif
			[mpcm_bit]           "M" (MPCM1_BIT),
		#ifdef USART1_USE_SOFT_RTS
			[rts_port]           "M" (_SFR_IO_ADDR(RTS1_PORT)),
			[rts_pin]            "M" (RTS1_IONUM),
		#endif
			[UCSRA_reg]          "n" (_SFR_MEM_ADDR(UCSR1A_REGISTER)),
			[UCSRA_reg_IO]       "M" (_SFR_IO_ADDR(UCSR1A_REGISTER)),
			[control_reg_IO]     "M" (_SFR_IO_ADDR(UCSR1B_REGISTER)),
			[control_reg]        "n" (_SFR_MEM_ADDR(UCSR1B_REGISTER)),
			[rxcie_bit]          "M" (RXCIE1_BIT)
		
			// no clobbers
		);
		
	}

#endif // NO_RX1_INTERRUPT

#ifndef NO_TX2_INTERRUPT

	ISR(UDRE2_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"
	
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"push	r16 \n\t"
			"in		r16, __SREG__ \n\t"
		#else
			"in		%[sreg_save], __SREG__ \n\t"
		#endif
	
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"push	r30 \n\t"
			"push	r31 \n\t"
		#else
			"movw	%[z_save], r30 \n\t"
		#endif
	
		#ifdef USART_UNSAFE_TX_INTERRUPT
			"lds	r31, %M[control_reg] \n\t"
			"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
		
			"sei \n\t"
		#endif
		
			TX2_EVERYCAL_EVENT
		
			"lds	r30, (tx2_Tail) \n\t"
			"lds	r31, (tx2_Head) \n\t"
		
		#ifdef USART_UNSAFE_TX_INTERRUPT
			"cp		r30, r31 \n\t"
			"breq	USART2_TX_EXIT \n\t"
		#endif
		
			"inc	r30 \n\t"
		
		#if (TX2_BUFFER_MASK != 0xff)
			"andi	r30, %M[mask]\n\t"
		#endif
		
		#ifndef USART_UNSAFE_TX_INTERRUPT
			"cpse	r30, r31 \n\t"
			"rjmp	USART2_TX_CONTINUE \n\t"
			
			"lds	r31, %M[control_reg] \n\t"
			"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
			
		"USART2_TX_CONTINUE: "
		#endif
			
			"sts	(tx2_Tail), r30 \n\t"
			
			"ldi	r31, 0x00 \n\t"
			"subi	r30, lo8(-(tx2_buffer)) \n\t"
			"sbci	r31, hi8(-(tx2_buffer)) \n\t"
			
			"ld		r30, Z \n\t"
			
			"sts	%M[UDR_reg], r30 \n\t"

			TX2_TRANSMIT_EVENT

		#ifdef USART_UNSAFE_TX_INTERRUPT
			"cli \n\t"
			
			"lds	r31, %M[control_reg] \n\t"
			"ori	r31, (1<<%M[udrie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
		#endif

		"USART2_TX_EXIT: "
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r31 \n\t"
			"pop	r30 \n\t"
		#else
			"movw	r30, %[z_save] \n\t"
		#endif
			
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#else
			"out	__SREG__, %[sreg_save] \n\t"
		#endif

			"reti \n\t"
			: // output operands
			USART_REG_SAVE_LIST
			
			: // input operands
			TX2_INPUT_OPERAND_LIST
			[UDR_reg]     "n" (_SFR_MEM_ADDR(UDR2_REGISTER)),
			[control_reg] "n" (_SFR_MEM_ADDR(UCSR2B_REGISTER)),
			[udrie_bit]   "M" (UDRIE2_BIT),
			[mask]        "M" (TX2_BUFFER_MASK)
		
			// no clobbers
		);
		
	}

#if defined(USART2_USE_TXC_INTERRUPT)

#ifdef USATR2_NO_NAKED_TXC_INTERRUPT
	ISR(TXC2_INTERRUPT)
#else
	ISR(TXC2_INTERRUPT, ISR_NAKED) // ISR is compiled to only one cbi instruction - no need for large prologue/epilogue
#endif
	{
	#ifdef USART2_RS485_MODE
		RS485_CONTROL2_PORT &= ~(1<<RS485_CONTROL2_IONUM); // set low after completed transaction
	#endif
	
		TXC2_interrupt_event();
	
	#ifndef USATR2_NO_NAKED_TXC_INTERRUPT
		reti();
	#endif
	}

#endif // USART2_RS485_MODE

#endif // NO_TX2_INTERRUPT

#ifndef NO_RX2_INTERRUPT

	ISR(RX2_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"push	r16 \n\t"
			"in		r16, __SREG__ \n\t"
		#else
			"in		%[sreg_save], __SREG__ \n\t"
		#endif

			"push	r25 \n\t"
			
		#ifdef USART2_PUSH_BEFORE_RX
			#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
				"push	r30 \n\t"
				"push	r31 \n\t"
			#else
				"movw	%[z_save], r30 \n\t"
			#endif
		#endif	
			
		#ifndef USART2_EXTEND_RX_BUFFER
			RX2_FRAMING_EVENT
			"lds	r25, %M[UDR_reg] \n\t"
		#endif
			
		#ifndef USART2_PUSH_BEFORE_RX
			#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
				"push	r30 \n\t"
				"push	r31 \n\t"
			#else
				"movw	%[z_save], r30 \n\t"
			#endif
		#endif
		
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"lds	r31, %M[control_reg] \n\t"
			"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
			
			"sei \n\t"
		#endif
		
			RX2_EVERYCALL_EVENT
		
			"lds	r30, (rx2_Head) \n\t"
			"lds	r31, (rx2_Tail) \n\t"
		
			"inc	r30 \n\t"
		
		#if (RX2_BUFFER_MASK != 0xff)
			"andi	r30, %M[mask]\n\t"
		#endif
		
			"cp		r31, r30 \n\t"
		#if defined(USART2_USE_SOFT_RTS)||(defined(USART2_EXTEND_RX_BUFFER)&&!defined(USART_UNSAFE_RX_INTERRUPT))
			"breq	USART2_DISABLE_RXCIE \n\t"           
		#elif defined(USART2_EXTEND_RX_BUFFER)&&defined(USART_UNSAFE_RX_INTERRUPT)
			"breq	USART2_RX_EXIT_SKIP \n\t"          
		#else
			"breq	USART2_RX_EXIT \n\t"           
		#endif
		
		#ifdef USART2_EXTEND_RX_BUFFER
			RX2_FRAMING_EVENT
			"lds	r25, %M[UDR_reg] \n\t"
		#endif
		
			RX2_EARLY_RECEIVE_EVENT
		
		#if defined(USART2_MPCM_MODE)&&!defined(MPCM2_MASTER_ONLY)
			"lds	r31, %M[UCSRA_reg] \n\t"
		
			"sbrs	r31, %M[mpcm_bit] \n\t"
			"rjmp	USART2_RX_CONTINUE \n\t"
			"cpi	r25, %M[mpcm_address] \n\t"
		#ifdef MPCM2_GCALL_ADDRESS
			"breq	p_%= \n\t"
			"cpi	r25, %M[mpcm_gcall_address] \n\t"
		#endif
			"brne	USART2_RX_EXIT \n\t"
		"p_%=: "
			"andi	r31, ~(1<<%M[mpcm_bit]) \n\t"
		
			"sts	%M[UCSRA_reg], r31 \n\t"
		
		"USART2_RX_CONTINUE: "
		#endif
			
			"sts	(rx2_Head), r30 \n\t"
		
			"ldi	r31, 0x00 \n\t"
			"subi	r30, lo8(-(rx2_buffer))\n\t"
			"sbci	r31, hi8(-(rx2_buffer))\n\t"
			"st		Z, r25 \n\t"
		
			RX2_LATE_RECEIVE_EVENT
		
		"USART2_RX_EXIT: "
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"cli \n\t"
			
			"lds	r31, %M[control_reg] \n\t"
			"ori	r31, (1<<%M[rxcie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
		
		"USART2_RX_EXIT_SKIP: "
		#endif
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r31 \n\t"
			"pop	r30 \n\t"
		#else
			"movw	r30, %[z_save] \n\t"
		#endif
			
			"pop	r25 \n\t"
	
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#else
			"out	__SREG__, %[sreg_save] \n\t"
		#endif

			"reti \n\t"
		
		#if defined(USART2_USE_SOFT_RTS)||(defined(USART2_EXTEND_RX_BUFFER)&&!defined(USART_UNSAFE_RX_INTERRUPT))
		"USART2_DISABLE_RXCIE: "
		
		#ifdef USART3_USE_SOFT_RTS
			"sbi	%M[rts_port], %M[rts_pin] \n\t"
		#endif
		
		#ifndef USART_UNSAFE_RX_INTERRUPT
			"lds	r31, %M[control_reg] \n\t"
			"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
		#endif
			
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"rjmp	USART2_RX_EXIT_SKIP \n\t"
		#else
			"rjmp	USART2_RX_EXIT \n\t"
		#endif
		#endif
			: // output operands
			USART_REG_SAVE_LIST
			
			: // input operands
			RX2_INPUT_OPERAND_LIST
			[UDR_reg]            "n" (_SFR_MEM_ADDR(UDR2_REGISTER)),
			[mask]               "M" (RX2_BUFFER_MASK),
			[mpcm_address]       "M" (MPCM2_ADDRESS),
		#ifdef MPCM2_GCALL_ADDRESS
			[mpcm_gcall_address] "M" (MPCM2_GCALL_ADDRESS),
		#endif
			[mpcm_bit]           "M" (MPCM2_BIT),
		#ifdef USART2_USE_SOFT_RTS
			[rts_port]           "M" (_SFR_IO_ADDR(RTS2_PORT)),
			[rts_pin]            "M" (RTS2_IONUM),
		#endif
			[UCSRA_reg]          "n" (_SFR_MEM_ADDR(UCSR2A_REGISTER)),
			[control_reg]        "n" (_SFR_MEM_ADDR(UCSR2B_REGISTER)),
			[rxcie_bit]          "M" (RXCIE2_BIT)
		
			// no clobbers
		);
		
	}
	
#endif // NO_RX2_INTERRUPT

#ifndef NO_TX3_INTERRUPT

	ISR(UDRE3_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"
	
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"push	r16 \n\t"
			"in		r16, __SREG__ \n\t"
		#else
			"in		%[sreg_save], __SREG__ \n\t"
		#endif
	
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"push	r30 \n\t"
			"push	r31 \n\t"
		#else
			"movw	%[z_save], r30 \n\t"
		#endif
		
		#ifdef USART_UNSAFE_TX_INTERRUPT
			"lds	r31, %M[control_reg] \n\t"
			"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
		
			"sei \n\t"
		#endif
			
			TX3_EVERYCAL_EVENT
			
			"lds	r30, (tx3_Tail) \n\t"
			"lds	r31, (tx3_Head) \n\t"
		
		#ifdef USART_UNSAFE_TX_INTERRUPT
			"cp		r30, r31 \n\t"
			"breq	USART3_TX_EXIT \n\t"
		#endif
		
			"inc	r30 \n\t"
		
		#if (TX3_BUFFER_MASK != 0xff)
			"andi	r30, %M[mask]\n\t"
		#endif
		
		#ifndef USART_UNSAFE_TX_INTERRUPT
			"cpse	r30, r31 \n\t"
			"rjmp	USART3_TX_CONTINUE \n\t"
			
			"lds	r31, %M[control_reg] \n\t"
			"andi	r31, ~(1<<%M[udrie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
		
		"USART3_TX_CONTINUE: "
		#endif
		
			"sts	(tx3_Tail), r30 \n\t"
		
			"ldi	r31, 0x00 \n\t"
			"subi	r30, lo8(-(tx3_buffer)) \n\t"
			"sbci	r31, hi8(-(tx3_buffer)) \n\t"
			
			"ld		r30, Z \n\t"
		
			"sts	%M[UDR_reg], r30 \n\t"
			
			TX3_TRANSMIT_EVENT
			
		#ifdef USART_UNSAFE_TX_INTERRUPT
			"cli \n\t"
			
			"lds	r31, %M[control_reg] \n\t"
			"ori	r31, (1<<%M[udrie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
		#endif
			
		"USART3_TX_EXIT: "
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r31 \n\t"
			"pop	r30 \n\t"
		#else
			"movw	r30, %[z_save] \n\t"
		#endif
			
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#else
			"out	__SREG__, %[sreg_save] \n\t"
		#endif

			"reti \n\t"
			: // output operands
			USART_REG_SAVE_LIST
			
			: // input operands
			TX3_INPUT_OPERAND_LIST
			[UDR_reg]     "n" (_SFR_MEM_ADDR(UDR3_REGISTER)),
			[control_reg] "n" (_SFR_MEM_ADDR(UCSR3B_REGISTER)),
			[udrie_bit]   "M" (UDRIE3_BIT),
			[mask]        "M" (TX3_BUFFER_MASK)
		
			// no clobbers
		);
		
	}
	
#if defined(USART3_USE_TXC_INTERRUPT)

#ifdef USATR3_NO_NAKED_TXC_INTERRUPT
	ISR(TXC3_INTERRUPT)
#else
	ISR(TXC3_INTERRUPT, ISR_NAKED) // ISR is compiled to only one cbi instruction - no need for large prologue/epilogue
#endif
	{
	#ifdef USART3_RS485_MODE
		RS485_CONTROL3_PORT &= ~(1<<RS485_CONTROL3_IONUM); // set low after completed transaction
	#endif
	
		TXC3_interrupt_event();
	
	#ifndef USATR3_NO_NAKED_TXC_INTERRUPT
		reti();
	#endif
	}

#endif // USART0_RS485_MODE	
	
#endif // NO_TX3_INTERRUPT

#ifndef NO_RX3_INTERRUPT

	ISR(RX3_INTERRUPT, ISR_NAKED)
	{
		asm volatile("\n\t"
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"push	r16 \n\t"
			"in		r16, __SREG__ \n\t"
		#else
			"in		%[sreg_save], __SREG__ \n\t"
		#endif
	
			"push	r25 \n\t"
		
		#ifdef USART3_PUSH_BEFORE_RX
			#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
				"push	r30 \n\t"
				"push	r31 \n\t"
			#else
				"movw	%[z_save], r30 \n\t"
			#endif
		#endif
				
		#ifndef USART3_EXTEND_RX_BUFFER
			RX3_FRAMING_EVENT
			"lds	r25, %M[UDR_reg] \n\t"
		#endif
			
		#ifndef USART3_PUSH_BEFORE_RX
			#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
				"push	r30 \n\t"
				"push	r31 \n\t"
			#else
				"movw	%[z_save], r30 \n\t"
			#endif
		#endif
		
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"lds	r31, %M[control_reg] \n\t"
			"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
		
			"sei \n\t"
		#endif
		
			RX3_EVERYCALL_EVENT
		
			"lds	r30, (rx3_Head) \n\t"
			"lds	r31, (rx3_Tail) \n\t"
		
			"inc	r30 \n\t"
		
		#if (RX3_BUFFER_MASK != 0xff)
			"andi	r30, %M[mask]\n\t"
		#endif
		
			"cp		r31, r30 \n\t"
		#if defined(USART3_USE_SOFT_RTS)||(defined(USART3_EXTEND_RX_BUFFER)&&!defined(USART_UNSAFE_RX_INTERRUPT))
			"breq	USART3_DISABLE_RXCIE \n\t"
		#elif defined(USART3_EXTEND_RX_BUFFER)&&defined(USART_UNSAFE_RX_INTERRUPT)
			"breq	USART3_RX_EXIT_SKIP \n\t"
		#else
			"breq	USART3_RX_EXIT \n\t"
		#endif
		
		#ifdef USART3_EXTEND_RX_BUFFER
			RX3_FRAMING_EVENT
			"lds	r25, %M[UDR_reg] \n\t"
		#endif
			
			RX3_EARLY_RECEIVE_EVENT
			
		#if defined(USART3_MPCM_MODE)&&!defined(MPCM3_MASTER_ONLY)
			"lds	r31, %M[UCSRA_reg] \n\t"

			"sbrs	r31, %M[mpcm_bit] \n\t"
			"rjmp	USART3_RX_CONTINUE \n\t"
			"cpi	r25, %M[mpcm_address] \n\t"
		#ifdef MPCM3_GCALL_ADDRESS
			"breq	p_%= \n\t"
			"cpi	r25, %M[mpcm_gcall_address] \n\t"
		#endif
			"brne	USART3_RX_EXIT \n\t"
		"p_%=: "
			"andi	r31, ~(1<<%M[mpcm_bit]) \n\t"

			"sts	%M[UCSRA_reg], r31 \n\t"

		"USART3_RX_CONTINUE: "
		#endif	
			
			"sts	(rx3_Head), r30 \n\t"
			
			"ldi	r31, 0x00 \n\t"
			"subi	r30, lo8(-(rx3_buffer))\n\t"
			"sbci	r31, hi8(-(rx3_buffer))\n\t"
			"st		Z, r25 \n\t"
		
			RX3_LATE_RECEIVE_EVENT
		
		"USART3_RX_EXIT: "
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"cli \n\t"
			
			"lds	r31, %M[control_reg] \n\t"
			"ori	r31, (1<<%M[rxcie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
			
		"USART3_RX_EXIT_SKIP: "
		#endif
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r31 \n\t"
			"pop	r30 \n\t"
		#else
			"movw	r30, %[z_save] \n\t"
		#endif
		
			"pop	r25 \n\t"
	
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#else
			"out	__SREG__, %[sreg_save] \n\t"
		#endif

			"reti \n\t"
		
		#if defined(USART3_USE_SOFT_RTS)||(defined(USART3_EXTEND_RX_BUFFER)&&!defined(USART_UNSAFE_RX_INTERRUPT))
		"USART3_DISABLE_RXCIE: "
		
		#ifdef USART3_USE_SOFT_RTS
			"sbi	%M[rts_port], %M[rts_pin] \n\t"
		#endif
		
		#ifndef USART_UNSAFE_RX_INTERRUPT
			"lds	r31, %M[control_reg] \n\t"
			"andi	r31, ~(1<<%M[rxcie_bit]) \n\t"
			"sts	%M[control_reg], r31 \n\t"
		#endif
			
		#ifdef USART_UNSAFE_RX_INTERRUPT
			"rjmp	USART3_RX_EXIT_SKIP \n\t"
		#else
			"rjmp	USART3_RX_EXIT \n\t"
		#endif
		#endif
			: // output operands
			USART_REG_SAVE_LIST
			
			: // input operands
			RX3_INPUT_OPERAND_LIST
			[UDR_reg]            "n" (_SFR_MEM_ADDR(UDR3_REGISTER)),
			[mask]               "M" (RX3_BUFFER_MASK),
			[mpcm_address]       "M" (MPCM3_ADDRESS),
		#ifdef MPCM3_GCALL_ADDRESS
			[mpcm_gcall_address] "M" (MPCM3_GCALL_ADDRESS),
		#endif
			[mpcm_bit]           "M" (MPCM3_BIT),
		#ifdef USART3_USE_SOFT_RTS
			[rts_port]           "M" (_SFR_IO_ADDR(RTS3_PORT)),
			[rts_pin]            "M" (RTS3_IONUM),
		#endif
			[UCSRA_reg]          "n" (_SFR_MEM_ADDR(UCSR3A_REGISTER)),
			[control_reg]        "n" (_SFR_MEM_ADDR(UCSR3B_REGISTER)),
			[rxcie_bit]          "M" (RXCIE3_BIT)
		
			// no clobbers
		);
		
	}

#endif // NO_RX3_INTERRUPT