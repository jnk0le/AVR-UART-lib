#ifndef USART_HPP
#define USART_HPP

/************************************************************************************
 *  Published on: 13-02-2015                                                        *
 *  Author: jnk0le@hotmail.com                                                      *
 *  https://github.com/jnk0le                                                       *
 *  This library is distributed under MIT license terms                             *
 ************************************************************************************/

// NOTE that this library is optimized for -Os optimization level (release build in IDEs)
// in any other case, it can generate larger code than neutral libraries

// for function calls that are twice smaller than normal, you can use -mrelax
// if any other uart library produces better code than this, you can contact me to fix this issue

// DO NOT DEFINE BUFFERS SIZES OR ANY SHARED MACROS IN 'main.cpp' CODE
// instead of this, define it in "Project Properties -> AVR C++ Compiler -> Symbols" or try to use -D gcc flag (eg. -DF_CPU=8000000)

//#define NO_USART_RX // disable all receiver code and dependencies
//#define NO_USART_TX // disable all transmitter code and dependencies

//#define USE_DOUBLE_SPEED // enables double speed for all available USART interfaces 

//#define NO_RX0_INTERRUPT // disables interrupt handling and frees RX0 gpio port // combining with NO_USART_RX is not necessary 
//#define NO_TX0_INTERRUPT // disables interrupt handling and frees TX0 gpio port // combining with NO_USART_TX is not necessary

//#define RX0_BINARY_MODE // prepare RX0 interrupt to binary transmission

//#define USART_DO_NOT_INLINE // disables inlining code typically executed once, that is heavily dependent of optimize flags

/*****************************multiple USART mcu's***********************************/

//#define NO_USART0 // disable usage of uart0
//#define NO_USART1 // disable usage of uart1 
//#define NO_USART2 // disable usage of uart2 
//#define NO_USART3 // disable usage of uart3

//#define NO_RX1_INTERRUPT // disables interrupt handling and frees RX1 gpio port // combining with NO_USART_RX is not necessary
//#define NO_RX2_INTERRUPT // disables interrupt handling and frees RX2 gpio port // combining with NO_USART_RX is not necessary
//#define NO_RX3_INTERRUPT // disables interrupt handling and frees RX3 gpio port // combining with NO_USART_RX is not necessary

//#define NO_TX1_INTERRUPT // disables interrupt handling and frees TX1 gpio port // combining with NO_USART_TX is not necessary
//#define NO_TX2_INTERRUPT // disables interrupt handling and frees TX2 gpio port // combining with NO_USART_TX is not necessary
//#define NO_TX3_INTERRUPT // disables interrupt handling and frees TX3 gpio port // combining with NO_USART_TX is not necessary

//#define RX1_BINARY_MODE // prepare RX1 interrupt to binary transmission 
//#define RX2_BINARY_MODE // prepare RX2 interrupt to binary transmission 
//#define RX3_BINARY_MODE // prepare RX3 interrupt to binary transmission 

//#define USART0_U2X_SPEED // enables double speed for USART0 // combining with USE_DOUBLE_SPEED is not necessary
//#define USART1_U2X_SPEED // enables double speed for USART1 // combining with USE_DOUBLE_SPEED is not necessary
//#define USART2_U2X_SPEED // enables double speed for USART2 // combining with USE_DOUBLE_SPEED is not necessary
//#define USART3_U2X_SPEED // enables double speed for USART3 // combining with USE_DOUBLE_SPEED is not necessary

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#ifndef F_CPU
	#error F_CPU is undefined, USART cannot work correctly without this parametr
#endif

#define BAUD_CALC(x) ((F_CPU+(x)*8UL) / (16UL*(x))-1UL) // macro calculating UBBR value
#define DOUBLE_BAUD_CALC(x) ((F_CPU+(x)*4UL) / (8UL*(x))-1UL) // macro calculating UBBR value for double speed

#ifdef DEBUG
	#warning defined DEBUG mode flag, if you want to reduce code size, switch to release mode instead
#endif

#ifndef DEFAULT_RX_BUFFER_SIZE
	#define DEFAULT_RX_BUFFER_SIZE 32 // Size of the ring buffers, must be power of 2
#endif

#ifndef DEFAULT_TX_BUFFER_SIZE
	#define DEFAULT_TX_BUFFER_SIZE 32 // Size of the ring buffers, must be power of 2
#endif

#ifndef TX0_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define TX0_BUFFER_SIZE DEFAULT_TX_BUFFER_SIZE
#endif
#ifndef RX0_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define RX0_BUFFER_SIZE DEFAULT_RX_BUFFER_SIZE
#endif
#ifndef TX1_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define TX1_BUFFER_SIZE DEFAULT_TX_BUFFER_SIZE
#endif
#ifndef RX1_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define RX1_BUFFER_SIZE DEFAULT_RX_BUFFER_SIZE
#endif
#ifndef TX2_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define TX2_BUFFER_SIZE DEFAULT_TX_BUFFER_SIZE
#endif
#ifndef RX2_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define RX2_BUFFER_SIZE DEFAULT_RX_BUFFER_SIZE
#endif
#ifndef TX3_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define TX3_BUFFER_SIZE DEFAULT_TX_BUFFER_SIZE
#endif
#ifndef RX3_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define RX3_BUFFER_SIZE DEFAULT_RX_BUFFER_SIZE
#endif

#define TX0_BUFFER_MASK (TX0_BUFFER_SIZE - 1)
#define RX0_BUFFER_MASK (RX0_BUFFER_SIZE - 1)

#define TX1_BUFFER_MASK (TX1_BUFFER_SIZE - 1)
#define RX1_BUFFER_MASK (RX1_BUFFER_SIZE - 1)

#define TX2_BUFFER_MASK (TX2_BUFFER_SIZE - 1)
#define RX2_BUFFER_MASK (RX2_BUFFER_SIZE - 1)

#define TX3_BUFFER_MASK (TX3_BUFFER_SIZE - 1)
#define RX3_BUFFER_MASK (RX3_BUFFER_SIZE - 1)

enum {locked, unlocked};
enum {COMPLETED = 0, BUFFER_EMPTY = 1};	

#ifdef NO_USART_RX // remove all RX interrupts
	#define NO_RX0_INTERRUPT
	#define NO_RX1_INTERRUPT
	#define NO_RX2_INTERRUPT
	#define NO_RX3_INTERRUPT
#endif

#ifdef NO_USART_TX // remove all TX interrupts
	#define NO_TX0_INTERRUPT
	#define NO_TX1_INTERRUPT
	#define NO_TX2_INTERRUPT
	#define NO_TX3_INTERRUPT
	
#endif

#ifdef USE_DOUBLE_SPEED 
	#define USART0_U2X_SPEED
	#define USART1_U2X_SPEED
	#define USART2_U2X_SPEED
	#define USART3_U2X_SPEED
#endif

#if defined(STDIO) && defined(STDIO_BAUDRATE) && defined(STDIO_RX_BUFSIZE) && defined(HW_TIMER) && defined(LED_PORT) && defined(LED_PIN)
	#define NO_USART0
#endif

#if defined(__AVR_ATtiny2313__)||defined(__AVR_ATtiny2313A__)||defined(__AVR_ATtiny4313)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART_RX_vect
	#define TX0_INTERRUPT		USART_TX_vect
	#define UDR0_REGISTER		UDR
	#define UBRR0L_REGISTER		UBRRL
	#define UBRR0H_REGISTER		UBRRH
	#define UCSR0A_REGISTER		UCSRA
	#define UCSR0B_REGISTER		UCSRB
	#define UCSR0C_REGISTER		UCSRC
	#define TXCIE0_BIT  		TXCIE
	#define RXCIE0_BIT  		RXCIE
	#define TXEN0_BIT   		TXEN
	#define RXEN0_BIT   		RXEN
	#define U2X0_BIT    		U2X
	
#endif //NO_USART0
#endif

#if defined(__AVR_ATmega48__)||defined(__AVR_ATmega88__)|| defined(__AVR_ATmega168__)\
||defined(__AVR_ATmega48P__) ||defined(__AVR_ATmega88P__)|| defined(__AVR_ATmega168P__)\
||defined(__AVR_ATmega328P__)||defined(__AVR_ATmega328__)||defined(__AVR_ATmega48PA__)\
||defined(__AVR_ATmega88PA__)|| defined(__AVR_ATmega168PA__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART_RX_vect
	#define TX0_INTERRUPT		USART_TX_vect
	#define UDR0_REGISTER		UDR0
	#define UBRR0L_REGISTER		UBRR0L
	#define UBRR0H_REGISTER		UBRR0H
	#define UCSR0A_REGISTER		UCSR0A
	#define UCSR0B_REGISTER		UCSR0B
	#define UCSR0C_REGISTER		UCSR0C
	#define TXCIE0_BIT  		TXCIE0
	#define RXCIE0_BIT   		RXCIE0
	#define TXEN0_BIT   		TXEN0
	#define RXEN0_BIT   		RXEN0
	#define U2X0_BIT    		U2X0

#endif //NO_USART0
#endif

#if defined(__AVR_ATmega8__)||defined(__AVR_ATmega8P__)||defined(__AVR_ATmega16__)\
||defined(__AVR_ATmega16L__)||defined(__AVR_ATmega32__)||defined(__AVR_ATmega32L__)\
||defined(__AVR_ATmega8A__)||defined(__AVR_ATmega8L__)||defined(__AVR_ATmega32A__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART_RXC_vect
	#define TX0_INTERRUPT		USART_TXC_vect
	#define UDR0_REGISTER		UDR
	#define UBRR0L_REGISTER		UBRRL
	#define UBRR0H_REGISTER		UBRRH
	#define UCSR0A_REGISTER		UCSRA
	#define UCSR0B_REGISTER		UCSRB
	#define UCSR0C_REGISTER		UCSRC
	#define TXCIE0_BIT  		TXCIE
	#define RXCIE0_BIT  		RXCIE
	#define TXEN0_BIT   		TXEN
	#define RXEN0_BIT   		RXEN
	#define U2X0_BIT    		U2X
	
#endif //NO_USART0
#endif

#if defined(__AVR_ATmega8515__)||defined(__AVR_ATmega8515L__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART_RX_vect
	#define TX0_INTERRUPT		USART_TX_vect
	#define UDR0_REGISTER		UDR
	#define UBRR0L_REGISTER		UBRRL
	#define UBRR0H_REGISTER		UBRRH
	#define UCSR0A_REGISTER		UCSRA
	#define UCSR0B_REGISTER		UCSRB
	#define UCSR0C_REGISTER		UCSRC
	#define TXCIE0_BIT  		TXCIE
	#define RXCIE0_BIT  		RXCIE
	#define TXEN0_BIT   		TXEN
	#define RXEN0_BIT   		RXEN
	#define U2X0_BIT    		U2X

#endif //NO_USART0
#endif

#if defined(__AVR_ATmega162__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART0_RXC_vect
	#define TX0_INTERRUPT		USART0_TXC_vect
	#define UDR0_REGISTER		UDR0
	#define UBRR0L_REGISTER		UBRR0L
	#define UBRR0H_REGISTER		UBRR0H
	#define UCSR0A_REGISTER		UCSR0A
	#define UCSR0B_REGISTER		UCSR0B
	#define UCSR0C_REGISTER		UCSR0C
	#define TXCIE0_BIT  		TXCIE0
	#define RXCIE0_BIT  		RXCIE0
	#define TXEN0_BIT   		TXEN0
	#define RXEN0_BIT   		RXEN0
	#define U2X0_BIT    		U2X0

#endif //NO_USART0

#ifndef NO_USART1
#define USE_USART1

	#define RX1_INTERRUPT		USART1_RXC_vect
	#define TX1_INTERRUPT		USART1_TXC_vect
	#define UDR1_REGISTER		UDR1
	#define UBRR1L_REGISTER		UBRR1L
	#define UBRR1H_REGISTER		UBRR1H
	#define UCSR1A_REGISTER		UCSR1A
	#define UCSR1B_REGISTER		UCSR1B
	#define UCSR1C_REGISTER		UCSR1C
	#define TXCIE1_BIT  		TXCIE1
	#define RXCIE1_BIT  		RXCIE1
	#define TXEN1_BIT   		TXEN1
	#define RXEN1_BIT   		RXEN1
	#define U2X1_BIT    		U2X1

#endif //NO_USART1
#endif

#if defined(__AVR_ATmega644__)||defined(__AVR_ATmega644P__)||defined(__AVR_ATmega644PA__)\
||defined(__AVR_ATmega1284__)||defined(__AVR_ATmega1284P__)||defined(__AVR_ATmega128__)\
||defined(__AVR_ATmega128L__)||defined(__AVR_ATmega64__)||defined(__AVR_ATmega64L__)\
||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)||defined(__AVR_ATmega640__)\
||defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART0_RX_vect
	#define TX0_INTERRUPT		USART0_TX_vect
	#define UDR0_REGISTER		UDR0
	#define UBRR0L_REGISTER		UBRR0L
	#define UBRR0H_REGISTER		UBRR0H
	#define UCSR0A_REGISTER		UCSR0A
	#define UCSR0B_REGISTER		UCSR0B
	#define UCSR0C_REGISTER		UCSR0C
	#define TXCIE0_BIT  		TXCIE0
	#define RXCIE0_BIT  		RXCIE0
	#define TXEN0_BIT   		TXEN0
	#define RXEN0_BIT   		RXEN0
	#define U2X0_BIT    		U2X0

#endif

#if !defined(NO_USART1) && !defined(__AVR_ATmega644__)
#define USE_USART1

	#define RX1_INTERRUPT		USART1_RX_vect
	#define TX1_INTERRUPT		USART1_TX_vect
	#define UDR1_REGISTER		UDR1
	#define UBRR1L_REGISTER		UBRR1L
	#define UBRR1H_REGISTER		UBRR1H
	#define UCSR1A_REGISTER		UCSR1A
	#define UCSR1B_REGISTER		UCSR1B
	#define UCSR1C_REGISTER		UCSR1C
	#define TXCIE1_BIT  		TXCIE1
	#define RXCIE1_BIT  		RXCIE1
	#define TXEN1_BIT   		TXEN1
	#define RXEN1_BIT   		RXEN1
	#define U2X1_BIT    		U2X1
	
#endif // NO_USART1 && 644

#endif

#if defined(__AVR_ATmega640__)||defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)

#ifndef NO_USART2
#define USE_USART2

	#define RX2_INTERRUPT		USART2_RX_vect
	#define TX2_INTERRUPT		USART2_TX_vect
	#define UDR2_REGISTER		UDR2
	#define UBRR2L_REGISTER		UBRR2L
	#define UBRR2H_REGISTER		UBRR2H
	#define UCSR2A_REGISTER		UCSR2A
	#define UCSR2B_REGISTER		UCSR2B
	#define UCSR2C_REGISTER		UCSR2C
	#define TXCIE2_BIT  		TXCIE2
	#define RXCIE2_BIT  		RXCIE2
	#define TXEN2_BIT   		TXEN2
	#define RXEN2_BIT   		RXEN2
	#define U2X2_BIT    		U2X2
	
#endif // NO_USART2

#ifndef NO_USART3
#define USE_USART3

	#define RX3_INTERRUPT		USART3_RX_vect
	#define TX3_INTERRUPT		USART3_TX_vect
	#define UDR3_REGISTER		UDR3
	#define UBRR3L_REGISTER 	UBRR3L
	#define UBRR3H_REGISTER		UBRR3H
	#define UCSR3A_REGISTER		UCSR3A
	#define UCSR3B_REGISTER		UCSR3B
	#define UCSR3C_REGISTER		UCSR3C
	#define TXCIE3_BIT  		TXCIE3
	#define RXCIE3_BIT  		RXCIE3
	#define TXEN3_BIT   		TXEN3
	#define RXEN3_BIT   		RXEN3
	#define U2X3_BIT    		U2X3
	
#endif // NO_USART3

#endif // 640/1280/2560 usart 2 & 3 

#if !defined(USE_USART0) && !defined(USE_USART1) && !defined(USE_USART2) && !defined(USE_USART3)
	#error USART not available or unknown mcu
#endif

#ifndef USE_USART0 
	#define NO_TX0_INTERRUPT
	#define NO_RX0_INTERRUPT
#endif

#ifndef USE_USART1
	#define NO_TX1_INTERRUPT
	#define NO_RX1_INTERRUPT
#endif

#ifndef USE_USART2
	#define NO_TX2_INTERRUPT
	#define NO_RX2_INTERRUPT
#endif

#ifndef USE_USART3
	#define NO_TX3_INTERRUPT
	#define NO_RX3_INTERRUPT
#endif

#ifndef USART0_CONFIG_B // set config bytes for UCSR0B_REGISTER
	
	#if defined(NO_RX0_INTERRUPT)
		#define USART0_CONFIG_B (1<<TXEN0_BIT)|(1<<TXCIE0_BIT)
	
	#elif defined(NO_TX0_INTERRUPT)
		#define USART0_CONFIG_B (1<<RXEN0_BIT)|(1<<RXCIE0_BIT)
	#else
		#define USART0_CONFIG_B (1<<TXEN0_BIT)|(1<<TXCIE0_BIT)|(1<<RXEN0_BIT)|(1<<RXCIE0_BIT)
	#endif 
#endif // USART0_CONFIG

#ifndef USART1_CONFIG_B // set config bytes for UCSR1B_REGISTER
	
	#if defined(NO_RX1_INTERRUPT)
		#define USART1_CONFIG_B (1<<TXEN1_BIT)|(1<<TXCIE1_BIT)

	#elif defined(NO_TX1_INTERRUPT)
		#define USART1_CONFIG_B (1<<RXEN1_BIT)|(1<<RXCIE1_BIT)
	#else
		#define USART1_CONFIG_B (1<<TXEN1_BIT)|(1<<TXCIE1_BIT)|(1<<RXEN1_BIT)|(1<<RXCIE1_BIT)
	#endif
#endif // USART1_CONFIG

#ifndef USART2_CONFIG_B // set config bytes for UCSR2B_REGISTER

	#if defined(NO_RX2_INTERRUPT)
		#define USART2_CONFIG_B (1<<TXEN2_BIT)|(1<<TXCIE2_BIT)

	#elif defined(NO_TX2_INTERRUPT)
		#define USART2_CONFIG_B (1<<RXEN2_BIT)|(1<<RXCIE2_BIT)
	#else
		#define USART2_CONFIG_B (1<<TXEN2_BIT)|(1<<TXCIE2_BIT)|(1<<RXEN2_BIT)|(1<<RXCIE2_BIT)
	#endif
#endif // USART2_CONFIG

#ifndef USART3_CONFIG_B // set config bytes for UCSR3B_REGISTER

	#if defined(NO_RX3_INTERRUPT)
		#define USART3_CONFIG_B (1<<TXEN3_BIT)|(1<<TXCIE3_BIT)

	#elif defined(NO_TX3_INTERRUPT)
		#define USART3_CONFIG_B (1<<RXEN3_BIT)|(1<<RXCIE3_BIT)
	#else
		#define USART3_CONFIG_B (1<<TXEN3_BIT)|(1<<TXCIE3_BIT)|(1<<RXEN3_BIT)|(1<<RXCIE3_BIT)
	#endif
#endif // USART3_CONFIG

#ifndef NO_TX0_INTERRUPT
	volatile uint8_t tx0_first_byte, tx0_last_byte, interrupt_semaphore0;
	char tx0_buffer[TX0_BUFFER_SIZE];
#endif

#ifndef NO_RX0_INTERRUPT
	volatile uint8_t rx0_first_byte, rx0_last_byte;
	char rx0_buffer[RX0_BUFFER_SIZE];
#endif

#ifndef NO_TX1_INTERRUPT
	volatile uint8_t tx1_first_byte, tx1_last_byte, interrupt_semaphore1;
	char tx1_buffer[TX1_BUFFER_SIZE];
#endif

#ifndef NO_RX1_INTERRUPT
	volatile uint8_t rx1_first_byte, rx1_last_byte;
	char rx1_buffer[RX1_BUFFER_SIZE];
#endif

#ifndef NO_TX2_INTERRUPT
	volatile uint8_t tx2_first_byte, tx2_last_byte, interrupt_semaphore2;
	char tx2_buffer[TX2_BUFFER_SIZE];
#endif

#ifndef NO_RX2_INTERRUPT
	volatile uint8_t rx2_first_byte, rx2_last_byte;
	char rx2_buffer[RX2_BUFFER_SIZE];
#endif

#ifndef NO_TX3_INTERRUPT
	volatile uint8_t tx3_first_byte, tx3_last_byte, interrupt_semaphore3;
	char tx3_buffer[TX3_BUFFER_SIZE];
#endif

#ifndef NO_RX3_INTERRUPT
	volatile uint8_t rx3_first_byte, rx3_last_byte;
	char rx3_buffer[RX3_BUFFER_SIZE];
#endif

/************************************************************************************
 *                            Initializers                                          *
 ************************************************************************************/
#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)
	
#ifndef USART_DO_NOT_INLINE
	// functions smaller then calling routines or executed once are inline for reducing
	// flash memory usage (eg. switch and if routines can be executed during compilation)

	inline void uart_init(uint8_t usartct, uint16_t ubbr_value)
	{
		switch(usartct)
		{
		#ifdef USE_USART0
			default:
			UBRR0L_REGISTER = (uint8_t) ubbr_value;
			
			if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
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
			
			if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
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
			
			if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
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
			
			if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
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
	
	// UCSRC_reg can be used to set other than 8n1 transmission
	inline void uart_set_UCSRC(uint8_t usartct, uint8_t UCSRC_reg)
	{
		switch(usartct)
		{
		#ifdef USE_USART0
			default: UCSR0C_REGISTER |= UCSRC_reg; break;
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
	
	inline void uart_set_U2X(uint8_t usartct) // use instead of USE_U2Xn_SPEED
	{
		switch(usartct)
		{
		#ifdef USE_USART0
			default: UCSR0A_REGISTER |= (1<<U2X0_BIT); break;
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
#else // USART_DO_NOT_INLINE
	
	void uart_init(uint8_t usartct, uint16_t ubbr_value);
	void uart_set_UCSRC(uint8_t usartct, uint8_t UCSRC_reg);
	void uart_set_U2X(uint8_t usartct); // use instead of USE_U2Xn_SPEED
	
#endif

#else // single USART mcu
	
#ifndef USART_DO_NOT_INLINE
	// functions smaller then calling routines or executed once are inline for reducing 
	// flash memory usage (eg. switch and if routines can be executed during compilation)
	
	inline void uart_init(uint16_t ubbr_value) // function called once
	{
		UBRR0L_REGISTER = (uint8_t) ubbr_value;
		
		if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
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
	}
	
#else // USART_DO_NOT_INLINE

	void uart_init(uint16_t ubbr_value);

#endif

	// UCSRC_reg can be used to set other than 8n1 transmission
	inline void uart_set_UCSRC(uint8_t UCSRC_reg) 
	{
		UCSR0C_REGISTER |= UCSRC_reg;
	} 
	
	inline void uart_set_U2X(void) // use instead of USE_DOUBLE_SPEED 
	{
		UCSR0A_REGISTER |= (1<<U2X0_BIT);
	} 

#endif // single/multi USART
	
/************************************************************************************
 *                          Transmitter functions                                   *
 ************************************************************************************/
#ifndef NO_USART_TX

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)
	
	void uart_putc(uint8_t usartct, char data); // put character/data into transmitter ring buffer
	
	void uart_putstrl(uint8_t usartct, char *string, uint8_t BytesToWrite); // in case of bascom users or buffers without NULL byte ending
	void uart_putstr(uint8_t usartct, char *string); // send string from the dynamic buffer 
	// stops when NULL byte is hit (NULL byte is not included into transmission)
	#ifdef __cplusplus
		#define uart_puts(_usartct,str) uart_putstr(_usartct,const_cast<char*>(str))// macro to avoid const *char conversion restrictions 
	#else
		#define uart_puts(_usartct,str) uart_putstr(_usartct,str)
	#endif
		// for deprecated usage only (wastes ram data memory to keep all string constants), instead of this try to use puts_P

	void uart_puts_p(uint8_t usartct, const char *string); // send string from flash memory 
		#define uart_puts_P(_usartct,__s)    uart_puts_p(_usartct,PSTR(__s)) 
		// macro to automatically put a string constant into flash
	
	void uart_putint(uint8_t usartct, int16_t data);
	void uart_put_hex(uint8_t usartct, int16_t data);
	void uart_putlong(uint8_t usartct, int32_t data);
	void uart_putfloat(uint8_t usartct, float data);
	void uart_fputfloat(uint8_t usartct, float data, uint8_t size, uint8_t precision);

#else // single USART mcu

	void uart_putc(char data); // put character/data into transmitter ring buffer
	
	void uart_putstrl(char *string, uint8_t BytesToWrite); // in case of bascom users or buffers without NULL byte ending
	void uart_putstr(char *string); // send string from the dynamic buffer
		// stops when NULL byte is hit (NULL byte is not included into transmission)
	#ifdef __cplusplus
		#define uart_puts(str) uart_putstr(const_cast<char*>(str))// macro to avoid const *char conversion restrictions
	#else
		#define uart_puts(str) uart_putstr(str)
	#endif
		// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P

	void uart_puts_p(const char *string); // send string from flash memory
		#define uart_puts_P(__s)    uart_puts_p(PSTR(__s))
		// macro to automatically put a string constant into flash
	
	void uart_putint(int16_t data);
	void uart_put_hex(int16_t data);
	void uart_putlong(int32_t data);
	void uart_putfloat(float data);
	void uart_fputfloat(float data, uint8_t size, uint8_t precision);

#endif // single/multi USART

#endif // NO_USART_TX

/************************************************************************************
 *                           Receiver functions                                     *
 ************************************************************************************/
#ifndef NO_USART_RX

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)
	
	char uart_getc(uint8_t usartct); // get character from receiver ring buffer
	void uart_gets(uint8_t usartct, char *buffer); // DEPRECATED, only in case of optimizing flash usage, instead of this try to use limited gets
	// to avoid stack/buffer overflows, temp buffer size have to be the same as ring buffer or bigger 
	// adds NULL byte at the end of string
	void uart_getsl(uint8_t usartct, char *buffer, uint8_t bufferlimit); // stops reading if NULL byte or bufferlimit-1 is hit 
	// adds NULL byte at the end of string (positioned at bufferlimit-1)
	uint8_t uart_getData(uint8_t usartct, uint8_t *data); // reads binary data from a buffer and loads it into *data byte 
	// in case of empty buffers returning flag is set to BUFFER_EMPTY (1) 
	// don't forget to set RX0_BINARY_MODE flag
	uint8_t uart_AvailableBytes(uint8_t usartct); // returns number of bytes waiting in the receiver buffer
	//uint8_t uart_peek(uint8_t usartct); CSIIWWTMMFA

#else // single USART mcu

	char uart_getc(void); // get character from receiver ring buffer
	void uart_gets(char *buffer); // DEPRECATED, only in case of optimizing flash usage, instead of this try to use limited gets
	// to avoid stack/buffer overflows, temp buffer size have to be the same as ring buffer or bigger
	// adds NULL byte at the end of string
	void uart_getsl(char *buffer, uint8_t bufferlimit); // stops reading if NULL byte or bufferlimit-1 is hit
	// adds NULL byte at the end of string (positioned at bufferlimit-1)
	uint8_t uart_getData(uint8_t *data); // reads binary data from a buffer and loads it into *data byte
	// in case of empty buffers returning flag is set to BUFFER_EMPTY (1)
	// don't forget to set RXn_BINARY_MODE flag
	uint8_t uart_AvailableBytes(void); // returns number of bytes waiting in the receiver buffer
	//uint8_t uart_peek(void); CSIIWWTMMFA

#endif // single/multi USART

#endif // NO_USART_RX


#endif // USART_HPP