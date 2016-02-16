#ifndef _USART_H_
#define _USART_H_

/************************************************************************************
 *  Published on: 21-02-2015                                                        *
 *  Author: jnk0le@hotmail.com                                                      *
 *  https://github.com/jnk0le                                                       *
 *  This library is distributed under MIT license terms                             *
 ************************************************************************************/

// DO NOT DEFINE BUFFERS SIZES OR ANY SHARED MACROS IN 'main.c' CODE
// instead of this, define it in "Project Properties -> AVR C Compiler -> Symbols" or try to use -D gcc flag (eg. -DF_CPU=8000000)

//#define NO_USART_RX // disable all receiver code and dependencies
//#define NO_USART_TX // disable all transmitter code and dependencies

//#define USART_RS485_MODE // globally enable rs485 operation mode // RE have to be shorted with DE

//#define USE_DOUBLE_SPEED // enables double speed for all available USART interfaces 

#define RX_STDIO_GETCHAR_ECHO // echoes back received characters in getchar() function (for reading in scanf()) 
#define RX_GETC_ECHO // echoes back received characters in getc() function

//#define PUTC_CONVERT_LF_TO_CRLF // allow for unix style (\n only) newline terminator in stored strings // not included into putc_noblock
#define RX_NEWLINE_MODE 2 // 0 - \r,  1 - \n,  2 - \r\n
// lot of terminals sends only \r character as a newline terminator, instead of \r\n or even unix style \n
// (BTW PuTTY doesn't allow to change this) but in return requires \r\n terminator to show not broken text

//#define USART_UNSAFE_RX_INTERRUPT // modify RX interrupt to meet 25 cycle restriction // UDRE is too hungry, only RX ISRs works now
/*****************************multiple USART mcu's***********************************/

//#define NO_USART0 // disable usage of uart0
//#define NO_USART1 // disable usage of uart1
//#define NO_USART2 // disable usage of uart2
//#define NO_USART3 // disable usage of uart3

//#define NO_RX0_INTERRUPT // disables interrupt handling and frees RX0 gpio port // combining with NO_USART_RX is not necessary 
//#define NO_RX1_INTERRUPT // disables interrupt handling and frees RX1 gpio port // combining with NO_USART_RX is not necessary
//#define NO_RX2_INTERRUPT // disables interrupt handling and frees RX2 gpio port // combining with NO_USART_RX is not necessary
//#define NO_RX3_INTERRUPT // disables interrupt handling and frees RX3 gpio port // combining with NO_USART_RX is not necessary

//#define NO_TX0_INTERRUPT // disables interrupt handling and frees TX0 gpio port // combining with NO_USART_TX is not necessary
//#define NO_TX1_INTERRUPT // disables interrupt handling and frees TX1 gpio port // combining with NO_USART_TX is not necessary
//#define NO_TX2_INTERRUPT // disables interrupt handling and frees TX2 gpio port // combining with NO_USART_TX is not necessary
//#define NO_TX3_INTERRUPT // disables interrupt handling and frees TX3 gpio port // combining with NO_USART_TX is not necessary

//#define USART0_U2X_SPEED // enables double speed for USART0 // combining with USE_DOUBLE_SPEED is not necessary
//#define USART1_U2X_SPEED // enables double speed for USART1 // combining with USE_DOUBLE_SPEED is not necessary
//#define USART2_U2X_SPEED // enables double speed for USART2 // combining with USE_DOUBLE_SPEED is not necessary
//#define USART3_U2X_SPEED // enables double speed for USART3 // combining with USE_DOUBLE_SPEED is not necessary

//#define RX0_GETC_ECHO
//#define RX1_GETC_ECHO
//#define RX2_GETC_ECHO
//#define RX3_GETC_ECHO

//#define USART0_RS485_MODE // set correct DE pin first 
//#define USART1_RS485_MODE	// set correct DE pin first 
//#define USART2_RS485_MODE	// set correct DE pin first 
//#define USART3_RS485_MODE	// set correct DE pin first 

/*****************************RS 485 config***********************************/

//#define RS485_CONTROL0_PORT D // A,B,C,D ... port naming - define valid port connected to DE + RE 
//#define RS485_CONTROL0_PIN 2 // 1,2,3,4 ... pin naming - define valid pin connected to DE + RE

//#define RS485_CONTROL1_PORT 
//#define RS485_CONTROL1_PIN 

//#define RS485_CONTROL2_PORT 
//#define RS485_CONTROL2_PIN

//#define RS485_CONTROL3_PORT
//#define RS485_CONTROL3_PIN

#include <avr/io.h> // for inline func

#ifndef F_CPU
	#error F_CPU is undefined, USART cannot work correctly without this parametr
#endif

#define BAUD_CALC(x) ((F_CPU+(x)*8UL) / (16UL*(x))-1UL) // macro calculating UBBR value
#define DOUBLE_BAUD_CALC(x) ((F_CPU+(x)*4UL) / (8UL*(x))-1UL) // macro calculating UBBR value for double speed

#ifndef __OPTIMIZE__
	#warning "Compiler optimizations disabled; functions from usart.h won't work as designed"
	//#define USART_DO_NOT_INLINE
#endif
	
#ifdef DEBUG
	//#define USART_DO_NOT_INLINE
#endif

#ifndef RX_BUFFER_SIZE
	#define RX_BUFFER_SIZE 32 // Size of the ring buffers, must be power of 2
#endif

#ifndef TX_BUFFER_SIZE
	#define TX_BUFFER_SIZE 32 // Size of the ring buffers, must be power of 2
#endif

#ifndef TX0_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define TX0_BUFFER_SIZE TX_BUFFER_SIZE
#endif
#ifndef RX0_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define RX0_BUFFER_SIZE RX_BUFFER_SIZE
#endif
#ifndef TX1_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define TX1_BUFFER_SIZE TX_BUFFER_SIZE
#endif
#ifndef RX1_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define RX1_BUFFER_SIZE RX_BUFFER_SIZE
#endif
#ifndef TX2_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define TX2_BUFFER_SIZE TX_BUFFER_SIZE
#endif
#ifndef RX2_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define RX2_BUFFER_SIZE RX_BUFFER_SIZE
#endif
#ifndef TX3_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define TX3_BUFFER_SIZE TX_BUFFER_SIZE
#endif
#ifndef RX3_BUFFER_SIZE // Size of the ring buffers, must be power of 2
	#define RX3_BUFFER_SIZE RX_BUFFER_SIZE
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
enum {COMPLETED = 1, BUFFER_EMPTY = 0, BUFFER_FULL=0};	

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

#ifdef RX_GETC_ECHO
	#define RX0_GETC_ECHO
	#define RX1_GETC_ECHO
	#define RX2_GETC_ECHO
	#define RX3_GETC_ECHO
#endif

#ifdef USART_RS485_MODE
	#define USART0_RS485_MODE
	#define USART1_RS485_MODE
	#define USART2_RS485_MODE
	#define USART3_RS485_MODE
#endif

#ifdef USART0_RS485_MODE
	#if !defined(RS485_CONTROL0_PORT)||!defined(RS485_CONTROL0_PIN)
		#error "define valid DE/RE output for USART0 RS485 operation"
	#endif
#endif

#ifdef USART1_RS485_MODE
	#if !defined(RS485_CONTROL1_PORT)||!defined(RS485_CONTROL1_PIN)
		#error "define valid DE/RE output for USART1 RS485 operation"
	#endif
#endif

#ifdef USART2_RS485_MODE
	#if !defined(RS485_CONTROL2_PORT)||!defined(RS485_CONTROL2_PIN)
		#error "define valid DE/RE output for USART2 RS485 operation"
	#endif
#endif

#ifdef USART3_RS485_MODE
	#if !defined(RS485_CONTROL3_PORT)||!defined(RS485_CONTROL3_PIN)
		#error "define valid DE/RE output for USART3 RS485 operation"
	#endif
#endif

#ifndef ___DDR
	#define ___DDR(x) ___XDDR(x)
#endif
#ifndef ___XDDR
	#define ___XDDR(x) (DDR ## x)
#endif

#ifndef ___DDR
	#define ___PORT(x) ___XPORT(x)
#endif
#ifndef ___DDR
	#define ___XPORT(x) (PORT ## x)
#endif

#ifdef RX_NEWLINE_MODE
	
	#if (RX_NEWLINE_MODE == 0)
		#define RX_NEWLINE_MODE_R
	#elif (RX_NEWLINE_MODE == 1)
		#define RX_NEWLINE_MODE_N
	#else // RX_NEWLINE_MODE == 2
		#define RX_NEWLINE_MODE_RN
	#endif
#else
	#define RX_NEWLINE_MODE_RN // 2
#endif

#ifdef __usbdrv_h_included__ // V-USB
	#define USART_UNSAFE_RX_INTERRUPT
#endif

#if defined(__AVR_ATtiny2313__)||defined(__AVR_ATtiny2313A__)||defined(__AVR_ATtiny4313)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART_RX_vect
	#define TXC0_INTERRUPT  	USART_TX_vect
	#define UDRE0_INTERRUPT		USART_UDRE_vect
	#define UDR0_REGISTER		UDR
	#define UBRR0L_REGISTER		UBRRL
	#define UBRR0H_REGISTER		UBRRH
	#define UCSR0A_REGISTER		UCSRA
	#define UCSR0B_REGISTER		UCSRB
	#define UCSR0C_REGISTER		UCSRC
	#define TXCIE0_BIT      	TXCIE
	#define UDRIE0_BIT    		UDRIE
	#define RXCIE0_BIT  		RXCIE
	#define TXEN0_BIT   		TXEN
	#define RXEN0_BIT   		RXEN
	#define U2X0_BIT    		U2X
	
#endif //NO_USART0
#endif

#if defined(__AVR_ATmega48__)||defined(__AVR_ATmega48P__)||defined(__AVR_ATmega48PA__)||defined(__AVR_ATmega48PB__)\
||defined(__AVR_ATmega88__)||defined(__AVR_ATmega88P__)||defined(__AVR_ATmega88PA__)||defined(__AVR_ATmega88PB__)\
||defined(__AVR_ATmega168__)||defined(__AVR_ATmega168P__)||defined(__AVR_ATmega168PA__)||defined(__AVR_ATmega168PB__)\
||defined(__AVR_ATmega328__)||defined(__AVR_ATmega328P__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART_RX_vect
	#define TXC0_INTERRUPT  	USART_TX_vect
	#define UDRE0_INTERRUPT		USART_UDRE_vect
	#define UDR0_REGISTER		UDR0
	#define UBRR0L_REGISTER		UBRR0L
	#define UBRR0H_REGISTER		UBRR0H
	#define UCSR0A_REGISTER		UCSR0A
	#define UCSR0B_REGISTER		UCSR0B
	#define UCSR0C_REGISTER		UCSR0C
	#define TXCIE0_BIT      	TXCIE0
	#define UDRIE0_BIT    		UDRIE0
	#define RXCIE0_BIT   		RXCIE0
	#define TXEN0_BIT   		TXEN0
	#define RXEN0_BIT   		RXEN0
	#define U2X0_BIT    		U2X0

#endif //NO_USART0
#endif

#if defined(__AVR_ATmega328PB__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART0_RX_vect
	#define TXC0_INTERRUPT		USART0_TX_vect
	#define UDRE0_INTERRUPT		USART0_UDRE_vect
	#define UDR0_REGISTER		UDR0
	#define UBRR0L_REGISTER		UBRR0L
	#define UBRR0H_REGISTER		UBRR0H
	#define UCSR0A_REGISTER		UCSR0A
	#define UCSR0B_REGISTER		UCSR0B
	#define UCSR0C_REGISTER		UCSR0C
	#define TXCIE0_BIT      	TXCIE0
	#define UDRIE0_BIT    		UDRIE0
	#define RXCIE0_BIT   		RXCIE0
	#define TXEN0_BIT   		TXEN0
	#define RXEN0_BIT   		RXEN0
	#define U2X0_BIT    		U2X0

#endif //NO_USART0

#ifndef NO_USART1
#define USE_USART1

	#define RX1_INTERRUPT		USART1_RX_vect
	#define TXC1_INTERRUPT  	USART1_TX_vect
	#define UDRE1_INTERRUPT		USART1_UDRE_vect
	#define UDR1_REGISTER		UDR1
	#define UBRR1L_REGISTER		UBRR1L
	#define UBRR1H_REGISTER		UBRR1H
	#define UCSR1A_REGISTER		UCSR1A
	#define UCSR1B_REGISTER		UCSR1B
	#define UCSR1C_REGISTER		UCSR1C
	#define TXCIE1_BIT      	TXCIE1
	#define UDRIE1_BIT    		UDRIE1
	#define RXCIE1_BIT   		RXCIE1
	#define TXEN1_BIT   		TXEN1
	#define RXEN1_BIT   		RXEN1
	#define U2X1_BIT    		U2X1

#endif //NO_USART1
#endif

#if defined(__AVR_ATmega8__)||defined(__AVR_ATmega8P__)||defined(__AVR_ATmega16__)\
||defined(__AVR_ATmega16A__)||defined(__AVR_ATmega32__)||defined(__AVR_ATmega32A__)\
||defined(__AVR_ATmega8A__)||defined(__AVR_ATmega8L__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART_RXC_vect
	#define TXC0_INTERRUPT		USART_TXC_vect
	#define UDRE0_INTERRUPT 	USART_UDRE_vect 
	#define UDR0_REGISTER		UDR
	#define UBRR0L_REGISTER		UBRRL
	#define UBRR0H_REGISTER		UBRRH
	#define UCSR0A_REGISTER		UCSRA
	#define UCSR0B_REGISTER		UCSRB
	#define UCSR0C_REGISTER		UCSRC
	#define TXCIE0_BIT	    	TXCIE
	#define UDRIE0_BIT    		UDRIE
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
	#define TXC0_INTERRUPT		USART_TX_vect
	#define UDRE0_INTERRUPT		USART_UDRE_vect 
	#define UDR0_REGISTER		UDR
	#define UBRR0L_REGISTER		UBRRL
	#define UBRR0H_REGISTER		UBRRH
	#define UCSR0A_REGISTER		UCSRA
	#define UCSR0B_REGISTER		UCSRB
	#define UCSR0C_REGISTER		UCSRC
	#define TXCIE0_BIT      	TXCIE
	#define UDRIE0_BIT    		UDRIE
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
	#define TXC0_INTERRUPT		USART0_TXC_vect
	#define UDRE0_INTERRUPT		USART0_UDRE_vect 
	#define UDR0_REGISTER		UDR0
	#define UBRR0L_REGISTER		UBRR0L
	#define UBRR0H_REGISTER		UBRR0H
	#define UCSR0A_REGISTER		UCSR0A
	#define UCSR0B_REGISTER		UCSR0B
	#define UCSR0C_REGISTER		UCSR0C
	#define TXCIE0_BIT      	TXCIE0
	#define UDRIE0_BIT    		UDRIE0
	#define RXCIE0_BIT  		RXCIE0
	#define TXEN0_BIT   		TXEN0
	#define RXEN0_BIT   		RXEN0
	#define U2X0_BIT    		U2X0

#endif //NO_USART0

#ifndef NO_USART1
#define USE_USART1

	#define RX1_INTERRUPT		USART1_RXC_vect
	/#define TXC1_INTERRUPT		USART1_TXC_vect
	#define UDRE1_INTERRUPT		USART1_UDRE_vect 
	#define UDR1_REGISTER		UDR1
	#define UBRR1L_REGISTER		UBRR1L
	#define UBRR1H_REGISTER		UBRR1H
	#define UCSR1A_REGISTER		UCSR1A
	#define UCSR1B_REGISTER		UCSR1B
	#define UCSR1C_REGISTER		UCSR1C
	#define TXCIE1_BIT      	TXCIE1
	#define UDRIE1_BIT    		UDRIE1
	#define RXCIE1_BIT  		RXCIE1
	#define TXEN1_BIT   		TXEN1
	#define RXEN1_BIT   		RXEN1
	#define U2X1_BIT    		U2X1

#endif //NO_USART1
#endif

#if defined(__AVR_ATmega644__)||defined(__AVR_ATmega644P__)||defined(__AVR_ATmega644PA__)\
||defined(__AVR_ATmega1284__)||defined(__AVR_ATmega1284P__)||defined(__AVR_ATmega128__)\
||defined(__AVR_ATmega128A__)||defined(__AVR_ATmega64__)||defined(__AVR_ATmega64A__)\
||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)||defined(__AVR_ATmega640__)\
||defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega164P__)\
||defined(__AVR_ATmega324P__)||defined(__AVR_ATmega324A__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART0_RX_vect
	#define TXC0_INTERRUPT		USART0_TX_vect
	#define UDRE0_INTERRUPT		USART0_UDRE_vect 
	#define UDR0_REGISTER		UDR0
	#define UBRR0L_REGISTER		UBRR0L
	#define UBRR0H_REGISTER		UBRR0H
	#define UCSR0A_REGISTER		UCSR0A
	#define UCSR0B_REGISTER		UCSR0B
	#define UCSR0C_REGISTER		UCSR0C
	#define TXCIE0_BIT      	TXCIE0
	#define UDRIE0_BIT    		UDRIE0
	#define RXCIE0_BIT  		RXCIE0
	#define TXEN0_BIT   		TXEN0
	#define RXEN0_BIT   		RXEN0
	#define U2X0_BIT    		U2X0

#endif

#if !defined(NO_USART1) && !defined(__AVR_ATmega644__)
#define USE_USART1

	#define RX1_INTERRUPT		USART1_RX_vect
	#define TXC1_INTERRUPT		USART1_TX_vect
	#define UDRE1_INTERRUPT		USART1_UDRE_vect 
	#define UDR1_REGISTER		UDR1
	#define UBRR1L_REGISTER		UBRR1L
	#define UBRR1H_REGISTER		UBRR1H
	#define UCSR1A_REGISTER		UCSR1A
	#define UCSR1B_REGISTER		UCSR1B
	#define UCSR1C_REGISTER		UCSR1C
	#define TXCIE1_BIT      	TXCIE1
	#define UDRIE1_BIT    		UDRIE1
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
	#define TXC2_INTERRUPT		USART2_TX_vect
	#define UDRE2_INTERRUPT		USART2_UDRE_vect 
	#define UDR2_REGISTER		UDR2
	#define UBRR2L_REGISTER		UBRR2L
	#define UBRR2H_REGISTER		UBRR2H
	#define UCSR2A_REGISTER		UCSR2A
	#define UCSR2B_REGISTER		UCSR2B
	#define UCSR2C_REGISTER		UCSR2C
	#define TXCIE2_BIT      	TXCIE2
	#define UDRIE2_BIT    		UDRIE2
	#define RXCIE2_BIT  		RXCIE2
	#define TXEN2_BIT   		TXEN2
	#define RXEN2_BIT   		RXEN2
	#define U2X2_BIT    		U2X2
	
#endif // NO_USART2

#ifndef NO_USART3
#define USE_USART3

	#define RX3_INTERRUPT		USART3_RX_vect
	#define TXC3_INTERRUPT		USART3_TX_vect
	#define UDRE3_INTERRUPT		USART3_UDRE_vect 
	#define UDR3_REGISTER		UDR3
	#define UBRR3L_REGISTER 	UBRR3L
	#define UBRR3H_REGISTER		UBRR3H
	#define UCSR3A_REGISTER		UCSR3A
	#define UCSR3B_REGISTER		UCSR3B
	#define UCSR3C_REGISTER		UCSR3C
	#define TXCIE3_BIT      	TXCIE3
	#define UDRIE3_BIT    		UDRIE3
	#define RXCIE3_BIT  		RXCIE3
	#define TXEN3_BIT   		TXEN3
	#define RXEN3_BIT   		RXEN3
	#define U2X3_BIT    		U2X3
	
#endif // NO_USART3

#endif // 640/1280/2560 usart 2 & 3 

#if defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega16U4__)\
||defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega32U6__)

#ifndef NO_USART0 // we will call the only usart, as an usart0
#define USE_USART0

	#define RX0_INTERRUPT		USART1_RX_vect
	#define TXC0_INTERRUPT		USART1_TX_vect
	#define UDRE0_INTERRUPT		USART1_UDRE_vect 
	#define UDR0_REGISTER		UDR1
	#define UBRR0L_REGISTER		UBRR1L
	#define UBRR0H_REGISTER		UBRR1H
	#define UCSR0A_REGISTER		UCSR1A
	#define UCSR0B_REGISTER		UCSR1B
	#define UCSR0C_REGISTER		UCSR1C
	#define TXCIE0_BIT      	TXCIE1
	#define UDRIE0_BIT    		UDRIE1
	#define RXCIE0_BIT  		RXCIE1
	#define TXEN0_BIT   		TXEN1
	#define RXEN0_BIT   		RXEN1
	#define U2X0_BIT    		U2X1

#endif // NO_USART0
#endif

#if defined(__AVR_ATmega169A__)||defined(__AVR_ATmega169__)||defined(__AVR_ATmega169P__)||defined(__AVR_ATmega169PA__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART0_RX_vect
	#define TXC0_INTERRUPT		USART0_TX_vect
	#define UDRE0_INTERRUPT		USART0_UDRE_vect 
	#define UDR0_REGISTER		UDR
	#define UBRR0L_REGISTER		UBRRL
	#define UBRR0H_REGISTER		UBRRH
	#define UCSR0A_REGISTER		UCSRA
	#define UCSR0B_REGISTER		UCSRB
	#define UCSR0C_REGISTER		UCSRC
	#define TXCIE0_BIT      	TXCIE
	#define UDRIE0_BIT    		UDRIE
	#define RXCIE0_BIT  		RXCIE
	#define TXEN0_BIT   		TXEN
	#define RXEN0_BIT   		RXEN
	#define U2X0_BIT    		U2X

#endif //NO_USART0
#endif

#if defined(__AVR_ATmega329__)||defined(__AVR_ATmega329P__)||defined(__AVR_ATmega329PA__)\
||defined(__AVR_ATmega329A__)||defined(__AVR_ATmega649__)||defined(__AVR_ATmega649A__)\
||defined(__AVR_ATmega649P__)||defined(__AVR_ATmega169P__)||defined(__AVR_ATmega169PA__)\
||defined(__AVR_ATmega325__)||defined(__AVR_ATmega325A__)||defined(__AVR_ATmega325P__)\
||defined(__AVR_ATmega325PA__)||defined(__AVR_ATmega645__)||defined(__AVR_ATmega645A__)\
||defined(__AVR_ATmega645P__)


#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART0_RX_vect
	#define TXC0_INTERRUPT		USART0_TX_vect
	#define UDRE0_INTERRUPT		USART0_UDRE_vect 
	#define UDR0_REGISTER		UDR0
	#define UBRR0L_REGISTER		UBRR0L
	#define UBRR0H_REGISTER		UBRR0H
	#define UCSR0A_REGISTER		UCSR0A
	#define UCSR0B_REGISTER		UCSR0B
	#define UCSR0C_REGISTER		UCSR0C
	#define TXCIE0_BIT      	TXCIE0
	#define UDRIE0_BIT    		UDRIE0
	#define RXCIE0_BIT  		RXCIE0
	#define TXEN0_BIT   		TXEN0
	#define RXEN0_BIT   		RXEN0
	#define U2X0_BIT    		U2X0

#endif //NO_USART0
#endif

#if defined(__AVR_ATmega3290__)||defined(__AVR_ATmega6490__)||defined(__AVR_ATmega3290P__)\
||defined(__AVR_ATmega3290PA__)||defined(__AVR_ATmega3290A__)||defined(__AVR_ATmega6490A__)\
||defined(__AVR_ATmega6490P__)||defined(__AVR_ATmega3250__)||defined(__AVR_ATmega3250A__)\
||defined(__AVR_ATmega3250P__)||defined(__AVR_ATmega3250PA__)||defined(__AVR_ATmega6450__)\
||defined(__AVR_ATmega6450A__)||defined(__AVR_ATmega6450P__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART_RX_vect // wtf
	#define TXC0_INTERRUPT		USART0_TX_vect
	#define UDRE0_INTERRUPT		USART0_UDRE_vect 
	#define UDR0_REGISTER		UDR0
	#define UBRR0L_REGISTER		UBRR0L
	#define UBRR0H_REGISTER		UBRR0H
	#define UCSR0A_REGISTER		UCSR0A
	#define UCSR0B_REGISTER		UCSR0B
	#define UCSR0C_REGISTER		UCSR0C
	#define TXCIE0_BIT      	TXCIE0
	#define UDRIE0_BIT    		UDRIE0
	#define RXCIE0_BIT  		RXCIE0
	#define TXEN0_BIT   		TXEN0
	#define RXEN0_BIT   		RXEN0
	#define U2X0_BIT    		U2X0

#endif //NO_USART0
#endif

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
		#define USART0_CONFIG_B (1<<TXEN0_BIT)
	
	#elif defined(NO_TX0_INTERRUPT)
		#define USART0_CONFIG_B (1<<RXEN0_BIT)|(1<<RXCIE0_BIT)
	#else
		#ifdef USART0_RS485_MODE
			#define USART0_CONFIG_B (1<<TXEN0_BIT)|(1<<TXCIE0_BIT)|(1<<RXEN0_BIT)|(1<<RXCIE0_BIT)
		#else
			#define USART0_CONFIG_B (1<<TXEN0_BIT)|(1<<RXEN0_BIT)|(1<<RXCIE0_BIT)
		#endif
	#endif 
#endif // USART0_CONFIG

#ifndef USART1_CONFIG_B // set config bytes for UCSR1B_REGISTER
	
	#if defined(NO_RX1_INTERRUPT)
		#define USART1_CONFIG_B (1<<TXEN1_BIT)

	#elif defined(NO_TX1_INTERRUPT)
		#define USART1_CONFIG_B (1<<RXEN1_BIT)|(1<<RXCIE1_BIT)
	#else
		#ifdef USART1_RS485_MODE
			#define USART1_CONFIG_B (1<<TXEN1_BIT)|(1<<TXCIE1_BIT)|(1<<RXEN1_BIT)|(1<<RXCIE1_BIT)
		#else
			#define USART1_CONFIG_B (1<<TXEN1_BIT)|(1<<RXEN1_BIT)|(1<<RXCIE1_BIT)
		#endif
	#endif
#endif // USART1_CONFIG

#ifndef USART2_CONFIG_B // set config bytes for UCSR2B_REGISTER

	#if defined(NO_RX2_INTERRUPT)
		#define USART2_CONFIG_B (1<<TXEN2_BIT)

	#elif defined(NO_TX2_INTERRUPT)
		#define USART2_CONFIG_B (1<<RXEN2_BIT)|(1<<RXCIE2_BIT)
	#else
		#ifdef USART2_RS485_MODE
			#define USART2_CONFIG_B (1<<TXEN2_BIT)|(1<<TXCIE2_BIT)|(1<<RXEN2_BIT)|(1<<RXCIE2_BIT)
		#else	
			#define USART2_CONFIG_B (1<<TXEN2_BIT)|(1<<RXEN2_BIT)|(1<<RXCIE2_BIT)
		#endif
	#endif
#endif // USART2_CONFIG

#ifndef USART3_CONFIG_B // set config bytes for UCSR3B_REGISTER

	#if defined(NO_RX3_INTERRUPT)
		#define USART3_CONFIG_B (1<<TXEN3_BIT)

	#elif defined(NO_TX3_INTERRUPT)
		#define USART3_CONFIG_B (1<<RXEN3_BIT)|(1<<RXCIE3_BIT)
	#else
		#ifdef USART3_RS485_MODE
			#define USART3_CONFIG_B (1<<TXEN3_BIT)|(1<<TXCIE3_BIT)|(1<<RXEN3_BIT)|(1<<RXCIE3_BIT)
		#else
			#define USART3_CONFIG_B (1<<TXEN3_BIT)|(1<<RXEN3_BIT)|(1<<RXCIE3_BIT)
		#endif
	#endif
#endif // USART3_CONFIG

/************************************************************************************
 *                            Initializers                                          *
 ************************************************************************************/
#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)
	
	// functions smaller then calling routines or executed once are inline for reducing
	// flash memory usage (eg. switch and if routines can be executed during compilation)

	void uart_reinit(uint8_t usartct, uint16_t ubbr_value); // for runtime reinitialization of uart module 

	inline void uart_init(uint8_t usartct, uint16_t ubbr_value) // have to be called once at startup with parameters known during compilation
	{
		switch(usartct)
		{
			default:
		#ifdef USE_USART0
			case 0:
			{
			#ifdef USART0_RS485_MODE
				___DDR(RS485_CONTROL0_PORT) |= (1<<RS485_CONTROL0_PIN); // default pin state is low
			#endif	
				UBRR0L_REGISTER = (uint8_t) ubbr_value;
			
				if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
					UBRR0H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART0_U2X_SPEED
				UCSR0A_REGISTER |= (1<<U2X0_BIT); // enable double speed
			#endif
			
				UCSR0B_REGISTER = USART0_CONFIG_B;
				// 8n1 is set by default, setting UCSRC is not needed
			
				break;
			}
		#endif // NO_USART0
		#ifdef USE_USART1
			case 1:
			{
			#ifdef USART1_RS485_MODE
				___DDR(RS485_CONTROL1_PORT) |= (1<<RS485_CONTROL1_PIN); // default pin state is low
			#endif
				UBRR1L_REGISTER = (uint8_t) ubbr_value;
			
				if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
					UBRR1H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART1_U2X_SPEED
				UCSR1A_REGISTER |= (1<<U2X1_BIT); // enable double speed
			#endif
			
				UCSR1B_REGISTER = USART1_CONFIG_B;
				// 8n1 is set by default, setting UCSRC is not needed
			
				break;
			}
		#endif // USE_USART1
		#ifdef USE_USART2
			case 2:
			{
			#ifdef USART2_RS485_MODE
				___DDR(RS485_CONTROL2_PORT) |= (1<<RS485_CONTROL2_PIN); // default pin state is low
			#endif
				UBRR2L_REGISTER = (uint8_t) ubbr_value;
			
				if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
					UBRR2H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART2_U2X_SPEED
				UCSR2A_REGISTER |= (1<<U2X2_BIT); // enable double speed
			#endif
			
				UCSR2B_REGISTER = USART2_CONFIG_B;
				// 8n1 is set by default, setting UCSRC is not needed
			
				break;
			}
		#endif // USE_USART2
		#ifdef USE_USART3
			case 3:
			{
			#ifdef USART3_RS485_MODE
				___DDR(RS485_CONTROL3_PORT) |= (1<<RS485_CONTROL3_PIN); // default pin state is low
			#endif
				UBRR3L_REGISTER = (uint8_t) ubbr_value;
			
				if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
					UBRR3H_REGISTER = (ubbr_value>>8);
			
			#ifdef USART3_U2X_SPEED
				UCSR3A_REGISTER |= (1<<U2X3_BIT); // enable double speed
			#endif
			
				UCSR3B_REGISTER = USART3_CONFIG_B;
				// 8n1 is set by default, setting UCSRC is not needed
			
				//break;
			}
		#endif // USE_USART3
		}
	}
	
	// UCSRC_reg can be used to set other than 8n1 transmission
	inline void uart_set_UCSRC(uint8_t usartct, uint8_t UCSRC_reg)
	{
		switch(usartct)
		{
			default:
		#ifdef USE_USART0
			case 0: UCSR0C_REGISTER = UCSRC_reg; break;
		#endif // USE_USART0
		#ifdef USE_USART1
			case 1: UCSR1C_REGISTER = UCSRC_reg; break;
		#endif // USE_USART1
		#ifdef USE_USART2
			case 2: UCSR2C_REGISTER = UCSRC_reg; break;
		#endif // USE_USART2
		#ifdef USE_USART3
			case 3: UCSR3C_REGISTER = UCSRC_reg; //break;
		#endif // USE_USART3
		}
	}
	
	inline void uart_set_U2X(uint8_t usartct) // use instead of USE_U2Xn_SPEED
	{
		switch(usartct)
		{
			default:
		#ifdef USE_USART0
			case 0: UCSR0A_REGISTER = (1<<U2X0_BIT); break;
		#endif // USE_USART0
		#ifdef USE_USART1
			case 1: UCSR1A_REGISTER = (1<<U2X1_BIT); break;
		#endif // USE_USART1
		#ifdef USE_USART2
			case 2: UCSR2A_REGISTER = (1<<U2X2_BIT); break;
		#endif // USE_USART2
		#ifdef USE_USART3
			case 3: UCSR3A_REGISTER = (1<<U2X3_BIT); //break;
		#endif // USE_USART3
		}
	}
	
#else // single USART mcu
	
	// functions smaller then calling routines or executed once are inline for reducing 
	// flash memory usage (eg. switch and if routines can be executed during compilation)
	
	void uart_reinit(uint16_t ubbr_value); // for runtime reinitialization of uart module 
	
	inline void uart_init(uint16_t ubbr_value) // have to be called once at startup with parameters known during compilation
	{
	#ifdef USART0_RS485_MODE
		___DDR(RS485_CONTROL0_PORT) |= (1<<RS485_CONTROL0_PIN); // default pin state is low
	#endif
		
		UBRR0L_REGISTER = (uint8_t) ubbr_value;
		
		if((ubbr_value>>8) != 0) // requires -Os flag - do not use in non-inline functions
			UBRR0H_REGISTER = (ubbr_value>>8);
		
	#ifdef USART0_U2X_SPEED
		UCSR0A_REGISTER |= (1<<U2X0_BIT); // enable double speed
	#endif
		
		UCSR0B_REGISTER = USART0_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
	}

	// UCSRC_reg can be used to set other than 8n1 transmission
	inline void uart_set_UCSRC(uint8_t UCSRC_reg) 
	{
		UCSR0C_REGISTER = UCSRC_reg;
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
	uint8_t uart_putc_noblock(uint8_t usartct, char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
	void uart_putstrl(uint8_t usartct, char *string, uint8_t BytesToWrite); // in case of bascom users or buffers without NULL byte ending
	void uart_putstr(uint8_t usartct, char *string); // send string from the memory buffer 
	// stops when NULL byte is hit (NULL byte is not included into transmission)
	
	// for deprecated usage only (wastes ram data memory to keep all string constants), instead of this try to use puts_P
	#ifdef __cplusplus
		#define uart_puts(_usartct,str) uart_putstr(_usartct,(const char*)(str))// macro to avoid const char* conversion restrictions 
	#else
		#define uart_puts(_usartct,str) uart_putstr(_usartct,str)
	#endif
		
	void uart_puts_p(uint8_t usartct, const char *string); // send string from flash memory 
		#define uart_puts_P(_usartct,__s)    uart_puts_p(_usartct,PSTR(__s)) 
		// macro to automatically put a string constant into flash
	
	void uart_putint(uint8_t usartct, int16_t data);
	void uart_putintr(uint8_t usartct, int16_t data, uint8_t radix);
	
	void uart_putuint(uint8_t usartct, uint16_t data);
	void uart_putuintr(uint8_t usartct, uint16_t data, uint8_t radix);
	
	void uart_put_hex(uint8_t usartct, uint16_t data);
	
	void uart_putlong(uint8_t usartct, int32_t data);
	void uart_putlongr(uint8_t usartct, int32_t data, uint8_t radix);
	
	void uart_putulong(uint8_t usartct, uint32_t data);
	void uart_putulongr(uint8_t usartct, uint32_t data, uint8_t radix);
	
	void uart_putfloat(uint8_t usartct, float data);
	void uart_fputfloat(uint8_t usartct, float data, uint8_t precision);

#else // single USART mcu

	void uart_putc(char data); // put character/data into transmitter ring buffer
	uint8_t uart_putc_noblock(char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
	void uart_putstrl(char *string, uint8_t BytesToWrite); // in case of bascom users or buffers without NULL byte ending
	void uart_putstr(char *string); // send string from the memory buffer
		// stops when NULL byte is hit (NULL byte is not included into transmission)
	#ifdef __cplusplus
		#define uart_puts(str) uart_putstr((const char*)(str))// macro to avoid const char* conversion restrictions
	#else
		#define uart_puts(str) uart_putstr(str)
	#endif
		// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P

	void uart_puts_p(const char *string); // send string from flash memory
		#define uart_puts_P(__s)    uart_puts_p(PSTR(__s))
		// macro to automatically put a string constant into flash
	
	void uart_putint(int16_t data);
	void uart_putintr(int16_t data, uint8_t radix);

	void uart_putuint(uint16_t data);
	void uart_putuintr(uint16_t data, uint8_t radix);
	
	void uart_put_hex(uint16_t data);
	
	void uart_putlong(int32_t data);
	void uart_putlongr(int32_t data, uint8_t radix);

	void uart_putulong(uint32_t data);
	void uart_putulongr(uint32_t data, uint8_t radix);
	
	void uart_putfloat(float data);
	void uart_fputfloat(float data, uint8_t precision);

#endif // single/multi USART

#endif // NO_USART_TX

/************************************************************************************
 *                           Receiver functions                                     *
 ************************************************************************************/
#ifndef NO_USART_RX

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)
	
	char uart_getc(uint8_t usartct); // get character from receiver ring buffer
	
	void uart_gets(uint8_t usartct, char *buffer, uint8_t bufferlimit); // reads whole receiver buffer or bufferlimit-1 characters
	// newline terminator will not be cut // adds NULL byte at the end of string
	void uart_getln(uint8_t usartct, char *buffer, uint8_t bufferlimit); // reads one line from the buffer, 
	// waits for newline terminator or reached bufferlimit // adds NULL byte at the end of string
	void uart_getlnToFirstWhiteSpace(uint8_t usartct, char *buffer, uint8_t bufferlimit); // read one line to the first whitespace after the string
	//cuts all whitespaces before string and one after the string
	
	char uart_skipWhiteSpaces(uint8_t usartct); // returns first nonspace character found in the buffer
	
	int16_t uart_getint(uint8_t usartct);
	int32_t uart_getlong(uint8_t usartct);
	float uart_getfloat(uint8_t usartct);
	
	uint8_t uart_getData(uint8_t usartct, uint8_t *data); // reads binary data from a buffer and loads it into *data byte 
	// in case of empty buffers returned flag is set to BUFFER_EMPTY - NULL
	uint8_t uart_AvailableBytes(uint8_t usartct); // returns number of bytes waiting in the receiver buffer
	//uint8_t uart_peek(uint8_t usartct); 

#else // single USART mcu

	char uart_getc(void); // get character from receiver ring buffer
	
	void uart_gets(char *buffer, uint8_t bufferlimit); // reads whole receiver buffer or bufferlimit-1 characters
	// newline terminator will not be cut // adds NULL byte at the end of string
	void uart_getln(char *buffer, uint8_t bufferlimit); // reads one line from the buffer
	// waits for newline terminator or reached bufferlimit // adds NULL byte at the end of string
	void uart_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit); // read one line to the first whitespace after the string
	// cuts all whitespaces before string and one after the string
	
	char uart_skipWhiteSpaces(void); // returns first nonspace character found in the buffer
	
	int16_t uart_getint(void);
	int32_t uart_getlong(void);
	float uart_getfloat(void);
	
	uint8_t uart_getData(uint8_t *data); // reads binary data from a buffer and loads it into *data byte
	// in case of empty buffers returning flag is set to BUFFER_EMPTY - NULL
	uint8_t uart_AvailableBytes(void); // returns number of bytes waiting in the receiver buffer
	//uint8_t uart_peek(void); 

#endif // single/multi USART

#endif // NO_USART_RX


#ifdef _STDIO_H_

#if defined(USE_USART1)||defined(USE_USART2)||defined(USE_USART3)

	// wrapper of stdio.h FDEV_SETUP_STREAM to allow setting udata; udata is used to store info about port ID
	#define FDEV_SETUP_STREAM_U(p, g, f, u) \
	{ \
		.put = p, \
		.get = g, \
		.flags = f, \
		.udata = u, \
	}
	
	#ifndef NO_USART_TX
	
		void uart_putchar(char data, FILE *stream)
		{
			if ( data == '\n') uart_putc((uint16_t) stream -> udata, '\r');
				
			uart_putc((uint16_t) stream -> udata, data);
		}
	
	#endif

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
	
	#endif
	
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
		
	#endif

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
	
	#endif
	
	#if defined(NO_RX0_INTERRUPT)
		FILE uart0_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
	
	#elif defined(NO_TX0_INTERRUPT)
		FILE uart0_in = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
	#else
		FILE uart0_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	#endif
	
#endif // single/multi USART

#endif // used _STDIO_H_


#endif // _USART_H_

// if any other uart library produces better code than this, you can contact me to fix this issue