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

//#define USART_USE_SOFT_CTS // CTS handlers also have to be placed into INT/PCINT interrupt in the application code, see example(flow control).c
//#define USART_USE_SOFT_RTS

//#define USART_RS485_MODE // globally enable half duplex rs485 operation mode
//#define USART_MPCM_MODE // globally enable MPCM operation mode // 9 bit data frame only // always set frame format to 8 data bits

//#define USE_DOUBLE_SPEED // enables double speed for all available USART interfaces

#define RX_STDIO_GETCHAR_ECHO // echoes back received characters in getchar() function (for reading in scanf()) 
#define RX_GETC_ECHO // echoes back received characters in getc() function

//#define PUTC_CONVERT_LF_TO_CRLF // allow for unix style (\n only) newline terminator in stored strings // not included into putc_noblock
#define RX_NEWLINE_MODE 2 // 0 - \r,  1 - \n,  2 - \r\n
// lot of terminals sends only \r character as a newline terminator, instead of \r\n or even unix style \n
// (BTW PuTTY doesn't allow to change this) but in return requires \r\n terminator to show not broken text

//#define USART_EXTEND_RX_BUFFER // extend RX buffer by hardware 2/3 byte FIFO // required for hardware and software RTS
//#define USART_PUTC_FAST_INSERTIONS // skip FIFO procedure and write directly data to the UDR register when possible // probably required for full bus utilization at highest speed (f_cpu/8)
//#define USART_NO_LOCAL_BUFFERS // do not allocate temporary buffers on stack and use globally visible u_tmp_buff[] instead // it have to be created in application part and have to be at least of 6-17 bytes wide (depending on what is being converted)
//#define USART_UNSAFE_TX_INTERRUPT // max 19 cycles of interrupt latency // 3+PC bytes on stack // will not interrupt itself
//#define USART_UNSAFE_RX_INTERRUPT // max 23 cycles of interrupt latency // 4+PC bytes on stack // will not interrupt itself
#define USART_REMAP_LAST_INTERFACE // remap hardware registers of USART1/2/3 to USART0 if only one interface is used
//#define USART_NO_DIRTY_HACKS // if UBBRH is not zero at startup, or if the code is not allowed to violate any conventions

/*****************************config for multiple USART mcu's***********************************/

//#define NO_USART0 // disable usage of uart0
//#define NO_USART1 // disable usage of uart1
//#define NO_USART2 // disable usage of uart2
//#define NO_USART3 // disable usage of uart3

//#define NO_RX0_INTERRUPT // disables interrupt handling and frees RX0 gpio port // combining with NO_USART_RX is not necessary 
//#define NO_RX1_INTERRUPT // disables interrupt handling and frees RX1 gpio port
//#define NO_RX2_INTERRUPT // disables interrupt handling and frees RX2 gpio port
//#define NO_RX3_INTERRUPT // disables interrupt handling and frees RX3 gpio port

//#define NO_TX0_INTERRUPT // disables interrupt handling and frees TX0 gpio port // combining with NO_USART_TX is not necessary
//#define NO_TX1_INTERRUPT // disables interrupt handling and frees TX1 gpio port
//#define NO_TX2_INTERRUPT // disables interrupt handling and frees TX2 gpio port
//#define NO_TX3_INTERRUPT // disables interrupt handling and frees TX3 gpio port

//#define USART0_U2X_SPEED // enables double speed for USART0
//#define USART1_U2X_SPEED // enables double speed for USART1
//#define USART2_U2X_SPEED // enables double speed for USART2
//#define USART3_U2X_SPEED // enables double speed for USART3

//#define RX0_GETC_ECHO
//#define RX1_GETC_ECHO
//#define RX2_GETC_ECHO
//#define RX3_GETC_ECHO

//#define USART0_EXTEND_RX_BUFFER
//#define USART1_EXTEND_RX_BUFFER
//#define USART2_EXTEND_RX_BUFFER
//#define USART3_EXTEND_RX_BUFFER

//#define USART0_PUTC_FAST_INSERTIONS
//#define USART1_PUTC_FAST_INSERTIONS
//#define USART2_PUTC_FAST_INSERTIONS
//#define USART3_PUTC_FAST_INSERTIONS

//#define USART0_MPCM_MODE
//#define USART1_MPCM_MODE
//#define USART2_MPCM_MODE
//#define USART3_MPCM_MODE

//#define USART0_USE_SOFT_CTS
//#define USART1_USE_SOFT_CTS
//#define USART2_USE_SOFT_CTS
//#define USART3_USE_SOFT_CTS

//#define USART0_USE_SOFT_RTS
//#define USART1_USE_SOFT_RTS
//#define USART2_USE_SOFT_RTS
//#define USART3_USE_SOFT_RTS

//#define USART0_RS485_MODE 
//#define USART1_RS485_MODE	
//#define USART2_RS485_MODE	
//#define USART3_RS485_MODE	

/*****************************soft flow control config***********************************/

//#define CTS0_IOPORTNAME D // A,B,C,D ... port naming
//#define CTS0_PIN 2 // 1,2,3,4 ... pin naming

//#define CTS1_IOPORTNAME
//#define CTS1_PIN

//#define CTS2_IOPORTNAME
//#define CTS2_PIN

//#define CTS3_IOPORTNAME
//#define CTS3_PIN

//#define RTS0_IOPORTNAME D // A,B,C,D ... port naming
//#define RTS0_PIN 6 // 1,2,3,4 ... pin naming

//#define RTS1_IOPORTNAME
//#define RTS1_PIN

//#define RTS2_IOPORTNAME
//#define RTS2_PIN

//#define RTS3_IOPORTNAME
//#define RTS3_PIN

/*****************************RS 485 config***********************************/

//#define RS485_CONTROL0_IOPORTNAME D // A,B,C,D ... port naming - define valid destination of pin connected to DE + RE 
//#define RS485_CONTROL0_PIN 2 // 1,2,3,4 ... pin naming - define valid pin connected to DE + RE

//#define RS485_CONTROL1_IOPORTNAME 
//#define RS485_CONTROL1_PIN 

//#define RS485_CONTROL2_IOPORTNAME
//#define RS485_CONTROL2_PIN

//#define RS485_CONTROL3_IOPORTNAME
//#define RS485_CONTROL3_PIN

/*****************************MPCM config***********************************/

#define MPCM0_ADDRESS 0x01
#define MPCM1_ADDRESS 0x02
#define MPCM2_ADDRESS 0x03
#define MPCM3_ADDRESS 0x04

//#define MPCM0_GCALL_ADDRESS 0x00
//#define MPCM1_GCALL_ADDRESS 0x00
//#define MPCM2_GCALL_ADDRESS 0x00
//#define MPCM3_GCALL_ADDRESS 0x00

//#define MPCM0_MASTER_ONLY // do not include slave code into RX ISR
//#define MPCM1_MASTER_ONLY // do not include slave code into RX ISR
//#define MPCM2_MASTER_ONLY // do not include slave code into RX ISR
//#define MPCM3_MASTER_ONLY // do not include slave code into RX ISR

#include <avr/io.h> // for inline func

#ifndef F_CPU
	#error F_CPU is undefined, USART cannot work correctly without this parametr
#endif

#define BAUD_CALC(x) ((F_CPU+(x)*8UL) / (16UL*(x))-1UL) // macro calculating UBBR value
#define BAUD_CALC_FAST(x) ((F_CPU)/(BAUD*16UL)-1) // for faster real time calculations ?
#define DOUBLE_BAUD_CALC(x) ((F_CPU+(x)*4UL) / (8UL*(x))-1UL) // macro calculating UBBR value for double speed

#ifndef __OPTIMIZE__
	#warning "Compiler optimizations disabled; functions from usart.h won't work as designed"
	#define USART_NO_DIRTY_HACKS
#endif
	
#ifdef DEBUG
	#define USART_NO_DIRTY_HACKS
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
	
#if defined(URSEL)||defined(URSEL0)||defined(URSEL1)||defined(URSEL2)||defined(URSEL3)

	#define USART_XCK_RISING_EDGE 0x80
	#define USART_XCK_FALLING_EDGE 0x81

	#define USART_5BIT_DATA 0x80
	#define USART_6BIT_DATA 0x82
	#define USART_7BIT_DATA 0x84
	#define USART_8BIT_DATA 0x86

	#define USART_1STOP_BIT 0x80
	#define USART_2STOP_BITS 0x88

	#define USART_NO_PARITY 0x80
	#define USART_EVEN_PARITY 0xA0
	#define USART_ODD_PARITY 0xB0
	
	#define USART_ASYNC_MODE 0x80
	#define USART_SYNC_MODE 0xC0
#else
	#define USART_XCK_RISING_EDGE 0x00
	#define USART_XCK_FALLING_EDGE 0x01

	#define USART_5BIT_DATA 0x00
	#define USART_6BIT_DATA 0x02
	#define USART_7BIT_DATA 0x04
	#define USART_8BIT_DATA 0x06

	#define USART_1STOP_BIT 0x00
	#define USART_2STOP_BITS 0x08

	#define USART_NO_PARITY 0x00
	#define USART_EVEN_PARITY 0x20
	#define USART_ODD_PARITY 0x30
	
	#define USART_ASYNC_MODE 0x00
	#define USART_SYNC_MODE 0x40
	#define USART_MSPI_MODE 0xC0
#endif

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

#ifdef USART_EXTEND_RX_BUFFER
	#define USART0_EXTEND_RX_BUFFER
	#define USART1_EXTEND_RX_BUFFER
	#define USART2_EXTEND_RX_BUFFER
	#define USART3_EXTEND_RX_BUFFER
#endif

#ifdef USART_PUTC_FAST_INSERTIONS
	#define USART0_PUTC_FAST_INSERTIONS
	#define USART1_PUTC_FAST_INSERTIONS
	#define USART2_PUTC_FAST_INSERTIONS
	#define USART3_PUTC_FAST_INSERTIONS
#endif

#ifdef USART_USE_SOFT_CTS
	#define USART0_USE_SOFT_CTS
	#define USART1_USE_SOFT_CTS
	#define USART2_USE_SOFT_CTS
	#define USART3_USE_SOFT_CTS
#endif

#ifdef USART_USE_SOFT_RTS
	#define USART0_USE_SOFT_RTS
	#define USART1_USE_SOFT_RTS
	#define USART2_USE_SOFT_RTS
	#define USART3_USE_SOFT_RTS
#endif

#ifdef USART_RS485_MODE
	#define USART0_RS485_MODE
	#define USART1_RS485_MODE
	#define USART2_RS485_MODE
	#define USART3_RS485_MODE
#endif

#ifdef USART_MPCM_MODE
	#define USART0_MPCM_MODE
	#define USART1_MPCM_MODE
	#define USART2_MPCM_MODE
	#define USART3_MPCM_MODE
#endif

#ifdef USART0_USE_SOFT_RTS
	#ifndef USART0_EXTEND_RX_BUFFER
		#define USART0_EXTEND_RX_BUFFER
	#endif
#endif

#ifdef USART1_USE_SOFT_RTS
	#ifndef USART1_EXTEND_RX_BUFFER
		#define USART1_EXTEND_RX_BUFFER
	#endif
#endif

#ifdef USART2_USE_SOFT_RTS
	#ifndef USART2_EXTEND_RX_BUFFER
		#define USART2_EXTEND_RX_BUFFER
	#endif
#endif

#ifdef USART3_USE_SOFT_RTS
	#ifndef USART3_EXTEND_RX_BUFFER
		#define USART3_EXTEND_RX_BUFFER
	#endif
#endif

#ifndef ___DDR
	#define ___DDR(x) ___XDDR(x)
#endif
#ifndef ___XDDR
	#define ___XDDR(x) (DDR ## x)
#endif

#ifndef ___PORT
	#define ___PORT(x) ___XPORT(x)
#endif
#ifndef ___XPORT
	#define ___XPORT(x) (PORT ## x)
#endif

#ifndef ___PIN
	#define ___PIN(x) ___XPIN(x)
#endif
#ifndef ___XPIN
	#define ___XPIN(x) (PIN ## x)
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

#if defined(__usbdrv_h_included__)&&!defined(USART_UNSAFE_RX_INTERRUPT)
	#warning "usb may not work with RX ISR's"
#endif

#if defined(__usbdrv_h_included__)&&!defined(USART_UNSAFE_TX_INTERRUPT)
	#warning "usb may not work with TX ISR's"
#endif

#if defined(__AVR_ATtiny102__)||defined(__AVR_ATtiny104__)

#if (TX0_BUFFER_SIZE > 8)||(RX0_BUFFER_SIZE > 8)
	#warning "TX or RX buffer may be too large for this mcu"
#endif

#define USART0_IN_IO_ADDRESS_SPACE
#define USART0_NOT_ACCESIBLE_FROM_CBI

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART0_RXC_vect
	#define TXC0_INTERRUPT  	USART0_TXC_vect
	#define UDRE0_INTERRUPT		USART0_DRE_vect
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
	#define UDRE0_BIT   		UDRE0
	#define RXC0_BIT    		RXC0
	#define U2X0_BIT    		U2X0
	#define MPCM0_BIT           MPCM0
	#define UCSZ02_BIT          UCSZ02
	#define TXB80_BIT           TXB80
	
#endif //NO_USART0
#endif

#if defined(__AVR_ATtiny2313__)||defined(__AVR_ATtiny2313A__)||defined(__AVR_ATtiny4313)

#define USART0_IN_IO_ADDRESS_SPACE

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
	#define UDRE0_BIT   		UDRE
	#define RXC0_BIT    		RXC
	#define U2X0_BIT    		U2X
	#define MPCM0_BIT           MPCM
	#define UCSZ02_BIT          UCSZ2
	#define TXB80_BIT           TXB8

#endif //NO_USART0
#endif

#if defined(__AVR_ATtiny1634__)

#define USART0_IN_IO_ADDRESS_SPACE
#define USART0_NOT_ACCESIBLE_FROM_CBI

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART0_RX_vect
	#define TXC0_INTERRUPT  	USART0_TX_vect
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
	#define UDRE0_BIT   		UDRE0
	#define RXC0_BIT    		RXC0
	#define U2X0_BIT    		U2X0
	#define MPCM0_BIT           MPCM0
	#define UCSZ02_BIT          UCSZ02
	#define TXB80_BIT           TXB80

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
	#define RXCIE1_BIT  		RXCIE1
	#define TXEN1_BIT   		TXEN1
	#define RXEN1_BIT   		RXEN1
	#define UDRE1_BIT   		UDRE1
	#define RXC1_BIT    		RXC1
	#define U2X1_BIT    		U2X1
	#define MPCM1_BIT           MPCM1
	#define UCSZ12_BIT          UCSZ12
	#define TXB81_BIT           TXB81

#endif //NO_USART1
#endif


#if defined(__AVR_ATtiny441__)||defined(__AVR_ATtiny841__)

#ifndef NO_USART0
#define USE_USART0

	#define RX0_INTERRUPT		USART0_RX_vect
	#define TXC0_INTERRUPT  	USART0_TX_vect
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
	#define UDRE0_BIT   		UDRE0
	#define RXC0_BIT    		RXC0
	#define U2X0_BIT    		U2X0
	#define MPCM0_BIT           MPCM0
	#define UCSZ02_BIT          UCSZ02
	#define TXB80_BIT           TXB80

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
	#define RXCIE1_BIT  		RXCIE1
	#define TXEN1_BIT   		TXEN1
	#define RXEN1_BIT   		RXEN1
	#define UDRE1_BIT   		UDRE1
	#define RXC1_BIT    		RXC1
	#define U2X1_BIT    		U2X1
	#define MPCM1_BIT           MPCM1
	#define UCSZ12_BIT          UCSZ12
	#define TXB81_BIT           TXB81

#endif //NO_USART1
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
	#define UDRE0_BIT   		UDRE0
	#define RXC0_BIT    		RXC0
	#define U2X0_BIT    		U2X0
	#define MPCM0_BIT           MPCM0
	#define UCSZ02_BIT          UCSZ02
	#define TXB80_BIT           TXB80	

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
	#define UDRE0_BIT   		UDRE0
	#define RXC0_BIT    		RXC0
	#define U2X0_BIT    		U2X0
	#define MPCM0_BIT           MPCM0
	#define UCSZ02_BIT          UCSZ02
	#define TXB80_BIT           TXB80

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
	#define UDRE1_BIT   		UDRE1
	#define RXC1_BIT    		RXC1
	#define U2X1_BIT    		U2X1
	#define MPCM1_BIT           MPCM1
	#define UCSZ12_BIT          UCSZ12
	#define TXB81_BIT           TXB81	

#endif //NO_USART1
#endif

#if defined(__AVR_ATmega8__)||defined(__AVR_ATmega8P__)||defined(__AVR_ATmega16__)\
||defined(__AVR_ATmega16A__)||defined(__AVR_ATmega32__)||defined(__AVR_ATmega32A__)\
||defined(__AVR_ATmega8A__)||defined(__AVR_ATmega8L__)

#define USART0_IN_IO_ADDRESS_SPACE

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
	#define UDRE0_BIT   		UDRE
	#define RXC0_BIT    		RXC
	#define U2X0_BIT    		U2X
	#define MPCM0_BIT           MPCM
	#define UCSZ02_BIT          UCSZ2
	#define TXB80_BIT           TXB8
	
#endif //NO_USART0
#endif

#if defined(__AVR_ATmega8515__)||defined(__AVR_ATmega8515L__)

#define USART0_IN_IO_ADDRESS_SPACE

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
	#define UDRE0_BIT   		UDRE
	#define RXC0_BIT    		RXC
	#define U2X0_BIT    		U2X
	#define MPCM0_BIT           MPCM
	#define UCSZ02_BIT          UCSZ2
	#define TXB80_BIT           TXB8

#endif //NO_USART0
#endif

#if defined(__AVR_ATmega162__)

#define USART0_IN_IO_ADDRESS_SPACE
#define USART1_IN_IO_ADDRESS_SPACE

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
	#define UDRE0_BIT   		UDRE0
	#define RXC0_BIT    		RXC0
	#define U2X0_BIT    		U2X0
	#define MPCM0_BIT           MPCM0
	#define UCSZ02_BIT          UCSZ02
	#define TXB80_BIT           TXB80

#endif //NO_USART0

#ifndef NO_USART1
#define USE_USART1

	#define RX1_INTERRUPT		USART1_RXC_vect
	#define TXC1_INTERRUPT		USART1_TXC_vect
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
	#define UDRE1_BIT   		UDRE1
	#define RXC1_BIT    		RXC1
	#define U2X1_BIT    		U2X1
	#define MPCM1_BIT           MPCM1
	#define UCSZ12_BIT          UCSZ12
	#define TXB81_BIT           TXB81

#endif //NO_USART1
#endif

#if defined(__AVR_ATmega128__)||defined(__AVR_ATmega128A__)||defined(__AVR_ATmega64__)\
||defined(__AVR_ATmega64A__)
	#define USART0_IN_IO_ADDRESS_SPACE
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
	#define UDRE0_BIT   		UDRE0
	#define RXC0_BIT    		RXC0
	#define U2X0_BIT    		U2X0
	#define MPCM0_BIT           MPCM0
	#define UCSZ02_BIT          UCSZ02
	#define TXB80_BIT           TXB80

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
	#define UDRE1_BIT   		UDRE1
	#define RXC1_BIT    		RXC1
	#define U2X1_BIT    		U2X1
	#define MPCM1_BIT           MPCM1
	#define UCSZ12_BIT          UCSZ12
	#define TXB81_BIT           TXB81
	
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
	#define UDRE2_BIT   		UDRE2
	#define RXC2_BIT    		RXC2
	#define U2X2_BIT    		U2X2
	#define MPCM2_BIT           MPCM2
	#define UCSZ22_BIT          UCSZ22
	#define TXB82_BIT           TXB82
	
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
	#define UDRE3_BIT   		UDRE3
	#define RXC3_BIT    		RXC3
	#define U2X3_BIT    		U2X3
	#define MPCM3_BIT           MPCM3
	#define UCSZ32_BIT          UCSZ32
	#define TXB83_BIT           TXB83
	
#endif // NO_USART3

#endif // 640/1280/2560 usart 2 & 3 

#if defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega16U4__)\
||defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega32U6__)

#define USART0_HARDWARE_FLOW_CONTROL_AVAILABLE

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
	#define UCSR0D_REGISTER		UCSR1D
	#define TXCIE0_BIT      	TXCIE1
	#define UDRIE0_BIT    		UDRIE1
	#define RXCIE0_BIT  		RXCIE1
	#define TXEN0_BIT   		TXEN1
	#define RXEN0_BIT   		RXEN1
	#define UDRE0_BIT   		UDRE1
	#define RXC0_BIT    		RXC1
	#define U2X0_BIT    		U2X1
	#define MPCM0_BIT           MPCM1
	#define UCSZ02_BIT          UCSZ12
	#define TXB80_BIT           TXB81
	#define CTSEN0_BIT          CTSEN
	#define RTSEN0_BIT          RTSEN

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
	#define UDRE0_BIT   		UDRE
	#define RXC0_BIT    		RXC
	#define U2X0_BIT    		U2X
	#define MPCM0_BIT           MPCM
	#define UCSZ02_BIT          UCSZ2
	#define TXB80_BIT           TXB8

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
	#define UDRE0_BIT   		UDRE0
	#define RXC0_BIT    		RXC0
	#define U2X0_BIT    		U2X0
	#define MPCM0_BIT           MPCM0
	#define UCSZ02_BIT          UCSZ02
	#define TXB80_BIT           TXB80

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
	#define UDRE0_BIT   		UDRE0
	#define RXC0_BIT    		RXC0
	#define U2X0_BIT    		U2X0
	#define MPCM0_BIT           MPCM0
	#define UCSZ02_BIT          UCSZ02
	#define TXB80_BIT           TXB80

#endif //NO_USART0
#endif

#if defined(USART_REMAP_LAST_INTERFACE)&&!defined(USE_USART0)&&defined(USE_USART1)&&!defined(USE_USART2)&&!defined(USE_USART3)
	#undef USE_USART1
	#define USE_USART0
	
	#define RX0_INTERRUPT		RX1_INTERRUPT
	#define TXC0_INTERRUPT		TXC1_INTERRUPT
	#define UDRE0_INTERRUPT		UDRE1_INTERRUPT
	#define UDR0_REGISTER		UDR1_REGISTER
	#define UBRR0L_REGISTER		UBRR1L_REGISTER
	#define UBRR0H_REGISTER		UBRR1H_REGISTER
	#define UCSR0A_REGISTER		UCSR1A_REGISTER
	#define UCSR0B_REGISTER		UCSR1B_REGISTER
	#define UCSR0C_REGISTER		UCSR1C_REGISTER
	#define TXCIE0_BIT      	TXCIE1_BIT
	#define UDRIE0_BIT    		UDRIE1_BIT
	#define RXCIE0_BIT  		RXCIE1_BIT
	#define TXEN0_BIT   		TXEN1_BIT
	#define RXEN0_BIT   		RXEN1_BIT
	#define UDRE0_BIT   		UDRE1_BIT
	#define RXC0_BIT    		RXC1_BIT
	#define U2X0_BIT    		U2X1_BIT
	#define MPCM0_BIT           MPCM1_BIT
	#define UCSZ02_BIT          UCSZ12_BIT
	#define TXB80_BIT           TXB81_BIT
#endif

#if defined(USART_REMAP_LAST_INTERFACE)&&!defined(USE_USART0)&&!defined(USE_USART1)&&defined(USE_USART2)&&!defined(USE_USART3)
	#undef USE_USART2
	#define USE_USART0
	
	#define RX0_INTERRUPT		RX2_INTERRUPT
	#define TXC0_INTERRUPT		TXC2_INTERRUPT
	#define UDRE0_INTERRUPT		UDRE2_INTERRUPT
	#define UDR0_REGISTER		UDR2_REGISTER
	#define UBRR0L_REGISTER		UBRR2L_REGISTER
	#define UBRR0H_REGISTER		UBRR2H_REGISTER
	#define UCSR0A_REGISTER		UCSR2A_REGISTER
	#define UCSR0B_REGISTER		UCSR2B_REGISTER
	#define UCSR0C_REGISTER		UCSR2C_REGISTER
	#define TXCIE0_BIT      	TXCIE2_BIT
	#define UDRIE0_BIT    		UDRIE2_BIT
	#define RXCIE0_BIT  		RXCIE2_BIT
	#define TXEN0_BIT   		TXEN2_BIT
	#define RXEN0_BIT   		RXEN2_BIT
	#define UDRE0_BIT   		UDRE2_BIT
	#define RXC0_BIT    		RXC2_BIT
	#define U2X0_BIT    		U2X2_BIT
	#define MPCM0_BIT           MPCM2_BIT
	#define UCSZ02_BIT          UCSZ22_BIT
	#define TXB80_BIT           TXB82_BIT
#endif

#if defined(USART_REMAP_LAST_INTERFACE)&&!defined(USE_USART0)&&!defined(USE_USART1)&&!defined(USE_USART2)&&defined(USE_USART3)
	#undef USE_USART3
	#define USE_USART0
	
	#define RX0_INTERRUPT		RX3_INTERRUPT
	#define TXC0_INTERRUPT		TXC3_INTERRUPT
	#define UDRE0_INTERRUPT		UDRE3_INTERRUPT
	#define UDR0_REGISTER		UDR3_REGISTER
	#define UBRR0L_REGISTER		UBRR3L_REGISTER
	#define UBRR0H_REGISTER		UBRR3H_REGISTER
	#define UCSR0A_REGISTER		UCSR3A_REGISTER
	#define UCSR0B_REGISTER		UCSR3B_REGISTER
	#define UCSR0C_REGISTER		UCSR3C_REGISTER
	#define TXCIE0_BIT      	TXCIE3_BIT
	#define UDRIE0_BIT    		UDRIE3_BIT
	#define RXCIE0_BIT  		RXCIE3_BIT
	#define TXEN0_BIT   		TXEN3_BIT
	#define RXEN0_BIT   		RXEN3_BIT
	#define UDRE0_BIT   		UDRE3_BIT
	#define RXC0_BIT    		RXC3_BIT
	#define U2X0_BIT    		U2X3_BIT
	#define MPCM0_BIT           MPCM3_BIT
	#define UCSZ02_BIT          UCSZ32_BIT
	#define TXB80_BIT           TXB83_BIT
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

#if (defined(USE_USART0)&&!defined(USE_USART1)&&!defined(USE_USART2)&&!defined(USE_USART3))
	#ifdef NO_TX0_INTERRUPT
		#define NO_USART_TX
	#endif
	
	#ifdef NO_RX0_INTERRUPT
		#define NO_USART_RX
	#endif
#endif

#if defined(USART0_USE_SOFT_CTS)&&defined(USE_USART0)
	#if !defined(CTS0_IOPORTNAME)||!defined(CTS0_PIN)
		#error "define valid CTS input for USART0"
	#endif
#endif

#if defined(USART1_USE_SOFT_CTS)&&defined(USE_USART1)
	#if !defined(CTS1_IOPORTNAME)||!defined(CTS1_PIN)
		#error "define valid CTS input for USART1"
	#endif
#endif

#if defined(USART2_USE_SOFT_CTS)&&defined(USE_USART2)
	#if !defined(CTS2_IOPORTNAME)||!defined(CTS2_PIN)
		#error "define valid CTS input for USART2"
	#endif
#endif

#if defined(USART3_USE_SOFT_CTS)&&defined(USE_USART3)
	#if !defined(CTS3_IOPORTNAME)||!defined(CTS3_PIN)
		#error "define valid CTS input for USART3"
	#endif
#endif

#if defined(USART0_USE_SOFT_RTS)&&defined(USE_USART0)
	#if !defined(RTS0_IOPORTNAME)||!defined(RTS0_PIN)
		#error "define valid RTS input for USART0"
	#endif
#endif

#if defined(USART1_USE_SOFT_RTS)&&defined(USE_USART1)
	#if !defined(RTS1_IOPORTNAME)||!defined(RTS1_PIN)
		#error "define valid RTS input for USART1"
	#endif
#endif

#if defined(USART2_USE_SOFT_RTS)&&defined(USE_USART2)
	#if !defined(RTS2_IOPORTNAME)||!defined(RTS2_PIN)
		#error "define valid RTS input for USART2"
	#endif
#endif

#if defined(USART3_USE_SOFT_RTS)&&defined(USE_USART3)
	#if !defined(RTS3_IOPORTNAME)||!defined(RTS3_PIN)
		#error "define valid RTS input for USART3"
	#endif
#endif

#if defined(USART0_RS485_MODE)&&defined(USE_USART0)
	#if !defined(RS485_CONTROL0_IOPORTNAME)||!defined(RS485_CONTROL0_PIN)
		#error "define valid DE/RE output for USART0 RS485 operation"
	#endif
#endif

#if defined(USART1_RS485_MODE)&&defined(USE_USART1)
	#if !defined(RS485_CONTROL1_IOPORTNAME)||!defined(RS485_CONTROL1_PIN)
		#error "define valid DE/RE output for USART1 RS485 operation"
	#endif
#endif

#if defined(USART2_RS485_MODE)&&defined(USE_USART2)
	#if !defined(RS485_CONTROL2_IOPORTNAME)||!defined(RS485_CONTROL2_PIN)
		#error "define valid DE/RE output for USART2 RS485 operation"
	#endif
#endif

#if defined(USART3_RS485_MODE)&&defined(USE_USART3)
	#if !defined(RS485_CONTROL3_IOPORTNAME)||!defined(RS485_CONTROL3_PIN)
		#error "define valid DE/RE output for USART3 RS485 operation"
	#endif
#endif

#ifndef USART0_CONFIG_B // set config bytes for UCSR0B_REGISTER
	
	#ifdef USART0_MPCM_MODE
		#if defined(NO_RX0_INTERRUPT)
			#define USART0_CONFIG_B (1<<TXEN0_BIT)|(1<<UCSZ02_BIT)
		
		#elif defined(NO_TX0_INTERRUPT)
			#define USART0_CONFIG_B (1<<RXEN0_BIT)|(1<<RXCIE0_BIT)|(1<<UCSZ02_BIT)
		#else
			#ifdef USART0_RS485_MODE
				#define USART0_CONFIG_B (1<<TXEN0_BIT)|(1<<TXCIE0_BIT)|(1<<RXEN0_BIT)|(1<<RXCIE0_BIT)|(1<<UCSZ02_BIT)
			#else
				#define USART0_CONFIG_B (1<<TXEN0_BIT)|(1<<RXEN0_BIT)|(1<<RXCIE0_BIT)|(1<<UCSZ02_BIT)
			#endif
		#endif
	#else
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
	#endif
#endif // USART0_CONFIG

#ifndef USART1_CONFIG_B // set config bytes for UCSR1B_REGISTER
	
	#ifdef USART1_MPCM_MODE
		#if defined(NO_RX1_INTERRUPT)
			#define USART1_CONFIG_B (1<<TXEN1_BIT)|(1<<UCSZ12_BIT)
	
		#elif defined(NO_TX1_INTERRUPT)
			#define USART1_CONFIG_B (1<<RXEN1_BIT)|(1<<RXCIE1_BIT)|(1<<UCSZ12_BIT)
		#else
			#ifdef USART1_RS485_MODE
				#define USART1_CONFIG_B (1<<TXEN1_BIT)|(1<<TXCIE1_BIT)|(1<<RXEN1_BIT)|(1<<RXCIE1_BIT)|(1<<UCSZ12_BIT)
			#else
				#define USART1_CONFIG_B (1<<TXEN1_BIT)|(1<<RXEN1_BIT)|(1<<RXCIE1_BIT)|(1<<UCSZ12_BIT)
			#endif
		#endif
	#else
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
	#endif
#endif // USART1_CONFIG

#ifndef USART2_CONFIG_B // set config bytes for UCSR2B_REGISTER

	#ifdef USART2_MPCM_MODE
		#if defined(NO_RX2_INTERRUPT)
			#define USART2_CONFIG_B (1<<TXEN2_BIT)|(1<<UCSZ22_BIT)
	
		#elif defined(NO_TX2_INTERRUPT)
			#define USART2_CONFIG_B (1<<RXEN2_BIT)|(1<<RXCIE2_BIT)|(1<<UCSZ22_BIT)
		#else
			#ifdef USART2_RS485_MODE
				#define USART2_CONFIG_B (1<<TXEN2_BIT)|(1<<TXCIE2_BIT)|(1<<RXEN2_BIT)|(1<<RXCIE2_BIT)|(1<<UCSZ22_BIT)
			#else
				#define USART2_CONFIG_B (1<<TXEN2_BIT)|(1<<RXEN2_BIT)|(1<<RXCIE2_BIT)|(1<<UCSZ22_BIT)
			#endif
		#endif
	#else
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
	#endif
#endif // USART2_CONFIG

#ifndef USART3_CONFIG_B // set config bytes for UCSR3B_REGISTER

	#ifdef USART3_MPCM_MODE
		#if defined(NO_RX3_INTERRUPT)
			#define USART3_CONFIG_B (1<<TXEN3_BIT)|(1<<UCSZ32_BIT)
	
		#elif defined(NO_TX3_INTERRUPT)
			#define USART3_CONFIG_B (1<<RXEN3_BIT)|(1<<RXCIE3_BIT)|(1<<UCSZ32_BIT)
		#else
			#ifdef USART3_RS485_MODE
				#define USART3_CONFIG_B (1<<TXEN3_BIT)|(1<<TXCIE3_BIT)|(1<<RXEN3_BIT)|(1<<RXCIE3_BIT)|(1<<UCSZ32_BIT)
			#else
				#define USART3_CONFIG_B (1<<TXEN3_BIT)|(1<<RXEN3_BIT)|(1<<RXCIE3_BIT)|(1<<UCSZ32_BIT)
			#endif
		#endif
	#else	
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
	#endif
#endif // USART3_CONFIG

/************************************************************************************
 *                            Initializers                                          *
 ************************************************************************************/
	
	// functions smaller then calling overhead (including context saves) or executed once are inline for reducing 
	// flash memory usage (eg. if statements routines can be executed during compilation)
	
#ifdef USE_USART0
	void uart0_reinit(uint16_t ubbr_value); // for runtime reinitialization of uart
	
	static inline void uart0_init(uint16_t ubbr_value) __attribute__((always_inline));
	static inline void uart0_init(uint16_t ubbr_value) // have to be called once at startup with parameters known during compilation
	{
	#ifdef USART0_RS485_MODE
		___DDR(RS485_CONTROL0_IOPORTNAME) |= (1<<RS485_CONTROL0_PIN); // default pin state is low
	#endif
		
		UBRR0L_REGISTER = (uint8_t) ubbr_value;
		
	#ifndef USART_NO_DIRTY_HACKS
		if(((ubbr_value>>8) != 0)) // requires -Os flag - do not use in non-inline functions
	#endif
			UBRR0H_REGISTER = (ubbr_value>>8);
		
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
		___DDR(RTS0_IOPORTNAME) |= (1<<RTS0_PIN);
	#endif
	}
	
	static inline void uart0_set_FrameFormat(uint8_t UCSRC_reg) __attribute__((always_inline));
	static inline void uart0_set_FrameFormat(uint8_t UCSRC_reg) // UCSRC_reg can be used to set other than 8n1 transmission
	{
		UCSR0C_REGISTER = UCSRC_reg;
	} 
	
	static inline void uart0_set_U2X(void) __attribute__((always_inline));
	static inline void uart0_set_U2X(void) // use instead of USE_DOUBLE_SPEED 
	{
		UCSR0A_REGISTER |= (1<<U2X0_BIT);
	} 

#ifdef USART0_MPCM_MODE
	static inline void uart0_mpcm_slave_return_idle(void) __attribute__((always_inline));
	static inline void uart0_mpcm_slave_return_idle(void) // return slave to mpcm idle mode (wait for own addres frame)
	{
		UCSR0A_REGISTER |= (1<<MPCM0_BIT);
	}
#endif

#ifdef USART0_HARDWARE_FLOW_CONTROL_AVAILABLE
	void uart0_hardware_flow_control_init(uint8_t ctsenable, uint8_t rtsenable) __attribute__((always_inline));
	void uart0_hardware_flow_control_init(uint8_t ctsenable, uint8_t rtsenable) // pass true to enable
	{
		if(ctsenable && rtsenable) // -Os dependent, do not use in non-inline functions 
			UCSR0D_REGISTER = (1<<CTSEN0_BIT)|(1<<RTSEN0_BIT);
		else if(ctsenable)
			UCSR0D_REGISTER = (1<<CTSEN0_BIT);
		else
			UCSR0D_REGISTER = (1<<RTSEN0_BIT);
	}
	#define uart_hardware_flow_control_init(__ctsenable, __rtsenable) uart0_hardware_flow_control_init(__ctsenable, __rtsenable)
#endif
	
	#define uart_init(__ubbr) uart0_init(__ubbr)
	#define uart_reinit(__ubbr) uart0_reinit(__ubbr)
	#define uart_set_FrameFormat(__ucsrc) uart0_set_FrameFormat(__ucsrc)
	#define uart_set_U2X() uart0_set_U2X()
	#define uart_mpcm_slave_return_idle() uart0_mpcm_slave_return_idle()
#endif // USE_USART0

#ifdef USE_USART1
	void uart1_reinit(uint16_t ubbr_value); // for runtime reinitialization of uart
	
	static inline void uart1_init(uint16_t ubbr_value) __attribute__((always_inline));
	static inline void uart1_init(uint16_t ubbr_value) // have to be called once at startup with parameters known during compilation
	{
	#ifdef USART1_RS485_MODE
		___DDR(RS485_CONTROL1_IOPORTNAME) |= (1<<RS485_CONTROL1_PIN); // default pin state is low
	#endif
		
		UBRR1L_REGISTER = (uint8_t) ubbr_value;
		
	#ifndef USART_NO_DIRTY_HACKS
		if(((ubbr_value>>8) != 0)) // requires -Os flag - do not use in non-inline functions
	#endif
		UBRR1H_REGISTER = (ubbr_value>>8);
		
	#ifdef USART1_U2X_SPEED
		#ifdef USART1_MPCM_MODE
			UCSR1A_REGISTER = (1<<U2X1_BIT)|(1<<MPCM1_BIT);
		#else
			UCSR1A_REGISTER = (1<<U2X1_BIT); // enable double speed
		#endif
	#elif defined(USART1_MPCM_MODE)
		UCSR1A_REGISTER |= (1<<MPCM1_BIT);
	#endif
		
		UCSR1B_REGISTER = USART1_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
		
	#ifdef USART1_USE_SOFT_RTS
		___DDR(RTS1_IOPORTNAME) |= (1<<RTS1_PIN);
	#endif
	}

	static inline void uart1_set_FrameFormat(uint8_t UCSRC_reg) __attribute__((always_inline));
	static inline void uart1_set_FrameFormat(uint8_t UCSRC_reg) // UCSRC_reg can be used to set other than 8n1 transmission
	{
		UCSR1C_REGISTER = UCSRC_reg;
	} 
	
	static inline void uart1_set_U2X(void) __attribute__((always_inline));
	static inline void uart1_set_U2X(void) // use instead of USE_DOUBLE_SPEED 
	{
		UCSR1A_REGISTER |= (1<<U2X1_BIT);
	} 

#ifdef USART1_MPCM_MODE
	static inline void uart1_mpcm_slave_return_idle(void) __attribute__((always_inline));
	static inline void uart1_mpcm_slave_return_idle(void) // return slave to mpcm idle mode (wait for own addres frame)
	{
		UCSR1A_REGISTER |= (1<<MPCM1_BIT);
	}
#endif
#endif // USE_USART1

#ifdef USE_USART2
	void uart2_reinit(uint16_t ubbr_value); // for runtime reinitialization of uart
	
	static inline void uart2_init(uint16_t ubbr_value) __attribute__((always_inline));
	static inline void uart2_init(uint16_t ubbr_value) // have to be called once at startup with parameters known during compilation
	{
	#ifdef USART2_RS485_MODE
		___DDR(RS485_CONTROL2_IOPORTNAME) |= (1<<RS485_CONTROL2_PIN); // default pin state is low
	#endif
		
		UBRR2L_REGISTER = (uint8_t) ubbr_value;
		
	#ifndef USART_NO_DIRTY_HACKS
		if(((ubbr_value>>8) != 0)) // requires -Os flag - do not use in non-inline functions
	#endif
			UBRR2H_REGISTER = (ubbr_value>>8);
		
	#ifdef USART2_U2X_SPEED
		#ifdef USART2_MPCM_MODE
			UCSR2A_REGISTER = (1<<U2X2_BIT)|(1<<MPCM2_BIT);
		#else
			UCSR2A_REGISTER = (1<<U2X2_BIT); // enable double speed
		#endif
	#elif defined(USART2_MPCM_MODE)
		UCSR2A_REGISTER |= (1<<MPCM2_BIT);
	#endif
		
		UCSR2B_REGISTER = USART2_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
		
	#ifdef USART2_USE_SOFT_RTS
		___DDR(RTS2_IOPORTNAME) |= (1<<RTS2_PIN);
	#endif
	}

	static inline void uart2_set_FrameFormat(uint8_t UCSRC_reg) __attribute__((always_inline));
	static inline void uart2_set_FrameFormat(uint8_t UCSRC_reg) // UCSRC_reg can be used to set other than 8n1 transmission
	{
		UCSR2C_REGISTER = UCSRC_reg;
	} 
	
	static inline void uart2_set_U2X(void) __attribute__((always_inline));
	static inline void uart2_set_U2X(void) // use instead of USE_DOUBLE_SPEED 
	{
		UCSR2A_REGISTER |= (1<<U2X2_BIT);
	} 

#ifdef USART2_MPCM_MODE
	static inline void uart2_mpcm_slave_return_idle(void) __attribute__((always_inline));
	static inline void uart2_mpcm_slave_return_idle(void) // return slave to mpcm idle mode (wait for own addres frame)
	{
		UCSR2A_REGISTER |= (1<<MPCM2_BIT);
	}
#endif
#endif // USE_USART2

#ifdef USE_USART3
	void uart3_reinit(uint16_t ubbr_value); // for runtime reinitialization of uart
	
	static inline void uart3_init(uint16_t ubbr_value) __attribute__((always_inline));
	static inline void uart3_init(uint16_t ubbr_value) // have to be called once at startup with parameters known during compilation
	{
	#ifdef USART3_RS485_MODE
		___DDR(RS485_CONTROL3_IOPORTNAME) |= (1<<RS485_CONTROL3_PIN); // default pin state is low
	#endif
		
		UBRR3L_REGISTER = (uint8_t) ubbr_value;
		
	#ifndef USART_NO_DIRTY_HACKS
		if(((ubbr_value>>8) != 0)) // requires -Os flag - do not use in non-inline functions
	#endif
			UBRR3H_REGISTER = (ubbr_value>>8);
		
	#ifdef USART3_U2X_SPEED
		#ifdef USART3_MPCM_MODE
			UCSR3A_REGISTER = (1<<U2X3_BIT)|(1<<MPCM3_BIT);
		#else
			UCSR3A_REGISTER = (1<<U2X3_BIT); // enable double speed
		#endif
	#elif defined(USART3_MPCM_MODE)
		UCSR3A_REGISTER |= (1<<MPCM3_BIT);
	#endif
		
		UCSR3B_REGISTER = USART3_CONFIG_B;
		// 8n1 is set by default, setting UCSRC is not needed
		
	#ifdef USART3_USE_SOFT_RTS
		___DDR(RTS3_IOPORTNAME) |= (1<<RTS3_PIN);
	#endif
	}

	static inline void uart3_set_FrameFormat(uint8_t UCSRC_reg) __attribute__((always_inline));
	static inline void uart3_set_FrameFormat(uint8_t UCSRC_reg) // UCSRC_reg can be used to set other than 8n1 transmission
	{
		UCSR3C_REGISTER = UCSRC_reg;
	} 
	
	static inline void uart3_set_U2X(void) __attribute__((always_inline));
	static inline void uart3_set_U2X(void) // use instead of USE_DOUBLE_SPEED 
	{
		UCSR3A_REGISTER |= (1<<U2X0_BIT);
	} 

#ifdef USART3_MPCM_MODE
	static inline void uart3_mpcm_slave_return_idle(void) __attribute__((always_inline));
	static inline void uart3_mpcm_slave_return_idle(void) // return slave to mpcm idle mode (wait for own addres frame)
	{
		UCSR3A_REGISTER |= (1<<MPCM3_BIT);
	}
#endif
#endif // USE_USART3
	
/************************************************************************************
 *                          Transmitter functions                                   *
 ************************************************************************************/
#ifndef NO_USART_TX
	
	#ifndef NO_TX0_INTERRUPT
		#ifdef USART_NO_DIRTY_HACKS
			void uart0_putc(char data); // put character/data into transmitter ring buffer
		#else
			void uart0_putc(char data) __attribute__ ((naked, noinline));
		#endif
			char uart0_putc_(char data) __attribute__ ((noinline)); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
		
		uint8_t uart0_putc_noblock(char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
		void uart0_putstrl(char *string, uint8_t BytesToWrite); // in case of bascom users or buffers without NULL byte ending
		void uart0_putstr(char *string); // send string from the memory buffer
			// stops when NULL byte is hit (NULL byte is not included into transmission)
		#ifdef __cplusplus
			#define uart0_puts(str) uart0_putstr((const char*)(str))// macro to avoid const char* conversion restrictions
		#else
			#define uart0_puts(str) uart0_putstr(str)
		#endif
			// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P

		void uart0_puts_p(const char *string); // send string from flash memory
			#define uart0_puts_P(__strP)    uart0_puts_p(PSTR(__strP))
			// macro to automatically put a string constant into flash
	
		void uart0_putint(int16_t data);
		void uart0_putintr(int16_t data, uint8_t radix);

		void uart0_putuint(uint16_t data);
		void uart0_putuintr(uint16_t data, uint8_t radix);
	
		void uart0_puthex(uint8_t data);

		void uart0_putlong(int32_t data);
		void uart0_putlongr(int32_t data, uint8_t radix);

		void uart0_putulong(uint32_t data);
		void uart0_putulongr(uint32_t data, uint8_t radix);
	
		void uart0_putfloat(float data);
		void uart0_fputfloat(float data, uint8_t precision);
	
		void uart0_flush(void); // flush tx buffer
	
		#ifdef USART0_MPCM_MODE
			void uart0_mpcm_transmit_addres_Frame(uint8_t dat);
		#endif
		
		#define uart_putc(__data) uart0_putc(__data)
		#define uart_putc_(__data) uart0_putc_(__data)
		#define uart_putc_noblock(__data) uart0_putc_noblock(__data)
		#define uart_putstrl(__str, __limit) uart0_putstrl(__str, __limit)
		#define uart_putstr(__str) uart0_putstr(__str)
		#define uart_puts(__str) uart0_puts(__str)
		#define uart_puts_p(__strP) uart0_puts_p(__strP)
		#define uart_puts_P(__str) uart0_puts_P(__str)
		#define uart_putint(__data) uart0_putint(__data)
		#define uart_putintr(__data, __radix) uart0_putintr(__data, __radix)
		#define uart_putuint(__data) uart0_putuint(__data)
		#define uart_putuintr(__data, __radix) uart0_putuintr(__data, __radix)
		#define uart_puthex(__data) uart0_puthex(__data)
		#define uart_putlong(__data) uart0_putlong(__data)
		#define uart_putlongr(__data, __radix) uart0_putlongr(__data, __radix)
		#define uart_putulong(__data) uart0_putulong(__data)
		#define uart_putulongr(__data, __radix) uart0_putulongr(__data, __radix)
		#define uart_putfloat(__data) uart0_putfloat(__data)
		#define uart_fputfloat(__data, __precision) uart0_fputfloat(__data, __precision)
		#define uart_flush() uart0_flush()
		
		#ifdef USART0_MPCM_MODE
			#define uart_mpcm_transmit_addres_Frame(__data) uart0_mpcm_transmit_addres_Frame(__data)
		#endif
	#endif // NO_TX0_INTERRUPT
	
	#ifndef NO_TX1_INTERRUPT
		#ifdef USART_NO_DIRTY_HACKS
			void uart1_putc(char data); // put character/data into transmitter ring buffer
		#else
			void uart1_putc(char data) __attribute__ ((naked, noinline));
		#endif
			char uart1_putc_(char data) __attribute__ ((noinline)); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
		
		uint8_t uart1_putc_noblock(char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
		void uart1_putstrl(char *string, uint8_t BytesToWrite); // in case of bascom users or buffers without NULL byte ending
		void uart1_putstr(char *string); // send string from the memory buffer
			// stops when NULL byte is hit (NULL byte is not included into transmission)
		#ifdef __cplusplus
			#define uart1_puts(str) uart1_putstr((const char*)(str))// macro to avoid const char* conversion restrictions
		#else
			#define uart1_puts(str) uart1_putstr(str)
		#endif
			// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P

		void uart1_puts_p(const char *string); // send string from flash memory
			#define uart1_puts_P(__s)    uart1_puts_p(PSTR(__s))
			// macro to automatically put a string constant into flash
	
		void uart1_putint(int16_t data);
		void uart1_putintr(int16_t data, uint8_t radix);

		void uart1_putuint(uint16_t data);
		void uart1_putuintr(uint16_t data, uint8_t radix);
	
		void uart1_puthex(uint8_t data);

		void uart1_putlong(int32_t data);
		void uart1_putlongr(int32_t data, uint8_t radix);

		void uart1_putulong(uint32_t data);
		void uart1_putulongr(uint32_t data, uint8_t radix);
	
		void uart1_putfloat(float data);
		void uart1_fputfloat(float data, uint8_t precision);
	
		void uart1_flush(void); // flush tx buffer
	
		#ifdef USART1_MPCM_MODE
			void uart1_mpcm_transmit_addres_Frame(uint8_t dat);
		#endif
	#endif // NO_TX1_INTERRUPT
	
	#ifndef NO_TX2_INTERRUPT
		#ifdef USART_NO_DIRTY_HACKS
			void uart2_putc(char data); // put character/data into transmitter ring buffer
		#else
			void uart2_putc(char data) __attribute__ ((naked, noinline));
		#endif
			char uart2_putc_(char data) __attribute__ ((noinline)); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
		
		uint8_t uart2_putc_noblock(char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
		void uart2_putstrl(char *string, uint8_t BytesToWrite); // in case of bascom users or buffers without NULL byte ending
		void uart2_putstr(char *string); // send string from the memory buffer
			// stops when NULL byte is hit (NULL byte is not included into transmission)
		#ifdef __cplusplus
			#define uart2_puts(str) uart2_putstr((const char*)(str))// macro to avoid const char* conversion restrictions
		#else
			#define uart2_puts(str) uart2_putstr(str)
		#endif
			// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P

		void uart2_puts_p(const char *string); // send string from flash memory
			#define uart2_puts_P(__s)    uart2_puts_p(PSTR(__s))
			// macro to automatically put a string constant into flash
	
		void uart2_putint(int16_t data);
		void uart2_putintr(int16_t data, uint8_t radix);

		void uart2_putuint(uint16_t data);
		void uart2_putuintr(uint16_t data, uint8_t radix);
	
		void uart2_puthex(uint8_t data);

		void uart2_putlong(int32_t data);
		void uart2_putlongr(int32_t data, uint8_t radix);

		void uart2_putulong(uint32_t data);
		void uart2_putulongr(uint32_t data, uint8_t radix);
	
		void uart2_putfloat(float data);
		void uart2_fputfloat(float data, uint8_t precision);
	
		void uart2_flush(void); // flush tx buffer
	
		#ifdef USART2_MPCM_MODE
			void uart2_mpcm_transmit_addres_Frame(uint8_t dat);
		#endif
	#endif // NO_TX2_INTERRUPT
	
	#ifndef NO_TX3_INTERRUPT
		#ifdef USART_NO_DIRTY_HACKS
			void uart3_putc(char data); // put character/data into transmitter ring buffer
		#else
			void uart3_putc(char data) __attribute__ ((naked, noinline));
		#endif
			char uart3_putc_(char data) __attribute__ ((noinline)); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
		
		uint8_t uart3_putc_noblock(char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
		void uart3_putstrl(char *string, uint8_t BytesToWrite); // in case of bascom users or buffers without NULL byte ending
		void uart3_putstr(char *string); // send string from the memory buffer
			// stops when NULL byte is hit (NULL byte is not included into transmission)
		#ifdef __cplusplus
			#define uart3_puts(str) uart3_putstr((const char*)(str))// macro to avoid const char* conversion restrictions
		#else
			#define uart3_puts(str) uart3_putstr(str)
		#endif
			// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P

		void uart3_puts_p(const char *string); // send string from flash memory
			#define uart3_puts_P(__s)    uart3_puts_p(PSTR(__s))
			// macro to automatically put a string constant into flash
	
		void uart3_putint(int16_t data);
		void uart3_putintr(int16_t data, uint8_t radix);

		void uart3_putuint(uint16_t data);
		void uart3_putuintr(uint16_t data, uint8_t radix);
	
		void uart3_puthex(uint8_t data);

		void uart3_putlong(int32_t data);
		void uart3_putlongr(int32_t data, uint8_t radix);

		void uart3_putulong(uint32_t data);
		void uart3_putulongr(uint32_t data, uint8_t radix);
	
		void uart3_putfloat(float data);
		void uart3_fputfloat(float data, uint8_t precision);
	
		void uart3_flush(void); // flush tx buffer
	
		#ifdef USART3_MPCM_MODE
			void uart3_mpcm_transmit_addres_Frame(uint8_t dat);
		#endif
	#endif // NO_TX0_INTERRUPT

#endif // NO_USART_TX

/************************************************************************************
 *                           Receiver functions                                     *
 ************************************************************************************/
#ifndef NO_USART_RX
	
	#ifndef NO_RX0_INTERRUPT
		char uart0_getc(void); // get character from receiver ring buffer
	
		void uart0_gets(char *buffer, uint8_t bufferlimit); // reads whole receiver buffer or bufferlimit-1 characters
		// newline terminator will not be cut // adds NULL byte at the end of string
		void uart0_getln(char *buffer, uint8_t bufferlimit); // reads one line from the buffer
		// waits for newline terminator or reached bufferlimit // adds NULL byte at the end of string
		void uart0_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit); // read one line to the first whitespace after the string
		// cuts all whitespaces before string and one after the string
	
		char uart0_skipWhiteSpaces(void); // returns first nonspace character found in the buffer
	
		int16_t uart0_getint(void);
		int32_t uart0_getlong(void);
		float uart0_getfloat(void);
	
		int16_t uart0_getData(void); // reads single byte from a buffer // returns negative value if buffer is empty (upper byte is non zero)
		uint8_t uart0_LoadData(uint8_t *data); // reads single byte from a buffer and loads it into *data byte
		// in case of empty buffers returned flag is set to BUFFER_EMPTY - NULL
		
		extern volatile uint8_t rx0_last_byte, rx0_first_byte;
		//static inline uint8_t uart0_AvailableBytes(void) __attribute__((always_inline)); // just let the compiler
		static inline uint8_t uart0_AvailableBytes(void) // returns number of bytes waiting in the receiver buffer
		{
			return (rx0_last_byte - rx0_first_byte) & RX0_BUFFER_MASK;
		}
		uint8_t uart0_peek(void); // returns next byte from buffer // returned byte is invalid if there is nothing to read
		
		#define uart_getc() uart0_getc()
		#define uart_gets(__buffptr, __limit) uart0_gets(__buffptr, __limit)
		#define uart_getln(__buffptr, __limit) uart0_getln(__buffptr, __limit)
		#define uart_getlnToFirstWhiteSpace(__buffptr, __limit) uart0_getlnToFirstWhiteSpace(__buffptr, __limit)
		#define uart_skipWhiteSpaces() uart0_skipWhiteSpaces()
		#define uart_getint() uart0_getint()
		#define uart_getlong() uart0_getlong()
		#define uart_getfloat() uart0_getfloat()
		#define uart_getData() uart0_getData()
		#define uart_LoadData(__dataptr) uart0_LoadData(__dataptr)
		#define uart_AvailableBytes() uart0_AvailableBytes()
		#define uart_peek() uart0_peek()
	#endif // NO_RX0_INTERRUPT
	
	#ifndef NO_RX1_INTERRUPT
		char uart1_getc(void); // get character from receiver ring buffer
		
		void uart1_gets(char *buffer, uint8_t bufferlimit); // reads whole receiver buffer or bufferlimit-1 characters
		// newline terminator will not be cut // adds NULL byte at the end of string
		void uart1_getln(char *buffer, uint8_t bufferlimit); // reads one line from the buffer
		// waits for newline terminator or reached bufferlimit // adds NULL byte at the end of string
		void uart1_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit); // read one line to the first whitespace after the string
		// cuts all whitespaces before string and one after the string
		
		char uart1_skipWhiteSpaces(void); // returns first nonspace character found in the buffer
		
		int16_t uart1_getint(void);
		int32_t uart1_getlong(void);
		float uart1_getfloat(void);
		
		int16_t uart1_getData(void); // reads single byte from a buffer // returns negative value if buffer is empty (upper byte is non zero)
		uint8_t uart1_LoadData(uint8_t *data); // reads single byte from a buffer and loads it into *data byte
		// in case of empty buffers returned flag is set to BUFFER_EMPTY - NULL
		
		extern volatile uint8_t rx1_last_byte, rx1_first_byte;
		//static inline uint8_t uart1_AvailableBytes(void) __attribute__((always_inline)); // just let the compiler
		static inline uint8_t uart1_AvailableBytes(void) // returns number of bytes waiting in the receiver buffer
		{
			return (rx1_last_byte - rx1_first_byte) & RX1_BUFFER_MASK;
		}
		uint8_t uart1_peek(void); // returns next byte from buffer // returned byte is invalid if there is nothing to read
	#endif // NO_RX0_INTERRUPT
	
	#ifndef NO_RX2_INTERRUPT
		char uart2_getc(void); // get character from receiver ring buffer
		
		void uart2_gets(char *buffer, uint8_t bufferlimit); // reads whole receiver buffer or bufferlimit-1 characters
		// newline terminator will not be cut // adds NULL byte at the end of string
		void uart2_getln(char *buffer, uint8_t bufferlimit); // reads one line from the buffer
		// waits for newline terminator or reached bufferlimit // adds NULL byte at the end of string
		void uart2_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit); // read one line to the first whitespace after the string
		// cuts all whitespaces before string and one after the string
		
		char uart2_skipWhiteSpaces(void); // returns first nonspace character found in the buffer
		
		int16_t uart2_getint(void);
		int32_t uart2_getlong(void);
		float uart2_getfloat(void);
		
		int16_t uart2_getData(void); // reads single byte from a buffer // returns negative value if buffer is empty (upper byte is non zero)
		uint8_t uart2_LoadData(uint8_t *data); // reads single byte from a buffer and loads it into *data byte
		// in case of empty buffers returned flag is set to BUFFER_EMPTY - NULL
		
		extern volatile uint8_t rx2_last_byte, rx2_first_byte;
		//static inline uint8_t uart2_AvailableBytes(void) __attribute__((always_inline)); // just let the compiler
		static inline uint8_t uart2_AvailableBytes(void) // returns number of bytes waiting in the receiver buffer
		{
			return (rx2_last_byte - rx2_first_byte) & RX2_BUFFER_MASK;
		}
		uint8_t uart2_peek(void); // returns next byte from buffer // returned byte is invalid if there is nothing to read
	#endif // NO_RX0_INTERRUPT
	
	#ifndef NO_RX3_INTERRUPT
		char uart3_getc(void); // get character from receiver ring buffer
		
		void uart3_gets(char *buffer, uint8_t bufferlimit); // reads whole receiver buffer or bufferlimit-1 characters
		// newline terminator will not be cut // adds NULL byte at the end of string
		void uart3_getln(char *buffer, uint8_t bufferlimit); // reads one line from the buffer
		// waits for newline terminator or reached bufferlimit // adds NULL byte at the end of string
		void uart3_getlnToFirstWhiteSpace(char *buffer, uint8_t bufferlimit); // read one line to the first whitespace after the string
		// cuts all whitespaces before string and one after the string
		
		char uart3_skipWhiteSpaces(void); // returns first nonspace character found in the buffer
		
		int16_t uart3_getint(void);
		int32_t uart3_getlong(void);
		float uart3_getfloat(void);
		
		int16_t uart3_getData(void); // reads single byte from a buffer // returns negative value if buffer is empty (upper byte is non zero)
		uint8_t uart3_LoadData(uint8_t *data); // reads single byte from a buffer and loads it into *data byte
		// in case of empty buffers returned flag is set to BUFFER_EMPTY - NULL
		
		extern volatile uint8_t rx3_last_byte, rx3_first_byte;
		//static inline uint8_t uart3_AvailableBytes(void) __attribute__((always_inline)); // just let the compiler
		static inline uint8_t uart3_AvailableBytes(void) // returns number of bytes waiting in the receiver buffer
		{
			return (rx3_last_byte - rx3_first_byte) & RX3_BUFFER_MASK;
		}
		uint8_t uart3_peek(void); // returns next byte from buffer // returned byte is invalid if there is nothing to read
	#endif // NO_RX0_INTERRUPT

#endif // NO_USART_RX

/************************************************************************************
 *                    soft flow control interrupt handlers                          *
 ************************************************************************************/

#if defined(USART0_USE_SOFT_CTS)&&!defined(NO_TX0_INTERRUPT) //&&defined(USE_USART0)
	extern volatile uint8_t tx0_first_byte, tx0_last_byte;

	static inline void cts0_isr_handler(void)
	{
	#if defined(USART0_IN_IO_ADDRESS_SPACE)&&!defined(USART0_NOT_ACCESIBLE_FROM_CBI)
		if(___PIN(CTS0_IOPORTNAME) & (1<<CTS0_PIN))
		{
			//UCSR0B_REGISTER &= ~(1<<UDRIE0_BIT);
		}
		else if (tx0_first_byte != tx0_last_byte)
		{
			UCSR0B_REGISTER |= (1<<UDRIE0_BIT);
		}
	#else
		uint8_t tmp = UCSR0B_REGISTER;
		if(___PIN(CTS0_IOPORTNAME) & (1<<CTS0_PIN))
		{
			tmp &= ~(1<<UDRIE0_BIT);
		}
		else if (tx0_first_byte != tx0_last_byte)
		{
			tmp |= (1<<UDRIE0_BIT);
		}
		UCSR0B_REGISTER = tmp;
	#endif
	}
#endif

#if defined(USART1_USE_SOFT_CTS)&&!defined(NO_TX1_INTERRUPT)
	extern volatile uint8_t tx1_first_byte, tx1_last_byte;

	static inline void cts1_isr_handler(void)
	{
	#if defined(USART1_IN_IO_ADDRESS_SPACE)
		if(___PIN(CTS1_IOPORTNAME) & (1<<CTS1_PIN))
		{
			UCSR1B_REGISTER &= ~(1<<UDRIE1_BIT);
		}
		else
		{
			if (tx1_first_byte != tx1_last_byte)
			UCSR1B_REGISTER |= (1<<UDRIE1_BIT);
		}
	#else
		uint8_t tmp = UCSR1B_REGISTER;
		if(___PIN(CTS1_IOPORTNAME) & (1<<CTS1_PIN))
		{
			tmp &= ~(1<<UDRIE1_BIT);
		}
		else
		{
			if (tx1_first_byte != tx1_last_byte)
			tmp |= (1<<UDRIE1_BIT);
		}
		UCSR1B_REGISTER = tmp;
	#endif
	}
#endif

#if defined(USART2_USE_SOFT_CTS)&&!defined(NO_TX2_INTERRUPT)
	extern volatile uint8_t tx2_first_byte, tx2_last_byte;

	static inline void cts2_isr_handler(void)
	{
		uint8_t tmp = UCSR2B_REGISTER;
		if(___PIN(CTS2_IOPORTNAME) & (1<<CTS2_PIN))
		{
			tmp &= ~(1<<UDRIE2_BIT);
		}
		else
		{
			if (tx2_first_byte != tx2_last_byte)
			tmp |= (1<<UDRIE2_BIT);
		}
		UCSR2B_REGISTER = tmp;
	}
#endif

#if defined(USART3_USE_SOFT_CTS)&&!defined(NO_TX3_INTERRUPT)
	extern volatile uint8_t tx3_first_byte, tx3_last_byte;

	static inline void cts3_isr_handler(void)
	{
		uint8_t tmp = UCSR3B_REGISTER;
		if(___PIN(CTS3_IOPORTNAME) & (1<<CTS3_PIN))
		{
			tmp &= ~(1<<UDRIE3_BIT);
		}
		else
		{
			if (tx3_first_byte != tx3_last_byte)
			tmp |= (1<<UDRIE3_BIT);
		}
		UCSR3B_REGISTER = tmp;
	}
#endif

/************************************************************************************
 *                              stdio.h stuff                                       *
 ************************************************************************************/
#include <stdio.h> // avoid compilation errors

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
		void uart_putchar(char data, FILE *stream);
	#endif

	#ifndef NO_USART_RX
		char uart_getchar(FILE *stream);
	#endif
	
	#ifdef USE_USART0
	
		#if defined(NO_RX0_INTERRUPT)
			extern FILE uart0_out;
		
		#elif defined(NO_TX0_INTERRUPT)
			extern FILE uart0_in;
		#else
			extern FILE uart0_io;
		#endif
		
	#endif // USE_USART0
	
	#ifdef USE_USART1
	
		#if defined(NO_RX1_INTERRUPT)
			extern FILE uart1_out;
		
		#elif defined(NO_TX1_INTERRUPT)
			extern FILE uart1_in;
		#else
			extern FILE uart1_io;
		#endif
		
	#endif // USE_USART1
	
	#ifdef USE_USART2
	
		#if defined(NO_RX2_INTERRUPT)
			extern FILE uart2_out;
		
		#elif defined(NO_TX2_INTERRUPT)
			extern FILE uart2_in;
		#else
			extern FILE uart2_io;
		#endif
		
	#endif // USE_USART2
		
	#ifdef USE_USART3
		
		#if defined(NO_RX3_INTERRUPT)
			extern FILE uart3_out;
		
		#elif defined(NO_TX3_INTERRUPT)
			extern FILE uart3_in;
		#else
			extern FILE uart3_io;
		#endif
		
	#endif // USE_USART3
	
#else // single USART mcu

	#ifndef NO_TX0_INTERRUPT
		void uart_putchar(char data, FILE *stream);
	#endif

	#ifndef NO_RX0_INTERRUPT
		char uart_getchar(FILE *stream);
	#endif
	
	#if defined(NO_RX0_INTERRUPT)
		extern FILE uart0_out;
	
	#elif defined(NO_TX0_INTERRUPT)
		extern FILE uart0_in;
	#else
		extern FILE uart0_io;
	#endif
	
#endif // single/multi USART

#endif // _USART_H_