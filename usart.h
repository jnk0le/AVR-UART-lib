#ifndef _USART_H_
#define _USART_H_

/************************************************************************************
 *  Author: jnk0le@hotmail.com                                                      *
 *  https://github.com/jnk0le                                                       *
 *  This library is distributed under MIT license terms                             *
 ************************************************************************************/

#include <avr/io.h> // for inline func
#include "usart_config.h"

#ifndef F_CPU
	#warning F_CPU is undefined, USART may not work correctly without this
#endif

#define BAUD_CALC(x) ((F_CPU+(x)*8UL) / (16UL*(x))-1UL) // macro calculating precise UBRR value
#define BAUD_CALC_FAST(x) ((F_CPU)/((x)*16UL)-1) // for faster real time calculations ? // not recommended
#define DOUBLE_BAUD_CALC(x) ((F_CPU+(x)*4UL) / (8UL*(x))-1UL) // macro calculating UBRR value for double speed

#if !defined(__OPTIMIZE__)&&!defined(USART_NO_ABI_BREAKING_PREMATURES)
	#warning Compiler optimizations disabled; functions from usart.h might not work as designed
#endif

#ifdef DEBUG
	#define USART_NO_ABI_BREAKING_PREMATURES
#endif

#ifndef __AVR_ARCH__ // compiler fault ?
	#define USART_NO_ABI_BREAKING_PREMATURES
#endif

#ifndef RX_BUFFER_SIZE
	#define RX_BUFFER_SIZE 32 // Size of the ring buffers, must be power of 2
#endif

#ifndef TX_BUFFER_SIZE
	#define TX_BUFFER_SIZE 32 // Size of the ring buffers, must be power of 2
#endif

#ifndef TX0_BUFFER_SIZE
	#define TX0_BUFFER_SIZE TX_BUFFER_SIZE
#endif
#ifndef RX0_BUFFER_SIZE
	#define RX0_BUFFER_SIZE RX_BUFFER_SIZE
#endif
#ifndef TX1_BUFFER_SIZE
	#define TX1_BUFFER_SIZE TX_BUFFER_SIZE
#endif
#ifndef RX1_BUFFER_SIZE
	#define RX1_BUFFER_SIZE RX_BUFFER_SIZE
#endif
#ifndef TX2_BUFFER_SIZE
	#define TX2_BUFFER_SIZE TX_BUFFER_SIZE
#endif
#ifndef RX2_BUFFER_SIZE
	#define RX2_BUFFER_SIZE RX_BUFFER_SIZE
#endif
#ifndef TX3_BUFFER_SIZE
	#define TX3_BUFFER_SIZE TX_BUFFER_SIZE
#endif
#ifndef RX3_BUFFER_SIZE
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

enum {COMPLETED = 1, BUFFER_EMPTY = 0, BUFFER_FULL = 0};
	
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

	#define USART_8N1 (USART_8BIT_DATA|USART_NO_PARITY|USART_1STOP_BIT)
	#define USART_8N2 (USART_8BIT_DATA|USART_NO_PARITY|USART_2STOP_BITS)
	#define USART_8E1 (USART_8BIT_DATA|USART_EVEN_PARITY|USART_1STOP_BIT)
	#define USART_8E2 (USART_8BIT_DATA|USART_EVEN_PARITY|USART_2STOP_BITS)
	#define USART_8O1 (USART_8BIT_DATA|USART_ODD_PARITY|USART_1STOP_BIT)
	#define USART_8O2 (USART_8BIT_DATA|USART_ODD_PARITY|USART_2STOP_BITS)
	#define USART_7N1 (USART_7BIT_DATA|USART_NO_PARITY|USART_1STOP_BIT)
	#define USART_7N2 (USART_7BIT_DATA|USART_NO_PARITY|USART_2STOP_BITS)
	#define USART_7E1 (USART_7BIT_DATA|USART_EVEN_PARITY|USART_1STOP_BIT)
	#define USART_7E2 (USART_7BIT_DATA|USART_EVEN_PARITY|USART_2STOP_BITS)
	#define USART_7O1 (USART_7BIT_DATA|USART_ODD_PARITY|USART_1STOP_BIT)
	#define USART_7O2 (USART_7BIT_DATA|USART_ODD_PARITY|USART_2STOP_BITS)

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

#ifdef PUTC_CONVERT_LF_TO_CRLF
	#define PUTC0_CONVERT_LF_TO_CRLF
	#define PUTC1_CONVERT_LF_TO_CRLF
	#define PUTC2_CONVERT_LF_TO_CRLF
	#define PUTC3_CONVERT_LF_TO_CRLF
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

#ifdef USART_MPCM_MODE
	#define USART0_MPCM_MODE
	#define USART1_MPCM_MODE
	#define USART2_MPCM_MODE
	#define USART3_MPCM_MODE
#endif

#if defined(CTS0_DDR)&&defined(CTS0_PORT)&&defined(CTS0_PIN)&&defined(CTS0_IONUM)
	#define USART0_USE_SOFT_CTS
#endif
#if defined(CTS1_DDR)&&defined(CTS1_PORT)&&defined(CTS1_PIN)&&defined(CTS1_IONUM)
	#define USART1_USE_SOFT_CTS
#endif
#if defined(CTS2_DDR)&&defined(CTS2_PORT)&&defined(CTS2_PIN)&&defined(CTS2_IONUM)
	#define USART2_USE_SOFT_CTS
#endif
#if defined(CTS3_DDR)&&defined(CTS3_PORT)&&defined(CTS3_PIN)&&defined(CTS3_IONUM)
	#define USART3_USE_SOFT_CTS
#endif

#if defined(RTS0_DDR)&&defined(RTS0_PORT)&&defined(RTS0_PIN)&&defined(RTS0_IONUM)
	#define USART0_USE_SOFT_RTS
#endif
#if defined(RTS1_DDR)&&defined(RTS1_PORT)&&defined(RTS1_PIN)&&defined(RTS1_IONUM)
	#define USART1_USE_SOFT_RTS
#endif
#if defined(RTS2_DDR)&&defined(RTS2_PORT)&&defined(RTS2_PIN)&&defined(RTS2_IONUM)
	#define USART2_USE_SOFT_RTS
#endif
#if defined(RTS3_DDR)&&defined(RTS3_PORT)&&defined(RTS3_PIN)&&defined(RTS3_IONUM)
	#define USART3_USE_SOFT_RTS
#endif

#if defined(RS485_CONTROL0_DDR)&&defined(RS485_CONTROL0_PORT)&&defined(RS485_CONTROL0_PIN)&&defined(RS485_CONTROL0_IONUM)
	#define USART0_RS485_MODE
#endif
#if defined(RS485_CONTROL1_DDR)&&defined(RS485_CONTROL1_PORT)&&defined(RS485_CONTROL1_PIN)&&defined(RS485_CONTROL1_IONUM)
	#define USART1_RS485_MODE
#endif
#if defined(RS485_CONTROL2_DDR)&&defined(RS485_CONTROL2_PORT)&&defined(RS485_CONTROL2_PIN)&&defined(RS485_CONTROL2_IONUM)
	#define USART2_RS485_MODE
#endif
#if defined(RS485_CONTROL3_DDR)&&defined(RS485_CONTROL3_PORT)&&defined(RS485_CONTROL3_PIN)&&defined(RS485_CONTROL3_IONUM)
	#define USART3_RS485_MODE
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

#ifdef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
	register uint8_t USART_SREG_SAVE_REG_NAME asm(USART_SREG_SAVE_REG_NUM); // have to be defined separately in every compilation unit
#endif

#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
	register uint16_t USART_Z_SAVE_REG_NAME asm(USART_Z_SAVE_REG_NUM); // have to be defined separately in every compilation unit
#endif

#if defined(USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE)&&defined(USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE)
	#define USART_REG_SAVE_LIST \
		[sreg_save] "+r" (USART_SREG_SAVE_REG_NAME), \
		[z_save] "+r" (USART_Z_SAVE_REG_NAME)
		
#elif defined(USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE)
	#define USART_REG_SAVE_LIST \
		[z_save] "+r" (USART_Z_SAVE_REG_NAME)
	
#elif defined(USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE)
	#define USART_REG_SAVE_LIST \
		[sreg_save] "+r" (USART_SREG_SAVE_REG_NAME)
#else
	#define USART_REG_SAVE_LIST
#endif

#if defined(__usbdrv_h_included__)&&!defined(USART_UNSAFE_RX_INTERRUPT)
	#warning "usb may not work with uart's RX ISR"
#endif

#if defined(__usbdrv_h_included__)&&!defined(USART_UNSAFE_TX_INTERRUPT)
	#warning "usb may not work with uart's TX ISR"
#endif

#if (defined(USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE)||defined(USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE))&&(defined(USART_UNSAFE_TX_INTERRUPT)||defined(USART_UNSAFE_RX_INTERRUPT))
	#warning "using globally reserved save registers with interruptable interrupts might create special conditions"
#endif

#if defined(__AVR_ATtiny102__)||defined(__AVR_ATtiny104__)||defined(__AVR_ATtiny2313__)||defined(__AVR_ATtiny2313A__)
	#define USART_USE_TINY_MEMORY_MODEL
#endif

#if defined(__AVR_ATtiny102__)||defined(__AVR_ATtiny104__)

#if (TX0_BUFFER_SIZE > 8)||(RX0_BUFFER_SIZE > 8)
	#warning "TX or RX buffer may be too large for this mcu"
#endif

#define USART0_IN_UPPER_IO_ADDRESS_SPACE

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80
	
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
	#define MPCM0_BIT   		MPCM
	#define UCSZ02_BIT  		UCSZ2
	#define TXB80_BIT   		TXB8

#endif //NO_USART0
#endif

#if defined(__AVR_ATtiny1634__)

#define USART0_IN_UPPER_IO_ADDRESS_SPACE

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80

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
	#define MPCM1_BIT   		MPCM1
	#define UCSZ12_BIT  		UCSZ12
	#define TXB81_BIT   		TXB81

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80

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
	#define MPCM1_BIT   		MPCM1
	#define UCSZ12_BIT  		UCSZ12
	#define TXB81_BIT   		TXB81

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80	

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80

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
	#define MPCM1_BIT   		MPCM1
	#define UCSZ12_BIT  		UCSZ12
	#define TXB81_BIT   		TXB81	

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
	#define MPCM0_BIT   		MPCM
	#define UCSZ02_BIT  		UCSZ2
	#define TXB80_BIT   		TXB8
	
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
	#define MPCM0_BIT   		MPCM
	#define UCSZ02_BIT  		UCSZ2
	#define TXB80_BIT   		TXB8

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80

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
	#define MPCM1_BIT   		MPCM1
	#define UCSZ12_BIT  		UCSZ12
	#define TXB81_BIT   		TXB81

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
||defined(__AVR_ATmega324P__)||defined(__AVR_ATmega324A__)||defined(__AVR_ATmega324PA__)\
||defined(__AVR_ATmega324PB__)

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80

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
	#define MPCM1_BIT   		MPCM1
	#define UCSZ12_BIT  		UCSZ12
	#define TXB81_BIT   		TXB81
	
#endif // NO_USART1 && 644

#endif

#if defined(__AVR_ATmega640__)||defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)||defined(__AVR_ATmega324PB__)

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
	#define MPCM2_BIT   		MPCM2
	#define UCSZ22_BIT  		UCSZ22
	#define TXB82_BIT   		TXB82
	
#endif // NO_USART2

#if !defined(NO_USART3)&&!defined(__AVR_ATmega324PB__)
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
	#define MPCM3_BIT   		MPCM3
	#define UCSZ32_BIT  		UCSZ32
	#define TXB83_BIT   		TXB83
	
#endif // NO_USART3
#endif // 640/1280/2560 usart 2 & 3 

#if defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega16U4__)\
||defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega32U6__)\
||defined(__AVR_AT90USB82__)||defined(__AVR_AT90USB162__)

#define USART1_HARDWARE_FLOW_CONTROL_AVAILABLE

#ifndef NO_USART1 // we will call the only usart, as an usart0
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
	#define UCSR1D_REGISTER		UCSR1D
	#define TXCIE1_BIT      	TXCIE1
	#define UDRIE1_BIT    		UDRIE1
	#define RXCIE1_BIT  		RXCIE1
	#define TXEN1_BIT   		TXEN1
	#define RXEN1_BIT   		RXEN1
	#define UDRE1_BIT   		UDRE1
	#define RXC1_BIT    		RXC1
	#define U2X1_BIT    		U2X1
	#define MPCM1_BIT   		MPCM1
	#define UCSZ12_BIT  		UCSZ12
	#define TXB81_BIT   		TXB81
	#define CTSEN1_BIT  		CTSEN
	#define RTSEN1_BIT  		RTSEN

#endif // NO_USART1
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
	#define MPCM0_BIT   		MPCM
	#define UCSZ02_BIT  		UCSZ2
	#define TXB80_BIT   		TXB8

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80

#endif //NO_USART0
#endif

#if defined(__AVR_AT90CAN32__)||defined(__AVR_AT90CAN64__)||defined(__AVR_AT90CAN128__)\
||defined(__AVR_ATmega64RFR2__)||defined(__AVR_ATmega128RFR2__)||defined(__AVR_ATmega256RFR2__)\
||defined(__AVR_ATmega644RFR2__)||defined(__AVR_ATmega1284RFR2__)||defined(__AVR_ATmega2564RFR2__)\
||defined(__AVR_ATmega128RFA1__)

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
	#define MPCM0_BIT   		MPCM0
	#define UCSZ02_BIT  		UCSZ02
	#define TXB80_BIT   		TXB80

#endif // NO_USART0

#ifndef NO_USART1
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
	#define MPCM1_BIT   		MPCM1
	#define UCSZ12_BIT  		UCSZ12
	#define TXB81_BIT   		TXB81
	
#endif // NO_USART1
#endif

#if defined(__AVR_AT90USB646__)||defined(__AVR_AT90USB647__)||defined(__AVR_AT90USB1286__)||defined(__AVR_AT90USB1287__)

#ifndef NO_USART1
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
	#define MPCM1_BIT   		MPCM1
	#define UCSZ12_BIT  		UCSZ12
	#define TXB81_BIT   		TXB81

#endif // NO_USART1
#endif

#if defined(USART_REMAP_LAST_INTERFACE)&&!defined(USE_USART0)&&defined(USE_USART1)&&!defined(USE_USART2)&&!defined(USE_USART3)
	#undef USE_USART1
	#define USE_USART0
	
	#ifdef USART1_HARDWARE_FLOW_CONTROL_AVAILABLE
		#define USART0_HARDWARE_FLOW_CONTROL_AVAILABLE
	
		#define UCSR0D_REGISTER    UCSR1D_REGISTER
		#define CTSEN0_BIT         CTSEN1_BIT
		#define RTSEN0_BIT         RTSEN1_BIT
	#endif
	
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
	#define MPCM0_BIT   		MPCM1_BIT
	#define UCSZ02_BIT  		UCSZ12_BIT
	#define TXB80_BIT   		TXB81_BIT
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
	#define MPCM0_BIT   		MPCM2_BIT
	#define UCSZ02_BIT   		UCSZ22_BIT
	#define TXB80_BIT   		TXB82_BIT
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
	#define MPCM0_BIT   		MPCM3_BIT
	#define UCSZ02_BIT   		UCSZ32_BIT
	#define TXB80_BIT   		TXB83_BIT
#endif

#if !defined(USE_USART0) && !defined(USE_USART1) && !defined(USE_USART2) && !defined(USE_USART3)
	#warning "USART not available or unknown mcu"
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

#if defined(USART0_RS485_MODE)&&!defined(NO_TX0_INTERRUPT)
	#define USART0_USE_TXC_INTERRUPT
#endif

#if defined(USART1_RS485_MODE)&&!defined(NO_TX1_INTERRUPT)
	#define USART1_USE_TXC_INTERRUPT
#endif

#if defined(USART2_RS485_MODE)&&!defined(NO_TX2_INTERRUPT)
	#define USART2_USE_TXC_INTERRUPT
#endif

#if defined(USART3_RS485_MODE)&&!defined(NO_TX3_INTERRUPT)
	#define USART3_USE_TXC_INTERRUPT
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

#ifdef __cplusplus
	extern "C" {
#endif

/************************************************************************************
 *                                   Initializers                                   *
 ************************************************************************************/
	
	// functions smaller then calling overhead (including context saves) or executed once are inline for reducing 
	// flash memory usage (eg. if() statements routines can be executed during compilation)
	
#ifdef USE_USART0
	void uart0_reinit(uint16_t ubrr_value); // for runtime reinitialization of uart
	
	static inline void uart0_init(uint16_t ubrr_value) __attribute__((always_inline));
	static inline void uart0_init(uint16_t ubrr_value) // have to be called once at startup
	{
	#ifdef USART0_RS485_MODE
		RS485_CONTROL0_DDR |= (1<<RS485_CONTROL0_IONUM); // default pin state is low
	#endif
		
		UBRR0L_REGISTER = (uint8_t) ubrr_value;
		
	#ifdef USART_SKIP_UBRRH_IF_ZERO
		if(__builtin_constant_p(ubrr_value))
			if(((ubrr_value>>8) != 0)) // requires -Os flag - do not use in non-inline functions
	#endif
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
	
	#define uart_init(__ubrr) uart0_init(__ubrr)
	#define uart_reinit(__ubrr) uart0_reinit(__ubrr)
	#define uart_set_FrameFormat(__ucsrc) uart0_set_FrameFormat(__ucsrc)
	#define uart_set_U2X() uart0_set_U2X()
	#define uart_mpcm_slave_return_idle() uart0_mpcm_slave_return_idle()
#endif // USE_USART0

#ifdef USE_USART1
	void uart1_reinit(uint16_t ubrr_value); // for runtime reinitialization of uart
	
	static inline void uart1_init(uint16_t ubrr_value) __attribute__((always_inline));
	static inline void uart1_init(uint16_t ubrr_value) // have to be called once at startup
	{
	#ifdef USART1_RS485_MODE
		RS485_CONTROL1_DDR |= (1<<RS485_CONTROL1_IONUM); // default pin state is low
	#endif
		
		UBRR1L_REGISTER = (uint8_t) ubrr_value;
		
	#ifdef USART_SKIP_UBRRH_IF_ZERO
		if(__builtin_constant_p(ubrr_value))
			if(((ubrr_value>>8) != 0)) // requires -Os flag - do not use in non-inline functions
	#endif
			UBRR1H_REGISTER = (ubrr_value>>8);
		
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
		RTS1_DDR |= (1<<RTS1_IONUM);
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

#ifdef USART1_HARDWARE_FLOW_CONTROL_AVAILABLE
	void uart1_hardware_flow_control_init(uint8_t ctsenable, uint8_t rtsenable) __attribute__((always_inline));
	void uart1_hardware_flow_control_init(uint8_t ctsenable, uint8_t rtsenable) // pass true to enable
	{
		if(ctsenable && rtsenable) // -Os dependent, do not use in non-inline functions 
			UCSR1D_REGISTER = (1<<CTSEN1_BIT)|(1<<RTSEN1_BIT);
		else if(ctsenable)
			UCSR1D_REGISTER = (1<<CTSEN1_BIT);
		else
			UCSR1D_REGISTER = (1<<RTSEN1_BIT);
	}
#endif

#endif // USE_USART1

#ifdef USE_USART2
	void uart2_reinit(uint16_t ubrr_value); // for runtime reinitialization of uart
	
	static inline void uart2_init(uint16_t ubrr_value) __attribute__((always_inline));
	static inline void uart2_init(uint16_t ubrr_value) // have to be called once at startup
	{
	#ifdef USART2_RS485_MODE
		RS485_CONTROL2_DDR |= (1<<RS485_CONTROL2_IONUM); // default pin state is low
	#endif
		
		UBRR2L_REGISTER = (uint8_t) ubrr_value;
		
	#ifdef USART_SKIP_UBRRH_IF_ZERO
		if(__builtin_constant_p(ubrr_value))
			if(((ubrr_value>>8) != 0)) // requires -Os flag - do not use in non-inline functions
	#endif
			UBRR2H_REGISTER = (ubrr_value>>8);
		
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
		RTS2_DDR |= (1<<RTS2_IONUM);
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
	void uart3_reinit(uint16_t ubrr_value); // for runtime reinitialization of uart
	
	static inline void uart3_init(uint16_t ubrr_value) __attribute__((always_inline));
	static inline void uart3_init(uint16_t ubrr_value) // have to be called once at startup
	{
	#ifdef USART3_RS485_MODE
		RS485_CONTROL3_DDR |= (1<<RS485_CONTROL3_IONUM); // default pin state is low
	#endif
		
		UBRR3L_REGISTER = (uint8_t) ubrr_value;
		
	#ifdef USART_SKIP_UBRRH_IF_ZERO
		if(__builtin_constant_p(ubrr_value))
			if(((ubrr_value>>8) != 0)) // requires -Os flag - do not use in non-inline functions
	#endif
			UBRR3H_REGISTER = (ubrr_value>>8);
		
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
		RTS3_DDR |= (1<<RTS3_IONUM);
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
		UCSR3A_REGISTER |= (1<<U2X3_BIT);
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
 *                              Transmitter functions                               *
 ************************************************************************************/
#ifndef NO_USART_TX
	
	#ifndef NO_TX0_INTERRUPT
		#ifdef USART_NO_ABI_BREAKING_PREMATURES
			void uart0_putc(char data);
			inline char uart0_putc_(char data) __attribute__ ((always_inline));
			inline char uart0_putc_(char data) { uart0_putc(data); return data; }
		#else
			void uart0_putc(char data) __attribute__ ((naked, noinline));
			char uart0_putc_(char data) __attribute__ ((noinline)); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
		#endif
		
		uint8_t uart0_putc_noblock(char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
		void uart0_putstr(char *string); // send string from the memory buffer // stops when NULL byte is hit (NULL byte is not included into transmission)
		void uart0_putstrl(char *string, uint8_t BytesToWrite); // send specified number of bytes from the pointed buffer (up to 255 bytes)
		
		#ifdef __cplusplus
			#define uart0_puts(str) uart0_putstr((const char*)(str))// macro to avoid const char* conversion restrictions
		#else
			#define uart0_puts(str) uart0_putstr(str)
		#endif
			// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P
	
		#ifdef __cplusplus
			void uart0_puts_p(const char *string); // send string from flash memory
		#else
			void uart0_puts_p(const __flash char *string); // send string from flash memory
		#endif
			#define uart0_puts_P(__strP) uart0_puts_p(PSTR(__strP))
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
		
		extern volatile uint8_t tx0_Head, tx0_Tail;
		inline uint8_t uart0_BytesToSend(void) { return (tx0_Head - tx0_Tail - 1) & TX0_BUFFER_MASK; }
		// returns number of bytes waiting in the transmit buffer
	
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
		#ifdef USART_NO_ABI_BREAKING_PREMATURES
			void uart1_putc(char data);
			inline char uart1_putc_(char data) __attribute__ ((always_inline));
			inline char uart1_putc_(char data) { uart1_putc(data); return data; }
		#else
			void uart1_putc(char data) __attribute__ ((naked, noinline));
			char uart1_putc_(char data) __attribute__ ((noinline)); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
		#endif
		
		uint8_t uart1_putc_noblock(char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
		void uart1_putstr(char *string); // send string from the memory buffer // stops when NULL byte is hit (NULL byte is not included into transmission)
		void uart1_putstrl(char *string, uint8_t BytesToWrite); // send specified number of bytes from the pointed buffer (up to 255 bytes)
		
		#ifdef __cplusplus
			#define uart1_puts(str) uart1_putstr((const char*)(str))// macro to avoid const char* conversion restrictions
		#else
			#define uart1_puts(str) uart1_putstr(str)
		#endif
			// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P

		#ifdef __cplusplus
			void uart1_puts_p(const char *string); // send string from flash memory
		#else	
			void uart1_puts_p(const __flash char *string); // send string from flash memory
		#endif
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
		
		extern volatile uint8_t tx1_Head, tx1_Tail;
		inline uint8_t uart1_BytesToSend(void) { return (tx1_Head - tx1_Tail - 1) & TX1_BUFFER_MASK; }
		// returns number of bytes waiting in the transmit buffer
	
		#ifdef USART1_MPCM_MODE
			void uart1_mpcm_transmit_addres_Frame(uint8_t dat);
		#endif
	#endif // NO_TX1_INTERRUPT
	
	#ifndef NO_TX2_INTERRUPT
		#ifdef USART_NO_ABI_BREAKING_PREMATURES
			void uart2_putc(char data);
			inline char uart2_putc_(char data) __attribute__ ((always_inline));
			inline char uart2_putc_(char data) { uart2_putc(data); return data; }
		#else
			void uart2_putc(char data) __attribute__ ((naked, noinline));
			char uart2_putc_(char data) __attribute__ ((noinline)); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
		#endif
		
		uint8_t uart2_putc_noblock(char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
		void uart2_putstr(char *string); // send string from the memory buffer // stops when NULL byte is hit (NULL byte is not included into transmission)
		void uart2_putstrl(char *string, uint8_t BytesToWrite); // send specified number of bytes from the pointed buffer (up to 255 bytes)
		
		#ifdef __cplusplus
			#define uart2_puts(str) uart2_putstr((const char*)(str)) // macro to avoid const char* conversion restrictions
		#else
			#define uart2_puts(str) uart2_putstr(str)
		#endif
			// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P

		#ifdef __cplusplus
			void uart2_puts_p(const char *string); // send string from flash memory
		#else
			void uart2_puts_p(const __flash char *string); // send string from flash memory
		#endif
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
		
		extern volatile uint8_t tx2_Head, tx2_Tail;
		inline uint8_t uart2_BytesToSend(void) { return (tx2_Head - tx2_Tail - 1) & TX2_BUFFER_MASK; }
		// returns number of bytes waiting in the transmit buffer	
	
		#ifdef USART2_MPCM_MODE
			void uart2_mpcm_transmit_addres_Frame(uint8_t dat);
		#endif
	#endif // NO_TX2_INTERRUPT
	
	#ifndef NO_TX3_INTERRUPT
		#ifdef USART_NO_ABI_BREAKING_PREMATURES
			void uart3_putc(char data);
			inline char uart3_putc_(char data) __attribute__ ((always_inline));
			inline char uart3_putc_(char data) { uart3_putc(data); return data; }
		#else
			void uart3_putc(char data) __attribute__ ((naked, noinline));
			char uart3_putc_(char data) __attribute__ ((noinline)); // alias for uart_putc that returns passed argument unaffected by omitting any existent rule
		#endif
		
		uint8_t uart3_putc_noblock(char data); // returns BUFFER_FULL (false) if buffer is full and character cannot be sent at the moment
	
		void uart3_putstr(char *string); // send string from the memory buffer // stops when NULL byte is hit (NULL byte is not included into transmission)
		void uart3_putstrl(char *string, uint8_t BytesToWrite); // send specified number of bytes from the pointed buffer (up to 255 bytes)
		
		#ifdef __cplusplus
			#define uart3_puts(str) uart3_putstr((const char*)(str))// macro to avoid const char* conversion restrictions
		#else
			#define uart3_puts(str) uart3_putstr(str)
		#endif
			// for deprecated usage only (wastes SRAM data memory to keep all string constants), instead of this try to use puts_P

		#ifdef __cplusplus
			void uart3_puts_p(const char *string); // send string from flash memory
		#else
			void uart3_puts_p(const __flash char *string); // send string from flash memory
		#endif
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
		
		extern volatile uint8_t tx3_Head, tx3_Tail;
		inline uint8_t uart3_BytesToSend(void) { return (tx3_Head - tx3_Tail - 1) & TX3_BUFFER_MASK; }
		// returns number of bytes waiting in the transmit buffer
	
		#ifdef USART3_MPCM_MODE
			void uart3_mpcm_transmit_addres_Frame(uint8_t dat);
		#endif
	#endif // NO_TX0_INTERRUPT

#endif // NO_USART_TX

/************************************************************************************
 *                                Receiver functions                                *
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
	
		inline char uart0_skipWhiteSpaces(void) { char c; do{ c = uart0_getc(); }while(c <= 32); return c; }
		// returns first nonspace character found in the buffer
		
		int16_t uart0_getint(void);
		int32_t uart0_getlong(void);
		float uart0_getfloat(void);
	
		int16_t uart0_getData(void); // reads single byte from a buffer // returns negative value if buffer is empty (upper byte is non zero)
		uint8_t uart0_LoadData(uint8_t *data); // reads single byte from a buffer and loads it into *data byte
		// in case of empty buffers returned flag is set to BUFFER_EMPTY - NULL
		
		extern volatile uint8_t rx0_Head, rx0_Tail;
		inline uint8_t uart0_AvailableBytes(void) { return (rx0_Head - rx0_Tail) & RX0_BUFFER_MASK; }
		// returns number of bytes waiting in the receiver buffer
		
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
		
		inline char uart1_skipWhiteSpaces(void) { char c; do{ c = uart1_getc(); }while(c <= 32); return c; }
		// returns first nonspace character found in the buffer
		
		int16_t uart1_getint(void);
		int32_t uart1_getlong(void);
		float uart1_getfloat(void);
		
		int16_t uart1_getData(void); // reads single byte from a buffer // returns negative value if buffer is empty (upper byte is non zero)
		uint8_t uart1_LoadData(uint8_t *data); // reads single byte from a buffer and loads it into *data byte
		// in case of empty buffers returned flag is set to BUFFER_EMPTY - NULL
		
		extern volatile uint8_t rx1_Head, rx1_Tail;
		inline uint8_t uart1_AvailableBytes(void) { return (rx1_Head - rx1_Tail) & RX1_BUFFER_MASK; }
		// returns number of bytes waiting in the receiver buffer
		
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
		
		inline char uart2_skipWhiteSpaces(void) { char c; do{ c = uart2_getc(); }while(c <= 32); return c; }
		// returns first nonspace character found in the buffer
		
		int16_t uart2_getint(void);
		int32_t uart2_getlong(void);
		float uart2_getfloat(void);
		
		int16_t uart2_getData(void); // reads single byte from a buffer // returns negative value if buffer is empty (upper byte is non zero)
		uint8_t uart2_LoadData(uint8_t *data); // reads single byte from a buffer and loads it into *data byte
		// in case of empty buffers returned flag is set to BUFFER_EMPTY - NULL
		
		extern volatile uint8_t rx2_Head, rx2_Tail;
		inline uint8_t uart2_AvailableBytes(void) { return (rx2_Head - rx2_Tail) & RX2_BUFFER_MASK; }
		// returns number of bytes waiting in the receiver buffer
		
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
		
		inline char uart3_skipWhiteSpaces(void) { char c; do{ c = uart3_getc(); }while(c <= 32); return c; } 
		// returns first nonspace character found in the buffer
		
		int16_t uart3_getint(void);
		int32_t uart3_getlong(void);
		float uart3_getfloat(void);
		
		int16_t uart3_getData(void); // reads single byte from a buffer // returns negative value if buffer is empty (upper byte is non zero)
		uint8_t uart3_LoadData(uint8_t *data); // reads single byte from a buffer and loads it into *data byte
		// in case of empty buffers returned flag is set to BUFFER_EMPTY - NULL
		
		extern volatile uint8_t rx3_Head, rx3_Tail;
		inline uint8_t uart3_AvailableBytes(void) { return (rx3_Head - rx3_Tail) & RX3_BUFFER_MASK; }
		// returns number of bytes waiting in the receiver buffer
		
		uint8_t uart3_peek(void); // returns next byte from buffer // returned byte is invalid if there is nothing to read
	#endif // NO_RX0_INTERRUPT

#endif // NO_USART_RX

/************************************************************************************
 *                       soft flow control interrupt handlers                       *
 ************************************************************************************/

#if defined(USART0_USE_SOFT_CTS)&&!defined(NO_TX0_INTERRUPT)
	extern volatile uint8_t tx0_Tail, tx0_Head;

	static inline void cts0_isr_handler(void) __attribute__((always_inline));
	static inline void cts0_isr_handler(void)
	{
	#if defined(USART0_IN_IO_ADDRESS_SPACE)
		if(CTS0_PIN & (1<<CTS0_IONUM))
		{
			UCSR0B_REGISTER &= ~(1<<UDRIE0_BIT);
		}
		else if (tx0_Tail != tx0_Head)
		{
			UCSR0B_REGISTER |= (1<<UDRIE0_BIT);
		}
	#else
		uint8_t tmp = UCSR0B_REGISTER;
		if(CTS0_PIN & (1<<CTS0_IONUM))
		{
			tmp &= ~(1<<UDRIE0_BIT);
		}
		else if (tx0_Tail != tx0_Head)
		{
			tmp |= (1<<UDRIE0_BIT);
		}
		UCSR0B_REGISTER = tmp;
	#endif
	}
	
	static inline void naked_cts0_isr_handler(void) __attribute__((always_inline));
	static inline void naked_cts0_isr_handler(void)
	{
		asm volatile("\n\t"
		
		#if defined(USART0_IN_IO_ADDRESS_SPACE)
			"cbi	%M[UCSRB_reg_IO], %M[UDRIE_bit] \n\t"
			"sbic	%M[cts_port], %M[cts_pin] \n\t"
			"reti \n\t"
			
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			
			"lds	%B[z_save], (tx0_Tail) \n\t"
			"lds	%A[z_save], (tx0_Head) \n\t"
			"cpse	%B[z_save], %A[z_save] \n\t"
			"sbi	%M[UCSRB_reg_IO], %M[UDRIE_bit] \n\t"
			
		#elif defined(USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE)
			
			"push	r25 \n\t"
			
			"lds	r25, (tx0_Tail) \n\t"
			"lds	%[sreg_save], (tx0_Head) \n\t"
			"cpse	r25, %[sreg_save] \n\t"
			"sbi	%M[UCSRB_reg_IO], %M[UDRIE_bit] \n\t"
			
			"pop	r25 \n\t"
		#else
			"push	r24 \n\t"
			"push	r25 \n\t"
		
			"lds	r25, (tx0_Tail) \n\t"
			"lds	r24, (tx0_Head) \n\t"
			"cpse	r25, r24 \n\t"
			"sbi	%M[UCSRB_reg_IO], %M[UDRIE_bit] \n\t"
		
			"pop	r25 \n\t"
			"pop	r24 \n\t"
		#endif

			"reti \n\t"
			
		#else // USART not in IO
			
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"in		%[sreg_save], __SREG__ \n\t"
		#else
			"push	r16 \n\t"
			"in		r16,__SREG__ \n\t"
		#endif
		
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			#ifdef __AVR_HAVE_MOVW__
				"movw	%[z_save], r24 \n\t"
			#else // in this case only 4 cycles are prematured out
				"mov	%A[z_save], r24 \n\t"
				"mov	%B[z_save], r25 \n\t"
			#endif
		#else
			"push	r24 \n\t"
		#endif
		
		#ifdef USART0_IN_UPPER_IO_ADDRESS_SPACE
			"in 	r24, %M[UCSRB_reg_IO] \n\t"
		#else
			"lds	r24, %M[UCSRB_reg] \n\t"
		#endif
			
			"andi	r24, ~(1<<%M[UDRIE_bit]) \n\t"
			"sbic	%M[cts_port], %M[cts_pin]\n\t"
			"rjmp	cts_apply_%= \n\t"
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"push	r25 \n\t"
		#endif
			"lds	r24, (tx0_Head) \n\t"
			"lds	r25, (tx0_Tail) \n\t"
			"cp 	r25, r24 \n\t"
			
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r25 \n\t" // branch after poping
		#endif
			"breq	cts_exit_%= \n\t" // UDRIE should be disabled in this case - no need to clear it
		
		#ifdef USART0_IN_UPPER_IO_ADDRESS_SPACE
			"in 	r24, %M[UCSRB_reg_IO] \n\t"
		#else
			"lds	r24, %M[UCSRB_reg] \n\t"
		#endif
			"ori	r24, (1<<%M[UDRIE_bit]) \n\t"
			
		"cts_apply_%=:"	
		#ifdef USART0_IN_UPPER_IO_ADDRESS_SPACE
			"out	%M[UCSRB_reg_IO], r24 \n\t"
		#else
			"sts	%M[UCSRB_reg], r24 \n\t"
		#endif
		
		"cts_exit_%=:"
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			#ifdef __AVR_HAVE_MOVW__
				"movw	r24, %[z_save] \n\t"
			#else // in this case only 4 cycles are prematured out
				"mov	r25, %B[z_save] \n\t"
				"mov	r24, %A[z_save] \n\t"
			#endif
		#else
			"pop	r24 \n\t"
		#endif
		
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, %[sreg_save] \n\t"
		#else
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#endif
		
			"reti \n\t"
		#endif
			: /* output operands */
			USART_REG_SAVE_LIST
			: /* input operands */
			[UCSRB_reg]      "n" (_SFR_MEM_ADDR(UCSR0B_REGISTER)),
			[UCSRB_reg_IO]   "M" (_SFR_IO_ADDR(UCSR0B_REGISTER)),
			[UDRIE_bit]      "M" (UDRIE0_BIT),
			[cts_port]       "M" (_SFR_IO_ADDR(CTS0_PIN)),
			[cts_pin]        "M" (CTS0_IONUM)
			: /* clobbers */
		);
	}
#endif

#if defined(USART1_USE_SOFT_CTS)&&!defined(NO_TX1_INTERRUPT)
	extern volatile uint8_t tx1_Tail, tx1_Head;

	static inline void cts1_isr_handler(void) __attribute__((always_inline));
	static inline void cts1_isr_handler(void)
	{
	#if defined(USART1_IN_IO_ADDRESS_SPACE)
		if(CTS1_PIN & (1<<CTS1_IONUM))
		{
			UCSR1B_REGISTER &= ~(1<<UDRIE1_BIT);
		}
		else if (tx1_Tail != tx1_Head)
		{
			UCSR1B_REGISTER |= (1<<UDRIE1_BIT);
		}
	#else
		uint8_t tmp = UCSR1B_REGISTER;
		if(CTS1_PIN & (1<<CTS1_IONUM))
		{
			tmp &= ~(1<<UDRIE1_BIT);
		}
		else if (tx1_Tail != tx1_Head)
		{
			tmp |= (1<<UDRIE1_BIT);
		}
		UCSR1B_REGISTER = tmp;
	#endif
	}
	
	static inline void naked_cts1_isr_handler(void) __attribute__((always_inline));
	static inline void naked_cts1_isr_handler(void)
	{
		asm volatile("\n\t"
		
		#if defined(USART1_IN_IO_ADDRESS_SPACE)
			"cbi	%M[UCSRB_reg_IO], %M[UDRIE_bit] \n\t"
			"sbic	%M[cts_port], %M[cts_pin] \n\t"
			"reti \n\t"
		
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			
			"lds	%B[z_save], (tx1_Tail) \n\t"
			"lds	%A[z_save], (tx1_Head) \n\t"
			"cpse	%B[z_save], %A[z_save] \n\t"
			"sbi	%M[UCSRB_reg_IO], %M[UDRIE_bit] \n\t"
			
		#elif defined(USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE)
			
			"push	r25 \n\t"
			
			"lds	r25, (tx1_Tail) \n\t"
			"lds	%[sreg_save], (tx1_Head) \n\t"
			"cpse	r25, %[sreg_save] \n\t"
			"sbi	%M[UCSRB_reg_IO], %M[UDRIE_bit] \n\t"
			
			"pop	r25 \n\t"
		#else
			"push	r24 \n\t"
			"push	r25 \n\t"
		
			"lds	r25, (tx1_Tail) \n\t"
			"lds	r24, (tx1_Head) \n\t"
			"cpse	r25, r24 \n\t"
			"sbi	%M[UCSRB_reg_IO], %M[UDRIE_bit] \n\t"
		
			"pop	r25 \n\t"
			"pop	r24 \n\t"
		#endif
		
			"reti \n\t"
		
		#else // USART not in IO
		
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"in		%[sreg_save], __SREG__ \n\t"
		#else
			"push	r16 \n\t"
			"in		r16,__SREG__ \n\t"
		#endif
			
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"movw	%[z_save], r24 \n\t"
		#else
			"push	r24 \n\t"
		#endif
			
			"lds	r24, %M[UCSRB_reg] \n\t"
			"andi	r24, ~(1<<%M[UDRIE_bit]) \n\t"
			"sbic	%M[cts_port], %M[cts_pin]\n\t"
			"rjmp	cts_apply_%= \n\t"
			
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"push	r25 \n\t"
		#endif
			"lds	r24, (tx1_Head) \n\t"
			"lds	r25, (tx1_Tail) \n\t"
			"cp 	r25, r24 \n\t"
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r25 \n\t"
		#endif
			"breq	cts_exit_%= \n\t" // UDRIE should be disabled in this case - no need to clear it
			
			"lds	r24, %M[UCSRB_reg] \n\t"
			"ori	r24, (1<<%M[UDRIE_bit]) \n\t"
		"cts_apply_%=:"	
			"sts	%M[UCSRB_reg], r24 \n\t"
		
		"cts_exit_%=:"
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE	
			"movw	r24, %[z_save] \n\t"
		#else
			"pop	r24 \n\t"
		#endif
			
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, %[sreg_save] \n\t"
		#else
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#endif

			"reti \n\t"
		#endif
			: /* output operands */
			USART_REG_SAVE_LIST
			: /* input operands */
			[UCSRB_reg]      "n" (_SFR_MEM_ADDR(UCSR1B_REGISTER)),
			[UCSRB_reg_IO]   "M" (_SFR_IO_ADDR(UCSR1B_REGISTER)),
			[UDRIE_bit]      "M" (UDRIE1_BIT),
			[cts_port]       "M" (_SFR_IO_ADDR(CTS1_PIN)),
			[cts_pin]        "M" (CTS1_IONUM)
			: /* clobbers */
		);
	}
#endif

#if defined(USART2_USE_SOFT_CTS)&&!defined(NO_TX2_INTERRUPT)
	extern volatile uint8_t tx2_Tail, tx2_Head;

	static inline void cts2_isr_handler(void) __attribute__((always_inline));
	static inline void cts2_isr_handler(void)
	{
		uint8_t tmp = UCSR2B_REGISTER;
		if(CTS2_PIN & (1<<CTS2_IONUM))
		{
			tmp &= ~(1<<UDRIE2_BIT);
		}
		else if (tx2_Tail != tx2_Head)
		{
			tmp |= (1<<UDRIE2_BIT);
		}
		UCSR2B_REGISTER = tmp;
	}
	
	static inline void naked_cts2_isr_handler(void) __attribute__((always_inline));
	static inline void naked_cts2_isr_handler(void)
	{
		asm volatile("\n\t"
			
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"in		%[sreg_save], __SREG__ \n\t"
		#else
			"push	r16 \n\t"
			"in		r16,__SREG__ \n\t"
		#endif
		
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"movw	%[z_save], r24 \n\t"
		#else
			"push	r24 \n\t"
		#endif
		
			"lds	r24, %M[UCSRB_reg] \n\t"
			"andi	r24, ~(1<<%M[UDRIE_bit]) \n\t"
			"sbic	%M[cts_port], %M[cts_pin]\n\t"
			"rjmp	cts_apply_%= \n\t"
			
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"push	r25 \n\t"
		#endif
			"lds	r24, (tx2_Head) \n\t"
			"lds	r25, (tx2_Tail) \n\t"
			"cp 	r25, r24 \n\t"
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r25 \n\t"
		#endif
			"breq	cts_exit_%= \n\t" // UDRIE should be disabled in this case - no need to clear it
			
			"lds	r24, %M[UCSRB_reg] \n\t"
			"ori	r24, (1<<%M[UDRIE_bit]) \n\t"
		"cts_apply_%=:"	
			"sts	%M[UCSRB_reg], r24 \n\t"
		
		"cts_exit_%=:"
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE	
			"movw	r24, %[z_save] \n\t"
		#else
			"pop	r24 \n\t"
		#endif
		
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, %[sreg_save] \n\t"
		#else
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#endif

			"reti \n\t"
			: /* output operands */
			USART_REG_SAVE_LIST
			: /* input operands */
			[UCSRB_reg]      "n" (_SFR_MEM_ADDR(UCSR2B_REGISTER)),
			[UDRIE_bit]      "M" (UDRIE2_BIT),
			[cts_port]       "M" (_SFR_IO_ADDR(CTS2_PIN)),
			[cts_pin]        "M" (CTS2_IONUM)
			: /* clobbers */
		);
	}
#endif

#if defined(USART3_USE_SOFT_CTS)&&!defined(NO_TX3_INTERRUPT)
	extern volatile uint8_t tx3_Tail, tx3_Head;

	static inline void cts3_isr_handler(void) __attribute__((always_inline));
	static inline void cts3_isr_handler(void)
	{
		uint8_t tmp = UCSR3B_REGISTER;
		if(CTS3_PIN & (1<<CTS3_IONUM))
		{
			tmp &= ~(1<<UDRIE3_BIT);
		}
		else if (tx3_Tail != tx3_Head)
		{
			tmp |= (1<<UDRIE3_BIT);
		}
		UCSR3B_REGISTER = tmp;
	}
	
	static inline void naked_cts3_isr_handler(void) __attribute__((always_inline));
	static inline void naked_cts3_isr_handler(void)
	{
		asm volatile("\n\t"
			
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"in		%[sreg_save], __SREG__ \n\t"
		#else
			"push	r16 \n\t"
			"in		r16,__SREG__ \n\t"
		#endif
		
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"movw	%[z_save], r24 \n\t"
		#else
			"push	r24 \n\t"
		#endif
		
			"lds	r24, %M[UCSRB_reg] \n\t"
			"andi	r24, ~(1<<%M[UDRIE_bit]) \n\t"
			"sbic	%M[cts_port], %M[cts_pin]\n\t"
			"rjmp	cts_apply_%= \n\t"
			
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"push	r25 \n\t"
		#endif
			"lds	r24, (tx3_Head) \n\t"
			"lds	r25, (tx3_Tail) \n\t"
			"cp 	r25, r24 \n\t"
		
		#ifndef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE
			"pop	r25 \n\t"
		#endif
			"breq	cts_exit_%= \n\t" // UDRIE should be disabled in this case - no need to clear it
			
			"lds	r24, %M[UCSRB_reg] \n\t"
			"ori	r24, (1<<%M[UDRIE_bit]) \n\t"
		"cts_apply_%=:"	
			"sts	%M[UCSRB_reg], r24 \n\t"
		
		"cts_exit_%=:"
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE	
			"movw	r24, %[z_save] \n\t"
		#else
			"pop	r24 \n\t"
		#endif
		
		#ifdef USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE
			"out	__SREG__, %[sreg_save] \n\t"
		#else
			"out	__SREG__, r16 \n\t"
			"pop	r16 \n\t"
		#endif

			"reti \n\t"
			: /* output operands */
			USART_REG_SAVE_LIST
			: /* input operands */
			[UCSRB_reg]      "n" (_SFR_MEM_ADDR(UCSR3B_REGISTER)),
			[UDRIE_bit]      "M" (UDRIE3_BIT),
			[cts_port]       "M" (_SFR_IO_ADDR(CTS3_PIN)),
			[cts_pin]        "M" (CTS3_IONUM)
			: /* clobbers */
		);
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
		int uart_putchar(char data, FILE *stream);
	#endif

	#ifndef NO_USART_RX
		int uart_getchar(FILE *stream);
	#endif
	
	#ifdef USE_USART0
	
		#if defined(NO_RX0_INTERRUPT)
			extern FILE uart0_out;
		
		#elif defined(NO_TX0_INTERRUPT)
			extern FILE uart0_in;
		#else
			extern FILE uart0_io;
			extern FILE uart0_in;
			extern FILE uart0_out;
		#endif
		
	#endif // USE_USART0
	
	#ifdef USE_USART1
	
		#if defined(NO_RX1_INTERRUPT)
			extern FILE uart1_out;
		
		#elif defined(NO_TX1_INTERRUPT)
			extern FILE uart1_in;
		#else
			extern FILE uart1_io;
			extern FILE uart1_in;
			extern FILE uart1_out;
		#endif
		
	#endif // USE_USART1
	
	#ifdef USE_USART2
	
		#if defined(NO_RX2_INTERRUPT)
			extern FILE uart2_out;
		
		#elif defined(NO_TX2_INTERRUPT)
			extern FILE uart2_in;
		#else
			extern FILE uart2_io;
			extern FILE uart2_in;
			extern FILE uart2_out;
		#endif
		
	#endif // USE_USART2
		
	#ifdef USE_USART3
		
		#if defined(NO_RX3_INTERRUPT)
			extern FILE uart3_out;
		
		#elif defined(NO_TX3_INTERRUPT)
			extern FILE uart3_in;
		#else
			extern FILE uart3_io;
			extern FILE uart3_in;
			extern FILE uart3_out;
		#endif
		
	#endif // USE_USART3
	
#else // single USART mcu

	#ifndef NO_TX0_INTERRUPT
		int uart_putchar(char data, FILE *stream);
	#endif

	#ifndef NO_RX0_INTERRUPT
		int uart_getchar(FILE *stream);
	#endif
	
	#if defined(NO_RX0_INTERRUPT)
		extern FILE uart0_out;
	
	#elif defined(NO_TX0_INTERRUPT)
		extern FILE uart0_in;
	#else
		extern FILE uart0_io;
		extern FILE uart0_in;
		extern FILE uart0_out;
	#endif
	
#endif // single/multi USART

#ifdef __cplusplus
	}
#endif

#endif // _USART_H_
