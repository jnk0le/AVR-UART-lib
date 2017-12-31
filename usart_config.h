#ifndef USART_CONFIG_H_
#define USART_CONFIG_H_

/************************************************************************************
 *  Author: jnk0le@hotmail.com                                                      *
 *  https://github.com/jnk0le                                                       *
 *  This library is distributed under MIT license terms                             *
 ************************************************************************************/

// DO NOT DEFINE F_CPU, BUFFERS SIZES OR ANY OTHER SHARED MACROS IN 'main.c' CODE
// instead of this, define it in makefile (-D flag) or "Project Properties -> AVR C Compiler -> Symbols"

//#define NO_USART_RX // disable all receiver code and dependencies
//#define NO_USART_TX // disable all transmitter code and dependencies

//#define USART_MPCM_MODE // globally enable MPCM operation mode // 9 bit data frame only // always set frame format to 8 data bits

//#define USE_DOUBLE_SPEED // enables double speed for all available USART interfaces

#define RX_STDIO_GETCHAR_ECHO // echoes back received characters in getchar() function (for reading in scanf())
//#define RX_GETC_ECHO // echoes back received characters in getc() function

//#define PUTC_CONVERT_LF_TO_CRLF // allow for unix style (\n only) newline terminator in stored strings // not included into putc_noblock
#define RX_NEWLINE_MODE 2 // 0 - \r,  1 - \n,  2 - \r\n
// lot of terminals sends only \r character as a newline terminator, instead of \r\n or even unix style \n
// (BTW PuTTY doesn't allow to change this) but in return requires \r\n terminator to show not broken text

//#define USART_NO_ABI_BREAKING_PREMATURES // do not use prematures that might break compilers ABI (non-gcc calling conventions), compilers that are not forcing constant number of call-used registers might generate even better code
//#define USART_PUTHEX_IN_UPPERCASE // use uppercase letters in uart_puthex() function
//#define USART_EXTEND_RX_BUFFER // extend RX buffer by hardware 2/3 byte FIFO // required for hardware and software RTS
//#define USART_PUTC_FAST_INSERTIONS // skip FIFO procedure and write directly data to the UDR register when possible // probably required for full bus utilization at highest speed (f_cpu/8)
//#define USART_NO_LOCAL_BUFFERS // do not allocate temporary buffers on stack for integer/float <-> asci conversions and use globally visible u_tmp_buff[] instead // it have to be declared in application part and have to be at least of 6-17 bytes wide (depending on what is being converted)
//#define USART_UNSAFE_TX_INTERRUPT // max 19 cycles of interrupt latency // 3+PC bytes on stack // will not interrupt itself
//#define USART_UNSAFE_RX_INTERRUPT // max 23 cycles of interrupt latency // 4+PC bytes on stack // will not interrupt itself
//#define USART_REMAP_LAST_INTERFACE // remap hardware registers of USART1/2/3 to USART0 if only one interface is used
//#define USART_SKIP_UBRRH_IF_ZERO // do not generate code for writing to ubrrh if calculated value is zero // prematures out 2 bytes if ubrr is compile time constant

//#define USART_USE_GLOBALLY_RESERVED_ISR_SREG_SAVE // prematures out 4 cycles from every isr run // requires one globally reserved lower register
//#define USART_USE_GLOBALLY_RESERVED_ISR_Z_SAVE    // prematures out 6 cycles from every isr run // requires pair of globally reserved lower registers
// usage of globally reserved register for temporary storage in interrupts, should be combined with other interrupts for best results. 
// special care have to be taken when doing so, since those registers can still be used by other compilation units (fixable in gcc by -ffixed-n flag, where n is a suppressed register),
// precompiled libraries (vprintf, vscanf, qsort, strtod, strtol, strtoul), or even assembly hardcoded libraries (fft, aes).
// registers r2-r7 should be used instead of the higher ones, since those are never used by gcc for eg. argument passing.

	#define USART_SREG_SAVE_REG_NAME G_sreg_save // ??? // have to be redeclared under the same name if the same registers are reused in other instances (libs)
	#define USART_SREG_SAVE_REG_NUM "r4"
	
	#define USART_Z_SAVE_REG_NAME G_z_save // ??? // have to be redeclared under the same name if the same registers are reused in other instances (libs)
	#define USART_Z_SAVE_REG_NUM "r2" // register pair rn and rn+1 (rn+1:rn gives "invalid register name")

//#define RX_BUFFER_SIZE 128 // Size of the ring buffers, must be power of 2 // default 32
//#define TX_BUFFER_SIZE 64 // Size of the ring buffers, must be power of 2 // default 32

/*****************************config for multiple USART mcu's***********************************/

//#define NO_USART0 // disable usage of uart0
//#define NO_USART1 // disable usage of uart1
//#define NO_USART2 // disable usage of uart2
//#define NO_USART3 // disable usage of uart3

//#define RX0_BUFFER_SIZE 128
//#define TX0_BUFFER_SIZE 64

//#define RX1_BUFFER_SIZE 128
//#define TX1_BUFFER_SIZE 64

//#define RX2_BUFFER_SIZE 128
//#define TX2_BUFFER_SIZE 64

//#define RX3_BUFFER_SIZE 128
//#define TX3_BUFFER_SIZE 64

//#define NO_RX0_INTERRUPT // removes whole receive code (including ISR) and frees RX0 pin // combining with NO_USART_RX is not necessary
//#define NO_RX1_INTERRUPT // removes whole receive code (including ISR) and frees RX1 pin
//#define NO_RX2_INTERRUPT // removes whole receive code (including ISR) and frees RX2 pin
//#define NO_RX3_INTERRUPT // removes whole receive code (including ISR) and frees RX3 pin

//#define NO_TX0_INTERRUPT // removes whole transmit code (including ISR) and frees TX0 pin // combining with NO_USART_TX is not necessary
//#define NO_TX1_INTERRUPT // removes whole transmit code (including ISR) and frees TX1 pin
//#define NO_TX2_INTERRUPT // removes whole transmit code (including ISR) and frees TX2 pin
//#define NO_TX3_INTERRUPT // removes whole transmit code (including ISR) and frees TX3 pin

//#define USART0_U2X_SPEED // enables double speed for USART0
//#define USART1_U2X_SPEED // enables double speed for USART1
//#define USART2_U2X_SPEED // enables double speed for USART2
//#define USART3_U2X_SPEED // enables double speed for USART3

//#define RX0_GETC_ECHO
//#define RX1_GETC_ECHO
//#define RX2_GETC_ECHO
//#define RX3_GETC_ECHO

//#define PUTC0_CONVERT_LF_TO_CRLF
//#define PUTC1_CONVERT_LF_TO_CRLF
//#define PUTC2_CONVERT_LF_TO_CRLF
//#define PUTC3_CONVERT_LF_TO_CRLF

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

/*****************************soft flow control config***********************************/
// define IO instance to enable software CTS  
// CTS handlers also have to be placed into INT/PCINT interrupt in the application code, see example(flow control).c

//#define CTS0_DDR // DDRB
//#define CTS0_PORT // PORTB
//#define CTS0_PIN // PINB
//#define CTS0_IONUM // 0 // pin number

//#define CTS1_DDR
//#define CTS1_PORT
//#define CTS1_PIN
//#define CTS1_IONUM

//#define CTS2_DDR
//#define CTS2_PORT
//#define CTS2_PIN
//#define CTS2_IONUM

//#define CTS3_DDR
//#define CTS3_PORT
//#define CTS3_PIN
//#define CTS3_IONUM

// define IO instance to enable software RTS

//#define RTS0_DDR // DDRB
//#define RTS0_PORT // PORTB
//#define RTS0_PIN // PINB
//#define RTS0_IONUM // 1 // pin number

//#define RTS1_DDR
//#define RTS1_PORT
//#define RTS1_PIN
//#define RTS1_IONUM

//#define RTS2_DDR
//#define RTS2_PORT
//#define RTS2_PIN
//#define RTS2_IONUM

//#define RTS3_DDR
//#define RTS3_PORT
//#define RTS3_PIN
//#define RTS3_IONUM

/*****************************RS 485 config***********************************/
// define IO instance to enable half duplex rs485 operation mode // used pin should be initially kept in low state before boot

//#define RS485_CONTROL0_DDR // DDRB
//#define RS485_CONTROL0_PORT // PORTB
//#define RS485_CONTROL0_PIN // PINB
//#define RS485_CONTROL0_IONUM // 2 // pin number

//#define RS485_CONTROL1_DDR
//#define RS485_CONTROL1_PORT
//#define RS485_CONTROL1_PIN
//#define RS485_CONTROL1_IONUM

//#define RS485_CONTROL2_DDR
//#define RS485_CONTROL2_PORT
//#define RS485_CONTROL2_PIN
//#define RS485_CONTROL2_IONUM

//#define RS485_CONTROL3_DDR
//#define RS485_CONTROL3_PORT
//#define RS485_CONTROL3_PIN
//#define RS485_CONTROL3_IONUM

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

// Optional macros for placing user-defined code that will be executed inside of USART interrupt handlers.
// Only inline asm and its input operand lists are allowed to be put here.
// Too large code may generate weird cryptic linker errors, what is caused by exceeded range of branch instructions.
// http://www.nongnu.org/avr-libc/user-manual/inline_asm.html

// example:
//#define STH_EVENT "nop \n\t"\                              //
//                  "ldi r31, %M[A_MASK] \n\t"\              //
//                  "out %M[TIMADDR], r31 \n\t"
//#define OPERAND_LIST [A_MASK] "M" (0x55),\                 //
//                     [TIMADDR]  "M" (_SFR_IO_ADDR(TCCR0A)),

// code executed on every ISR call, before feeding UDR (for this racing implementation only), can be placed here // r30 and r31 are free to use
#define TX0_EVERYCAL_EVENT "\n\t"

// code executed on every byte transmission, can be placed here // r30 and r31 are free to use // r30 contains currently transmitted data byte
#define TX0_TRANSMIT_EVENT "\n\t"

#define TX0_INPUT_OPERAND_LIST 

#if !defined(USART0_EXTEND_RX_BUFFER) // DO NOT CHANGE
	// code executed before reading UDR register can be placed here // r25 is free to use // executed before enabling interrupts in unsafe mode
	#define RX0_FRAMING_EVENT "\n\t"
	
	//#define USART0_PUSH_BEFORE_RX // frees r30 an r31 for FRAMING_EVENT
#else
	// code executed before reading UDR register can be placed here // r25 and r31 are free to use
	#define RX0_FRAMING_EVENT "\n\t"
#endif

// code executed on every ISR call, can be placed here // r30 and r31 are free to use // r25 contains received data byte if 'extended buffer' mode is not used, free to use otherwise
#define RX0_EVERYCALL_EVENT "\n\t"

// code executed only when databyte was received, before buffer store, can be placed here // r31 is free to use // r25 contains received data byte, r30 rxn_last_byte buffer index // MPCM ??
#define RX0_EARLY_RECEIVE_EVENT "\n\t"

// code executed only when databyte was received, can be placed here // r25,r30,r31 are free to use // r25 contains received data byte
#define RX0_LATE_RECEIVE_EVENT "\n\t"

#define RX0_INPUT_OPERAND_LIST

//************************************************

#define TX1_EVERYCAL_EVENT "\n\t"
#define TX1_TRANSMIT_EVENT "\n\t"

#define TX1_INPUT_OPERAND_LIST

#if !defined(USART1_EXTEND_RX_BUFFER) // DO NOT CHANGE
	#define RX1_FRAMING_EVENT "\n\t"
	//#define USART1_PUSH_BEFORE_RX
#else
	#define RX1_FRAMING_EVENT "\n\t"
#endif

#define RX1_EVERYCALL_EVENT "\n\t"
#define RX1_EARLY_RECEIVE_EVENT "\n\t"
#define RX1_LATE_RECEIVE_EVENT "\n\t"

#define RX1_INPUT_OPERAND_LIST

//************************************************

#define TX2_EVERYCAL_EVENT "\n\t"
#define TX2_TRANSMIT_EVENT "\n\t"

#define TX2_INPUT_OPERAND_LIST

#if !defined(USART2_EXTEND_RX_BUFFER) // DO NOT CHANGE
	#define RX2_FRAMING_EVENT "\n\t"
	//#define USART2_PUSH_BEFORE_RX
#else
	#define RX2_FRAMING_EVENT "\n\t"
#endif

#define RX2_EVERYCALL_EVENT "\n\t"
#define RX2_EARLY_RECEIVE_EVENT "\n\t"
#define RX2_LATE_RECEIVE_EVENT "\n\t"

#define RX2_INPUT_OPERAND_LIST

//************************************************

#define TX3_EVERYCAL_EVENT "\n\t"
#define TX3_TRANSMIT_EVENT "\n\t"

#define TX3_INPUT_OPERAND_LIST

#if !defined(USART3_EXTEND_RX_BUFFER) // DO NOT CHANGE
	#define RX3_FRAMING_EVENT "\n\t"
	//#define USART3_PUSH_BEFORE_RX
#else
	#define RX3_FRAMING_EVENT "\n\t"
#endif

#define RX3_EVERYCALL_EVENT "\n\t"
#define RX3_EARLY_RECEIVE_EVENT "\n\t"
#define RX3_LATE_RECEIVE_EVENT "\n\t"

#define RX3_INPUT_OPERAND_LIST

// events executed inside transmit complete interrupts (last byte has been transmitted, UDR buffer is empty)
// if USATRn_NO_NAKED_TXC_INTERRUPT is not defined then inline asm is required here // any modified regs have to be pushed first
//inline void TXCn_interrupt_event(void)
//{
//	asm volatile("\n\t"
//		"nop \n\t"
//		"lpm \n\t"
//		"st Z+, r0 \n\t"
//	:: );
//}
// mpcm ??

//#define USATR0_NO_NAKED_TXC_INTERRUPT
//#define USART0_USE_TXC_INTERRUPT // if rs485 is not used

inline void TXC0_interrupt_event(void) __attribute__((always_inline));
inline void TXC0_interrupt_event(void)
{
}

//#define USATR1_NO_NAKED_TXC_INTERRUPT 
//#define USART1_USE_TXC_INTERRUPT

inline void TXC1_interrupt_event(void) __attribute__((always_inline));
inline void TXC1_interrupt_event(void)
{
}

//#define USATR2_NO_NAKED_TXC_INTERRUPT 
//#define USART2_USE_TXC_INTERRUPT

inline void TXC2_interrupt_event(void) __attribute__((always_inline));
inline void TXC2_interrupt_event(void)
{
}

//#define USATR3_NO_NAKED_TXC_INTERRUPT 
//#define USART3_USE_TXC_INTERRUPT

inline void TXC3_interrupt_event(void) __attribute__((always_inline));
inline void TXC3_interrupt_event(void)
{
}

#endif /* USART_CONFIG_H_ */