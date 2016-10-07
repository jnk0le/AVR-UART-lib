//**************************************************************
// ******  *******
//**************************************************************
//Compiler          : AVR-GCC
//Author            : jnk0le@hotmail.com
//                    https://github.com/jnk0le
//Created           : 30.09.2016
//License           : MIT
//**************************************************************

#ifndef USART_EVENTS_H_
#define USART_EVENTS_H_

// This is an optional file for placing user-defined code that will be executed inside of USART interrupt handlers.
// Only inline asm and its input operand lists are allowed to be put here.
// Too large code may generate weird cryptic linker errors, what is caused by exceeded range of branch instructions. 
// http://www.nongnu.org/avr-libc/user-manual/inline_asm.html

// example:
//#define STH_EVENT "nop \n\t"\
//                  "ldi r31, %M[A_MASK]"\
//                  "out %M[TIMADDR], r31"
	
//#define OPERAND_LIST [A_MASK] "M" (0x55),\
//                     [TIMADDR]  "M" (_SFR_IO_ADDR(TCCR0A)),

// TODO: TXC

// code executed on every ISR call, before feeding UDR can (for this racing implementation only), can be placed here // r30 and r31 are free to use
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

// code executed only when databyte was received, before buffer store, can be placed here // r31 is free to use // r25 contains received data byte, r30 rxn_last_byte buffer index // MPCM
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

#endif /* USART_EVENTS_H_ */