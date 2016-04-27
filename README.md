# Easy-AVR-USART-C-Library
An interrupt-driven USART (RS232) library for AVR microcontrollers, with support for multiple UARTs, using ring
buffers for receive/transmit. Designed to be easy to use, especially like arduino libraries (with less overload).

# Features
- easy to use
- intuitive frame format settings
- support for up to 4 USART's
- extremly light interrupts (43 cycles for tx/rx in worst case, +2 on >256k and -1 if uart is placed in IO address space)
- RS 485 compatibility
- MPCM master and slave mode support
- printf()/scanf() streams compatibility
- V-USB compatibility (25 or 33 cycle ISR restriction)
- optimized as much as possible to reduce code size
- and much more

simple "hello world" code on mega328p can give:

	Program Memory Usage 	:	402 bytes   1,2 % Full
	Data Memory Usage 		:	82 bytes   4,0 % Full

same code on mega8 gives:

	Program Memory Usage 	:	312 bytes   3,8 % Full
	Data Memory Usage 		:	82 bytes   8,0 % Full

Meanwhile Arduino generates 2KB of code.

For this result additional flag -mrelax is required in many IDE's (Atmel studio 7, Arduino etc.)

# Notes
Lot of terminals sends only CR character as a newline terminator, instead of CRLF or even unix style LF
(BTW PuTTY doesn't allow to change this) but in return requires CRLF terminator to show not broken text.
This behaviour can be covered by RX_NEWLINE_MODE macro, by default set to CRLF.

- 0 - CR
- 1 - LF
- 2 - CRLF (default)

In case of reinitializing uart on the fly (especially with non-constant ubbr) try to use uart_reint() or define USART_NO_DIRTY_HACKS macro.