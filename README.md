# Easy-AVR-USART-C-Library
An interrupt-driven USART (RS232) library for AVR microcontrollers, with support for multiple UARTs, using ring
buffers for receive/transmit. Designed to be easy to use, with absolutely minimum "feature-overhead".

# Features
- easy to use
- intuitive frame format settings
- support for up to 4 USART's
- extremly light interrupts 
- RS 485 compatibility
- MPCM master and slave mode support
- printf()/scanf() streams compatibility
- V-USB compatibility (25 or 33 cycle ISR restriction)
- optimized as much as possible to reduce code size
- and much more

simple "hello world" code on mega328p can give:

	Program Memory Usage 	:	398 bytes   1,2 % Full
	Data Memory Usage 		:	82 bytes   4,0 % Full

same code on mega8 gives:

	Program Memory Usage 	:	308 bytes   3,8 % Full
	Data Memory Usage 		:	82 bytes   8,0 % Full

Meanwhile Arduino generates 2KB of code.

For this result additional flag -mrelax is required in many IDE's (Atmel studio, Arduino etc.)

# Notes
Lot of terminals sends only CR character as a newline terminator, instead of CRLF or even unix style LF
(BTW PuTTY doesn't allow to change this) but in return requires CRLF terminator to show not broken text.
This behaviour can be covered by RX_NEWLINE_MODE macro, by default set to CRLF.

- 0 - CR
- 1 - LF
- 2 - CRLF (default)

In case of reinitializing uart on the fly (especially with non-constant ubbr) try to use uart_reint() or define USART_NO_DIRTY_HACKS macro.

# ISR timmings

- TX best case - send byte from buffer: 39 cycles (38 if uart is placed in IO address space)
- TX worst case - send byte from buffer and disable UDRIE interrupt: 47 cycles (41 if uart is placed in IO address space) 

- RX best case - load byte and do nothing: 37 cycles (36 if uart is placed in IO address space)
- RX worst case - load byte and put it into buffer: 43 cycles (42 if uart is placed in IO address space)

- Any case: +2 on >128k mcu's, -1 for 256 byte buffers

# todo
- soft rts/cts