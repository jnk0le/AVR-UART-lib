# Easy-AVR-USART-C-Library
An interrupt-driven USART (RS232) library for AVR microcontrollers, with support for multiple UARTs, using ring
buffers for receive/transmit. Designed to be easy to use, with zero overhead from unused features.

# Features
- easy to use
- intuitive frame format settings
- support for up to 4 USART's
- extremly light interrupts 
- software and hardware flow control (DCE naming)
- RS 485 compatibility
- MPCM master and slave mode support
- printf()/scanf() streams compatibility
- V-USB compatibility (25 cycle ISR restriction)
- optimized as much as possible to reduce code size and execution times
- and much more

simple minimal "hello world" (with delay) code on mega328p can give following results:

	Program Memory Usage 	:	390 bytes   1,2 % Full
	Data Memory Usage 		:	82 bytes   4,0 % Full

same code on mega8 gives:

	Program Memory Usage 	:	300 bytes   3,7 % Full 
	Data Memory Usage 		:	82 bytes   8,0 % Full

Meanwhile Arduino generates 2KB of code.

For this result additional flag -mrelax is required in many IDE's (eg. Atmel studio)

# Notes
Lot of terminals sends only CR character as a newline terminator, instead of CRLF or even unix style LF
(BTW PuTTY doesn't allow to change this) but in return requires CRLF terminator to show not broken text.
This behaviour can be covered by RX_NEWLINE_MODE macro, by default set to CRLF.

- 0 - CR
- 1 - LF
- 2 - CRLF (default)

In case of reinitializing uart on the fly (especially with non-constant ubbr) try to use uart_reint() or define USART_NO_DIRTY_HACKS macro.

In half duplex (rs485) transmission modes, the aplication code is responsible of starting transmission only when bus is idle.
If RE and DE are shorted together additional pullup on RX pin is required.

In MPCM mode first received byte is address by which device was called (own or general call), application is also responsible of restoring into "idle listening" state withing described bus silence time after receiving entire packet.

For software CTS, all used pins have to be configured as an input, and both edge interrupt source (INT/PCINT).
The application code should call cts_isr_handlers from interrupts corresponding to used pins. (see example(flow control).c)
If CTS line goes high during transmission, only one additional byte can be transmitted. (due to 2 level transmit register)

For software RTS, all used pins have to be configured as input without pullup or output in low state.
If interrupts are not missed, the receiver can accept up to 2 additional bytes, one from ongoing transmission 
and another if transmitter misses RTS signal (last one is stored in shift register).

For proper operation of hardware RTS, USART_EXTEND_RX_BUFFER have to be defined.

# ISR timmings

- TX best case - send byte from buffer: 40 cycles (39 if uart is placed in IO address space)
- TX worst case - send byte from buffer and disable UDRIE interrupt: 44 cycles (40 if uart is placed in IO address space) 

- RX best case - load byte and do nothing (buffer full): 37 cycles (36 if uart is placed in IO address space, up to 45 if soft RTS/extended buffer is used)
- RX worst case - load byte and put it into buffer: 43 cycles (42 if uart is placed in IO address space)

- Any case: +2 on >128k and some older mcu's, -1 for 256 byte buffers