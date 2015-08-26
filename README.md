# Easy-AVR-USART-C-Library
An interrupt-driven USART (RS232) library for AVR microcontrollers, with support for multiple UARTs, using ring
buffers for receive/transmit. Designed to be easy to use, especially like arduino libraries (with less overload).

# Features
- easy to use
- support for up to 4 USART's
- allow for binary transmission
- optimized as much as possible to reduce code size
- printf()/scanf() streams compatibility
- and much more

simple "hello world" code gives

		   text	   data	    bss	    dec	    hex	filename
		    428	     14	     68	    510	    1fe	avrt.elf

Program Memory Usage 	:	442 bytes   1,3 % Full
Data Memory Usage 		:	82 bytes   4,0 % Full

defined  NO_USART_RX flag gives

		   text	   data	    bss	    dec	    hex	filename
		    350	     14	     34	    398	    18e	avrt.elf

Program Memory Usage 	:	364 bytes   1,1 % Full
Data Memory Usage 		:	48 bytes   2,3 % Full

For this result additional flag -mrelax is required in many IDE's (eg. Atmel studio 6.2)


# Notes
Lot of terminals sends only CR character as a newline terminator, instead of CRLF or even unix style LF
(BTW PuTTY doesn't allow to change this) but in return requires CRLF terminator to show not broken text.
This behaviour can be covered by RX_NEWLINE_MODE macro, by default set to CRLF.

- 0 - CR
- 1 - LF
- 2 - CRLF (default)

