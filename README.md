# Easy-AVR-USART-C-Library
An interrupt-driven USART (RS232) library for AVR microcontrollers, with support for multiple hardware UARTs, using ring
buffers for receive/transmit. Designed especially for real-time or high throughput applications, with narrow resource limits.

##Features
- easy to use
- intuitive frame format settings
- selectable support for up to 4 USART's
- up to 255 byte FIFOs
- extremly light interrupts 
- software(RTS/CTS) and hardware flow control support (DCE naming)
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

##Notes
- Lot of terminals sends only CR character as a newline terminator, instead of CRLF or even unix style LF
(BTW PuTTY doesn't allow to change this) but in return requires CRLF terminator to show not broken text.
This behaviour can be covered by RX_NEWLINE_MODE macro, by default set to CRLF.

	- 0 - CR
	- 1 - LF
	- 2 - CRLF (default)

- In case of reinitializing uart on the fly (especially with non-constant ubbr) try to use uart_reint().

- Any used external IO pin have to be accesible from bottom IO address space. (eg. few ports on mega2560 cannot be used as a control IO) 

- In half duplex (RS485) transmission modes, the aplication code is responsible of starting transmission only when bus is idle.
If RE and DE are shorted together additional pullup on RX pin is required.
Pin used as a RS485 control line have to be kept in low state during boot process via a pulldown resistor or at least not driving it high even by an internall pull-up.

- In MPCM mode first received byte is address by which device was called (own or general call), application is also responsible of restoring into "idle listening" state withing described bus silence time after receiving entire packet.

- For software CTS, all used pins have to be configured as an input, and both edge interrupt source (INT/PCINT).
The application code should call cts_isr_handlers from interrupts corresponding to used pins. (see example(flow control).c)
If CTS line goes high during transmission, only one additional byte can be transmitted. (due to 2 level transmit register)

- For software RTS, all used pins have to be configured as input without pullup or output in low state.
If interrupts are not missed, the receiver can accept up to 2 additional bytes, one from ongoing transmission 
and another if transmitter misses RTS signal (last one is stored in shift register).

- For proper operation of hardware RTS, USART_EXTEND_RX_BUFFER have to be defined.

- uart_putc() function is not thread/interrupt-safe nor reentrant. It shouldn't be called from within atomic blocks or interrupt handlers since it re-enables interrupt flag on exit or even hangs in infinite loop waiting for execution of UDRE interrupt.

##ISR timmings (cycles)

| ISR case | attiny2313 | atmega8 | atmega328 | atmega2560 | lgt8f88A | 
| --- | --- | --- | --- | --- | --- |
| **TX best** | 37 | 40 | 40 | 42 | 30 |
| **TX worst** | 38 | 41 | 44 | 46 | 35 |
| **RX best** | 36 | 36 | 37 | 39 | 27 |
| **RX worst** | 40 | 42 | 43 | 45 | 32 |
| **RX best with rts** | 41 | 41 | 44 | 46 | 32 |
| **RX worst with rts** | 40 | 42 | 43 | 45 | 32 |
| **BUFFER_SIZE = 256** | x | -1 | -1 | -1 | -1 |

- TX best case - send byte from buffer
- TX worst case - send byte from buffer and disable UDRIE interrupt
- RX best case - load byte and do nothing (buffer full)
- RX worst case - load byte and put it into buffer

##todo
- func sizes
- xmega, tiny87
- some notes about flow control
- modbus
- xmodem timeouts/fallback