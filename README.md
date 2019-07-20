# AVR-UART-lib
An interrupt-driven USART (RS232) library for AVR microcontrollers, with support for multiple hardware UARTs, using ring
buffers for receive/transmit. Designed especially for real-time or high throughput applications, with narrow resource limits.

## Features
- selectable support for up to 4 USART's
- up to 255 byte FIFOs
- only 2 (+1 wasted slot) byte memory footprint except actual buffer for every implemented RX or TX path
- no dynamic memory allocations
- extremly light interrupts
- software(RTS/CTS) and hardware flow control support (DCE naming)
- RS 485 compatibility
- MPCM master and slave mode support
- printf()/scanf() streams compatibility
- V-USB compatibility (25 cycle ISR restriction)
- optimized as much as possible to reduce code size and execution times

## Notes
- Lot of terminals sends only CR character as a newline terminator, instead of CRLF or even unix style LF
(BTW PuTTY doesn't allow to change this) but in return requires CRLF terminator to show not broken text.
This behaviour can be covered by RX_NEWLINE_MODE macro, by default set to CRLF.

	- 0 - CR
	- 1 - LF
	- 2 - CRLF (default)

- In order to reinitialize uart baudrate after any traffic was occurring, function uart_reinit() have to be used. (memory FIFO buffers will not be flushed)

- Any used IO pin have to be accessible from bottom IO address space. (eg. few ports on mega2560 cannot be used as a control IO) 

- In half duplex (RS485) transmission modes, the application code is responsible of starting transmission only when bus is idle.
If RE and DE are shorted together additional pullup on RX pin is required.
Pin used as a RS485 control line have to be kept in low state during boot process via a pulldown resistor or at least not driving it high even by an internall pull-up. (especially on multi node buses)

- In MPCM mode first received byte is address by which device was called (own or general call), application is also responsible of restoring into "idle listening" state withing described bus silence time after receiving entire packet.

- uart_putc() function is not thread/interrupt-safe nor reentrant. It shouldn't be called from within atomic blocks or interrupt handlers since it re-enables interrupt flag on exit or even hangs in infinite loop waiting for execution of UDRE interrupt.

- this library is written in c99/gnu99 dialect standard.

## Flow control

This library provides a software implementation of CTS/RTS control lines (DTR/DSR respectively) to signal whether buffer is full (not an 80' style handshaking).
It should be used, in order to achieve stable bidirectional >100kbps transmissions, especially when using non deterministic usb-serial converters (ft232, ch340 etc).
The legendary "noise" that causes losing blocks of characters (not damaging) can also be fixed by that.

### Config
- soft CTS pins have to be configured as an input, and both edge interrupt source (INT/PCINT), before entering uart_init().
The application code should call cts_isr_handlers from interrupts corresponding to used pins. (see [example](example(flow control).c))
If CTS line goes high during transmission (before UDRE interrupt is fired), only one additional byte will be transmitted. (due to 2 level transmit register)

- soft RTS pins have to be configured as input without pullup or output in low state, before entering uart_init().
If interrupts are not missed, the receiver can accept up to 2 additional bytes after buffer is filled, one from next or ongoing transmission 
and another one if transmitter misses RTS signal (last one is stored in shift register).

- Any used IO pin have to be accessible from bottom IO address space. (eg. few ports on mega2560 cannot be used as a flow control IO) 

- For proper operation of hardware RTS, USART_EXTEND_RX_BUFFER have to be defined.

## ISR timmings (cycles)

| ISR case | attiny2313 | atmega8 | atmega328 | atmega2560 | lgt8f88A | 
| --- | --- | --- | --- | --- | --- |
| **TX best** | 37 | 40 | 40 | 42 | 30 |
| **TX worst** | 38 | 41 | 44 | 46 | 35 |
| **RX best** | 36 | 36 | 37 | 39 | 27 |
| **RX worst** | 40 | 42 | 43 | 45 | 32 |
| **RX rts full** | 41 | 41 | 44 | 46 | 32 |
| **RX rts normal** | 40 | 42 | 43 | 45 | 32 |
| **BUFFER_SIZE = 256** | x | -1 | -1 | -1 | -1 |
| **G_SREG_SAVE** | -4 | -4 | -4 | -4 | -2 |
| **G_Z_SAVE** | -6 | -6 | -6 | -6 | -2 |

- TX best case - send byte from buffer
- TX worst case - send byte from buffer and disable UDRIE interrupt (will not generate double shot)
- RX best case - load byte and do nothing (buffer full)
- RX worst case - load byte and put it into buffer
- RX rts full - rise RTS line and disable RXCIE interrupt
- RX rts normal - load byte and put it into buffer if there is available space

