/*!
 * \brief
 *
 * \author Jan Oleksiewicz <jnk0le@hotmail.com>
 * \license SPDX-License-Identifier: MIT
 */

#include <avr/io.h>
#include <util/delay.h>
#include <util/crc16.h>

#include "usart.h"

#define SETUP_RETRIES_DELAY 1000 /* in ms */
#define SETUP_CRC_RETRIES_TO_FALLBACK 15
#define SETUP_CKSUM_RETRIES_TO_GIVE_UP 2

#define PACKET_RETRY_DELAY 1000 /* in ms / timeouts if nothing is received or packet is too short */
#define PACKET_MAX_TIMEOUT_RETRANSMITS 15 /* 0-255 */

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NACK 0x15
#define CAN 0x18
//#define SUB 0x1a

#define bad_packet 0x00
#define good_packet 0x01
#define duplicate 0x02

typedef enum {
	ready_for_transmission = 0,
	awaiting_for_command,
	preparing_transmission,
	preparing_timeout,
	transmission_in_progres,
	//purge ?
	transmission_completed,
	transmission_max_retransmits,
	transmission_aborted
} ProgramStatus;

struct XmodemPacket {
	uint8_t buff[133];
	uint8_t position;
	uint8_t last_packet; // last successfully received packet
	uint8_t retrycount; // timeout retries (no packets or too small)
	uint16_t starttime; // uint16_t gives 65 sec margin
	uint8_t fallback;
	//uint16_t crc;
	//ProgramStatus transmission_status;
} xmdm;

uint8_t HandleIncomingData(uint8_t dat);
uint8_t validate_packet(uint8_t *bufptr, uint8_t *packet_number, uint8_t fallback);

uint16_t calcrc(uint8_t *bufptr, uint8_t size);
uint8_t calchecksum(uint8_t *bufptr, uint8_t size);

void MoveData(uint8_t *bufptr, uint8_t BytesToMove);
void HexDump16(uint8_t *bufptr, uint16_t Length);

uint32_t milis;

//inline uint32_t getMilis(void) __attribute__((always_inline));
uint32_t getMilis(void) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { return milis; } return 0; }

void timer1_init(void) // using timer 1 for milis is so wastefull, but still more usable than wasting it exclusively
{
	TCCR1A = 0;
	TCCR1B = (1 << CS10) | (1 << WGM12) | (1 << WGM13); //CTC mode, overflow on ICR1
	TIMSK1 = (1 << ICIE1); // CAPT vector is the overflow now
	ICR1 = F_CPU / 1000; // 1 millisecond accurate up to crystal ppm
}

ISR(TIMER1_CAPT_vect)
{
	milis++;
}

//just an example buffer to put downloaded content somewhere, it could be just spi flash
uint8_t file[1024];
uint16_t fileposition;

int main(void)
{
	ProgramStatus transmission_status = ready_for_transmission;
	char cmd;

	uart_init(BAUD_CALC(115200));
	timer1_init();

	sei();

	while (1)
	{
		switch (transmission_status)
		{
		case ready_for_transmission:
			uart_puts_P("Awaiting for command, type 'h' for help...\r\n");
			uart_putc('>');

			transmission_status = awaiting_for_command;

			break;
		case awaiting_for_command:

			cmd = uart_getc();
			if (cmd) // if received character
			{
				uart_puts("\r\n");
				switch (cmd)
				{
				case 'd':
					uart_puts_P("File transfer can be terminated by double ^X sequence before waiting for receiver timeout\r\n\n");
					uart_puts_P("Waiting for transmission ...\r\n");

					xmdm.last_packet = 0;
					xmdm.retrycount = 0;
					xmdm.fallback = 0;
					xmdm.starttime = getMilis();

					transmission_status = preparing_transmission;

					break;
				case 't':
				case 's':
					uart_puts_P("unsupported feature\r\n");
					transmission_status = ready_for_transmission;

					break;
				case 'x':
					uart_puts_P("hexdump of sent file : \r\n");

					HexDump16(file, 1024);

					transmission_status = ready_for_transmission;

					break;
				default:
					uart_puts_P("Usage:\r\n");
					uart_puts_P("d\t download file from pc\r\n");
					uart_puts_P("t\t send file to pc [not supported by now]\r\n");
					uart_puts_P("s\t look for any readable strings in received file [not supported by now]\r\n");
					uart_puts_P("x\t show hexdump of stored file\r\n");

					uart_putc('>');
				}
			}

			break;
		case preparing_transmission:

			cmd = uart_getc();
			if (cmd == SOH) // ignore everything before SOH
			{
				xmdm.retrycount = 0;
				xmdm.starttime = getMilis();

				transmission_status = HandleIncomingData(cmd);
				break;
			}
			else if (cmd == CAN) // 
				transmission_status = transmission_aborted;

			if ((uint16_t)getMilis() - xmdm.starttime > SETUP_RETRIES_DELAY)
			{
				if (xmdm.retrycount < SETUP_CRC_RETRIES_TO_FALLBACK + SETUP_CKSUM_RETRIES_TO_GIVE_UP)
				{
					if (xmdm.fallback)
						uart_putc(NACK);
					else
						uart_putc('C');

					xmdm.starttime = getMilis();
					xmdm.retrycount++;
				}
				else
				{
					transmission_status = preparing_timeout;
				}

				if (xmdm.retrycount >= SETUP_CRC_RETRIES_TO_FALLBACK)
				{
					xmdm.fallback = 1; // will be used in other parts	
				}
			}

			break;
		case preparing_timeout:
			uart_putc(CAN);
			uart_putc(CAN); // CAN in case if host started right now

			uart_puts_P("\r\nfailed to establish transmission with host\r\n");
			transmission_status = ready_for_transmission;

			break;
		case transmission_in_progres:
			//uint8_t incoming_data;
			//while (BUFFER_EMPTY != uart_LoadData(&incoming_data))
			//	transmission_status = HandleIncomingData(incoming_data);

			//while (1) // read all data from buffer
			//{
				//int16_t tmp = uart_getData();

				//if (tmp >= 0)
					//transmission_status = HandleIncomingData((uint8_t)tmp);
				//else // negative value - buffer empty
					//break;
			//}

			while (uart_AvailableBytes())
				transmission_status = HandleIncomingData(uart_getc());

			if ((uint16_t)getMilis() - xmdm.starttime > PACKET_RETRY_DELAY)
			{
				if (xmdm.retrycount > PACKET_MAX_TIMEOUT_RETRANSMITS)
				{
					transmission_status = transmission_aborted;
				}
				else // no need to purge in this case
				{
					uart_putc(NACK);
					xmdm.position = 0;
					xmdm.retrycount++;
				}
			}

			break;
		//case purge: ??
			//break;
		case transmission_max_retransmits:
			uart_putc(CAN);
			uart_putc(CAN);

			uart_puts_P("Timeout or short packets retransmits limit hit ... Aborting\r\n");

			transmission_status = ready_for_transmission;

			break;
		case transmission_completed:
			uart_puts_P("Transmission completed, thanks for your attention\r\n");
			transmission_status = ready_for_transmission;

			break;
		case transmission_aborted:
			uart_puts_P("\r\n^X, Aborting ... \r\n");

			transmission_status = ready_for_transmission;
		}

		//AnotherRealTimeTaskDuringTransmission(); // why not ?
	}
}

uint8_t HandleIncomingData(uint8_t dat)
{
	if (dat == EOT && xmdm.position == 0)
	{
		uart_putc(ACK);
		uart_putc(ACK);
		return transmission_completed;
	}

	if (xmdm.position == 1 && dat == CAN && xmdm.buff[0] == CAN) // double check - should arrive after timeout / whole packet
		return transmission_aborted;

	xmdm.buff[xmdm.position++] = dat;

	if (xmdm.position == (133 - xmdm.fallback)) // 133 or 132 bytes
	{
		xmdm.position = 0;

		uint8_t packet_status = validate_packet(xmdm.buff, &xmdm.last_packet, xmdm.fallback);
		switch (packet_status)
		{
		case good_packet:
			uart_putc(ACK);
			xmdm.starttime = getMilis();

			MoveData(&xmdm.buff[3], 128);
			// insert a data handler here, for example, write buffer to a flash device

			return transmission_in_progres;

		case duplicate:
			uart_putc(ACK); // just ack this

			xmdm.starttime = getMilis();
			//count duplicates

			break;
		case bad_packet:
			uart_putc(NACK);

			xmdm.starttime = getMilis();
			//count badpacket

			return transmission_in_progres;

		//default:
		}

	}

	return transmission_in_progres;
}

uint8_t validate_packet(uint8_t *bufptr, uint8_t *packet_number, uint8_t fallback)
{
	if (bufptr[0] != SOH) // valid start
		return bad_packet;

	if ((bufptr[1] + bufptr[2]) != 0xff) // sanity check before duplicating
		return bad_packet;

	if (bufptr[1] == *packet_number)
		return duplicate;

	if (bufptr[1] != ((*packet_number + 1) & 0xff)) // check if it is the next packet
		return bad_packet;

	if (fallback)
	{
		uint8_t cksum = calchecksum(&bufptr[3], 128);

		if (bufptr[131] != cksum)
			return bad_packet;
	}
	else
	{
		uint16_t crc = calcrc(&bufptr[3], 128); // compute CRC and validate it

		if ((bufptr[131] != (uint8_t)(crc >> 8)) || (bufptr[132] != (uint8_t)(crc)))
			return bad_packet;
	}

	*packet_number = *packet_number + 1;
	return good_packet;
}

uint16_t calcrc(uint8_t *bufptr, uint8_t size)
{
	uint16_t crc = 0;

	while (size--)
	{
		crc = _crc_xmodem_update(crc, *bufptr++); // util/crc16.h
		//crc = small_crc_update(crc, *bufptr++);
	}

	return crc;
}

uint8_t calchecksum(uint8_t *bufptr, uint8_t size)
{
	uint8_t cksum = 0;

	while (size--) cksum += *bufptr++;

	return cksum;
}

void MoveData(uint8_t *bufptr, uint8_t BytesToMove) // insert a data handler here, for example, write buffer to a flash device
{
	for (uint8_t i = 0; i < BytesToMove; i++)
	{
		file[fileposition++] = bufptr[i];
		fileposition &= 1023; // this is an example, so just overwrite our file if sent file is >1K
	}
}

void HexDump16(uint8_t *bufptr, uint16_t Length)
{
	uint16_t i;
	char buff[17];
	buff[16] = 0;

	// Process every byte in the data.
	for (i = 0; i < Length; i++)
	{
		// Multiple of 16 means new line (with line offset).

		if ((i % 16) == 0)
		{
			// Just don't print ASCII for the zeroth line.
			if (i != 0)
			{
				uart_puts("  ");
				uart_puts(buff);
				uart_puts("\r\n");
			}

			// Output the offset.
			uart_puts("  ");

			uart_puthex(i >> 8);
			uart_puthex(i);

			uart_putc(' ');
		}

		// Now the hex code for the specific character.
		uart_putc(' ');

		uart_puthex(bufptr[i]);

		// And store a printable ASCII character for later.
		if ((bufptr[i] < 0x20) || (bufptr[i] > 0x7e))
			buff[i % 16] = '.';
		else
			buff[i % 16] = bufptr[i];
	}

	// Pad out last line if not exactly 16 characters.
	while ((i % 16) != 0)
	{
		buff[i % 16] = 0;
		uart_puts("   ");
		i++;
	}

	// And print the final ASCII bit.
	uart_puts("  ");
	uart_puts(buff);
	uart_puts("\r\n");
}
