// TODO: add timeouts error handling

#include <util/delay.h>
#include <util/crc16.h>

#include "usart.h"

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NACK 0x15
#define CAN 0x18

#define bad_packet 0x00
#define good_packet 0x01
#define duplicate 0x02

typedef enum {
	ready_for_transmission,
	awaiting_for_command,
	preparing_transmission,
	transmission_in_progres,
	transmission_completed,
	transmission_aborted
} ProgramStatus;

#define XMODEM_VALIDATION_CRC // comment out to use checksum instead of CRC
// some terminals/devices still doesn't provide support for crc error handling

uint8_t HandleIncomingData(uint8_t dat);
uint8_t validate_packet(uint8_t *bufptr, uint8_t *packet_number);
uint16_t calcrc(uint8_t *bufptr, uint8_t size);

uint8_t calchecksum(uint8_t *bufptr, uint8_t size);

void MoveData(uint8_t *bufptr, uint8_t BytesToMove);
void HexDump16(uint8_t *bufptr, uint16_t ByteCount);

//static inline uint16_t small_crc_update(uint16_t crc, uint8_t data)
//{
//	uint8_t i;
//
//	crc = crc ^ ((uint16_t)data << 8);
//	for (i = 0; i<8; i++)
//	{
//		if (crc & 0x8000)
//			crc = (crc << 1) ^ 0x1021;
//		else
//			crc <<= 1;
//	}
//
//	return crc;
//}

//static inline uint8_t cksum_update(uint8_t cksum, uint8_t data)
//{
//	return cksum + data;
//}

//clear crc to 0 

//just a buffer to put downloaded content somewhere
uint8_t file[1024];
uint16_t fileposition;

struct XmodemPacket {
	uint8_t buff[133];
	uint8_t position;
	uint8_t last_packet; // represents 'last successfully received packet'
} xmdm;

int main(void)
{
	ProgramStatus transmission_status = ready_for_transmission;
	char cmd;

	uart_init(BAUD_CALC(9600));

	sei();
	
	while (1)
	{
		switch (transmission_status)
		{
			case ready_for_transmission:
				uart_puts_P("Awaiting for command, type 'h' for help...\r\n");
				uart_putc('>');

				transmission_status++;

				break;
			case awaiting_for_command:
				
				cmd = uart_getc();
				if (cmd) // if received character
				{
					uart_puts("\r\n");
					switch (cmd)
					{
						case 'd':
							transmission_status = preparing_transmission;
							uart_puts_P("Waiting for transmission ...\r\n");
							break;
						case 't': 
						case 's':
							transmission_status = transmission_aborted;

							break;
						case 'x':
							uart_puts_P("hexdump of sent file : \r\n");

							HexDump16(file, 1024);
							
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
				
				if(uart_AvailableBytes())
				{
					transmission_status = transmission_in_progres;
					break;
				}
			#ifdef XMODEM_VALIDATION_CRC
				uart_putc('C');
			#else
				uart_putc(NACK);
			#endif
				
				_delay_ms(1000); // poor delay
				
				break;
			case transmission_in_progres:
				//looks clean but not most optimal
				//uint8_t incoming_data;
				//while (BUFFER_EMPTY != uart_LoadData(&incoming_data))
				//	transmission_status = HandleIncomingData(incoming_data);
				
				while (1)
				{
					int16_t tmp = uart_getData();

					if (tmp >= 0)
						transmission_status = HandleIncomingData((uint8_t)tmp);
					else // negative value - buffer empty
						break;
				}

				break;
			case transmission_completed:
				uart_puts_P("Tansmission completed, thanks for your attention\r\n");
				transmission_status = ready_for_transmission;

				break;
			case transmission_aborted:
				uart_puts_P("Fatal error occured ... Aborting\r\n");
				transmission_status = ready_for_transmission;
		}

		//AnotherRealTimeTaskDuringTransmission(); // why not ?
	
	}
	
}

uint8_t HandleIncomingData(uint8_t dat)
{
	xmdm.buff[xmdm.position++] = dat;
	
	if(dat == EOT && xmdm.position == 1)
	{
		uart_putc(ACK);
		return transmission_completed;
	}
	
#ifdef XMODEM_VALIDATION_CRC
	if(xmdm.position == 133)
#else
	if(xmdm.position == 132)
#endif
	{
		xmdm.position = 0;
		
		uint8_t packet_status = validate_packet(xmdm.buff, &xmdm.last_packet);
		switch(packet_status)
		{
			case good_packet:
				MoveData(&xmdm.buff[3], 128);
				uart_putc(ACK);
					// insert a data handler here,
					// for example, write buffer to a flash device
				return transmission_in_progres;
			
			case duplicate:
				uart_putc(ACK);
					// a counter for duplicate packets could be added here, to enable a
					// for example, exit gracefully if too many consecutive duplicates,
					// otherwise do nothing, we will just ack this
				break;
			case bad_packet:
				uart_putc(NACK);
				return transmission_in_progres;
			
			default:
				uart_putc(CAN);
				uart_putc(CAN);
				return transmission_aborted;
					// bad, timeout or error -
					// if required, insert an error handler of some description,
					// for example, exit gracefully if too many errors
		}
		
	}

	return transmission_in_progres;
}

uint8_t validate_packet(uint8_t *bufptr, uint8_t *packet_number)
{
	if (bufptr[0] != SOH) // valid start
		return bad_packet;
		
	if (bufptr[1] != ((*packet_number+1) & 0xff) || (bufptr[1] + bufptr[2]) != 0xff) // block number and checksum are ok?
		return bad_packet;

	if (bufptr[1] == *packet_number)
		return duplicate;
	
#ifdef XMODEM_VALIDATION_CRC
	uint16_t crc = calcrc(&bufptr[3],128);      // compute CRC and validate it
	
	if ((bufptr[131] == (uint8_t)(crc >> 8)) && (bufptr[132] == (uint8_t)(crc)))
#else 
	uint8_t cksum = calchecksum(uint8_t *bufptr, uint8_t size)
	
	if (bufptr[131] == cksum)
#endif
	{
		*packet_number = *packet_number + 1; // good packet ... ok to increment
		return good_packet;
	}
	else
		return bad_packet;
}

uint16_t calcrc(uint8_t *bufptr, uint8_t size)
{
	uint16_t crc = 0;
	
	while(size--)
	{
		crc = _crc_xmodem_update(crc, *bufptr++);
	}

	return crc;
}

uint8_t calchecksum(uint8_t *bufptr, uint8_t size)
{
	uint8_t cksum = 0;
		
	while(size--) cksum += *bufptr++;
		
	return cksum;
}

void MoveData(uint8_t *bufptr, uint8_t BytesToMove)
{
	for(uint8_t i = 0; i < BytesToMove ;i++)
	{
		file[fileposition] = bufptr[i];
		fileposition = (fileposition+1);
		
		if(fileposition == 1024) fileposition = 0; // overwrite our file if sent file is >1K
	}
}

void HexDump16(uint8_t *bufptr, uint16_t ByteCount)
{
	uint16_t i;
	char buff[17];
	buff[16] = 0;
	
	// Process every byte in the data.
    for (i = 0; i < ByteCount; i++) 
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
			
			uart_puthex(i>>2);
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
