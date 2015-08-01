// TODO: add timeouts error handling

#include <util/delay.h>

#include "usart.h"

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NACK 0x15
#define CAN 0x18

#define bad_packet 0x00
#define good_packet 0x01
#define duplicate 0x02

#define transmission_in_progres 0x00
#define transmission_completed 0x01
#define transmission_aborted 0x02

#define XMODEM_VALIDATION_CRC 1 // 1 - crc, 0 - checksum
//#define XMODEM_VALIDATION_CRC 0 // some terminals/devices still doesn't provide support for crc error handling

uint8_t HandleIncomingData(uint8_t dat);
uint8_t validate_packet(uint8_t *bufptr, uint8_t *packet_number);
uint16_t calcrc(uint8_t *bufptr, uint8_t size);
uint8_t calchecksum(uint8_t *bufptr, uint8_t size);
void MoveData(uint8_t *bufptr, uint8_t BytesToMove);

uint8_t file[1024];
uint16_t fileposition;

uint8_t xbuff[133];
uint8_t xbuffposition;
uint8_t last_packet; // represents 'last successfully received packet'

int main(void)
{
	uint8_t incoming_data, transmission_status = 0;
	
	uart_init(BAUD_CALC(9600));

	sei();
	
	while(!uart_getc())
	{
		uart_puts_P("Press any key to continue ...\r\n");
		_delay_ms(1000);
	}
	
	uart_puts_P("Transmission will start in 10 seconds ...\r\n");
	_delay_ms(5000);
	_delay_ms(5000);
	
#if XMODEM_VALIDATION_CRC // 1 - crc
	uart_putc('C');
#else // 0 - checksum
	uart_putc(NACK);
#endif
	
	
	while(transmission_status == transmission_in_progres)
	{
		
		if(BUFFER_EMPTY != uart_getData(&incoming_data))
			transmission_status = HandleIncomingData(incoming_data);

		//AnotherRealTimeTaskDuringTransmission(); // why not ?
	}
	
	_delay_ms(3000);
	
	uart_puts_P("Transmission completed, thanks for your attention\r\n");
	uart_puts_P("hexdump of sent file : \r\n");
	
	for (uint16_t i = 0, j = 1; i < 1024; i++, j++)
	{
		uart_put_hex(file[i]);
		uart_putc(' ');
		
		if(j == 8)
		{
			uart_puts("\r\n");
			j = 0;
		}
		
	}

	while(1); // nothing to do
}

uint8_t HandleIncomingData(uint8_t dat)
{
	xbuff[xbuffposition++] = dat;
	
	if(dat == EOT && xbuffposition == 1)
	{
		uart_putc(ACK);
		return transmission_completed;
	}
	
#if XMODEM_VALIDATION_CRC  // 1 - crc
	if(xbuffposition == 133)
#else // 0 - checksum
	if(xbuffposition == 132)
#endif
	{
		xbuffposition = 0;
		
		uint8_t packet_status = validate_packet(xbuff, &last_packet);
		switch(packet_status)
		{
			case good_packet:
				MoveData(&xbuff[3], 128);
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
	
#if XMODEM_VALIDATION_CRC  // 1 - crc
	uint16_t crc = calcrc(&bufptr[3],128);      // compute CRC and validate it
	
	if ((bufptr[131] == (uint8_t)(crc >> 8)) && (bufptr[132] == (uint8_t)(crc)))
#else // 0 - checksum
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
		crc = crc ^ (uint16_t) *bufptr++ << 8;
		for(uint8_t i = 0; i < 8; i++)
		{
			if (crc & 0x8000)
				crc = crc << 1 ^ 0x1021;
			else
				crc = crc << 1;
		}
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
