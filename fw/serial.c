// Part of ZSIM - Reactance Simulator using Mutual Inductors. 
// Firmware for AVR controller of the simulator.
//
// Target MCU: ATmega88
// Fuse bits: use default setup: RC oscillator and /8 divider (1MHz)
//
// (c) 2020, Stanislav Maslan, CMI, smaslan@cmi.cz, s.maslan@seznam.cz
// The script is distributed under MIT license, https://opensource.org/licenses/MIT. 
//
// Simulator details: 
//   https://github.com/smaslan/Z-sim-mutual
//
// This project was co-developed in scope of EMPIR project LiBforSecUse
// "Lithium Batteries for Second Life Applications". Project url:
//   https://www.ptb.de/empir2018/libforsecuse/home/

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay_basic.h>
#include <util/delay.h>
#include <string.h>

#include "main.h"
#include "serial.h"

//----------------------------------------------------------------------------------
// UART STUFF
//----------------------------------------------------------------------------------


// USART ISR
ISR(USART_RX_vect)
{
	
	uint8_t ptr = rxd_ptr; // work with local copy - faster
	if(ptr < RX_BUF_SZ - 1)
	{
		uint8_t dbyte = UDR0;
		rxd[ptr++] = dbyte;
		rxd[ptr]   = '\0'; // add terminator

		if(dbyte == '\0' || dbyte == '\n')
			sbi(rxd_stat,RX_DONE); // command done
	}
	rxd_ptr = ptr;

}

// init USART
void serial_init(void)
{
	// init RX/TX
	UCSR0A = (1<<U2X0);
	UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
	UCSR0C = (0<<UMSEL00) | (0<<UPM00) | (0<<USBS0) | (3<<UCSZ00);
	UBRR0 = (uint16_t)((F_CPU/(USART_BAUDRATE*8ul)) - 1);

	rxd[0] = '\0';
	rxd_ptr = 0; // read pointer
	rxd_stat = (0<<RX_DONE); // no command yet
}

// decode command, supports following format:
//  "my:command:or:whatver[<space(s)>parameter]"
uint8_t serial_decode(char **par)
{
	// check command completness
	if(!bit_is_set(rxd_stat,RX_DONE))
		return(0); // not rx done
	
	// command rx done - store data size
	uint8_t size = rxd_ptr;
	rxd[size] = '\0';
		
	// parse till token
	char *p = strchr(rxd,' ');
	if(!p)
	{
		// no parameter
		*par = NULL;

		// search end of command string
		p = rxd;
		while(*p != ';' && *p != '\n' && *p != '\r' && *p != '\0')
			p++;
		*p = '\0'; // mark end of command

	}
	else
	{
		// possibly parameter

		// mark end of command header
		*p++ = '\0';

		// skip whites
		while(*p == ' ')
			p++;	
		
		// return parameter start
		*par = p;

		// search end of parameter string
		while(*p != ';' && *p != '\n' && *p != '\r' && *p != '\0')
			p++;
		*p = '\0'; // mark end of parameter

		// make parameter upper case for simple decoding
		strupr(*par);

	}

	// make command upper case for simple decoding
	strupr(rxd);

	return(1); // command detected
}

// clear rx command buffer
void serial_flush_command(void)
{	
	// reset data pointer
	rxd_ptr = 0;

	// clear command flag
	cbi(rxd_stat,RX_DONE);
}


// send byte
void serial_tx_byte(uint8_t byte)
{
	loop_until_bit_is_set(UCSR0A,UDRE0);
	UDR0 = byte;
}

// send string from progmem
void serial_tx_cstr(const char *str)
{
	char byte;
	while((byte = pgm_read_byte(str++)) != '\0')
		serial_tx_byte(byte);
}

// send string from progmem
void serial_tx_str(char *str)
{
	char byte;
	while((byte = *str++) != '\0')
		serial_tx_byte(byte);
}




// --- SCPI error generator ---

// err: error code; info: optional error message; mode: flags
void serial_error(int16_t err,const char *info,uint8_t mode)
{
	static uint16_t err_mem = 0;
	static char *info_mem = NULL;
	
	// remember last error
	if(mode&SCPI_ERR_STORE)
	{
		err_mem = err;
		info_mem = (char*)info;
	}

	if(mode&SCPI_ERR_SEND)
	{
		switch(err_mem)
		{
			case SCPI_ERR_undefinedHeader:
				serial_tx_cstr(PSTR("-113, Undefined command header.")); break;
			case SCPI_ERR_wrongParamType:
				serial_tx_cstr(PSTR("-104, Wrong parameter type or value.")); break;
			case SCPI_ERR_tooFewParameters:
				serial_tx_cstr(PSTR("-109, Missing parameters.")); break;
			case SCPI_ERR_std_mediaProtected:
				serial_tx_cstr(PSTR("-258, EEPROM write protected.")); break;
			case SCPI_ERR_noError:
				serial_tx_cstr(PSTR("0, No error.")); break;
			default:
				break;

		}
		if(info_mem)
			serial_tx_cstr(info_mem);
		serial_tx_cstr(PSTR("\n"));

		err_mem = 0;
		info_mem = NULL;
	}
	
}
