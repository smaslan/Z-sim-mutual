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
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay_basic.h>
#include <util/delay.h>
#include <util/twi.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "serial.h"


// power bias control (active low)
#define PBIASp PORTD
#define PBIASd DDRD
#define PBIAS PD2

// FAN control
#define PFANp PORTC
#define PFANd DDRC
#define PFAN PC3

// main timer tick [s]
#define MTICK 500e-6

// control states
typedef struct{
	int8_t bias;
	uint8_t ind;
	uint8_t range;
	uint8_t com;
	uint8_t power;
	uint16_t bias_pwr;
	uint16_t bias_pot;
}TCTRL;
volatile TCTRL ctrl = {0,0,0,0,0};




//----------------------------------------------------------------------------------
// special function bistable relays controled via MAX4820
//----------------------------------------------------------------------------------

// special relays state variable
volatile uint8_t specr;

// special relays flags in status variable
#define RE_RNG 0 // range
#define RE_POL 1 // polarity
#define RE_COM 2 // common grounds bypass
#define RE_PWR 3 // power bypass relay
#define REMODF 7 // command in progress flag (do not change manually!)

// default special relays state
#define RES_DEF (0)

// special relay pulse length in ticks count (2ms should be enough)
#define RESLEN ((uint8_t)(2e-3/MTICK))

// update special relays state (states: new flag states, mask: 0's to inhibit state change)
// call this to initiate transition (IRQ must be enabled!)
void spec_relay_update(uint8_t states,uint8_t mask)
{
	uint8_t temp = specr;

	// update relay flags
	temp &= ~mask;
	temp |= (states & mask) | (1<<REMODF);
	
	// store to state variable
	specr = temp;
}


// SPI bus states
#define SPID DDRB
#define SPIP PORTB
#define SS PB2
#define SCK PB5
#define MOSI PB3

// send special relays MAX4820 controler command via SPI
void spec_relay_set(uint8_t data)
{
	// CS active
	cbi(SPIP,SS);
	
	// send data
	SPDR = data;
	
	// wait for transfer done
	while(!bit_is_set(SPSR,SPIF));

	// disable CS
	sbi(SPIP,SS);
}

// initialize special relays MAX4820 controler (SPI bus)
void spec_relay_init(void)
{
	// set SPI pin directions
	sbi(SPID,SS);
	sbi(SPID,SCK);
	sbi(SPID,MOSI);

	// enable SPI at full speed
	SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR0);
	SPSR |= (1<<SPI2X);

	// chip select to inactive
	sbi(SPIP,SS);

	// all drivers to idle state
	spec_relay_set(0x00);
}
// relay decoder table (state flags to control pulses for MAX4820)
const uint8_t srdec_lut[] PROGMEM = {0x55,0x56,0x59,0x5A,0x65,0x66,0x69,0x6A,0x95,0x96,0x99,0x9A,0xA5,0xA6,0xA9,0xAA};

// generate MAX4820 control pulse defined by low nibble bits of 'states'
void spec_relay_generate_pulse(uint8_t states)
{
	// start pulse
	spec_relay_set(pgm_read_byte(&srdec_lut[states&0x0Fu]));
}

// initiate assynchronous update of relay states (ISR must be enabled!)
//   optionally wait for completion of the command
//   always waits for completion of previous command@
void set_relays(volatile TCTRL *ctrl,uint8_t wait)
{
	// wait for previous command to end
	while(bit_is_set(specr,REMODF));

	// make new command
	specr = ((!ctrl->ind)<<RE_POL) | (ctrl->range<<RE_RNG) | ((!ctrl->com)<<RE_COM) | (ctrl->power<<RE_PWR) | (1<<REMODF);

	// optional wait for completion
	if(wait)
		while(bit_is_set(specr,REMODF));
		
}

// sequencer that generates control pulses for the spec function relays
//   note: this must be called periodically with MTICK period
void spec_rel_tick(void)
{
	static uint8_t spec_rel = 0x00; // local state variable for the relay control
	static uint8_t spec_rel_cnt = 0; // relay control timing counter
	
	if(!spec_rel_cnt && bit_is_set(specr,REMODF) && !bit_is_set(spec_rel,REMODF))
	{
		// --- user command to change relay states found:
		
		// store desired new states
		spec_rel = specr;

		// start control pulse timer
		spec_rel_cnt = RESLEN;
				
		// start control pulse
		spec_relay_generate_pulse(spec_rel);
		
	}
	else if(!spec_rel_cnt && bit_is_set(spec_rel,REMODF))
	{
		// --- relay pulse end reached:
		
		// clear modified flag in local state variable
		cbi(spec_rel,REMODF);

		// clear modified flag in user command states variable (indicates the command is done)
		cbi(specr,REMODF);

		// end control pulse
		spec_relay_set(0x00);

	}
	else if(spec_rel_cnt > 0)
		spec_rel_cnt--;
}



//----------------------------------------------------------------------------------
// main tick timer
//----------------------------------------------------------------------------------

// button commons
#define MB0p PORTD
#define MB0d DDRD
#define MB0  PD7
#define MB1p PORTD
#define MB1d DDRD
#define MB1  PD6
// LED cathodes
#define ML0p PORTB
#define ML0d DDRB
#define ML0  PB6
#define ML1p PORTD
#define ML1d DDRD
#define ML1  PD5
#define ML2p PORTB
#define ML2d DDRB
#define ML2  PB7
// LED anodes/buttons
#define MD0p PORTC
#define MD0i PINC
#define MD0d DDRC
#define MD0  PC2
#define MD1p PORTD
#define MD1i PIND
#define MD1d DDRD
#define MD1  PD3
#define MD2p PORTC
#define MD2i PINC
#define MD2d DDRC
#define MD2  PC1
#define MD3p PORTD
#define MD3i PIND
#define MD3d DDRD
#define MD3  PD4
#define MD4p PORTC
#define MD4i PINC
#define MD4d DDRC
#define MD4  PC0

#define LED_CYCLES 1
#define LED_DEADTIME 5


// --- TIMEOUT STUFF ---
// timeout timer [ms]
volatile uint16_t timeout = 0;
// set mew timeout value
void timeout_set(uint16_t time_ms)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// set new time
		timeout = time_ms;		
	}	
}
// check if timeout is done
uint8_t timeout_done(void)
{
	uint8_t done;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// set new time
		done = (timeout == 0);
	}	
	return(done);
}



// button states
volatile uint8_t btns = 0x00;
volatile uint8_t btns_down = 0x00;

// tick ISR
ISR(TIMER0_COMPA_vect)
{
	
	static uint8_t mux_cycle = 0;
	static uint8_t btns_old = 0x00;

	if(mux_cycle == 0)
	{
		// --- button B0:

		// change mux states
		cbi(MD0d,MD0);	// mux port to inputs
		cbi(MD1d,MD1);
		cbi(MD2d,MD2);
		cbi(MD3d,MD3);
		cbi(MD4d,MD4);
		
		sbi(MD0p,MD0);	// mux port pullups
		sbi(MD1p,MD1);
		sbi(MD2p,MD2);
		sbi(MD3p,MD3);
		sbi(MD4p,MD4);

		sbi(ML1p,ML1);	// clear LEDs
		cbi(MB0p,MB0);  // set B0

	}
	else if(mux_cycle == 1)
	{
		// --- button B1:

		// read B0 states to button state variable
		uint8_t tmp = btns;
		bcopy(tmp,0,MD0i,MD0);
		bcopy(tmp,1,MD1i,MD1);
		bcopy(tmp,2,MD2i,MD2);
		bcopy(tmp,3,MD3i,MD3);
		bcopy(tmp,4,MD4i,MD4);
		btns = tmp;

		// change mux states
		sbi(MB0p,MB0);	// clear B0
		cbi(MB1p,MB1);  // set B1

	}
	else if(mux_cycle == 2+LED_DEADTIME)
	{
		// --- LED0 common:

		// read B1 states to button state variable
		uint8_t tmp = btns;
		bcopy(tmp,5,MD0i,MD0);
		bcopy(tmp,6,MD1i,MD1);
		bcopy(tmp,7,MD2i,MD2);
		btns = tmp;

		// change mux states
		sbi(MB1p,MB1);	// clear B1
		sbi(MD0d,MD0);	// mux port to outputs
		sbi(MD1d,MD1);
		sbi(MD2d,MD2);
		sbi(MD3d,MD3);
		sbi(MD4d,MD4);

		// set LEDs
		cbi(MD3p,MD3);
		cbi(MD2p,MD2);
		cbi(MD1p,MD1);
		cbi(MD0p,MD0);			
		if(ctrl.bias == 0)
		{
			// bias A
			sbi(MD3p,MD3);
		}
		else if(ctrl.bias == 1)
		{
			// bias B
			sbi(MD2p,MD2);
		}
		else if(ctrl.bias == 2)
		{
			// bias C
			sbi(MD1p,MD1);
		}
		else if(ctrl.bias == 3)
		{
			// bias D
			sbi(MD0p,MD0);			
		}
		else
		{
			// custom bias
			sbi(MD3p,MD3);
			sbi(MD0p,MD0);
		}
		// range LED A
		bcopy_v(MD4p,MD4, (!ctrl.range), 0);

		// change mux states
		cbi(ML0p,ML0);  // set L0

	}
	else if(mux_cycle == 2 + LED_DEADTIME + LED_CYCLES)
	{
		// --- LED1 common:

		// change mux states
		sbi(ML0p,ML0);	// clear L0
		
		// range LED B
		bcopy_v(MD0p,MD0, ctrl.range, 0);
		// inductive mode
		bcopy_v(MD1p,MD1, !ctrl.ind, 0);
		bcopy_v(MD2p,MD2,  ctrl.ind, 0);
		// volt-curr joint
		bcopy_v(MD3p,MD3, ctrl.com, 0);
		// power bias active
		bcopy_v(MD4p,MD4, ctrl.power, 0);
		

		// change mux states
		cbi(ML1p,ML1);  // set L1

	}
	
	mux_cycle++;	// next cycle
	if(mux_cycle >= 2 + LED_DEADTIME + LED_CYCLES*2)
	{
		mux_cycle = 0;	// mux done

		// detect button down events
		btns_down |= ((btns^btns_old)&(~btns));
		
		// store old button states
		btns_old = btns;
	}


	// --- latching relays sequencer ---
	spec_rel_tick();

	
	// --- update timeout timer ---
	static uint8_t time_div = 0;	
	time_div++;
	if(time_div >= (uint8_t)(1e-3/MTICK))
	{
		// timeout timer tick (1ms)
		time_div = 0;
		if(timeout)
			timeout--;
	}
}


//----------------------------------------------------------------------------------
// I2C STUFF
//----------------------------------------------------------------------------------

#define i2c_start() TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN)
#define i2c_stop() TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO)
#define i2c_data_out(data)	TWDR = (data); TWCR = (1<<TWINT) | (1<<TWEN)

// wait for I2C state change with timout (ISR must be enabled!)
uint8_t i2c_wait(uint16_t timeout_ms)
{
	if(timeout_ms)
	{
		// timeout mode
		timeout_set(timeout_ms);	
		while(!bit_is_set(TWCR,TWINT) && !timeout_done());
		return(timeout_done());
	}
	else
		while(!bit_is_set(TWCR,TWINT)); // no timeout mode

	return(0);
}

// I2C write routine with timeouts (ISR must be enabled)
uint8_t i2c_write_data(uint8_t address,uint8_t *data,uint8_t count,uint16_t timeout_ms)
{
	// start
	i2c_start();	
	if(i2c_wait(timeout_ms))
	{
		// timeout
		i2c_stop();
		return(1);
	}	
	if((TW_STATUS) != TW_START)
	{
		i2c_stop();
		return(1);
	}

	// send address + write
	i2c_data_out(address<<1);
	if(i2c_wait(timeout_ms))
	{
		// timeout
		i2c_stop();
		return(2);
	}	
	if((TW_STATUS) != TW_MT_SLA_ACK)
	{
		i2c_stop();
		return(2);		
	}

	// transmit data bytes
	for(uint8_t k = 0;k < count;k++) 
	{
		// send byte
		i2c_data_out(data[k]);
		if(i2c_wait(timeout_ms))		
		{
			// timeout
			i2c_stop();
			return(3);
		}		
		if((TW_STATUS) != TW_MT_DATA_ACK)
		{
			i2c_stop();
			return(3);			
		}
	}

	// stop
	i2c_stop();

	
	return(0);
}

// timeout in case I2C fails [ms]
#define I2C_TIMEOUT 3000

// initialization of DAC
uint8_t LTC2631_init(uint8_t address)
{
	uint8_t data[3] = {0,0,0};
	data[0] = 0x60;
	return(i2c_write_data(address, data, 3, I2C_TIMEOUT));
}

// set DAC value
uint8_t LTC2631_set(uint8_t address, uint16_t value)
{
	uint8_t data[3];

	// build command
	data[0] = 0x30;	// write and set
	data[1] = (uint8_t)(value>>4);
	data[2] = (uint8_t)((value<<4)&0x00F0u);
	
	// send it
	return(i2c_write_data(address, data, 3, I2C_TIMEOUT));
}


//----------------------------------------------------------------------------------
// MAIN
//----------------------------------------------------------------------------------

// I2C clock rate
#define I2C_FREQ 10000.0
#define I2C_ADDR_PWR 0x10
#define I2C_ADDR_POT 0x12

// button flags in edge event register
#define BTN_BIAS 0
#define BTN_RANGE 1
#define BTN_IND 2
#define BTN_COM 3
#define BTN_POWER 4

// GUI and REMOTE command flags
#define CMD_RELAYS 0 // modify relays
#define CMD_POT 1 // modify potential DAC
#define CMD_PWR 2 // modify power DAC
#define CMD_PWR_WAIT 3 // wait for power DAC response
#define CMD_ACK 4 // send remote command ACK after command executed


int main(void)
{
	// disable WDT if enabled (sometimes it fucks up ...)
	MCUSR = 0;
  	wdt_disable();

	// disable all ports
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;

	// enable MUX common lines
	sbi(MB0d,MB0);
	sbi(MB1d,MB1);
	sbi(ML0d,ML0);
	sbi(ML1d,ML1);

	// power bias state
	sbi(PBIASd,PBIAS);
	sbi(PBIASp,PBIAS);

	// FAN to LOW
	sbi(PFANd,PFAN);
	cbi(PFANp,PFAN);
	
	// timer 0 tick
	TCCR0A = (2<<WGM00);
	TCCR0B = (2<<CS00);
	OCR0A = (uint8_t)(F_CPU/8.0*MTICK) - 1;
	TIMSK0 |= (1<<OCIE0A);

	// I2C as master
	TWCR = (1<<TWEN)|(1<<TWEA);
	TWBR = (uint8_t)((F_CPU - 16.0*I2C_FREQ)/(2.0*I2C_FREQ));
	TWSR = (0<<TWPS0);
	PORTC |= (1<<PC4) | (1<<PC5);

	// init UART
	serial_init();

	// init SPI for relay driver
	spec_relay_init();
		
	// enable ISR
	sei();

	// put relays to default state
	set_relays(&ctrl,1);
	
	// default voltage set
	#define u2dac(voltage) ((uint16_t)(4095.0*voltage/5.0))
	uint16_t volts[4] = {u2dac(0.0), u2dac(3.0), u2dac(4.0), u2dac(5.0)};


	while(1)
	{

		// HW setup modified flags
		uint8_t modf = 0;
		
		
		// --- BUTTON PROCESSING ---
		// get and clear button event flags (this should be atomic)
		uint8_t flags;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			flags = btns_down;
			btns_down = 0x00;
		}


		if(bit_is_set(flags,BTN_BIAS))
		{
			// cycle predefined bias levels
			ctrl.bias++;
			if(ctrl.bias > 3)
				ctrl.bias = 0;	

			// set mew voltages
			ctrl.bias_pot = volts[ctrl.bias];
			ctrl.bias_pwr = ctrl.bias_pot;
									
			modf |= (1<<CMD_POT) | (1<<CMD_PWR);
		}
		else if(bit_is_set(flags,BTN_RANGE))
		{
			// cycle ranges
			ctrl.range++;
			if(ctrl.range > 1)
				ctrl.range = 0;

			sbi(modf,CMD_RELAYS);
			
		}
		else if(bit_is_set(flags,BTN_IND))
		{
			// toggle polarity
			ctrl.ind++;
			if(ctrl.ind > 1)
				ctrl.ind = 0;

			sbi(modf,CMD_RELAYS);
		}
		else if(bit_is_set(flags,BTN_COM))
		{
			// toggle pot-cur joint switch
			ctrl.com++;
			if(ctrl.com > 1)
				ctrl.com = 0;

			sbi(modf,CMD_RELAYS);
		}
		else if(bit_is_set(flags,BTN_POWER))
		{
			// toggle current path bias source
			ctrl.power++;
			if(ctrl.power > 1)
				ctrl.power = 0;	

			if(ctrl.power)
			{
				// power was just turned on - wait for actual power up				
				modf |= (1<<CMD_PWR) | (1<<CMD_PWR_WAIT);
			}
			sbi(modf,CMD_RELAYS);
		}

		// --- REMOTE PROCESSING ---
		char *par;
		if(serial_decode(&par))
		{
			// --- command detected ---

			if(!strcmp_P(rxd,PSTR("MODE")))
			{
				// "MODE mode" to set polarity of potential to "mode"

				if(!par)
					serial_error(SCPI_ERR_tooFewParameters,NULL,SCPI_ERR_STORE); // missing parameter
				else if(!strcmp_P(par,PSTR("IND")))
					ctrl.ind = 1;
				else if(!strcmp_P(par,PSTR("CAP")))
					ctrl.ind = 0;
				else
					serial_error(SCPI_ERR_wrongParamType,PSTR(" Only IND or CAP supported."),SCPI_ERR_STORE); // invalid

				sbi(modf,CMD_RELAYS);

			}
			else if(!strcmp_P(rxd,PSTR("RANGE")))
			{
				// "RANGE range" to set range (1 or 2)

				if(!par)
					serial_error(SCPI_ERR_tooFewParameters,NULL,SCPI_ERR_STORE); // missing parameter
				else
				{
					uint8_t val = atoi(par);
					if(val < 1 || val > 2)
						serial_error(SCPI_ERR_wrongParamType,PSTR(" Only ranges 1 and 2 supported."),SCPI_ERR_STORE); // invalid
					else
						ctrl.range = val - 1;

					sbi(modf,CMD_RELAYS);
				}
					
			}
			else if(!strcmp_P(rxd,PSTR("COMMON:STATE")))
			{
				// "COMMON:STATE state" to set potential and bias grounds connections (0/1)

				if(!par)
					serial_error(SCPI_ERR_tooFewParameters,NULL,SCPI_ERR_STORE); // missing parameter
				else
				{
					if(!strcmp_P(par,PSTR("ON")) || !strcmp_P(par,PSTR("1")))
						ctrl.com = 1;
					else if(!strcmp_P(par,PSTR("OFF")) || !strcmp_P(par,PSTR("0")))
						ctrl.com = 0;
					else
						serial_error(SCPI_ERR_wrongParamType,PSTR(" Only ON/OFF or 1/0 supported."),SCPI_ERR_STORE); // invalid

					sbi(modf,CMD_RELAYS);
				}
			}
			else if(!strcmp_P(rxd,PSTR("POWER:STATE")))
			{
				// "POWER:STATE state" to set enable state of power bias

				if(!par)
					serial_error(SCPI_ERR_tooFewParameters,NULL,SCPI_ERR_STORE); // missing parameter
				else
				{
					if(!strcmp_P(par,PSTR("ON")) || !strcmp_P(par,PSTR("1")))
						ctrl.power = 1;
					else if(!strcmp_P(par,PSTR("OFF")) || !strcmp_P(par,PSTR("0")))
						ctrl.power = 0;
					else
						serial_error(SCPI_ERR_wrongParamType,PSTR(" Only ON/OFF or 1/0 supported."),SCPI_ERR_STORE); // invalid

					if(ctrl.power)
					{
						// power was just turned on - wait for actual power up				
						sbi(modf,CMD_PWR);
						sbi(modf,CMD_PWR_WAIT);
					}					
					
					sbi(modf,CMD_RELAYS);
				}
			}
			else if(!strcmp_P(rxd,PSTR("BIAS:VOLT")))
			{
				// "BIAS:VOLT level" to set bias voltage in [mV]
				// this sets the same bias to both sources

				if(!par)
					serial_error(SCPI_ERR_tooFewParameters,NULL,SCPI_ERR_STORE); // missing parameter
				else
				{
					uint16_t mvlev = atol(par);
					if(mvlev > 5000)
						serial_error(SCPI_ERR_wrongParamType,PSTR(" Maximum supported level is 5000mV."),SCPI_ERR_STORE); // invalid range

					// custom bias set
					ctrl.bias = -1;
					
					// set new voltages to both sources
					ctrl.bias_pot = (uint16_t)(((uint32_t)mvlev*4095ul)/5000ul);
					ctrl.bias_pwr = ctrl.bias_pot;

					// update DAC(s)
					sbi(modf,CMD_PWR);
					sbi(modf,CMD_POT);
				}
			}
			else if(!strcmp_P(rxd,PSTR("BIAS:POT:VOLT")))
			{
				// "BIAS:POT:VOLT level" to set bias voltage in [mV]
				// this sets level to potential only

				if(!par)
					serial_error(SCPI_ERR_tooFewParameters,NULL,SCPI_ERR_STORE); // missing parameter
				else
				{
					uint16_t mvlev = atol(par);
					if(mvlev > 5000)
						serial_error(SCPI_ERR_wrongParamType,PSTR(" Maximum supported level is 5000mV."),SCPI_ERR_STORE); // invalid range

					// custom bias set
					ctrl.bias = -1;
					
					// set new voltages to both sources
					ctrl.bias_pot = (uint16_t)(((uint32_t)mvlev*4095ul)/5000ul);
					
					// update DAC(s)
					sbi(modf,CMD_POT);

				}
			}
			else if(!strcmp_P(rxd,PSTR("BIAS:PWR:VOLT")))
			{
				// "BIAS:PWR:VOLT level" to set bias voltage in [mV]
				// this sets level to power only

				if(!par)
					serial_error(SCPI_ERR_tooFewParameters,NULL,SCPI_ERR_STORE); // missing parameter
				else
				{
					uint16_t mvlev = atol(par);
					if(mvlev > 5000)
						serial_error(SCPI_ERR_wrongParamType,PSTR(" Maximum supported level is 5000mV."),SCPI_ERR_STORE); // invalid range

					// custom bias set
					ctrl.bias = -1;
					
					// set new voltages to both sources
					ctrl.bias_pwr = (uint16_t)(((uint32_t)mvlev*4095ul)/5000ul);
					
					// update DAC(s)
					sbi(modf,CMD_PWR);

				}
			}
			else if(!strcmp_P(rxd,PSTR("FAN")))
			{
				// "FAN state" to set potential and bias grounds connections (LO/HI)

				if(!par)
					serial_error(SCPI_ERR_tooFewParameters,NULL,SCPI_ERR_STORE); // missing parameter
				else
				{
					if(!strcmp_P(par,PSTR("LO")) || !strcmp_P(par,PSTR("LOW")))
						cbi(PFANp,PFAN)
					else if(!strcmp_P(par,PSTR("HIGH")) || !strcmp_P(par,PSTR("HI")))
						sbi(PFANp,PFAN)
					else
						serial_error(SCPI_ERR_wrongParamType,PSTR(" Only LO/HI or LOW/HIGH supported."),SCPI_ERR_STORE); // invalid
				}
			}
			else if(!strcmp_P(rxd,PSTR("*OPC?")))
			{
				// "*OPC?" returns "+1" when ready to execute new command
				//  note: this whole code executes in a loop, so the commands are executed
				//        before we get here again, so OPC? should always return 1 here
				serial_tx_cstr(PSTR("+1\n"));
			}
			else if(!strcmp_P(rxd,PSTR("*RST")))
			{
				// *RST - restarts controller
				cli();
				//serial_flush_command();
				wdt_enable(WDTO_15MS);
				while(1);				
			}
			else if(!strcmp_P(rxd,PSTR("*IDN?")))
			{
				// "*IDN?" to return IDN string
				serial_tx_cstr(PSTR("Z-Simulator Bias Source V1.1, s.n. 20200214\n"));
			}
			else if(!strcmp_P(rxd,PSTR("SYST:ERR?")))
			{
				// "SYST:ERR?" return error buffer
				serial_error(SCPI_ERR_noError,NULL,SCPI_ERR_SEND);				
			}			
			else
			{
				// invalid
				serial_error(SCPI_ERR_undefinedHeader,NULL,SCPI_ERR_STORE);
			}

			// command processed
			serial_flush_command();
			

		}


		
		
		// --- UPDATE HW STATE ---

		// set DCDC state (no need for flags)
		bcopy_v(PBIASp,PBIAS, !ctrl.power, 0);

		// update power DAC
		if(bit_is_set(modf,CMD_PWR))
		{
			// wait for I2C ACK response - that should indicate power is up
			if(bit_is_set(modf,CMD_PWR_WAIT))
				while(LTC2631_init(I2C_ADDR_PWR) != 0);

			// set value
			LTC2631_init(I2C_ADDR_PWR);
			LTC2631_set(I2C_ADDR_PWR, ctrl.bias_pwr);
		}
		
		// update potential DAC
		if(bit_is_set(modf,CMD_POT))
		{		
			LTC2631_init(I2C_ADDR_POT);
			LTC2631_set(I2C_ADDR_POT, ctrl.bias_pot);
		}

		// update relays
		if(bit_is_set(modf,CMD_RELAYS))
			set_relays(&ctrl,1);

		// send remote ACK? ###note: obsolete
		/*if(bit_is_set(modf,CMD_ACK))
			serial_tx_cstr(PSTR("0\n"));*/
			
		
		// clear HW mod flags
		modf = 0;

	}

}


