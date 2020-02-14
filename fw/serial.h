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

#ifndef SERIAL_H
#define SERIAL_H

// --- USART config ---
#define USART_BAUDRATE 9600 /* baud rate (do not set too high!) */
#define RX_BUF_SZ 128 /* receive buffer size (max 255!) */
#define RX_DONE 0 /* command received flag */


// --- SCPI errors ---
// serial error flags
#define SCPI_ERR_STORE 1 /* store error message to buffer */
#define SCPI_ERR_SEND 2 /* send error from internal buffer */

// return SCPI error
#define SCPI_ERR_noError 0l
#define SCPI_ERR_undefinedHeader -113l
#define SCPI_ERR_wrongParamType -104l
#define SCPI_ERR_tooFewParameters -109l
#define SCPI_ERR_std_mediaProtected -258l



// rx data buffer
char rxd[RX_BUF_SZ];
// rx data position
volatile uint8_t rxd_ptr;
// flags
volatile uint8_t rxd_stat;

// prototypes
void serial_init(void);
uint8_t serial_decode(char **par);
void serial_flush_command(void);
void serial_tx_byte(uint8_t byte);
void serial_tx_cstr(const char *str);
void serial_tx_str(char *str);
void serial_error(int16_t err,const char *info,uint8_t mode);


#endif
