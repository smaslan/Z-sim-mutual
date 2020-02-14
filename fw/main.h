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

#ifndef MAIN_H
#define MAIN_H

// general macros
#define sbi(port,pin) {port|=(1<<pin);}
#define cbi(port,pin) {port&=~(1<<pin);}
#define min(a,b) ((a<b)?(a):(b))
#define max(a,b) ((a>b)?(a):(b))
#define bcopy(dreg,dpin,srreg,srpin) if(bit_is_set(srreg,srpin)){sbi(dreg,dpin);}else{cbi(dreg,dpin);}
#define bcopy_v(dreg,dpin,srreg,srpin) if((srreg)&(1<<srpin)){sbi(dreg,dpin);}else{cbi(dreg,dpin);}


#endif
