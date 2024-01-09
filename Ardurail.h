/* Ardurail 1.0

   Copyright (C) 2012 Jan Weller jan.weller@gmx.de Rewritten
   and reorganized 2013 by Holger Wirtz <dcoredump@googlemail.com>

   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free 
   Software Foundation; either version 3 of the License, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful, but WITHOUT 
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

 */

#ifndef _ARDURAIL_H
#define _ARDURAIL_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#if defined(__AVR_ATmega328P__)
#define __UNO__ 1
#define __BOARD__ "Arduino Uno"
#elif defined(__AVR_ATmega32U4__)
#define __LEONARDO__ 1
#define __BOARD__ "Arduino Leonardo"
#elif defined(__AVR_ATmega2560__)
#define __MEGA__ 1
#define __BOARD__ "Arduino Mega 2560"
#else
#warning Unsupported board.
//#error Unsupported board. Please adjust library.
#endif

#include <avr/interrupt.h>
#include <avr/io.h>

// 
// ######################### DEFAULTS #############################
// 

#define __LED13__ 0						// used for debugging
#define USE_S88 1 						// S88 enabled
#define USE_SHORT_INTERRUPT 0			// pin 2 only!


// Port for S88-Module
#define S88_Data	A0
#define S88_Clock	A1
#define S88_PS		A2
#define S88_Reset	A3

// setup maximum values
#define MAX_REFRESH    		80 			// 80 locomotives
#define MAX_TRACK_COMMANDS	80 			// 80 commands can be buffered
#define MAX_S88        		31			// 31 modules of 2 bytes

// 
// DO NOT CHANGE ANYTHING BEHIND THIS LINE IF YOU DO NOT KNOW WHAT YOU ARE
// DOING!!!
// 

// 
// ######################### DEFINITIONS #############################
// 

// States in DCC-ISR
#define PREAMBLE	0
#define SEPERATOR	1
#define SENDBYTE	2
#define TERMINATOR	3
#define BREAK		4

// Status for the next telegram
#define MM_LOCO 	0
#define MM_DEVICE 	1   
#define DCC 		2


enum decoder_type
{
	DEF,
    M1,
    M2,
    M2_28_80A,
    M2_14_256A,
    M2_28_256A,
    //MFX,
	DCC_28,
	DCC_128
};

#define M3 M2_28_80A
#define M4 M2_14_256A
#define M5 M2_28_256A

// DCC packet
struct DCC_packet
{
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
	uint8_t byte4;
	uint8_t byte5;
	uint8_t byte6;
	uint8_t used_bytes;
};

// refresh packet (packets which are sent out constantly) 
// and all informations abaut locos
struct Refresh
{
	uint8_t address;
	uint8_t type:4;
	uint8_t speed;
	uint8_t dir:1;
	uint8_t changedir:1;
	uint8_t function:1;
	uint8_t f1:1;
	uint8_t f2:1;
	uint8_t f3:1;
	uint8_t f4:1;
	uint8_t f5:1;
	uint8_t f6:1;
	uint8_t f7:1;
	uint8_t f8:1;
	uint8_t f9:1;
	uint8_t f10:1;
	uint8_t f11:1;
	uint8_t f12:1;
	uint8_t f13:1;
	uint8_t f14:1;
	uint8_t f15:1;
	uint8_t f16:1;
	uint8_t ready:1;
};

// packet for Trackswitch
struct MMTrackswitch
{
  uint8_t address;
  uint8_t subaddress:3;
  uint8_t state:1;
  uint8_t ready:1;
};


// timer frequency is  16 MHz / 8 = 2 MHz
#define TIMER_MM_DEVICE		233	// 13 usec for devices
#define TIMER_MM_LOCO		214	// 26 usec for locomotives

#define TIMER_DCC			145	// 58 usec for dcc

// wait timer definitons
#define TIME_T1				40	// 1248 us between data packets (inside a double packet)
#define TIME_T2				162	// 4,2 ms between two double packets
#define TIME_T3				235	// 235 6,0 ms after two double packets

#define TIME_DCC_BREAK		160	// 5,0 ms between two dcc packets

// a set contains 8 messages (8x2 double packets)
#define MAXMSG  8

// 
// ######################### CLASS #############################
// 

class Ardurail
{
  public:
	Ardurail();
	void init(byte pin);
	void init(byte pin,byte shortcut_pin,byte go_signal_pin);
	void S88(void);
	byte get_S88(byte modul, byte area);
	boolean get_power(void);
	void set_power(boolean power);
	boolean get_halt(void);
	void set_halt(void);
	void set_go(void);
	void set_stop(void);
	boolean get_shortcut(void);
	void add_loco(byte addr);
	void add_loco(byte addr,byte protocol);
	byte get_loco(byte addr);
	byte get_speed(byte addr);
	byte get_speedsteps(byte addr);
	void set_speed(byte addr, byte speed);
	boolean get_dir(byte addr);
	void set_dir(byte addr, boolean direction);
	void change_dir(byte addr);
	boolean get_function(byte addr, byte function_nr);
	void set_function(byte addr, byte function, boolean state);
	void emergency_stop(byte addr);
	void reset_locos(void);
	void commit(void);
	void commit(byte addr);
	void set_track(byte addr, byte sub_addr, boolean state);
	static inline void handle_timer2_interrupt(void);
	static inline void handle_short_interrupt(void);
#ifdef USE_S88
	byte _s88_max_s88;
#endif

  protected:
  	uint8_t _shortcut_signal_pin;
	uint8_t _go_signal_pin;
	boolean _power;
	boolean _halt;
	boolean _short;
	volatile struct Refresh _refresh[MAX_REFRESH];		// maximum of MAX_REFRESH (80) locomotives
	uint8_t _refresh_max;								// current number of locomotives
	struct MMTrackswitch tswitch[MAX_TRACK_COMMANDS];	// buffer of MAX_TRACK_COMMANDS (80) for trackswitchcommands
	int8_t _waiting_track_commands;
	
#ifdef USE_S88
	byte _s88_data_pin;
	byte _s88_clock_pin;
	byte _s88_ps_pin;
	byte _s88_reset;
	uint8_t _s88_state[MAX_S88 * 2];
#endif

	// vars for TIMER2 interrupt handler
	volatile boolean _break_t1;				// flag for break between the data packets
	volatile boolean _break_t2;				// flag for break between double packets
	volatile boolean _break_t3;				// flag for break after two double packets
	volatile byte _step;					// step counter
	volatile byte _bit_counter;				// counts bits in each double packet (9 Trits = 18 Bits)
	volatile byte _packet_counter;			// counter for double packets (there are every time two of them (STAR WARS)
	volatile byte _msg_counter;				// counts the messages
	volatile uint8_t _timer;				// timer setup value
	volatile uint8_t _outbit:1;				// bit to write out
	volatile uint8_t _mode:2; 				// 0 = MM_LOCO 1 = MM_DEVICE 2 = DCC 
	volatile uint8_t _dcc_state;			// state of DCC: PREAMBL, SEPERATOR, SENDBYTE, TERMINATOR, BREAK

	volatile int8_t _refresh_pointer;		// the next loco
	volatile int8_t _updated_loco;			// the loco has new information to send out first (with priority)

	volatile uint32_t _msg[MAXMSG];			// MM1-5 Outputregister
	volatile struct DCC_packet _msg_dcc[MAXMSG];	// DCC Outputregister
	
	// vars for digital signal
	volatile uint8_t _digital_signal;
	volatile uint8_t* _digital_reg;

	
	uint8_t	_s88modulcycleaddr;

	void _generate(byte index);
	void _generate_m1(byte index);
	void _generate_m2(byte index);
	void _generate_m3(byte index);
	void _generate_m4(byte index);
	void _generate_m5(byte index);
	void _generate_dcc_28(uint8_t index);
	void _generate_dcc_128(uint8_t index);
	void _generate_trackswitch(byte address, byte subaddress, boolean on);
	byte _calc_function_f4(byte speed, boolean function);
	byte _calc_function_f3(byte speed, boolean function);
	byte _calc_function_f2(byte speed, boolean function);
	byte _calc_function_f1(byte speed, boolean function);
	byte _calc_speed(byte speed);
	byte _calc_speed_and_direction(byte speed, boolean dir);
	uint8_t _calc_dcc28_speed_and_direction(uint8_t speed, uint8_t dir);
	uint8_t _calc_dcc128_speed(uint8_t speed);
	uint8_t _calc_dcc128_speed_and_direction(uint8_t speed, uint8_t dir);
	byte _calc_trinary_address(byte address);
	uint8_t _calc_dcc_address(uint8_t address);
	uint8_t _lsb_msb_replacement(uint8_t inbyte);
	void _write_mm_output_register(uint8_t message, uint8_t address, uint8_t function, uint8_t data);
	void _commit(byte index);
	int8_t _create_refresh(byte address);
	int8_t _find_refresh(byte address);
	void _timersetup(void);
	void _do_timer2_irq_MM(void);
	void _do_timer2_irq_DCC(void);
	void _do_update(void);
	static Ardurail *_active_object;
};

// 
// ######################### FUNCTIONS #############################
// 

#ifdef __LED13__
inline void __led13__(boolean state);
#endif
#endif
