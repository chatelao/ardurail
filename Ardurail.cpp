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

#include "Ardurail.h"
#include <avr/pgmspace.h>

#define SERIALPORT Serial

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))

void _short_irq()
{
	Ardurail::handle_short_interrupt();
}

// 
// ######################### GLOBAL VARS #############################
// 
Ardurail *Ardurail::_active_object = 0;
unsigned long _switch_time;
// 
// ######################### PUBLIC METHODS #############################
// 

Ardurail::Ardurail(void)
{
	_active_object = 0;
	_power = false;
	_halt = false;
	_short = false;
	_refresh_pointer = 0;
	_refresh_max = 0;
	_updated_loco = -1;
	_timer = TIMER_MM_LOCO;
	_break_t1 = false;
	_break_t2 = false;
	_break_t3 = false;
	_step = 0;
	_bit_counter = 0;
	_packet_counter = 0;
	_msg_counter = 0;
	_waiting_track_commands = 0;
	_shortcut_signal_pin = -1; // default: not used
	_go_signal_pin = -1; // default: not used
}

const uint8_t extended_MM_addr[] PROGMEM =
{
  0b00000000, 0b01000000, 0b01100000, 0b10010111, 0b01110000,
  0b01001000, 0b01101000, 0b01011000, 0b01111000, 0b01000100,
  0b01100100, 0b01010100, 0b01110100, 0b01001100, 0b01101100,
  0b01011100, 0b01111100, 0b01000010, 0b01100010, 0b01010010,
  0b01110010, 0b01001010, 0b01101010, 0b01011010, 0b01111010,
  0b01000110, 0b01100110, 0b01010110, 0b01110110, 0b01001110,
  0b01101110, 0b01011110, 0b01111110, 0b01000001, 0b01100001,
  0b01010001, 0b01110001, 0b01001001, 0b01101001, 0b01011001,
  0b01111001, 0b01000101, 0b01100101, 0b10011111, 0b01110101,
  0b01001101, 0b01101101, 0b01011101, 0b01111101, 0b01000011,
  0b01100011, 0b01010011, 0b01110011, 0b01001011, 0b01101011,
  0b01011011, 0b01111011, 0b01000111, 0b01100111, 0b01010111,
  0b01110111, 0b01001111, 0b01101111, 0b01011111, 0b01111111,
  0b00010000, 0b00011000, 0b00010100, 0b00011100, 0b00010010,
  0b00011010, 0b00010110, 0b00011110, 0b00010001, 0b00011001,
  0b00010101, 0b00011101, 0b00010011, 0b00011011, 0b00010111,
  0b00011111, 0b11010000, 0b11011000, 0b11010100, 0b11011100,
  0b11010010, 0b11011010, 0b11010110, 0b11011110, 0b11010001,
  0b11011001, 0b11010101, 0b11011101, 0b11010011, 0b11011011,
  0b11010111, 0b11011111, 0b10010000, 0b10011000, 0b10010100,
  0b10011100, 0b10010010, 0b10011010, 0b10010110, 0b10011110,
  0b10010001, 0b10011001, 0b10010101, 0b10011101, 0b10010011,
  0b10011011, 0b01010000, 0b01010101, 0b00000100, 0b00000110,
  0b00000101, 0b00000111, 0b11000100, 0b11000110, 0b11000101,
  0b11000111, 0b10000100, 0b10000110, 0b10000101, 0b10000111,
  0b00110100, 0b00110110, 0b00110101, 0b00110111, 0b11110100,
  0b11110110, 0b11110101, 0b11110111, 0b10110100, 0b10110110,
  0b10110101, 0b10110111, 0b00100100, 0b00100110, 0b00100101,
  0b00100111, 0b11100100, 0b11100110, 0b11100101, 0b11100111,
  0b10100100, 0b10100110, 0b10100101, 0b10100111, 0b00000001,
  0b11000001, 0b10000001, 0b00110001, 0b11110001, 0b10110001,
  0b00100001, 0b11100001, 0b10100001, 0b00001101, 0b11001101,
  0b10001101, 0b00111101, 0b11111101, 0b10111101, 0b00101101,
  0b11101101, 0b10101101, 0b00001001, 0b11001001, 0b10001001,
  0b00111001, 0b11111001, 0b10111001, 0b00101001, 0b11101001,
  0b10101001
};

void Ardurail::init(byte pin)
{
	_active_object = this;

#ifdef __LED13__
	pinMode(13,OUTPUT);
	__led13__(true);
#endif

	// Pin->Port
	pinMode(pin, OUTPUT);
	_digital_signal = digitalPinToBitMask(pin);
	_digital_reg = portOutputRegister(digitalPinToPort(pin));

	// Portsetup for S88-Decoder
#ifdef USE_S88
	pinMode(S88_Data, INPUT);
	pinMode(S88_Clock, OUTPUT);
	digitalWrite(S88_Clock, LOW);
	pinMode(S88_PS, OUTPUT);
	digitalWrite(S88_PS, HIGH);
	pinMode(S88_Reset, OUTPUT);
	digitalWrite(S88_Reset, LOW);
#endif

	// if we have no loco, fill with Idlepackets in MM Loco Mode
    for (uint8_t i = 0; i < MAXMSG; i++)
	{
		_write_mm_output_register(i, 0b10101010, 0b00, 0b00000000);
	}

	// dcc idle packets too
	for (uint8_t i = 0; i < MAXMSG; i++)
	{
		_msg_dcc[i].byte1 = 0xff;
		_msg_dcc[i].byte2 = 0;
		_msg_dcc[i].byte3 = 0xff;
		_msg_dcc[i].byte4 = 0;
		_msg_dcc[i].byte5 = 0;
		_msg_dcc[i].byte6 = 0;
		_msg_dcc[i].used_bytes = 3;
	}

	_mode = MM_LOCO;
	_timer = TIMER_MM_LOCO;

	// Timerstart
	TCCR2A = 0x00;				// Timer2 arbeitet als normaler Timer
	TCCR2B = (1 << CS21);		// Timer2 Control Reg B: Timer Prescaler 8
	TIMSK2 = (1 << TOIE2);		// Timer2 Overflow Interrupt Enable 

	sei();						// enable all Interrupts
	delay(50);

#ifdef __LED13__
	pinMode(13,OUTPUT);
	__led13__(false);
#endif
}

void Ardurail::init(byte digital_signal_pin,byte shortcut_pin,byte go_signal_pin)
{
	_shortcut_signal_pin = shortcut_pin;
	_go_signal_pin = go_signal_pin;

	if (shortcut_pin == 2)
	{
		attachInterrupt(1, _short_irq, FALLING);
	}
	pinMode(_shortcut_signal_pin, INPUT);
	digitalWrite(_shortcut_signal_pin, HIGH);		// Pullup
	pinMode(_go_signal_pin, OUTPUT);
	digitalWrite(_go_signal_pin, LOW);
	init(digital_signal_pin);
	_s88_max_s88 = MAX_S88;
}

void Ardurail::S88(void)
{
#ifdef USE_S88
	uint8_t S88value = 0;

	if (_s88modulcycleaddr == 0) 
	{
		digitalWrite(S88_Clock, HIGH);
		delayMicroseconds(15);
		digitalWrite(S88_Clock, LOW);
		delayMicroseconds(15);

		digitalWrite(S88_Reset, HIGH);
		delayMicroseconds(15);
		digitalWrite(S88_Reset, LOW);
	}

	digitalWrite(S88_PS, LOW);
	delayMicroseconds(15);

	// 2 Bytes auslesen und weiterschieben
	for (uint8_t byte_nr = 0; byte_nr <= 1; byte_nr++)
	{

		for (uint8_t clock = 0; clock <= 7; clock++)
		{
			S88value |= (digitalRead(S88_Data) << (7 - clock));
			
			digitalWrite(S88_Clock, HIGH);
			delayMicroseconds(10);
			digitalWrite(S88_Clock, LOW);
			delayMicroseconds(10);
		}
		_s88_state[_s88modulcycleaddr * 2 + byte_nr] = S88value;
		S88value = 0;
	}

	_s88modulcycleaddr++;
	if (_s88modulcycleaddr == _s88_max_s88)
	{
		_s88modulcycleaddr = 0;
		digitalWrite(S88_PS, HIGH);
	}
#else
	;
#endif
}

byte Ardurail::get_S88(byte modul, byte byte_nr)
{
#ifdef USE_S88
	return ((byte)_s88_state[modul * 2 + byte_nr]);
#else
	return ((byte)0x00);
#endif
}

boolean Ardurail::get_power(void)
{
	return (_power);
}

void Ardurail::set_power(boolean power)
{
	_power = power;
	if(power)
	{
		set_go();
	 	_halt = false;
		_short = false;
	}
	else
	  set_stop();
}

boolean Ardurail::get_halt(void)
{
	return(_halt);
}

void Ardurail::set_halt(void)
// not ready
{
	_halt = true;
	for (int i = 0; i <= _refresh_max; i++)
	{
		emergency_stop(_refresh[i].address);
	}
}

void Ardurail::set_go(void)
{
	if((_go_signal_pin >= 0) && (get_shortcut() == false))
		digitalWrite(_go_signal_pin, HIGH);
}

void Ardurail::set_stop(void)
{
	if(_go_signal_pin >= 0)
		digitalWrite(_go_signal_pin, LOW);
}

boolean Ardurail::get_shortcut(void)
{
	if(_shortcut_signal_pin >= 0)
	{
		if (digitalRead(_shortcut_signal_pin) == HIGH)
		{
			_short = true;
		}
		else
		{
			_short = false;
		}
	}
	return (_short);
}

void Ardurail::add_loco(byte addr)
{
	add_loco(addr,M2);
}

void Ardurail::add_loco(byte addr, byte protocol)
{
	if(_find_refresh(addr)<0)
	{
		int8_t refresh_position = _create_refresh(addr);
      _refresh[refresh_position].type = protocol;
      _refresh[refresh_position].speed = 0;
      _refresh[refresh_position].dir = true;
      
      byte i=0;
		for(i=0;i<5;i++)
		{
			set_function(addr,i,false);
		}
	}
}

byte Ardurail::get_loco(byte addr)
{
	int8_t refresh_position = (-1);
	refresh_position = _find_refresh(addr);
	
	if (refresh_position >= 0)
	{
		return(_refresh[refresh_position].type);
	}
	else
	{
		return(-1);
	}
}

byte Ardurail::get_speed(byte addr)
{
	int8_t refresh_position = (-1);
	refresh_position = _find_refresh(addr);
	
	if (refresh_position >= 0)
	{
		return (_refresh[refresh_position].speed);
	}
	else
	{
		return (-1);
	}
}

byte Ardurail::get_speedsteps(byte addr)
{
	switch(_refresh[_find_refresh(addr)].type)
	{
		case M1:
		case M2:
		case M2_14_256A:
			return(14);
		case M2_28_80A:
		case M2_28_256A:
			return(28);
		default:
			return(-1);
	}
}

void Ardurail::set_speed(byte addr, byte speed)
{
	int8_t refresh_position = -1;

	if (_halt)
		return;

	refresh_position = _create_refresh(addr);
	if (refresh_position >= 0)
	{
		_refresh[refresh_position].ready = false;
		_refresh[refresh_position].speed = speed;
	}
}

boolean Ardurail::get_dir(byte addr)
{
	int8_t refresh_position = -1;
	
	refresh_position = _find_refresh(addr);
	if (refresh_position >= 0)
	{
	  return(_refresh[refresh_position].dir);
	}
	else
	{
	  return (-1);
	}
}

void Ardurail::set_dir(byte addr, boolean direction)
{
	int8_t refresh_position = -1;

	refresh_position = _find_refresh(addr);
	if (refresh_position >= 0)
	{
		_refresh[refresh_position].ready = false;
		if (_refresh[refresh_position].dir != direction)
		{
		  _refresh[refresh_position].ready = false;
		  _refresh[refresh_position].changedir = true;
		  _refresh[refresh_position].dir = direction;
		}
	}
}

void Ardurail::change_dir(byte addr)
{
	int8_t refresh_position = -1;

	refresh_position = _find_refresh(addr);
	if (refresh_position >= 0)
	{
		_refresh[refresh_position].ready = false;
		_refresh[refresh_position].changedir = true;
		_refresh[refresh_position].dir = ~(_refresh[refresh_position].dir);
	}
}

boolean Ardurail::get_function(byte addr, byte function_nr)
{
	int8_t refresh_position = _find_refresh(addr);
	boolean function_value = false;
	
	if (refresh_position >= 0)
	{
		switch (function_nr)
		{
			case 0:
				function_value = (boolean)(_refresh[refresh_position].function);
				break;
			case 1:
				function_value = (boolean)(_refresh[refresh_position].f1);
				break;
			case 2:
				function_value = (boolean)(_refresh[refresh_position].f2);
				break;
			case 3:
				function_value = (boolean)(_refresh[refresh_position].f3);
				break;
			case 4:
				function_value = (boolean)(_refresh[refresh_position].f4);
				break;
			case 5:
				function_value = (boolean)(_refresh[refresh_position].f5);
				break;
			case 6:
				function_value = (boolean)(_refresh[refresh_position].f6);
				break;
			case 7:
				function_value = (boolean)(_refresh[refresh_position].f7);
				break;
			case 8:
				function_value = (boolean)(_refresh[refresh_position].f8);
				break;
		}
	}
	return(function_value);
}

void Ardurail::set_function(byte addr, byte function, boolean state)
{
	int8_t refresh_position = -1;

	if ((refresh_position = _find_refresh(addr)) >= 0)
	{
		_refresh[refresh_position].ready = false;
		switch (function)
		{
			case 0:
				_refresh[refresh_position].function = state;
				break;
			case 1:
				_refresh[refresh_position].f1 = state;
				break;
			case 2:
				_refresh[refresh_position].f2 = state;
				break;
			case 3:
				_refresh[refresh_position].f3 = state;
				break;
			case 4:
				_refresh[refresh_position].f4 = state;
				break;
			case 5:
				_refresh[refresh_position].f5 = state;
				break;
			case 6:
				_refresh[refresh_position].f6 = state;
				break;
			case 7:
				_refresh[refresh_position].f7 = state;
				break;
			case 8:
				_refresh[refresh_position].f8 = state;
				break;
		}
	}
}

void Ardurail::emergency_stop(byte addr)
{
	set_speed(addr,15);					// für alte Decoder
	commit(addr);
	set_dir(addr,!get_dir(addr));
	set_speed(addr,0);
	commit(addr);
	set_dir(addr,!get_dir(addr));
	commit(addr);
}

// The following function is not ready yet... HW
void Ardurail::reset_locos(void)
{
	byte i=0;
	byte n=0;
	uint8_t r=_refresh_max;

	for(i=0;i<MAX_REFRESH;i++)
	{
		set_speed(i,0);
		set_dir(i,true);
		for(n=0;n<=8;n++)
			set_function(i,n,0);
		commit(i);
	}

	_refresh_max=r;
}

void Ardurail::commit(void)
{
	byte i=0;

	for(i=0;i<=_refresh_max;i++)
	{
		_commit(i);
	}
}

void Ardurail::commit(byte addr)
{
	_commit(_find_refresh(addr));
}

void Ardurail::set_track(byte addr, byte sub_addr, boolean state)
{ 
	// we have one new command
	_waiting_track_commands++;
  
  	if (_waiting_track_commands > 1)
  	{
		// move all commands
		for (uint8_t i = _waiting_track_commands; i > 0; i--) 
		{
			tswitch[i] = tswitch[i-1];
		}
	}

	// insert the new command
	tswitch[0].address = addr;
	tswitch[0].subaddress = sub_addr;
	tswitch[0].state = state;
	tswitch[0].ready = true;
}

// 
// ######################### PRIVATE METHODS #############################
// 

void Ardurail::_commit(byte index)
{
	_refresh[index].ready = true;
	_updated_loco = index;
}


int8_t Ardurail::_create_refresh(byte address)
{
	int8_t refresh_position = -1;

	if (_refresh_max >= MAX_REFRESH)
	{
		return (-1);			// maximum refresh position reached!
	}

	if ((refresh_position = _find_refresh(address)) < 0)
	{
		refresh_position = ++_refresh_max;
		_refresh[refresh_position].address = address;
		_refresh[refresh_position].ready = false;
	}

	return (refresh_position);
}

int8_t Ardurail::_find_refresh(byte address)
{
	int8_t i = 0;

	for (i = 0; i <= _refresh_max; i++)
	{
		if (_refresh[i].address == address)
		{
			return(i);
		}
	}

	return (-1);
}

uint8_t Ardurail::_lsb_msb_replacement(uint8_t inbyte)
{
	uint8_t outbyte = 
		((inbyte & 1) << 7) |
		((inbyte & 2) << 5) |
		((inbyte & 4) << 3) |
		((inbyte & 8) << 1) |
		((inbyte & 16) >> 1) |
		((inbyte & 32) >> 3) |
		((inbyte & 64) >> 5) |
		((inbyte & 128) >> 7);

	return(outbyte);
}

void Ardurail::_write_mm_output_register(uint8_t message, uint8_t address, uint8_t function, uint8_t data)
{
	uint8_t  value1 = _lsb_msb_replacement(address);
	uint16_t value2 = (_lsb_msb_replacement(function) & 0b11000000) >> 6;
	uint32_t value3 = _lsb_msb_replacement(data);

	_msg[message] = value1 | (value2 << 8) | (value3 << 10);
}

byte Ardurail::_calc_trinary_address(byte address)
{
	uint8_t divident = address;
	uint8_t new_address = 0;
	uint8_t i=0;

	for (i = 0; i < 4; i++)
	{
		uint8_t stellenwert = divident % 3;

		if (stellenwert == 1)
		{
			new_address |= (0b11 << (6 - 2 * i));
		}
		if (stellenwert == 2)
		{
			new_address |= (0b10 << (6 - 2 * i));
		}
		divident = divident / 3;
	}
	return (new_address);
}

uint8_t Ardurail::_calc_dcc_address(uint8_t address)
{
	return (address);
}

uint8_t Ardurail::_calc_dcc128_speed(uint8_t speed)
{
	if (speed > 0)
	{
    	speed++;
    }
    return(speed);
}

uint8_t Ardurail::_calc_dcc128_speed_and_direction(uint8_t speed, uint8_t dir)
{
	uint8_t out = _calc_dcc128_speed(speed);

	// 1 = forward
	// 0 = backward

	if (dir == 1)
	{
		out |= 0b10000000;
	}

	return(out);
}

byte Ardurail::_calc_speed(byte speed)
{
	if (speed > 0)
	{
    	speed++;
    }

	byte out = 
		((speed & 1) << 7) | 
		((speed & 2) << 4) | 
		((speed & 4) << 1) | 
		((speed & 8) >> 2);

  return out;
}


byte Ardurail::_calc_speed_and_direction(byte speed, boolean dir)
{
	byte out = _calc_speed(speed);

	if (dir == false)
	{
		if (speed >= 7)
		{
			out |= 0b01000100;
		}
		else
		{
			out |= 0b01000101;
		}
	}
	else
	{
		if (speed <= 6)
		{
			out |= 0b00010001;
		}
		else
		{
			out |= 0b00010000;
		}
	}
	return out;
}

uint8_t Ardurail::_calc_dcc28_speed_and_direction(uint8_t speed, uint8_t dir)
{
	// http://www.nmra.org/sites/default/files/s-92-2004-07.pdf

	uint8_t out;

	if (speed > 0)
	{
    	speed += 3;
    }

	out = ((speed & 0b1) << 4) | ((speed & 0b11110) >> 1);
	
	if (dir == 1)
	{
		// 1 = forward
		out |= 0x20;
	}

	// opcode
	out |= 0x40; 

	return out;
}

byte Ardurail::_calc_function_f1(byte speed, boolean function)
{
	byte out = _calc_speed(speed);

	if ((speed == 2) && (function == false))
	{							// 1. Ausnahme
		out |= 0b01000100;
	}
	else
	{
		if ((speed == 10) && (function == true))
		{						// 2. Ausnahme
			out |= 0b00010001;
		}
		else
		{						// keine Ausnahme
			if (function == true)
			{
				out |= 0b01010001;
			}
			else
			{
				out |= 0b01010000;
			}
		}
	}
	return out;
}


byte Ardurail::_calc_function_f2(byte speed, boolean function)
{
	byte out = _calc_speed(speed);

	if ((speed == 3) && (function == false))
	{							// 1. Ausnahme
		out |= 0b01000100;
	}
	else
	{
		if ((speed == 11) && (function == true))
		{						// 2. Ausnahme
			out |= 0b00010001;
		}
		else
		{						// keine Ausnahme
			if (function == true)
			{
				out |= 0b00000101;
			}
			else
			{
				out |= 0b00000100;
			}
		}
	}
	return out;
}


byte Ardurail::_calc_function_f3(byte speed, boolean function)
{
	byte out = _calc_speed(speed);

	if ((speed == 5) && (function == false))
	{							// 1. Ausnahme
		out |= 0b01000100;
	}
	else
	{
		if ((speed == 13) && (function == true))
		{						// 2. Ausnahme
			out |= 0b00010001;
		}
		else
		{						// keine Ausnahme
			if (function == true)
			{
				out |= 0b00010101;
			}
			else
			{
				out |= 0b00010100;
			}
		}
	}
	return out;
}


byte Ardurail::_calc_function_f4(byte speed, boolean function)
{
	byte out = _calc_speed(speed);

	if ((speed == 6) && (function == false))
	{							// 1. Ausnahme
		out |= 0b01000100;
	}
	else
	{
		if ((speed == 14) && (function == true))
		{						// 2. Ausnahme
			out |= 0b00010001;
		}
		else
		{						// keine Ausnahme
			if (function == true)
			{
				out |= 0b01010101;
			}
			else
			{
				out |= 0b01010100;
			}
		}
	}
	return out;
}

void Ardurail::_generate(byte index)
{
	switch(_refresh[index].type)
	{
		case M1:
			_generate_m1(index);
		break;
		
		case DEF:
		case M2:
			_generate_m2(index);
		break;
		
		case M2_28_80A:
			_generate_m3(index);
		break;
		
		case M2_14_256A:
		   _generate_m4(index);
		break;
		
		case M2_28_256A:
		   _generate_m5(index);
		break;
		
		case DCC_28:
			_generate_dcc_28(index);	
		break;

		case DCC_128:
			_generate_dcc_128(index);
		break;
	}
}

void Ardurail::_generate_m1(byte index)
{
	// if no commit
	if(_refresh[index].ready == false)
	{
		return;
	}
	
	// just for counting
	byte i = 0;
	
	// get all informations of loco
	byte address = _refresh[index].address;
	byte speed = _refresh[index].speed;
	boolean changedir = _refresh[index].changedir;
	boolean function = _refresh[index].function;
	
	// timing for loco
	_mode = MM_LOCO;
	_timer = TIMER_MM_LOCO;

	// calculate data
	byte tri_address = _calc_trinary_address(address);
	byte f0 = (function & 1) | ((function & 1) << 1);
	byte data;
	
	// KEIN Fahrtrichtungswechsel
	if (changedir == false)
	{
	  data = _calc_speed(speed);			// Speedinformation A0B0C0D0
	  data |= ((data & 0b101010) >> 1);		// MM1              AABBCCDD
	}
	// Fahrtrichtungswechsel
	else
	{
	  data = 0b11000000;
	  _refresh[index].changedir = false;
	}

	for(i=0;i<8;i++)
	{
		_write_mm_output_register(i, tri_address, f0, data);
	}
}

void Ardurail::_generate_m2(byte index)
{
	// if no commit
	if(_refresh[index].ready == false)
	{
		return;
	}
	
	// get all informations of loco
	uint8_t address = _refresh[index].address;
	uint8_t speed = _refresh[index].speed;
	uint8_t dir = _refresh[index].dir;
	uint8_t changedir = _refresh[index].changedir;
	uint8_t function = _refresh[index].function;
	uint8_t f1 = _refresh[index].f1;
	uint8_t f2 = _refresh[index].f2;
	uint8_t f3 = _refresh[index].f3;
	uint8_t f4 = _refresh[index].f4;
	
	// timing for loco
	_mode = MM_LOCO;
	_timer = TIMER_MM_LOCO;
		
	// Adresse und Funktion berechnen
	uint8_t tri_address = _calc_trinary_address(address);
	uint8_t f0 = ((function & 1) << 1) | (function & 1);

	// KEIN Fahrtrichtungswechsel
	if (changedir == false)
	{
		// Die Pakete 0,2,4 und 6 fuer Geschwindigkeit und Richtung
		uint8_t data = _calc_speed_and_direction(speed, dir);

		for (uint8_t i = 0; i < 8; i += 2)
		{
			_write_mm_output_register(i, tri_address, f0, data);
		}

		// Die Pakete 1,3,5 und 7 fuer Geschwindigkeit und functionen (f1 - f4)
		data = _calc_function_f1(speed, f1);
		_write_mm_output_register(1, tri_address, f0, data);
		
		data =_calc_function_f2(speed, f2);
		_write_mm_output_register(3, tri_address, f0, data);

		data =_calc_function_f3(speed, f3);
		_write_mm_output_register(5, tri_address, f0, data);

		data =_calc_function_f4(speed, f4);
		_write_mm_output_register(7, tri_address, f0, data);
	}
	// Fahrtrichtungswechsel
	else
	{
		_refresh[index].changedir = false;
		for (uint8_t i = 0; i < 8; i++)
		{
			_write_mm_output_register(i, tri_address, 0b00, 0b11000000);
		}
	}
}

void Ardurail::_generate_m3(byte index)
{
	// if no commit
	if(_refresh[index].ready == false)
	{
		return;
	}

	// just for counting
	uint8_t i = 0;
	
	// get all informations of loco
	uint8_t address = _refresh[index].address;
	boolean dir = _refresh[index].dir;
	boolean changedir = _refresh[index].changedir;
	boolean function = _refresh[index].function;
	boolean f1 = _refresh[index].f1;
	boolean f2 = _refresh[index].f2;
	boolean f3 = _refresh[index].f3;
	boolean f4 = _refresh[index].f4;
	
	uint8_t speed = (_refresh[index].speed + 1) / 2;
	boolean zfs = false;
	
	if (speed != 0)
	{
	  zfs = (boolean)(_refresh[index].speed + 1) % 2;
	}

	// timing for loco
	_mode = MM_LOCO;
	_timer = TIMER_MM_LOCO;

	// Alle Pakete mit der richtigen Adresse und Funktion versehen
	uint8_t tri_address = _calc_trinary_address(address);
	uint8_t f0;
	
	if (zfs == false) // Zwischenfahrstufe :)
	{
		f0 = (function & 1) | ((function & 1) << 1);
	} else
	{
		if (function == true)
		{
			f0 = 0b10;
		} else
		{
			f0 = 0b01;
		}
	}

	// kein Fahrtrichtungswechsel
	if (changedir == false)
	{
		// Die Pakete 0,2,4 und 6 fuer Geschwindigkeit und Richtung
		uint8_t data = _calc_speed_and_direction(speed, dir);
		for (i = 0; i < 8; i += 2)
		{
			_write_mm_output_register(i, tri_address, f0, data);
		}

		// Die Pakete 1,3,5 und 7 fuer Geschwindigkeit und functionen (f1 - f4) 
		data = _calc_function_f1(speed, f1);
		_write_mm_output_register(1, tri_address, f0, data);
		data = _calc_function_f2(speed, f2);
		_write_mm_output_register(3, tri_address, f0, data);
		data = _calc_function_f3(speed, f3);
		_write_mm_output_register(5, tri_address, f0, data);
		data = _calc_function_f4(speed, f4);
		_write_mm_output_register(7, tri_address, f0, data);
	}
	
	// Fahrtrichtungswechsel
	else
	{
		_refresh[index].changedir = false;

		for (i = 0; i < 8; i++)
		{
			_write_mm_output_register(i, tri_address, f0, 0b11000000);
		}
	}
}

void Ardurail::_generate_m4(byte index)
{
	// if no commit
	if(_refresh[index].ready == false)
	{
		return;
	}
	
	// just for counting
	uint8_t i = 0;
	
	// get all informations of loco
	uint8_t address = _refresh[index].address;
	uint8_t speed = _refresh[index].speed;
	boolean dir = _refresh[index].dir;
	boolean changedir = _refresh[index].changedir;
	boolean function = _refresh[index].function;
	boolean f1 = _refresh[index].f1;
	boolean f2 = _refresh[index].f2;
	boolean f3 = _refresh[index].f3;
	boolean f4 = _refresh[index].f4;
	
	// timing for loco
	_mode = MM_LOCO;
	_timer = TIMER_MM_LOCO;

	uint8_t tri_address;
	
	// if we have address >= 80 this is an extended address (M4) 
	if (address < 80)
	{
	  tri_address = _calc_trinary_address(address);
	} else
	{
	  tri_address = extended_MM_addr[address-80];
	}
	
	uint8_t f0 = (function & 1) | ((function & 1) << 1);

	// KEIN Fahrtrichtungswechsel
	if (changedir == false)
	{
		// Die Pakete 0,2,4 und 6 fuer Geschwindigkeit und Richtung
		uint8_t data = _calc_speed_and_direction(speed, dir);
		for (i = 0; i < 8; i += 2)
		{
			_write_mm_output_register(1, tri_address, f0, data);
		}

		// Die Pakete 1,3,5 und 7 fuer Geschwindigkeit und functionen (f1 - f4) 
		data = _calc_function_f1(speed, f1);
		_write_mm_output_register(1, tri_address, f0, data);
		data = _calc_function_f2(speed, f2);
		_write_mm_output_register(3, tri_address, f0, data);
		data = _calc_function_f3(speed, f3);
		_write_mm_output_register(5, tri_address, f0, data);
		data = _calc_function_f4(speed, f4);
		_write_mm_output_register(7, tri_address, f0, data);
	}
	
	// Fahrtrichtungswechsel
	else
	{
		_refresh[index].changedir = false;
		for (i = 0; i < 8; i++)
		{
			_write_mm_output_register(i, tri_address, f0, 0b11000000);
		}
	}
}

void Ardurail::_generate_m5(byte index)
{
	// if no commit
	if(_refresh[index].ready == false)
	{
		return;
	}
	
	// just for counting
	uint8_t i = 0;
	
	// get all informations of loco
	uint8_t address = _refresh[index].address;
	boolean dir = _refresh[index].dir;
	boolean changedir = _refresh[index].changedir;
	boolean function = _refresh[index].function;
	boolean f1 = _refresh[index].f1;
	boolean f2 = _refresh[index].f2;
	boolean f3 = _refresh[index].f3;
	boolean f4 = _refresh[index].f4;
	
	uint8_t speed = (_refresh[index].speed + 1) / 2;
	boolean zfs = false;
	
	if (speed != 0)
	{
	  zfs = (boolean)(_refresh[index].speed + 1) % 2;
	}

	// timing for loco
	_mode = MM_LOCO;
	_timer = TIMER_MM_LOCO;

	uint8_t tri_address;
	uint8_t f0;
	
	// if we have address >= 80 this is an extended address (M4) 
	if (address < 80)
	{
	  tri_address = _calc_trinary_address(address);
	} else
	{
	  tri_address = extended_MM_addr[address-80];
	}

	if (zfs == false)
	{
		f0 = (function & 1) | ((function & 1) << 1);
	} else
	{
		if (function == true)
		{
			f0 = 0b10;
		} else
		{
			f0 = 0b01;
		}
	}

	// kein Fahrtrichtungswechsel
	if (changedir == false)
	{
		// Die Pakete 0,2,4 und 6 fuer Geschwindigkeit und Richtung
		uint8_t data = _calc_speed_and_direction(speed, dir);
		for (i = 0; i < 8; i += 2)
		{
			_write_mm_output_register(i, tri_address, f0, data);
		}

		// Die Pakete 1,3,5 und 7 fuer Geschwindigkeit und functionen (f1 - f4) 
		data = _calc_function_f1(speed, f1);
		_write_mm_output_register(1, tri_address, f0, data);
		data = _calc_function_f2(speed, f2);
		_write_mm_output_register(3, tri_address, f0, data);
		data = _calc_function_f3(speed, f3);
		_write_mm_output_register(5, tri_address, f0, data);
		data = _calc_function_f4(speed, f4);
		_write_mm_output_register(7, tri_address, f0, data);
	}
	
	// Fahrtrichtungswechsel
	else
	{
		_refresh[index].changedir = false;
		for (i = 0; i < 8; i++)
		{
			_write_mm_output_register(i, tri_address, f0, 0b11000000);
		}
	}
}

void Ardurail::_generate_dcc_28(uint8_t index)
{
	// if no commit
	if(_refresh[index].ready == false)
	{
		return;
	}
	
	// get all informations of loco
	uint8_t address = _calc_dcc_address(_refresh[index].address);
	uint8_t speed = _refresh[index].speed;
	uint8_t dir = _refresh[index].dir;

	uint8_t speed_and_dir = _calc_dcc28_speed_and_direction(speed, dir);

	/*
	boolean changedir = _refresh[index].changedir;
	boolean function = _refresh[index].function;
	boolean f1 = _refresh[index].f1;
	boolean f2 = _refresh[index].f2;
	boolean f3 = _refresh[index].f3;
	boolean f4 = _refresh[index].f4;
	*/

	_msg_dcc[0].used_bytes = 3;
	_msg_dcc[0].byte1 = address;
	_msg_dcc[0].byte2 = speed_and_dir;
	_msg_dcc[0].byte3 = address ^ speed_and_dir;
	_bit_counter = 0;
	_dcc_state = PREAMBLE;

	// DCC-Frequency
	_mode = DCC;
	_timer = TIMER_DCC;
}

void Ardurail::_generate_dcc_128(uint8_t index)
{
	// if no commit
	if(_refresh[index].ready == false)
	{
		return;
	}
	
	uint8_t address = _calc_dcc_address(_refresh[index].address);
	uint8_t speed_and_dir = _calc_dcc28_speed_and_direction(_refresh[index].speed, _refresh[index].dir);

	/*
	boolean changedir = _refresh[index].changedir;
	*/

	_msg_dcc[0].used_bytes = 4;
	_msg_dcc[0].byte1 = address;
	_msg_dcc[0].byte2 = 0b00111111;		// extended command:001 128 speedsteps: 11111
	_msg_dcc[0].byte3 = speed_and_dir;
	_msg_dcc[0].byte4 = address ^ 0b00111111 ^ speed_and_dir;

	uint8_t functions = 
		(0b100 << 5) |					// functiongroup 1
		(_refresh[index].function << 4) |
		(_refresh[index].f4 << 3) |
		(_refresh[index].f3 << 2) |
		(_refresh[index].f2 << 1) |
		_refresh[index].f1;

	_msg_dcc[1].used_bytes = 3;
	_msg_dcc[1].byte1 = address;
	_msg_dcc[1].byte2 = functions;
	_msg_dcc[1].byte3 = address ^ functions;

	functions = 
		(0b101 << 5) |					// functiongroup 2
		(1 << 4 ) |						// S = 1
		(_refresh[index].f8 << 3) |
		(_refresh[index].f7 << 2) |
		(_refresh[index].f6 << 1) |
		_refresh[index].f5;

	_msg_dcc[2].used_bytes = 3;
	_msg_dcc[2].byte1 = address;
	_msg_dcc[2].byte2 = functions;
	_msg_dcc[2].byte3 = address ^ functions;

	functions = 
		(0b101 << 5) |					// functiongroup 2
										// S = 0
		(_refresh[index].f12 << 3) |
		(_refresh[index].f11 << 2) |
		(_refresh[index].f10 << 1) |
		_refresh[index].f9;

	_msg_dcc[3].used_bytes = 3;
	_msg_dcc[3].byte1 = address;
	_msg_dcc[3].byte2 = functions;
	_msg_dcc[3].byte3 = address ^ functions;

	_bit_counter = 0;
	_step = 0;
	_dcc_state = PREAMBLE;

	// DCC-Frequency
	_mode = DCC;
	_timer = TIMER_DCC;

}

// erzeugt 1 Doppelpaket mit dem Weichen-Befehl
void Ardurail::_generate_trackswitch(byte address, byte subaddress, boolean state)
{
	// Weichenfrequenz und Mode setzen
	_mode = MM_DEVICE;
	_timer = TIMER_MM_DEVICE;

	uint8_t tri_address = _calc_trinary_address(address);
	uint8_t data = 
			((subaddress & 1) << 7) | ((subaddress & 1) << 6) |
			((subaddress & 2) << 4) | ((subaddress & 2) << 3) |
			((subaddress & 4) << 1) | (subaddress & 4) |
			((state & 1) << 1) | (state & 1);	

	for (uint8_t i=0; i<1; i++) {
		_write_mm_output_register(i, tri_address, 0b00, data);
	}
}

void Ardurail::_do_timer2_irq_MM(void)
{	
	TCNT2 = _timer;

	if (_mode == DCC)
	{
		_do_timer2_irq_DCC();
	}
	else
	{

	if ((_break_t1 + _break_t2 + _break_t3) == 0)
	{
		// rising edge
		if (_step == 0)
		{
			// high
			*_digital_reg |= _digital_signal;

			_outbit = (_msg[_msg_counter] >> _bit_counter) & 1;
		}

		// faling edge 
		else if (((_step == 1) && (_outbit == 0)) || ((_step == 7) && (_outbit == 1)))
		{
			// low
			*_digital_reg &= ~_digital_signal;
		}

		// 8. step is the next step 0
		if (++_step == 8)
		{
			_step = 0;
			_bit_counter++;

			if (_bit_counter == 18)
			{
				_bit_counter = 0;
				_packet_counter++;

				if ((_packet_counter == 1) || (_packet_counter == 3))
				{
					_break_t1 = true;
				}

				else if (_packet_counter == 2)
				{
					_break_t2 = true;
				}

				else if (_packet_counter == 4)
				{
					_break_t3 = true;
					_packet_counter = 0;
					_msg_counter++;
				}
			}
		}
	}
	else
	{
		_step++;

		if ((_step >= TIME_T1) && (_break_t1 == true))
		{
			_break_t1 = false;
			_step = 0;
		}

		else if ((_step >= TIME_T2) && (_break_t2 == true))
		{
			_break_t2 = false;
			_step = 0;
		}

		else if ((_step >= TIME_T3) && (_break_t3 == true))
		{
			// this is the end of breake t3
			_break_t3 = false;
			_step = 0;

			// the 8. loco msg and 1 devise msg is number 0
			if ((_msg_counter == MAXMSG) || ((_msg_counter == 2) && (_mode == MM_DEVICE)))
			{
			  	_msg_counter = 0;
			  	_bit_counter = 0;
			  	// we can load a new set of commands
			  	_do_update();
			}
		}
	}
}
}

void Ardurail::_do_timer2_irq_DCC(void)
{
	// next state
	switch (_dcc_state)
	{
		case PREAMBLE:
			// Preamble: 15 1-bits
			_outbit = 1;

			// end of Preamble
			if (_bit_counter == 15)
			{
				_bit_counter = 0;
				_dcc_state = SEPERATOR;
			}
		break;

		case SEPERATOR:
			// one 0-bit
			_outbit = 0;

			if (_bit_counter == 1)
			{
				_bit_counter = 0;
				_dcc_state = SENDBYTE;
			}
		break;

		case SENDBYTE:

			if (_bit_counter < 8)
			{	
				if (_step == 0)
				{
					// byte to send
					uint8_t sendout = 0;

					switch (_packet_counter)
					{
						case 0:
							sendout = _msg_dcc[_msg_counter].byte1;
						break;

						case 1:
							sendout = _msg_dcc[_msg_counter].byte2;
						break;

						case 2:
							sendout = _msg_dcc[_msg_counter].byte3;
						break;

						case 3:
							sendout = _msg_dcc[_msg_counter].byte4;
						break;

						case 4:
							sendout = _msg_dcc[_msg_counter].byte5;
						break;

						case 5:
							sendout = _msg_dcc[_msg_counter].byte6;
						break;
					}

					_outbit = (sendout >> (7 - _bit_counter)) & 1;
				}
			}
			else // _bit_counter = 8
			{	
				_bit_counter = 0;
				_step = 0;
				_packet_counter++;

				if (_packet_counter == _msg_dcc[_msg_counter].used_bytes)
				{
					_dcc_state = TERMINATOR;
					_packet_counter = 0;
				}
				else
				{
					_dcc_state = SEPERATOR;
				}
			}
		break;

		case TERMINATOR:
			// one 1-bit
			_outbit = 1;

			if (_bit_counter == 1)
			{
				_dcc_state = BREAK;
				_bit_counter = 0;
				_step = 0;
			}
		break;

		case BREAK:
			_step++;
			
			if (_step == TIME_DCC_BREAK)
			{
				_dcc_state = PREAMBLE;
				_step = 0;
				_msg_counter++;
				if (_msg_counter == MAXMSG)
				{
					_msg_counter = 0;
					_do_update();
				}
			}
		break;
	}

	if (_dcc_state != BREAK)
	{
		if (_outbit == 1)
		{
			if ((_step == 0) || (_step == 1))
			{
				*_digital_reg ^= _digital_signal;
			}
			_step++;
			if (_step == 2)
			{
				_step = 0;
				_bit_counter++;
			}
		}
		else
		{
			if ((_step == 0) || (_step == 2))
			{
				*_digital_reg ^= _digital_signal;
			}
			_step++;
			if (_step == 4)
			{
				_step = 0;
				_bit_counter++;
			}
		}
	}
}

void Ardurail::_do_update(void)
{
	#ifdef __LED13__
		__led13__(HIGH);
	#endif

	if (_power == true) {
		
		// if we have an update
		if(_updated_loco >= 0) {
			_generate(_updated_loco);
			_updated_loco = -1;
		}
		else {
			// if we have a regular command for trackswitch
			if(_waiting_track_commands > 0) {
				_waiting_track_commands--;

  				_generate_trackswitch(
  					tswitch[_waiting_track_commands].address, 
  					tswitch[_waiting_track_commands].subaddress, 
  					tswitch[_waiting_track_commands].state);

  				/*
  				tswitch[_waiting_track_commands].address = 0;
  				tswitch[_waiting_track_commands].subaddress = 0;
  				tswitch[_waiting_track_commands].state = 0;
  				tswitch[_waiting_track_commands].ready = false;
				*/
			}
			else {
				if (_refresh_max == 0) {
    				// if we have no loco, fill with Idlepackets
    				for (uint8_t i = 0; i < MAXMSG; i++) {
				
						// MM
						_write_mm_output_register(i, 0b10101010, 0b00, 0b00000000);
				
						// DCC
						_msg_dcc[i].byte1 = 0xff;
						_msg_dcc[i].byte2 = 0;
						_msg_dcc[i].byte3 = 0xff;
						_msg_dcc[i].byte4 = 0;
						_msg_dcc[i].byte5 = 0;
						_msg_dcc[i].byte6 = 0;
						_msg_dcc[i].used_bytes = 3;
					}

					// timing for loco
					_mode = MM_LOCO;
					_timer = TIMER_MM_LOCO;
				}
				else {
    				// the "regular" refresh
					_generate(_refresh_pointer++);	   		

					if(_refresh_pointer > _refresh_max) {
	    				_refresh_pointer = 0;
					}
				}
			}
		}
	}

	#ifdef __LED13__
  		__led13__(LOW);
	#endif
}

inline void Ardurail::handle_timer2_interrupt(void)
{
	if (_active_object)
	{
		_active_object->_do_timer2_irq_MM();
	}
}

inline void Ardurail::handle_short_interrupt(void)
{
	if (_active_object)
	{
		boolean State = true;
		for (int counter = 0; counter < 5; counter++)
		{
			//delay(1);
			State = (boolean)digitalRead(_active_object->_shortcut_signal_pin);
			if (State == false)
			{
				_active_object->_short = false;
				break;
			}
		}

		if (State == true)
		{
			_active_object->set_power(false);
			_active_object->_short = true;
		}
	}
}


// 
// ######################### FUNCTIONS #############################
// 

#ifdef __LED13__
inline void __led13__(boolean state)
{
#ifdef __MEGA__
	if (state == HIGH)
	{
		PORTB |= 1 << 7;		// high
	}
	else
	{
		PORTB &= ~(1 << 7);		// low
	}
#else
	if (state == HIGH)
	{
		PORTB |= 1 << 5;		// high
	}
	else
	{
		PORTB &= ~(1 << 5);		// low
	}
#endif
}
#endif

// 
// Timer2 Interrupt function for generation of the signal
//
#ifdef USE_SHORT_INTERRUPT
ISR(TIMER2_OVF_vect)
{
	Ardurail::handle_timer2_interrupt();
}
#endif



