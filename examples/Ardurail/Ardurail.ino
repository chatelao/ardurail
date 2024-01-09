/* Ardurail 1.0

   (C) 2012 Jan Weller jan.weller@gmx.de Rewritten
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

#include <Ardurail.h>

#define LOCO		18		// Locoaddress 
//#define PROTOCOL	M1		// Maerklin-Motorola I
#define PROTOCOL	M2		// Maerklin-Motorola II
//#define PROTOCOL	M2_28_80A	// Maerklin-Motorola II, 28 speedsteps
//#define PROTOCOL	M2_14_256A	// Maerklin-Motorola II, 256 addresses
//#define PROTOCOL	M2_28_256A	// Maerklin-Motorola II, 28 speedsteps, 256 addresses

Ardurail MM_CENTRAL;

void setup(void)
{
  MM_CENTRAL.init(6,7,8);
  delay(100);
  MM_CENTRAL.set_power(true);		// Go
  MM_CENTRAL.add_loco(LOCO,PROTOCOL);
}

void loop(void)
{
  delay(5000);				// wait 5 secs
  MM_CENTRAL.set_speed(LOCO, 0);	// speed 0
  MM_CENTRAL.set_dir(LOCO, true);	// forward
  MM_CENTRAL.set_function(LOCO, 0, true);	// lights on
  MM_CENTRAL.commit(LOCO);
  delay(1000);
  
  MM_CENTRAL.set_function(LOCO, 0, false);	// lights off
  MM_CENTRAL.commit(LOCO);
  delay(1000);

#if PROTOCOL!=M1
  MM_CENTRAL.set_function(LOCO, 1, true);	// function 1 on
  MM_CENTRAL.commit(LOCO);
  delay(1000);
  
  MM_CENTRAL.set_function(LOCO, 2, true);	// function 2 on
  MM_CENTRAL.commit(LOCO);
  delay(1000);

  MM_CENTRAL.set_function(LOCO, 3, true);	// function 3 on
  MM_CENTRAL.commit(LOCO);
  delay(1000);

  MM_CENTRAL.set_function(LOCO, 4, true);	// function 4 on
  MM_CENTRAL.commit(LOCO);
  delay(1000);

  MM_CENTRAL.set_function(LOCO, 1, false);	// function 1, 2, 3 and 4 off
  MM_CENTRAL.set_function(LOCO, 2, false);
  MM_CENTRAL.set_function(LOCO, 3, false);
  MM_CENTRAL.set_function(LOCO, 4, false);
#endif
  MM_CENTRAL.set_function(LOCO, 0, true);	// lights on
  MM_CENTRAL.commit(LOCO);
  delay(1000);

#if PROTOCOL==DEF || PROTOCOL==M1 || PROTOCOL==M2
	#define HALFSPEED 7
	#define FULLSPEED 14
#else
	#define HALFSPEED 14
	#define FULLSPEED28 
#endif
  MM_CENTRAL.set_speed(LOCO, HALFSPEED);	// half speed
  MM_CENTRAL.commit(LOCO);
  delay(10000);
 
  MM_CENTRAL.set_speed(LOCO, 0);		// speed 0 with brake retardation
  MM_CENTRAL.commit(LOCO);
  delay(10000);
  
  MM_CENTRAL.set_speed(LOCO, FULLSPEED);	// full speed
  MM_CENTRAL.set_dir(LOCO, false);		// backwards
  MM_CENTRAL.commit(LOCO);
  delay(10000);
  
  MM_CENTRAL.set_speed(LOCO, 0);		// emergency stop
  MM_CENTRAL.set_dir(LOCO, true);
  MM_CENTRAL.commit(LOCO);
  delay(10000);
   
  MM_CENTRAL.set_power(false);			// Stop
  delay(1000);
  MM_CENTRAL.set_power(true);			// Go
  delay(1000);
}
