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


#include "Ardurail.h"

Ardurail MM;

void setup(void)
{
  MM.init(6,2,8);           // use pin 6 for digital signal,
                            // pin 7 as shortcut detection and
                            // pin 8 as "GO" signal
}

void loop(void)
{
  delay(1000);              // wait 1 secs
  MM.set_power(true);       // go

  while (1)
  { 
    // set track to turnout (red)
    MM.set_track(2, 4, 1);  // address: 20  decoder: 5  subaddress: 6  state: 1
    delay(250);             // circuit time
    MM.set_track(2, 4, 0);  // off

    delay(3000);            // wait 3 secs

    // set track to straigh (green)
    MM.set_track(2, 5, 1);  // address: 20  decoder: 5  subaddress: 7  state: 1
    delay(250);             // circuit time
    MM.set_track(2, 5, 0);  // off

    delay(3000);            // wait 3 secs
  }
}
