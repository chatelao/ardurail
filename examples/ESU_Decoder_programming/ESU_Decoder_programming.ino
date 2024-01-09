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

// Just for testing the svn....

#include "Ardurail.h"

Ardurail MM_CENTRAL;

void setup(void)
{
  MM_CENTRAL.init(6,7,8);
  MM_CENTRAL.add_loco(80, M2_14_256A); // progaddress for ESU-Decoder
  delay(1000);
}

void loop(void)
{
  // activate the progmodus
  MM_CENTRAL.change_dir(80);
  MM_CENTRAL.commit(80);
  MM_CENTRAL.set_power(true);
  delay(1000);
  
  // CV 01 address
  MM_CENTRAL.add_loco(01, M2_14_256A);
  MM_CENTRAL.change_dir(01);
  MM_CENTRAL.commit(01);
  delay(1000);
  
  // new address is 82
  MM_CENTRAL.add_loco(82, M2_14_256A);
  MM_CENTRAL.change_dir(82);
  MM_CENTRAL.commit(82);
  delay(1000);
  
  // end for progmodus
  MM_CENTRAL.change_dir(80);
  MM_CENTRAL.commit(80);
  while(1);
}
