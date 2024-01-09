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

#ifndef _ARDURAIL_P50_H
#define _ARDURAIL_P50_H

#include "Ardurail.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//
// ######################### DEFAULTS #############################
//

//
// DO NOT CHANGE ANYTHING BEHIND THIS LINE IF YOU DO NOT KNOW WHAT YOU ARE DOING!!!
//

//
// ######################### DEFINITIONS #############################
//

//
// ######################### CLASS #############################
//

class Ardurail_P50 : public Ardurail
{
public:
  void init(byte pin);
  void init(byte pin,byte shortcut_pin,byte go_signal_pin);
  void p50(void);

protected:
  int _generate_trackswitch_address;			// für p50-Protokoll wie bei Ma 6051, zum Abschalten der Weichenspule muss man sich die letzte Adresse merken
};

//
// ######################### FUNCTIONS #############################
//

#endif 

