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

#include "Ardurail_P50.h"

#define SERIALPORT Serial

//
// ######################### GLOBAL VARS #############################
//

//
// ######################### PUBLIC METHODS #############################
//

void Ardurail_P50::init(byte pin)
{
  delay(500);
  Ardurail::init(pin);
  SERIALPORT.begin(2400);
}

void Ardurail_P50::init(byte digital_signal_pin,byte shortcut_pin,byte go_signal_pin)
{
  delay(500);
  Ardurail::init(digital_signal_pin,shortcut_pin,go_signal_pin);
  SERIALPORT.begin(2400);
}

// Verarbeitet p50-Befehle auf der Seriellen Schnittstelle
void Ardurail_P50::p50(void)
{
#ifdef USE_S88
  S88();
#endif

  if (SERIALPORT.available() > 0)
  {
	  byte incomingByte = SERIALPORT.read();
	  
	  byte speed;			// p50-speed: 0-14  Fahrtrichtungswechsel: 15
    boolean function;		// Licht
    boolean f1,f2,f3,f4;	// function
    byte weichenspule;

    // Go-Befehl
    if (incomingByte == 96) 
    {
      set_power(true);
    }

    // Stop-Befehl
    if (incomingByte == 97) 
    {  
      set_power(false);
    }

    // Weichenspule abschalten
    if (incomingByte == 32) 
    {
      set_track(_generate_trackswitch_address,0,0);
    }

    // S88-Decoder auslesen (mehrere)
    if ((incomingByte > 128) && (incomingByte <= 192)) 
    { 
    	// Modulanzahl berechnen
    	byte numberofdevices = incomingByte - 128; 	
      
      // Werte der Rückmelder ausgeben
      for (byte modul_nr = 0; modul_nr < numberofdevices; modul_nr++)
      {
#ifdef USE_S88
        SERIALPORT.write((byte)get_S88(modul_nr, 0));
        SERIALPORT.write((byte)get_S88(modul_nr, 1));
#else
        SERIALPORT.write((byte)0);
        SERIALPORT.write((byte)0);
#endif
      }
    }

    // S88-Decoder auslesen (einzeln)
    if ((incomingByte > 192) && (incomingByte <= 255)) 
    { 
      // Modulnummer berechnen
      byte modul_nr = incomingByte - 192;  
      
#ifdef USE_S88
      SERIALPORT.write((byte)get_S88(modul_nr, 0));
      SERIALPORT.write((byte)get_S88(modul_nr, 1));
#else
      SERIALPORT.write((byte)0);
      SERIALPORT.write((byte)0);
#endif
    }



    // 2-Byte-Befehle
    // Loks: 1.Byte: Geschwindigkeit und Funktionicht (0-31) oder function (64-79) 2.Byte: Adresse (1-80)
    // Weichen: 1.Byte: Richtung (33: Gerade 34: Abbiegen)  2.Byte: Adresse (1-255)

	 // Weichen-Befehl
    if ((incomingByte == 33) || (incomingByte == 34))
    {		
      delay(1);
      while (SERIALPORT.available() == 0);  
      byte secondincomingByte = SERIALPORT.read();
      
      _generate_trackswitch_address = ((secondincomingByte-1)/4)+1;
      weichenspule = ((secondincomingByte-1)%4)*2;
      
      if (incomingByte == 33)
      {
        weichenspule++;
      }	
      set_track(_generate_trackswitch_address,weichenspule,1);
    }
    
    // Lok-Befehl												 
    if (incomingByte <= 31) 
    {									
      delay(1);
      while (SERIALPORT.available() == 0);
      byte address = SERIALPORT.read();

      add_loco(address); // needed since r38!

      speed = incomingByte & 0b00001111;
      function = (incomingByte & 0b00010000) >> 4;
      
      if(speed == 15)
        change_dir(address);
      else
        set_speed(address, speed);
        
      set_function(address, 0, function);
      commit(address);
    }
    
    if ((incomingByte >= 64) && (incomingByte <= 79))
    {
      delay(1);
      while (SERIALPORT.available() == 0);
      byte address = SERIALPORT.read();
      
      add_loco(address); //  quick hack, needed since r38!
      
      incomingByte -= 64;      
      f1 = incomingByte & 1;
      f2 = (incomingByte & 2) >> 1;
      f3 = (incomingByte & 4) >> 2;
      f4 = (incomingByte & 8) >> 3;
      
      set_function(address,1,f1);
      set_function(address,2,f2);
      set_function(address,3,f3);
      set_function(address,4,f4);
      commit(address);
    }
  }
}
