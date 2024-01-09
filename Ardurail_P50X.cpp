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

#include "Ardurail_P50X.h"

//
// ######################### GLOBAL VARS #############################
//
static unsigned char p50_p50b_input_line[15];
uint8_t p50_p50b_lenght = 0;
static unsigned char p50a_input_line[80];
boolean disable_p50_p50b = false;
unsigned long _last_serialbyte;
//
// ######################### PUBLIC METHODS #############################
//


#define BAUD 9600
#define SERIALPORT Serial
//#define SERIALPORT_DEBUG Serial1


void Ardurail_P50X::init(byte pin) {
  Ardurail::init(pin);
  SERIALPORT.begin(BAUD);

  for (uint8_t counter = 0; counter < _s88_max_s88; counter++) {
    Ardurail::S88();
  }

  _s88_max_s88 = MAX_S88;
  _s88_autoreset = true;
  _mode = MIXED;
}

void Ardurail_P50X::init(byte digital_signal_pin, byte shortcut_pin, byte go_signal_pin) {
  Ardurail::init(digital_signal_pin, shortcut_pin, go_signal_pin);
  SERIALPORT.begin(BAUD);
  // For Debug
  //SERIALPORT_DEBUG.begin(9600);
  
  for (uint8_t counter = 0; counter < _s88_max_s88; counter++) {
    Ardurail::S88();
  }

  _s88_max_s88 = MAX_S88;
  _s88_autoreset = true;
  _mode = MIXED;
}

void Ardurail_P50X::new_trackswitchcommand(uint8_t address, uint8_t subaddress, uint8_t state) {

  if ((get_power() == true) && (millis() >= _next_switch_time)) {
    // sending to ardurail library
    set_track(address, subaddress, state);
    if (state == 1)
    {
      _next_switch_time = millis() + SWITCH_TIME_MAX;
    }
  }
  else {
    tswitch_buffer[_waiting_trackswitch_commands].address = address;
    tswitch_buffer[_waiting_trackswitch_commands].subaddress = subaddress;
    tswitch_buffer[_waiting_trackswitch_commands].state = state;
    _waiting_trackswitch_commands++;
  }

  if (state == 1){
    tswitch_buffer[_waiting_trackswitch_commands].address = address;
    tswitch_buffer[_waiting_trackswitch_commands].subaddress = subaddress;
    tswitch_buffer[_waiting_trackswitch_commands].state = 0;
    _waiting_trackswitch_commands++;
  }    
}

void Ardurail_P50X::blow_out_trackswitchcommand_buffer() {

  // Sending buffered trackswitch commands
  if ((_waiting_trackswitch_commands > 0) && (get_power() == true)) {
    
    // sending to ardurail library
    if (millis() >= _next_switch_time) {
      set_track(tswitch_buffer[0].address, tswitch_buffer[0].subaddress, tswitch_buffer[0].state);
      if (tswitch_buffer[0].state == 1)
      {
        _next_switch_time = millis() + SWITCH_TIME_MAX;
      }
    
      // first in first out
      for (uint8_t i=0; i < _waiting_trackswitch_commands; i++) {
        tswitch_buffer[i] = tswitch_buffer[i+1];
      }

      // command is send
      _waiting_trackswitch_commands--;
    }
  }
}

uint8_t Ardurail_P50X::get_S88_accumulator(uint8_t modul_nr, uint8_t byte_nr) {
  uint8_t returnvalue = 0;

  if (byte_nr == 0) {
    returnvalue = feedbackdevice[modul_nr].lsb;
  }
  if (byte_nr == 1) {
    returnvalue = feedbackdevice[modul_nr].msb;
  }
  return(returnvalue);
}

void Ardurail_P50X::reset_S88_accumulator(uint8_t modul_nr) {
  feedbackdevice[modul_nr].lsb = get_S88(modul_nr, 0);
  feedbackdevice[modul_nr].msb = get_S88(modul_nr, 1);
  feedbackdevice[modul_nr].modified = false;
}

void Ardurail_P50X::S88(void) {
  
  // Das nächste Modul einlesen
  Ardurail::S88();
  
  uint8_t modul_nr = _s88modulcycleaddr;

  if (modul_nr == 0) {
    modul_nr = _s88_max_s88 - 1;
  }
  else {
    modul_nr--;
  }

  // Debug
  /*
  SERIALPORT_DEBUG.print(" S88-Bus Modul: ");
  SERIALPORT_DEBUG.print(modul_nr);
  SERIALPORT_DEBUG.print("  ");
  SERIALPORT_DEBUG.print(get_S88(modul_nr, 0));
  SERIALPORT_DEBUG.print(" ");
  SERIALPORT_DEBUG.println(get_S88(modul_nr, 1));
  delay(1000);
  */

  uint8_t lsb = get_S88(modul_nr, 0);
  uint8_t msb = get_S88(modul_nr, 1);

  if ((lsb != feedbackdevice[modul_nr].lsb) || (msb != feedbackdevice[modul_nr].msb)) {
    feedbackdevice[modul_nr].lsb |= lsb;
    feedbackdevice[modul_nr].msb |= msb;
    feedbackdevice[modul_nr].modified = true;
    _event_sensors = true;
  }
}

boolean Ardurail_P50X::process_p50_command(unsigned char data[80], unsigned int length) {
  boolean result = false;
  
  // Go-Befehl
  if ((data[0] == 96) && (length == 1)) {
    set_power(true);
    result = true;
  }

  // Stop-Befehl
  if ((data[0] == 97) && (length == 1)) {  
    set_power(false);
    result = true;
  }

  // Weichenspule abschalten
  if ((data[0] == 32) && (length == 1)) {
    
    new_trackswitchcommand(_last_trackswitch_address, 0, 0);
    result = true;
  }

  // S88-Autoreset off
  if ((data[0] == 0x80) && (length == 1)) {  
    _s88_autoreset = true;
    result = true;
  }

  // S88-Autoreset on
  if ((data[0] == 0xC0) && (length == 1)) {  
    _s88_autoreset = false;
    result = true;
  }

  // S88-Decoder auslesen
  if (((data[0] > 0x80) && (data[0] <=0x9F)) && (length == 1)) {

    /* Wieder ein Bug in Rocrail: Rocrail sendet in P50X 0x80 + die Anzahl der Halbmodule (Bytes)
    und nicht die Anzahl der Module zu je zwei Bytes, wie es Maerklin in P50 vorgesehen hat. :(
    */

    uint8_t sensormodules = data[0] - 0x80;
      
    for (uint8_t modul_nr = 0; modul_nr < sensormodules; modul_nr++) {

#ifdef USE_S88
      SERIALPORT.write((byte)get_S88(modul_nr, 0));
      SERIALPORT.write((byte)get_S88(modul_nr, 1));
#else
      SERIALPORT.write((byte)0x00);
      SERIALPORT.write((byte)0x00);
#endif
    }
    
    /*
    uint8_t sensorbytes = data[0] - 0x80;

    for (uint8_t byte_nr = 1; byte_nr <= sensorbytes; byte_nr++) {
#ifdef USE_S88
      SERIALPORT.write((byte)get_S88(((byte_nr - 1) / 2), ((byte_nr - 1) % 2)));
#else
      SERIALPORT.write((byte)0x00);
#endif
    }
    */

    result = true;
  }

  if ((data[0] > 0xC0) && (data[0] <= 0xDF) && (length == 1)) {
    uint8_t modul_nr = data[0] - 0x80;
#ifdef USE_S88
      SERIALPORT.write((uint8_t)get_S88(modul_nr, 0));
      SERIALPORT.write((uint8_t)get_S88(modul_nr, 1));
#else
      SERIALPORT.write((uint8_t)0x00);
      SERIALPORT.write((uint8_t)0x00);
#endif
  }

  // Weichen-Befehl
  if (((data[0] == 33) || (data[0] == 34)) && (length == 2)) { 
    uint8_t subaddress  = ((data[1] - 1) % 4) * 2; 
    _last_trackswitch_address = ((data[1] - 1) / 4) + 1;

    if (data[0] == 33) {
      subaddress++;
    }
    
    new_trackswitchcommand(_last_trackswitch_address, subaddress, 1);
    result = true;
  }

  // Lok-Befehl
  if ((data[0] <= 31) && (length == 2)) {                 
    uint8_t speed = data[0] & 0b00001111;
    uint8_t function = (data[0] & 0b00010000) >> 4;
    uint8_t address = data[1];

    if (address < 80) {
      add_loco(address, PROTOCOL);
    }
    else {
      add_loco(address, PROTOCOL_II);
    }

    if(speed == 15) {
      change_dir(address);
    }
    else {
      set_speed(address, speed);
    }
        
    set_function(address, 0, function);
    commit(address);
    result = true;
  }

  if ((data[0] >= 64) && (data[0] <= 79) && (length == 2)) {
    
    uint8_t functions = data[0] - 64;
    uint8_t address = data[1];
      
    add_loco(address, PROTOCOL);
    
    uint8_t f1 = functions & 1;
    uint8_t f2 = (functions & 2) >> 1;
    uint8_t f3 = (functions & 4) >> 2;
    uint8_t f4 = (functions & 8) >> 3;
      
    set_function(address, 1, f1);
    set_function(address, 2, f2);
    set_function(address, 3, f3);
    set_function(address, 4, f4);
    commit(address);
    result = true;
  }


  // Debug
  /*
  SERIALPORT_DEBUG.print(" P50: ");

  for (unsigned int counter = 0; counter < length; counter++)
  {
    SERIALPORT_DEBUG.print(data[counter], HEX);
    SERIALPORT_DEBUG.print(" ");
  }

  SERIALPORT_DEBUG.print("  result: ");
  SERIALPORT_DEBUG.println(result, BIN);
  */

  return(result);
}

boolean Ardurail_P50X::process_p50a_command(unsigned char* data, unsigned int length) {
  
  boolean result = false;

  // Help
  if ((strcmp((char*)data, "?") == 0) || (strcmp((char*)data, "H") == 0)) {
    SERIALPORT.println("Ardurail V 1.0.0 P50X Copyright (C) 2012 Jan Weller jan.weller@gmx.de");
    SERIALPORT.println();
    SERIALPORT.println("HL, HT, HF, STOP, GO, HALT, L");
    result = true;
  }
  else

  // Loco Help
  if (strcmp((char*)data, "HL") == 0) {
    SERIALPORT.print("L {Lok#, [Speed], [FL], [Dir], [F1], [F2], [F3], [F4]}");
    result = true;
  }
  else

  // Turnout Help
  if (strcmp((char*)data, "HT") == 0) {
    SERIALPORT.print("T {Trnt#, [Color], [Status]}");
    result = true;
  }
  else

  // Function Help
  if (strcmp((char*)data, "HF") == 0) {
    SERIALPORT.print("F {Lok#, [F1], [F2], [F3], [F4], [F5], [F6], [F7], [F8]}");
    result = true;
  }
  else

  // Stop
  if ((strcmp((char*)data, ".") == 0) || (strcmp((char*)data, "STOP") == 0)) {
    set_power(false);
    SERIALPORT.print("Pwr off");
    result = true;  
  }
  else

  // Go
  if ((strcmp((char*)data, "!") == 0) || (strcmp((char*)data, "GO") == 0)) {
    set_power(true);
    SERIALPORT.print("Pwr on");
    result = true;  
  }
  else

  // Halt-command
  if (strcmp((char*)data, "HALT") == 0) {
    set_halt();
    SERIALPORT.print("Halted!");
    result = true;
  }
  else {

    // extract Command
    char* command = strtok((char*)data, "{");
    trim((unsigned char*)command);

    // Loco-command
    if (strcmp(command, "L") == 0) {

      uint8_t speed;
      uint8_t f0;
      uint8_t dir;
      uint8_t f1;
      uint8_t f2;
      uint8_t f3;
      uint8_t f4;

      uint8_t address = (uint8_t)atoi(strtok(NULL, ","));
      char* ptr_string = strtok(NULL, ",");

      if (ptr_string != NULL) {
        speed = (uint8_t)atoi(ptr_string);
        f0 = (uint8_t)atoi(strtok(NULL, ","));
        dir = (uint8_t)atoi(strtok(NULL, ","));
        f1 = (uint8_t)atoi(strtok(NULL, ","));
        f2 = (uint8_t)atoi(strtok(NULL, ","));
        f3 = (uint8_t)atoi(strtok(NULL, ","));
        f4 = (uint8_t)atoi(strtok(NULL, ","));

        if (address < 80) {
          add_loco(address, PROTOCOL);
        }
        else {
          add_loco(address, PROTOCOL_II);
        }

        set_function(address, 0, f0);
        set_function(address, 1, f1);
        set_function(address, 2, f2);
        set_function(address, 3, f3);
        set_function(address, 4, f4);

        /*
        speed = 0: halt
        speed = 1: emergency stop
        speed = 2: FS 1
          ...
        speed = 127: FS max
        */

        if (speed == 1) {
          emergency_stop(address);
          result = true;
          return(true);
        }

        if (speed >= 2)
          speed--;

        uint8_t protocol = get_loco(address);
        switch (protocol) {
          case M1:
            speed = (speed + 8) / 9;
          break;
          
          case M2:
            speed = (speed + 8) / 9;
          break;
          
          case M2_28_80A:
            speed = (speed * 2 + 7) / 9;
          break;

          case M2_14_256A:
            speed = (speed + 8) / 9;
          break;

          case M2_28_256A:
            speed = (speed * 2 + 7) / 9;
          break;
        }

        set_dir(address, dir);
        set_speed(address, speed);
        commit(address);
      }

      // Antwort
      speed = get_speed(address);

      if (speed > 1) {
  
        uint8_t protocol = get_loco(address);

        switch (protocol) {
          case M1:
            speed = speed * 9 + 1;
          break;
          
          case M2:
            speed = speed * 9 + 1;
          break;
          
          case M2_28_80A:
            speed = speed /2 * 9;
          break;

          case M2_14_256A:
            speed = speed * 9 + 1;
          break;

          case M2_28_256A:
            speed = speed / 2 * 9;
          break;
        }
      }

      SERIALPORT.print("L ");
      SERIALPORT.print(address);
      SERIALPORT.write(0x20);
      SERIALPORT.print(speed);
      SERIALPORT.write(0x20);
      SERIALPORT.print(get_function(address, 0));
      SERIALPORT.write(0x20);
      SERIALPORT.print(get_dir(address));
      SERIALPORT.write(0x20);
      SERIALPORT.print(get_function(address, 1));
      SERIALPORT.write(0x20);
      SERIALPORT.print(get_function(address, 2));
      SERIALPORT.write(0x20);
      SERIALPORT.print(get_function(address, 3));
      SERIALPORT.write(0x20);
      SERIALPORT.print(get_function(address, 4));

      result = true;
    }
    else

    // Turnout-command
    if (strcmp(command, "T") == 0) {
      uint8_t address = (uint8_t)atoi(strtok(NULL, ","));
      char* ptr_string = strtok(NULL, ",");

      if (ptr_string != NULL) {
        char color = (char)(*ptr_string);
        uint8_t state = (uint8_t)(atoi(strtok(NULL, ",")));
        if ((color == '0') || (color == 'r')) {
          color = 0;
        } else {
          if ((color == '1') || (color == 'g')) {
            color = 1;
          }
        }
        if (state > 1) {
          state = 1;
        }
        _last_trackswitch_address = ((address - 1) / 4) + 1;
        uint8_t subaddress = ((address - 1) % 4) * 2;
        if (color == 1) {
          subaddress++;
        }

        new_trackswitchcommand(_last_trackswitch_address, subaddress, state);
        result = true;
      }
    }
  }

  // Prompt
  if (result == true) {
    SERIALPORT.write(0x0A); // LF
    SERIALPORT.write(0x0D); // CR
    SERIALPORT.write(0x5D); // [
  }

  // Debug
  /*
  SERIALPORT_DEBUG.print(" P50a: ");

  length = strlen((char*)data);

  for (uint8_t counter = 0; counter < length; counter++)
  {
    SERIALPORT_DEBUG.print(data[counter], HEX);
    SERIALPORT_DEBUG.print(" ");
  }

  SERIALPORT_DEBUG.print("  result: ");
  SERIALPORT_DEBUG.println(result, BIN);
  */

  return(result);
}

boolean Ardurail_P50X::process_p50b_command(unsigned char data[80], unsigned int length) {
  boolean result = false;

  switch (data[1])
  {
    // XLok
    case 0x80:
      if (length == 6) {
        uint8_t lsb = data[2];
        //byte msb = data[3];
        uint8_t speed = data[4];
        uint8_t functions = data[5];
        
        // 16 Bit Address
        //uint16_t address = (msb << 8) | lsb;

        // 8 Bit Address
        uint8_t address = lsb;

        if (address < 80) {
          add_loco(address, PROTOCOL);
        }
        else {
          add_loco(address, PROTOCOL_II);
        }

        boolean dir = (functions & 0b00100000) >> 5;
        boolean fl = (functions & 0b00010000) >> 4;

        if ((functions & 0b10000000) == 0b10000000) {
          set_function(address, 1, functions & 0b00000001);
          set_function(address, 2, (functions & 0b00000010) >> 1);
          set_function(address, 3, (functions & 0b00000100) >> 2);
          set_function(address, 4, (functions & 0b00001000) >> 3);
        }

        /*
        speed = 0: halt
        speed = 1: emergency stop
        speed = 2: FS 1
        ...
        speed = 127: FS max
        */

        if (speed == 1) {
          emergency_stop(address);
          SERIALPORT.write((byte)0x00);
          result = true;
          break;
        }

        if (speed >= 2)
          speed--;

        byte protocol = get_loco(address);
        switch (protocol) {
          
          case M1:
            speed = (speed + 8) / 9;
          break;
          
          case M2:
            speed = (speed + 8) / 9;
          break;
          
          case M2_28_80A:
            speed = (speed * 2 + 7) / 9;
          break;

          case M2_14_256A:
            speed = (speed + 8) / 9;
          break;

          case M2_28_256A:
            speed = (speed * 2 + 7) / 9;
          break;

          case DCC_28:
            speed = (speed * 2 + 7) / 9;
          break;

          case DCC_128:
          break;
        }

        set_function(address, 0, fl);
        set_dir(address, dir);
        set_speed(address, speed);
        commit(address);

        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

    // XFunc
    case 0x88:
      if (length == 5) {
        uint8_t lsb = data[2];
        //uint8_t msb = data[3];
        uint8_t functions = data[4];

        // 16 Bit Address
        //uint16_t address = (msb << 8) | lsb;

        // 8 Bit Address
        uint8_t address = lsb;

        set_function(address, 1, functions & 0b00000001);
        set_function(address, 2, (functions & 0b00000010) >> 1);
        set_function(address, 3, (functions & 0b00000100) >> 2);
        set_function(address, 4, (functions & 0b00001000) >> 3);
        set_function(address, 5, (functions & 0b00010000) >> 4);
        set_function(address, 6, (functions & 0b00100000) >> 5);
        set_function(address, 7, (functions & 0b01000000) >> 6);
        set_function(address, 8, (functions & 0b10000000) >> 7);
        
        commit(address);
        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

    // XTrnt
    case 0x90:
      if (length == 4) {
        uint8_t lsb = data[2];
        uint8_t msb = data[3];
        boolean color = (boolean)((msb & 0b10000000) >> 7);
        boolean state = (boolean)((msb & 0b01000000) >> 6);
        uint8_t address = ((lsb - 1) / 4) + 1;
        uint8_t subaddress = ((lsb - 1) % 4) * 2;
        if (color == 1) 
        {
          // 0 = red = turnout
          // 1 = green = straight
          subaddress++;
        }

        new_trackswitchcommand(address, subaddress, state);

        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

    // XSensor 
    case 0x98:
      if (length == 3) {
        uint8_t modul_nr = data[2];
        if (modul_nr > _s88_max_s88) {
          SERIALPORT.write(0x02);   // Moduladresse außerhalb des gültigen Bereichs
        }
        else {
          SERIALPORT.write((byte)0x00);
          SERIALPORT.write((byte)get_S88_accumulator(modul_nr, 0));    // Kontakte #1..8 (Bits #7..0)
          SERIALPORT.write((byte)get_S88_accumulator(modul_nr, 1));    // Kontakte #9..16 (Bits #7..0)
          result = true;
          if (_s88_autoreset) {
            reset_S88_accumulator(modul_nr);
          }
        }
      }
    break;

    // XSensOff
    case 0x99:
      if (length == 2) {
        for (int modul_nr = 0; modul_nr < _s88_max_s88; modul_nr++) {
          if ((get_S88_accumulator(modul_nr, 0) != 0) || (get_S88_accumulator(modul_nr, 1) != 0)) {
            feedbackdevice[modul_nr].modified = true;
            _event_sensors = true;
          }
        }
        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

    // X88PGet 
    case 0x9C:
      if (length == 3) {
        uint8_t parameter_nr = data[2];
        if (parameter_nr == 0x00) {
          SERIALPORT.write((byte)0x00);
          SERIALPORT.write((byte)_s88_max_s88 * 2);
        }
        if (parameter_nr == 0x03) {
          SERIALPORT.write((byte)0x00);
          SERIALPORT.write((byte)_s88_autoreset);
        }
        result = true;
      }
    break;

    // X88PSet 
    case 0x9D:
      if (length == 4) {
        uint8_t parameter_nr = data[2];
        uint8_t parameter_value = data[3];
        if (parameter_nr == 0x00) {
          if ((parameter_value & 1) == 1) {
            _s88_max_s88 = parameter_value / 2 + 1;
          }
          else {
            _s88_max_s88 = parameter_value / 2;
          }
          
        }
        if (parameter_nr == 0x03) {
          _s88_autoreset = (boolean)parameter_value;
        }
        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

    // Versionsnummer
    case 0xA0:
      if (length == 2) {

        // Versionsnummer: 2.0.1a
        SERIALPORT.write(0x04);
        SERIALPORT.write(0x02); 
        SERIALPORT.write((byte)0x00);
        SERIALPORT.write(0x01);
        SERIALPORT.print('a');

        // Seriennummer: 0001
        SERIALPORT.write(0x04);
        SERIALPORT.write((byte)0x00); 
        SERIALPORT.write((byte)0x00);
        SERIALPORT.write((byte)0x00);
        SERIALPORT.write(0x01);
        SERIALPORT.write((byte)0x00);
        
        result = true;
      }
    break;
    
    // XStatus
    case 0xA2:
      if (length == 2) {
        uint8_t status = ((get_power() & 1) << 3) | ((get_halt() & 1) << 4);
        SERIALPORT.write((byte)status);
        result = true;
      }
    break;

    // XHalt
    case 0xA5:
      if (length == 2) {
        set_halt();
        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

    // XPwrOff
    case 0xA6:
      if (length == 2) {
        set_power(false);
        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

    // XPwrOn
    case 0xA7:
      if (length == 2) {
        set_power(true);
        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

    // XNop
    case 0xC4:
      if (length == 2) {
        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

     // XEvent
    case 0xC8:
      if (length == 2) {
        uint8_t eventbyte0 = 0x80 | (((byte)_event_trnt & 1) << 5) | (((byte)_event_pwoff & 1) << 3) 
                                  | (((byte)_event_sensors & 1) << 2) | ((byte)_event_lok & 1);
        uint8_t eventbyte1 = 0x80 | (((byte)_event_sts & 1) << 6) | (((byte)_event_short) & 1);
        uint8_t eventbyte2 = 0x00;

        SERIALPORT.write((byte)eventbyte0);
        SERIALPORT.write((byte)eventbyte1);
        SERIALPORT.write((byte)eventbyte2);

        _event_trnt = false;
        _event_pwoff = false;
        _event_sensors = false;
        _event_lok = false;
        _event_sts = false;
        _event_short = false;

        result = true;
      }
    break;

    // XEvtLok
    case 0xC9:
      if (length == 2) {
        SERIALPORT.write(0x80);
        result = true;
      }
    break;

    // XEvtTrn
    case 0xCA:
      if (length == 2) {
        SERIALPORT.write((byte)0x00);
        result = true;
      }
    break;

    // XEvtSen 
    case 0xCB:
      if (length == 2) {
        for (byte modul_nr = 0; modul_nr < _s88_max_s88; modul_nr++) {
          if (feedbackdevice[modul_nr].modified) {
            uint8_t lsb = get_S88(modul_nr, 0);
            uint8_t msb = get_S88(modul_nr, 1);
            SERIALPORT.write(modul_nr + 1);
            SERIALPORT.write((byte)lsb);
            SERIALPORT.write((byte)msb);
            if (_s88_autoreset) {
              feedbackdevice[modul_nr].modified = false;
              feedbackdevice[modul_nr].lsb = lsb;
              feedbackdevice[modul_nr].msb = msb;
            }
          }
        }
        SERIALPORT.write((byte)0x0);
        result = true;
      }
    break;

    default:
      result = false;
    break;
  }

  // Debug
  /*
  SERIALPORT_DEBUG.print(" P50b: ");

  for (unsigned int counter = 0; counter < length; counter++) {
    SERIALPORT_DEBUG.print(data[counter], HEX);
    SERIALPORT_DEBUG.print(" ");
  }

  SERIALPORT_DEBUG.print("  result: ");
  SERIALPORT_DEBUG.println(result, BIN);
  */

  return(result);
}

void Ardurail_P50X::p50X(void)
{ 
  boolean result = false;
  uint8_t len;

  S88();

  if (SERIALPORT.available() > 0) {
    _last_serialbyte = millis();

    unsigned char buffer = SERIALPORT.read();
  
    p50_p50b_input_line[p50_p50b_lenght++] = buffer;

    len = strlen((char*)p50a_input_line);
    p50a_input_line[len] = buffer;
    p50a_input_line[len + 1] = '\0';
  }

  // Mixed-mode (P50 and P50X)
  if (_mode == MIXED) {

    // P50Xb or P50
    if ((_last_serialbyte + (1000*30/BAUD)) <= millis()) {
      
      if (((p50_p50b_input_line[0] == 'x') || (p50_p50b_input_line[0] == 'X')) && (p50_p50b_lenght > 1)) {
        result = process_p50b_command(p50_p50b_input_line, p50_p50b_lenght);
      }
      else {
        result = process_p50_command(p50_p50b_input_line, p50_p50b_lenght);
      }
    }

    /*
    // P50Xa
    len = strlen((char*)p50a_input_line);
    if ((len > 0) && ((p50a_input_line[0] == 'x') || (p50a_input_line[0] == 'X')) && (p50a_input_line[len - 1] == '\r')) {
      p50a_input_line[0] = 0x20;
      p50a_input_line[len - 1] = '\0';
      trim(p50a_input_line);
      len = strlen((char*)p50a_input_line);
      result = process_p50a_command(p50a_input_line, len);
      p50a_input_line[0] = '\0';
    }
    */
  }
  
  // P50X-only-mode (only P50X without 'x' or 'X')
  else {
    if ((p50_p50b_lenght > 0) && (_last_serialbyte + (1000*30/BAUD)) <= millis()) {
      result = process_p50b_command(p50_p50b_input_line, p50_p50b_lenght);
    }
  }

  // Command successful
  if (result == true) {
    p50_p50b_input_line[0] = '\0';
    p50_p50b_lenght = 0;
    p50a_input_line[0] = '\0';
    disable_p50_p50b = false;
  }
  // or timeout for P50 and P50b
  else {
    if ((_last_serialbyte + 500) <= millis()) {
      p50_p50b_input_line[0] = '\0';
      p50_p50b_lenght = 0;
      disable_p50_p50b = false;
    }
  }

  blow_out_trackswitchcommand_buffer();

}

void Ardurail_P50X::trim(unsigned char* data) {
  uint8_t left = 0;
  uint8_t right = strlen((char*)data);

  if (right == 0){
    return;
  }

  while (((char*)data)[left] == 0x20) {
    left++;
  }

  for (uint8_t loop = left; loop <= right; loop++) {
    ((char*)data)[loop - left] = ((char*)data)[loop];
    ((char*)data)[loop] = '\0';
  }

  right = strlen((char*)data);

  while (((char*)data)[right - 1] == 0x20) {
    ((char*)data)[--right] = '\0';
  }
}
