

#ifndef _MAERKLINO_P50X_H
#define _MAERKLINO_P50X_H

#include "Ardurail.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define MIXED 0
#define P50X  1

#define SWITCH_TIME_MAX 300
//#define SWITCH_TIME_MIN 200

//
// ######################### DEFAULTS #############################
//

//
// DO NOT CHANGE ANYTHING BEHIND THIS LINE IF YOU DO NOT KNOW WHAT YOU ARE DOING!!!
//

//
// ######################### DEFINITIONS #############################
//

// Protokoll für alle Adressen von 0 bis 79
//#define PROTOCOL  M1          // Maerklin-Motorola I
#define PROTOCOL  M2            // Maerklin-Motorola II
//#define PROTOCOL  M2_28_80A   // Maerklin-Motorola II, 28 speedsteps
//#define PROTOCOL  M2_14_256A  // Maerklin-Motorola II, 256 addresses
//#define PROTOCOL  M2_28_256A  // Maerklin-Motorola II, 28 speedsteps, 256 addresses
//#define PROTOCOL  DCC_28      // old and simple DCC, 28 speedsteps
//#define PROTOCOL  DCC_128     // state of the art DCC, 128 speedsteps
 
// Protokoll für alle Adressen von 80 bis 255
#define PROTOCOL_II  DCC_128

struct feedbackbuffertype
{
  uint8_t  lsb;
  uint8_t  msb;
  boolean modified:1;
};

// packet for Trackswitch
struct MMTrackswitch_buffer
{
  byte address;
  byte subaddress:3;
  boolean state:1;
};

//
// ######################### CLASS #############################
//

class Ardurail_P50X : public Ardurail
{
public:
  void init(byte pin);
  void init(byte pin, byte shortcut_pin, byte go_signal_pin);
  void new_trackswitchcommand(uint8_t address, uint8_t subaddress, uint8_t state);
  void blow_out_trackswitchcommand_buffer();
  uint8_t get_S88_accumulator(uint8_t modul_nr, uint8_t byte_nr);
  void reset_S88_accumulator(uint8_t modul_nr);
  void S88(void);
  void p50X(void);

protected:
  boolean _event_sensors;
  boolean _event_short;
  boolean _event_trnt;
  boolean _event_sts;
  boolean _event_pwoff;
  boolean _event_lok;
  boolean _s88_autoreset;
  uint8_t _mode;
  uint8_t _last_trackswitch_address;
  volatile unsigned long _next_switch_time;
  MMTrackswitch_buffer tswitch_buffer[80];         // Buffer for 80 Trackswitch Commands
  int8_t _waiting_trackswitch_commands;
  feedbackbuffertype feedbackdevice[MAX_S88];

  void trim(unsigned char* data);
  boolean process_p50_command(unsigned char data[80], unsigned int length);
  boolean process_p50a_command(unsigned char* data, unsigned int length);
  boolean process_p50b_command(unsigned char data[80], unsigned int length);
};

//
// ######################### FUNCTIONS #############################
//


#endif 
