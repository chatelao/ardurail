/* ArdurailSRCPSerial 1.0
 
 (C) 2013 Holger Wirtz <dcoredump@googlemail.com>
 
 SRCP code originally written and taken with permission of Marcel Bernet from
 https://github.com/mc-b/microSRCP/wiki/Srcp-protokoll
 
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

#include "SRCPCommand.h"
#include "SRCPMessages.h"
#include "SRCPParser.h"

// SRCP Kommando Struktur
srcp::command_t cmd;
// I/O Buffer
char buf[64];
// SRCP Parser
srcp::SRCPParser parser;
// Anzahl Session bzw. GO
int counter = 0;

Ardurail MM_CENTRAL;

void setup(void)
{
  Serial.begin(115200);

  MM_CENTRAL.init(6,7,8);
  delay(100);
}

void loop(void)
{
  if(receive())
  {
    char* rc=dispatch();
    Serial.println(rc);
  }
}

int receive()
{
  int count = 0;
  
  if(Serial.available())
  {
    while(true)
    {
      if(!Serial.available())
        continue;

      int i=Serial.read();
      // NL beendet lesen
      if(i=='\n')
        break;
      // Sonderzeichen ignorieren
      if (i=='\r')
        continue;
      buf[count++]=i;
    }
    buf[count]='\0';

    // SRCP ASCII parsen und in command_t abstellen
    parser.parse(cmd,buf);
    return(true);
  }
  return(false);
}

char* dispatch()
{
  switch (cmd.cmd)
  {
    case srcp::GO:
      MM_CENTRAL.set_power(true);
      return(srcp::Messages.go(++counter));
    case srcp::INIT:
      if(cmd.values[0]!='M')
        return(srcp::Messages.error(420));
      else
        return(srcp::Messages.ok());
    case srcp::SET:
      switch (cmd.device)
      {
        // Generic Loco
        case srcp::GL:
          // direction
          switch(cmd.values[0])
          {
            case 0:
              if(MM_CENTRAL.get_dir(cmd.addr)==true)
                MM_CENTRAL.set_dir(cmd.addr,false);
              break;
            case 1:
              if(MM_CENTRAL.get_dir(cmd.addr)==false)
                MM_CENTRAL.set_dir(cmd.addr,true);
              break;
            case 2:
              MM_CENTRAL.emergency_stop(cmd.addr);
              break;
          }
          // speed
          if(cmd.values[1]>=0 && cmd.values[1]<=cmd.values[2] && cmd.values[2]>0)
            MM_CENTRAL.set_speed(cmd.addr,(byte)(cmd.values[1]/cmd.values[2]*(float)MM_CENTRAL.get_speedsteps(cmd.addr)+0.5));
          else
            return(srcp::Messages.error(412));
          // function
          for(byte i=0;i<4;i++)
          {
            char tmp[10];
            
            if(strncasecmp(itoa(cmd.values[i+3],tmp,10),"=",1)!=0)
              MM_CENTRAL.set_function(cmd.addr,i,cmd.values[i+3]);
          }
          MM_CENTRAL.commit(cmd.addr);
          return(srcp::Messages.ok());
        // Service mode
        case srcp::SM:
          return(srcp::Messages.error(425));
        // Generic Accessoire   
        case srcp::GA:
          MM_CENTRAL.set_track(cmd.addr,cmd.values[0],cmd.values[1]);
          MM_CENTRAL.commit(cmd.addr);
          return(srcp::Messages.ok());
        // Feedback sensors
        case srcp::FB:
          return(srcp::Messages.error(425));
        default:
          return(srcp::Messages.error(421));
      }
      break;
    case srcp::GET:
      switch (cmd.device)
      {
        // Generic Loco
        case srcp::GL:
          char data[64];
          switch(MM_CENTRAL.get_function(cmd.addr,1))
          {
             case false:
               sprintf(data,"%d %d %d %d",MM_CENTRAL.get_dir(cmd.addr),MM_CENTRAL.get_speed(cmd.addr),MM_CENTRAL.get_speedsteps(cmd.addr),MM_CENTRAL.get_function(cmd.addr,0));
               break;
             case true:
               sprintf(data,"%d %d %d %d %d %d %d %d",MM_CENTRAL.get_dir(cmd.addr),MM_CENTRAL.get_speed(cmd.addr),MM_CENTRAL.get_speedsteps(cmd.addr),MM_CENTRAL.get_function(cmd.addr,0),MM_CENTRAL.get_function(cmd.addr,1),MM_CENTRAL.get_function(cmd.addr,2),MM_CENTRAL.get_function(cmd.addr,3),MM_CENTRAL.get_function(cmd.addr,4));
               break;
          }
          return(srcp::Messages.info(cmd.bus,cmd.addr,data));
        // Service mode
        case srcp::SM:
          return(srcp::Messages.error(425));
        // Generic Accessoire   
        case srcp::GA:
          return(srcp::Messages.error(416));
        // Feedback sensors
        case srcp::FB:
          return(srcp::Messages.error(425));
        default:
          return(srcp::Messages.error(421));
      }
      break;
    case srcp::CONNECTIONMODE:
      return(srcp::Messages.ok202());
    case srcp::PROTOCOL:
      return(srcp::Messages.ok201());
    default:
      return(srcp::Messages.ok());
  }
  return(srcp::Messages.ok());
}
