/**********************************************************************
* ModbusSensor class
* A class to collect data from a Modbus energy monitor 
*
* version 0.3 ALPHA 21/09/2015
* 
* Author: Jaime García  @peninquen
* License: Apache License Version 2.0.
*
**********************************************************************/

#ifndef ModbusSensor_h
#define ModbusSensor_h

#include "Arduino.h"
#include "ModbusMaster.h"

// Serial ports available
// Arduino UNO
// Arduino LEONARDO
// Arduino MEGA 

struct EnergyMonitor {
  uint16_t voltage;
  
};

class ModbusSensor {
  public:
    //constructor
    ModbusSensor();

    // Setup instance variables
    void begin(int ModbusSerial, unsigned int idModbus, unsigned int interval);

    // check interval and update data
    void refreshData();

    // read v in defined units
    float read();

    // reset
    void reset();

    // is a new data available?
    boolean available();

  private:
    bool _flag;                        // true when data is processed, false when data is readed
    EnergyMonitor energyMonitor;       // struct, buffer with last data received
};

#endif
