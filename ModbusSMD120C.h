/**********************************************************************
* ModbusSensor class
* A class to collect data from a Modbus energy monitor 
*
* version 0.3 ALPHA 21/09/2015
* 
* Author: Jaime García  @peninquen
* License: Apache License Version 2.0.
*
*
*READ_INPUT_REGISTERS
*
**********************************************************************/

#ifndef ModbusSensor_h
#define ModbusSensor_h

#include "Arduino.h"
#include <SimpleModbusMaster.h>

// Serial ports available
// Arduino UNO
// Arduino LEONARDO
// Arduino MEGA 

extern dataSDM120C;

class ModbusSDM120C {
  public:
    //constructor
    ModbusSDM120C();

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
    unsigned char _id;               // Modbus id
    bool _flag;             // true when data is processed, false when data is readed
    dataSMD120C _voltage;
    dataSMD120C _current;
    dataSMD120C _power;
    dataSMD120C _voltamps;
    dataSMD120C _powerFactor;
    dataSMD120C _FrecuencyOfVoltage;
    dataSMD120C _TotalEnergy;
    dataSMD120C _PositiveEnergy;
    dataSMD120C _ReverseEnergy;
    
};

#endif
