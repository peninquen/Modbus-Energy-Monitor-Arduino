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
//------------------------------------------------------------------------------
// Poner DEBUG a 1 para depuración.

#define DEBUG  0

//------------------------------------------------------------------------------
// Debug directives

#if DEBUG
#   define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
#   define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
#   define DEBUG_PRINT(...)
#   define DEBUG_PRINTLN(...)
#endif


#include "ModbusSensor.h"

