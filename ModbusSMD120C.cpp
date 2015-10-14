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


#include <SimpleModbusMaster.h>
#include "ModbusSMD120C.h"
/*
Constants are provided for:
  Function 1  - READ_COIL_STATUS
  Function 2  - READ_INPUT_STATUS
  Function 3  - READ_HOLDING_REGISTERS 
  Function 4  - READ_INPUT_REGISTERS
  Function 15 - FORCE_MULTIPLE_COILS
  Function 16 - PRESET_MULTIPLE_REGISTERS 
*/

// Direcciones registros de datos solo lectura. Valores tipo float.
// Utilizar funcion 04 lectura, numero de registros 16-bits 2.

#define VOL_ADR 0X0000    // Voltage
#define CUR_ADR 0X0006    // Corriente
#define POW_ADR 0X000C    // Potencia 
#define VAM_ADR 0X0012    // VoltAmperios.
#define PFA_ADR 0X001E    // Factor de potencia.
#define FRE_ADR 0X0046    // Frecuencia.
#define TOE_ADR 0X0201    // Energia consumida KWH
#define POE_ADR 0XF101    // Energía instantanea consumida.
#define REE_ADR 0XF201    // Energía instantanea generada.
 


#define BAUD_RATE 9600
#define TIMEOUT 1000
#define RETRY_COUNT 10
enum
{
  VOLTAGE,
  CURRENT,
  POWER,
  VOLTAMPS,
  POWERFACTOR,
  FREQUENCYOFVOLTAGE,
  TOTALENERGY
  POSITIVEENERGY,
  REVERSEENERGY,
  TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

packetPointer packet1 = &packets[PACKET1];
union dataSDM120C {
  float value;
  uint16_t data[2];
};


unsigned int readRegs[2];

