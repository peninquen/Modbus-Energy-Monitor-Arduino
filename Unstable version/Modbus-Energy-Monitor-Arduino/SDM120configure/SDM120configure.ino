/**********************************************************************
  SDM120 configure
  Sketch to configure and test de parameters of a modbus energy monitor
  model EASTRON SDM120, SDM220, SDM320, SDM630
  this sketch uses two hardsare serial ports:
  - Serial to comunicate to monitor serial via USB
  - Serial1 to comunicate to modbus RS485 SDM single device.
  TX enable pin 17
  Default SDM configuration:
  - Id number: 1
  - Baud rate: 2400
  - parity bit: none

    version 0.5 BETA 4/01/2016

  Author: Jaime García  @peninquen
  License: Apache License Version 2.0.

**********************************************************************/

#include "ModbusSensor.h"
#include "SDMdefines.h"

#define MB_SERIAL_PORT &Serial1   // Arduino has only one serial port, Mega has 3 serial ports.

#define MB_BYTEFORMAT     SERIAL_8N2    // Prty n
#define TxEnablePin       17

// Forward declarations
void  id_configure();
void  baudrate_configure();
void  turnDisplay_configure();
void  pulse1kwh_configure();
void  energyMode_configure();
void  pulse1Mode_configure();


uint8_t idNumber = 1;                  // Id 001  modbus id of the energy monitor
uint16_t baudRate = 2400;              // b 2400
uint16_t byteFormat = SERIAL_8N2;      // Prty n
#define REFRESH_INTERVAL  1000         // refresh time, 1 SECOND

void setup() {

  Serial.begin(9600);
  delay(95);
  Serial.println(F("Config SDM120 - Modbus"));
  MBSerial.config(MB_SERIAL_PORT, TxEnablePin, REFRESH_INTERVAL);
  MBSerial.begin(baudRate, byteFormat);

  id_configure();
  baudrate_configure();
  turnDisplay_configure();
  pulse1kwh_configure();
  energyMode_configure();
  pulse1Mode_configure();
}

void loop() {} // no hace nada...

//-----------------------------------------------------------------------------------------
//Procesa el valor de registro id
void id_configure() {
  Serial.println(F("Enter expected Id: "));
  while (!Serial.available()) {}
  idNumber = Serial.parseInt();

  modbusSensor id(idNumber, DEVICE_ID, HOLD_VALUE, sizeof(float), READ_HOLDING_REGISTERS);
  while (!MBSerial.available()) {}

  Serial.print(F("Meter Id: ")); Serial.println(id.read(), 0);
  Serial.print(F("New Id: "));
  while (!Serial.available()) {}
  int16_t iParameter = Serial.parseInt();
  if (iParameter >= 1 && iParameter <= 247) {
  float fParameter = (float)iParameter;
    id.preset(fParameter);
    if (id.printStatus() != MB_VALID_DATA) return;
    idNumber == iParameter; // actuliza el id para las siguientes peticiones
  }
  else Serial.println(F("Skip"));
}

//-----------------------------------------------------------------------------------------
//Procesa el valor de registro baud rate
void baudrate_configure() {
  modbusSensor baudrate(idNumber, BAUD_RATE, HOLD_VALUE, sizeof(float), READ_HOLDING_REGISTERS);
  while (!MBSerial.available()) {}
  float defBaudRate = baudrate.read(); //default
  Serial.print(F("Baud rate (0:2400 1:4800 2:9600 5:1200): "));
  Serial.println(defBaudRate, 0);
  Serial.print(F("New baud rate: "));
  while (!Serial.available()) {}
  int16_t iParameter = Serial.parseInt();
  switch (iParameter) {
    case 0:
      baudRate = 2400;
      break;
    case 1:
      baudRate = 4800;
      break;
    case 2:
      baudRate = 9600;
      break;
    case 5:
      baudRate = 1200;
      break;
    default:
      Serial.println(F("Skip"));
      return;
  }
  float fParameter = (float)iParameter;
  Serial.println(fParameter, 0);
  baudrate.preset(fParameter);
  if(baudrate.printStatus() != MB_VALID_DATA) return;
  if (defBaudRate != iParameter) {
    MBSerial.end();
    MBSerial.begin(baudRate, byteFormat);
  }
}

//-----------------------------------------------------------------------------------------
//Procesa el valor de registro 'tiempo entre pantallas'
void turnDisplay_configure() {
  uint16_t bcd;
  modbusSensor turnDisplay(idNumber, TIME_DISP, HOLD_VALUE, sizeof(bcd), READ_HOLDING_REGISTERS);
  while (!MBSerial.available()) {}
  Serial.print(F("Time of display in turns (0 - 30 seconds): "));
  Serial.print(turnDisplay.read(bcd), HEX);
  Serial.print(F("New time: "));
  while (!Serial.available()) {}
  bcd = Serial.parseInt();
  if (bcd >= 0 && bcd <= 30) {
    bcd = bcd % 10 | (bcd / 10 << 4); //convert to BCD
    Serial.println(bcd, HEX);
    turnDisplay.preset(bcd);
    if (turnDisplay.printStatus() != MB_VALID_DATA) return;
  }
  else Serial.println(F("Skip"));
}

//-----------------------------------------------------------------------------------------
//Procesa el valor de registro 'salida pulso 1'
void pulse1kwh_configure() {
  uint16_t hex;
  modbusSensor pulse1kwh(idNumber, PULSE1_KWH, HOLD_VALUE, sizeof(hex), READ_HOLDING_REGISTERS);
  while (!MBSerial.available()) {}
  Serial.print(F("Pulse 1 output (0:1000, 1:100, 2:10 3:1 imp/Kwh): "));
  Serial.println(pulse1kwh.read(hex), HEX);
  Serial.print(F("New pulse 1 output value: "));
  while (!Serial.available()) {}
  hex = Serial.parseInt();
  if (hex >= 0 && hex <= 3) {
    Serial.println(hex, HEX);
    pulse1kwh.preset(hex);
    if (pulse1kwh.printStatus() != MB_VALID_DATA) return;
  }
  else Serial.println(F("Skip"));
}

//-----------------------------------------------------------------------------------------
//Procesa el valor de registro 'modo medida de energía'
void energyMode_configure() {
  uint16_t hex;
  modbusSensor enrgMode(idNumber, TOT_MODE, HOLD_VALUE, sizeof(hex), READ_HOLDING_REGISTERS);
  while (!MBSerial.available()) {}
  Serial.print(F("Measurement mode (0:mode 1, 1:mode 2, 2:mode 3): "));
  Serial.println(enrgMode.read(hex), HEX);
  Serial.print(F("New Measure mode: "));
  while (!Serial.available()) {}
  hex = Serial.parseInt();
  if (hex >= 0 && hex <= 2) {
    Serial.println(hex, HEX);
    enrgMode.preset(hex);
    if (enrgMode.printStatus() != MB_VALID_DATA) return;
  }
  else Serial.println(F("Skip"));
}

//-----------------------------------------------------------------------------------------
//Procesa el valor de registro 'modo salida pulso 1'
void pulse1Mode_configure() {
  uint16_t hex;
  modbusSensor pulse1Mode(idNumber, PULSE1_MODE, HOLD_VALUE, sizeof(hex), READ_HOLDING_REGISTERS);
  while (!MBSerial.available()) {}
  Serial.print(F("Pulse 1 output mode (0:imp+exp, 1:imp, 2:exp): "));
  Serial.println(pulse1Mode.read(hex), HEX);
  Serial.print(F("New pulse 1 output mode: "));
  while (!Serial.available()) {}
  hex = Serial.parseInt();
  if (hex >= 0 && hex <= 2) {
    Serial.println(hex, HEX);
    pulse1Mode.preset(hex);
    if (pulse1Mode.printStatus() != MB_VALID_DATA) return;
  }
  else Serial.println(F("Skip"));

  // Salir del modo -SET- para que se hagan efectivos los cambios, después apagar y encender el SMD120
  Serial.println(F("Now long press for 3 seconds to exit the -SET- mode. Then restart the SMD120 to aply the changes."));
}




