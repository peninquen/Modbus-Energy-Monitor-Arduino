/**********************************************************************
* ModbusEnergyMonitor example
* An example to collect data from a Modbus energy monitor using ModbusSensor class
* to datalogger, include a RTC DS3231 and a SD card
* version 0.3 ALPHA 21/09/2015
* 
* Author: Jaime García  @peninquen
* License: Apache License Version 2.0.
*
**********************************************************************/
/*

*/
#define SERIAL_OUTPUT 1

#if SERIAL_OUTPUT
#   define SERIAL_BEGIN(...) Serial.begin(__VA_ARGS__)
#   define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#   define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#   define SERIAL_BEGIN(...)
#   define SERIAL_PRINT(...)
#   define SERIAL_PRINTLN(...)
#endif

#include "ModbusSDM120C.h"
#define TxEnablePin 2

#define MODBUS_SERIAL_PORT &Serial1 // Arduino has only one serial port, Mega has 3 serial ports.
// if use Serial 0, remember disconect Tx (pin0) when upload sketch, then re-conect
#define ID_SDM120C      // modbus id of the energy monitor
#define REFRESH_INTERVAL  5000      // refresh time, 5 SECONDS, equivalent to poll time
#define WRITE_INTERVAL 300000UL  // values send to serial port, 15 minutes (5 * 60 * 1000)
#define KWH_2_WS 36000000 
ModbusSDM120C wattmeter; // instance to collect data
//variables to process and send values
float voltage;
float maxVoltage;
float minVoltage;
float current;
float maxCurrent;
float minCurrent;
float power;
float maxPower;
float minPower;
float energy;
float lastEnergy;
boolean firstData;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;


void setup() {
  SERIAL_BEGIN(9600);
  wattmeter.begin(SERIAL_PORT, ID_SLAVE, REFRESH_INTERVAL);
  SERIAL_PRINTLN("time(s), average power(W), max power(W), min power(W), energy(KWh)");

  firstData = false;
  power = 0;
  maxPower = 0;    // in case it has been recorded, use it
  minPower = 0;
  lastEnergy = 0;  // in case it has been recorded, use it
  energy = lastEnergy;
}

void loop() {
  sei();
  if (wattmeter.available()) {
    power = wattmeter.read();
    if (!firstData) {
      if (maxPower < power) maxPower = power;
      if (minPower > power) minPower = power;
    }
    else {
      maxPower = power;
      minPower = power;
      firstData = false;
    }
    // backup energy, maxPower
  }

  currentMillis = millis();
  if (currentMillis - previousMillis >= WRITE_INTERVAL) {
    previousMillis = currentMillis;
    energy = wattmeter.readAcum();
    power = (energy - lastEnergy) * KWH_2_WS * 1000 / WRITE_INTERVAL; //average power KWh/s to W
    lastEnergy = energy;
    firstData = true;

    SERIAL_PRINT(currentMillis / 1000);
    SERIAL_PRINT(",");
    SERIAL_PRINT(power);
    SERIAL_PRINT(",");
    SERIAL_PRINT(maxPower);
    SERIAL_PRINT(",");
    SERIAL_PRINT(minPower);
    SERIAL_PRINT(",");
    SERIAL_PRINTLN(energy);

  }
}

