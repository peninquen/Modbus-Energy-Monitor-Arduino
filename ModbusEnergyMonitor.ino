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
PULSE ELECTRICAL ENERGY MONITOR
DDS238-1 SINGLE-PHASE DIN-RAIL TYPE WATT-HOUR METER
TECHNICAL DETAILS
STANDARD IEC 62053-21 (IEC61036)
Nominal voltage: 120/220/230/240 V +-10%
Basic current (It): 5 A
Maximun current (Ib1) 32 A
Minimun current: 0.02 A
Frequency: 50/60 Hz
Consumption: <2w / 10 VA
Accurancy class: 1
Display: Mechanical 5+1 digits
         LCD 5+1 / 6+1 digits
Interface: Open colector output (SO)  SO+ ---------------------- PIN 2 (INT0) INTERNAL PULLUP RESISTOR
           18-27V 27mA                SO- ---------------------- GND
Impulse length: >= 30ms
units conversion:
Energy: 3200 imp = 1 KWh -> 1 imp = 0,3125 Wh;
Power: 1 imps/s = 1125 W;
Current(voltage 230V)-> 1 imp/s = 4.89 A;
maximum impulse frequency (30 A): 6.13 imp/s -> period 163 ms
basic frequency (5 A): 1.02 imp/s -> period 978.3 ms
minimun impulse frequency (0.02 A): 1.0 imp/min-> period 60000 ms
Mounting: DIN rail 18 mm (1 module)
include flasing led proportional to load (1000 imp/KWh)
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

#include "ModbusSensor.h"

#define SERIAL_PORT 0 // Arduino only one serial port, Mega has 3 serial ports.
#define ID_SLAVE 1
#define REFRESH_INTERVAL  5000      // refresh time, 5 SECONDS
#define WRITE_INTERVAL 300000UL  // values send to serial port, 15 minutes (5 * 60 * 1000)
#define KWH_2_WS 36000000 
ModbusSensor wattmeter; // instance to collect data
//variables to process and send values
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

