/**********************************************************************
  ModbusEnergyMonitor example
  An example to collect data from a Modbus energy monitor using ModbusSensor class
  to datalogger, include a RTC DS3231 and a SD card
  version 0.4 BETA 31/12/2015

  Author: Jaime García  @peninquen
  License: Apache License Version 2.0.

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

#include "ModbusSensor.h"
#include "SDMdefines.h"

#define MB_SERIAL_PORT &Serial2   // Arduino has only one serial port, Mega has 3 serial ports.
// if use Serial 0, remember disconect Tx (pin0) when upload sketch, then re-conect
#define MB_BAUDRATE       2400          // b 2400
#define MB_BYTEFORMAT     SERIAL_8N2    // Prty n
#define TxEnablePin       17

#define ID_1  1                       // id 001  modbus id of the energy monitor
#define REFRESH_INTERVAL  5000        // refresh time, 5 SECONDS
#define WRITE_INTERVAL 20000UL        // values send to serial port, 1 minute ( 60 * 1000)
#define KWH_2_WS 36000000

// multiplication factor, store value as an integer
#define VOL_FAC 10
#define CUR_FAC 100
#define POW_FAC 10
#define PFA_FAC 100
#define FRE_FAC 10
#define ENE_FAC 100


// global variables to poll, process and send values
modbusSensor volt(ID_1, VOLTAGE, CHANGE_TO_ZERO);
modbusSensor curr(ID_1, CURRENT, CHANGE_TO_ZERO);
modbusSensor pwr(ID_1, POWER, CHANGE_TO_ZERO);
modbusSensor enrg(ID_1, IAENERGY, HOLD_VALUE);
modbusSensor freq(ID_1, FREQUENCY, CHANGE_TO_ZERO);
modbusSensor aPwr(ID_1, APOWER, CHANGE_TO_ZERO);
modbusSensor pwrFact(ID_1, PFACTOR, CHANGE_TO_ONE);

uint16_t voltage, maxVoltage, minVoltage; // integer, factor x10
uint16_t current, maxCurrent, minCurrent; // integer, factor x100
uint16_t power, maxPower, minPower;       // integer, factor x10
uint16_t lastEnergy, energy, avgPower;    // integer, factor x100
uint16_t frequency, maxFreq, minFreq;     // integer, factor x100
uint16_t aPower, maxApower, minApower;    // integer, factor x10
uint16_t powerFactor, maxPF, minPF;       // integer, factor x100

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
boolean       firstData;

void setup() {
  SERIAL_BEGIN(9600);
  MBSerial.config(MB_SERIAL_PORT, TxEnablePin, REFRESH_INTERVAL);
  MBSerial.begin(MB_BAUDRATE, MB_BYTEFORMAT);
  delay(95);
  SERIAL_PRINTLN("time(s), maxVolt(V), minVolt(V), maxCurr(A) minCurr(A), maxPower(W), minPower(W), maxApPower(VA), minApPower(VA), maxFreq(Hz), minFreq(Hz), AvgPower (W), Energy(Kwh)");

  firstData = false;
  power = 0;
  maxPower = 0;    // in case it has been recorded, use it
  minPower = 0;
  lastEnergy = 0;  // in case it has been recorded, use it
  energy = lastEnergy;
}

void loop() {
  sei();
  if (MBSerial.available()) {
    voltage = volt.read(VOL_FAC);
    current = curr.read(CUR_FAC);
    power = pwr.read(POW_FAC);
    aPower = aPwr.read(POW_FAC);
    frequency = freq.read(FRE_FAC);
    energy = enrg.read(ENE_FAC);

    if (!firstData) {
      if (maxVoltage < voltage) maxVoltage = voltage;
      if (minVoltage > voltage) minVoltage = voltage;
      if (maxCurrent < current) maxCurrent = current;
      if (minCurrent > current) minCurrent = current;
      if (maxPower < power) maxPower = power;
      if (minPower > power) minPower = power;
      if (maxApower < aPower) maxApower = aPower;
      if (minApower > aPower) minApower = aPower;
      if (maxFreq < frequency) maxFreq = frequency;
      if (minFreq > frequency) minFreq = frequency;
      if (maxPower < power) maxPower = power;
      if (minPower > power) minPower = power;
    }
    else {
      maxVoltage = voltage;
      minVoltage = voltage;
      maxCurrent = current;
      minCurrent = current;
      maxPower = power;
      minPower = power;
      maxApower = aPower;
      minApower = aPower;
      maxFreq = frequency;
      minFreq = frequency;
      firstData = false;
    }
  }

  currentMillis = millis();
  if (currentMillis - previousMillis >= WRITE_INTERVAL) {
    previousMillis = currentMillis;
    avgPower = (energy - lastEnergy) * KWH_2_WS / (WRITE_INTERVAL / 1000); //average power KWh/s to W
    lastEnergy = energy;
    firstData = true;

    SERIAL_PRINT(currentMillis / 1000);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)maxVoltage / VOL_FAC, 1);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)minVoltage / VOL_FAC, 1);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)maxCurrent / CUR_FAC, 2);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)minCurrent / CUR_FAC, 2);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)maxPower / POW_FAC, 2);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)minPower / POW_FAC, 2);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)maxApower / POW_FAC, 2);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)minApower / POW_FAC, 2);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)maxFreq / FRE_FAC, 2);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)minFreq / FRE_FAC, 2);
    SERIAL_PRINT(",");
    SERIAL_PRINT((float)avgPower / ENE_FAC, 2);
    SERIAL_PRINT(",");
    SERIAL_PRINTLN((float)energy / ENE_FAC, 2);

  }
}


