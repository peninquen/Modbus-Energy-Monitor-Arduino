/**********************************************************************
* ModbusEnergyMonitor example
* An example to collect data from a Modbus energy monitor using ModbusSensor class
* to datalogger, include a RTC DS3231 and a SD card
* version 0.1 ALPHA 14/12/2015
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

#include "ModbusSensor.h"

#define MB_SERIAL_PORT &Serial1   // Arduino has only one serial port, Mega has 3 serial ports.
// if use Serial 0, remember disconect Tx (pin0) when upload sketch, then re-conect
#define MB_BAUDRATE       2400          // b 2400
#define MB_BYTEFORMAT     SERIAL_8N2    // Prty n
#define TxEnablePin       17
#define TIMEOUT           200


#define ID_1  1                       // id 001  modbus id of the energy monitor
#define REFRESH_INTERVAL  5000        // refresh time, 5 SECONDS
#define WRITE_INTERVAL 60000UL        // values send to serial port, 1 minute ( 60 * 1000)
#define KWH_2_WS 36000000 

// Direcciones registros de datos solo lectura. Valores tipo float.
// Utilizar funcion 04 lectura, numero de registros 16-bits 2.

#define VOL_ADR 0x0000    // VOLTAJE.
#define CUR_ADR 0x0006    // CORRIENTE.
#define POW_ADR 0x000C    // POTENCIA ACTIVA. 
#define APO_ADR 0x0012    // Potencia Aparente.
#define PFA_ADR 0x001E    // Factor de potencia.
#define FRE_ADR 0x0046    // Frecuencia.
#define PEN_ADR 0x0048    // ENERGIA IMPORTADA KWH
#define REN_ADR 0x004A    // Energia exportada.
#define TEN_ADR 0x0156    // Energia activa Total.
#define TRE_ADR 0x0158    // Energia reactiva Total.

// multiplication factor, store value as an integer
#define VOL_FAC 10
#define CUR_FAC 100
#define POW_FAC 10
#define PFA_FAC 100
#define FRE_FAC 10
#define ENE_FAC 100


modbusMaster MBserial(MB_SERIAL_PORT, TxEnablePin);  // instance to collect data using Modbus protocol over RS485

//variables to poll, process and send values
modbusSensor volt(&MBserial, ID_1, VOL_ADR, CHANGE_TO_ZERO);
modbusSensor curr(&MBserial, ID_1, CUR_ADR, CHANGE_TO_ZERO);
modbusSensor pwr(&MBserial, ID_1, POW_ADR, CHANGE_TO_ZERO);
modbusSensor enrg(&MBserial, ID_1, PEN_ADR, HOLD_VALUE);
modbusSensor freq(&MBserial, ID_1, FRE_ADR, CHANGE_TO_ZERO);
modbusSensor aPwr(&MBserial, ID_1, APO_ADR, CHANGE_TO_ZERO);
modbusSensor pwrFact(&MBserial, ID_1, PFA_ADR, CHANGE_TO_ONE);

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
  MBserial.begin(MB_BAUDRATE, MB_BYTEFORMAT, TIMEOUT, REFRESH_INTERVAL);
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
  if (MBserial.available()) {
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
    SERIAL_PRINT(maxVoltage / VOL_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(minVoltage / VOL_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(maxCurrent /CUR_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(minCurrent / CUR_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(maxPower / POW_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(minPower /POW_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(maxApower / POW_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(minApower /POW_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(maxFreq / FRE_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(minFreq /FRE_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINT(avgPower /ENE_FAC);
    SERIAL_PRINT(",");
    SERIAL_PRINTLN(energy / ENE_FAC);

  }
}

