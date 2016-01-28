/**********************************************************************
  ModbusEnergyMonitor example
  An example to collect data from a Modbus energy monitor using ModbusSensor class
  to datalogger, include a RTC DS3231 and a SD card
  version 0.5 BETA 4/01/2016

  Author: Jaime García  @peninquen
  License: Apache License Version 2.0.

**********************************************************************/


#include <ESP8266WiFi.h>      //ESP library from http://github.com/esp8266/Arduino
#include <PubSubClient.h>     // MQTT library from https://github.com/knolleary/pubsubclient
#include "ModbusSensor.h"
#include "SDMdefines.h"

// Update these with values suitable for your network.

const char* ssid = "........";
const char* password = "........";
const char* mqtt_server = "broker.mqtt-dashboard.com";

WiFiClient espClient;
PubSubClient client(espClient);
uint8_t msg[50];

#define MB_SERIAL_PORT &Serial   // ESP8266 has two serial ports, but Serial1 only Tx
// Serial.swap() change pins to Tx and Rx

#define MB_BAUDRATE       2400          // b 2400
#define MB_BYTEFORMAT     SERIAL_8N2    // Prty n
#define TxEnablePin       2

#define ID_1  1                       // id 001  modbus id of the energy monitor
#define REFRESH_INTERVAL  5000        // refresh time, 5 SECONDS


struct three_phase {
  float line3, line2, line1;
} voltage, current, power;

float energy = 0.0;

// global variables to poll, process and send values

//modbusSensor(uint8_t id, uint8_t fc, uint16_t adr, uint8_t hold, uint8_t sizeofValue)
modbusSensor volt(ID_1, VOLTAGE, CHANGE_TO_ZERO, sizeof(three_phase));
modbusSensor curr(ID_1, CURRENT, CHANGE_TO_ZERO, sizeof(three_phase));
modbusSensor pwr(ID_1, POWER, CHANGE_TO_ZERO, sizeof(three_phase));
//modbusSensor(uint8_t uint16_t adr, uint8_t hold)
modbusSensor enrg(ID_1, IAENERGY, HOLD_VALUE);

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  setup_wifi();
  client.setServer(mqtt:server,1883);
  client.setCallback(callback);
  
  Serial.begin(9600);
  MBSerial.config(MB_SERIAL_PORT, TxEnablePin, REFRESH_INTERVAL);
  MBSerial.begin(MB_BAUDRATE, MB_BYTEFORMAT);
  
  delay(95);
  Serial.println("time(s),Volt1(V), Volt2(V), Volt3(V), Curr1(A) Curr2(A), Curr3(A), Power1(W), Power2(W), Power3(W), Energy(Kwh)");
}

void loop() {

  if (MBSerial.available()) {
    volt.read(voltage);
    curr.read(current);
    pwr.read(power);
    energy = enrg.read();

    Serial.print(millis() / 1000);
    Serial.print(",");
    Serial.print(voltage.line1, 1);
    Serial.print(",");
    Serial.print(voltage.line2, 1);
    Serial.print(",");
    Serial.print(voltage.line3, 1);
    Serial.print(",");
    Serial.print(current.line1, 2);
    Serial.print(",");
    Serial.print(current.line2, 2);
    Serial.print(",");
    Serial.print(current.line3, 2);
    Serial.print(",");
    Serial.print(power.line1, 2);
    Serial.print(",");
    Serial.print(power.line2, 2);
    Serial.print(",");
    Serial.print(power.line3, 2);
    Serial.print(",");
    Serial.println(energy, 2);
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());    
}

