/**********************************************************************
  ModbusSensor.h
  create ModbusSensor and ModbusMaster classes to process values from
  a Eastron SMD120 and family.

  version 0.3 BETA 22/12/2015

  Author: Jaime García  @peninquen
  License: Apache License Version 2.0.

**********************************************************************/

#ifndef ModbusSensor_h
#define ModbusSensor_h

#include "Arduino.h"
#define MAX_SENSORS 16
// The maximum number of bytes in a modbus packet is 256 bytes.
// The serial buffer limits this to 128 bytes.
// We can reduce it to maximum data sending by a slave SMD120 9 bytes, SMD 630 is 85 bytes
// Three phase meters are 3 values, 6 registers and 12 bytes, plus 5 frame bytes, total 17
#define BUFFER_SIZE      32
#define TIMEOUT          110  // time to fail a request 
#define WAITING_INTERVAL 40   // time required by SDM120 to be prepared to receive a new request

// What happens when _status is diferent to MB_VALID_DATA?
#define CHANGE_TO_ZERO 0x00
#define CHANGE_TO_ONE  0x01
#define HOLD_VALUE     0xFF

class modbusMaster;

union dataFloat {
  float f;
  uint8_t array[4];
};

//------------------------------------------------------------------------------
class modbusSensor {
  public:
    // Constructor
    modbusSensor(modbusMaster *mbm, uint8_t id, uint16_t adr, uint8_t hold);

    // Constructor
    //    modbusSensor(uint8_t id, uint16_t adr, uint8_t hold);

    // read value in defined units
    float read();

    // read value as a integer multiplied by factor
    uint16_t read(uint16_t factor);

    // get status of the value
    inline uint8_t getStatus();

    // write sensor value
    inline void write(float value);

    //  change status, return new status
    inline uint8_t putStatus(uint8_t status);

    // get pointer to _poll frame
    inline uint8_t *getFramePtr();

  private:
    uint8_t     _frame[8];
    dataFloat   _value;
    uint8_t     _status;
    uint8_t     _hold;
};

//------------------------------------------------------------------------------
class modbusMaster {
  public:
    //constructor
    modbusMaster(HardwareSerial *mbSerial, uint8_t TxEnPin);

    // Connect a modbusSensor to modbusMaster array of queries
    void connect(modbusSensor *mbSensor);

    // Disconnect a modbusSensor to modbusMaster array of queries
    void disconnect(modbusSensor *mbSensor);

    // begin communication using ModBus protocol over RS485
    void begin(uint16_t baudrate, uint8_t byteFormat, uint16_t pollInterval);

    // end communication over serial port
    void end();

    // Finite State Machine core, process FSM and check if the array of sensors has been requested
    boolean available();

  private:
    inline void sendFrame();
    inline void readBuffer(uint8_t frameSize);
    uint8_t  _state;                    // Modbus FSM status (SENDING, RECEIVING, STANDBY, WAINTING_NEXT_POLL)
    uint8_t  _TxEnablePin;              // pin to enable transmision in MAX485
    uint8_t  _totalSensors;             // constant, max number of sensors to poll
    uint16_t _pollInterval;             // constant, time between polling same data
    uint32_t _T2_5;                     // time between characters in a frame, in microseconds
    uint8_t  _buffer[BUFFER_SIZE];      // buffer to process rececived frame
    //    uint8_t  _availableSensors;         // number of refreshed sensors, decrement when read, increment when write
    //    uint16_t _availableSensorsFlag;     // array of flags to register new available sensor values
    HardwareSerial  *_MBSerial;
    modbusSensor    *_mbSensorsPtr[MAX_SENSORS]; // array of modbusSensor's pointers
    modbusSensor    *_mbSensorPtr;
    uint8_t         *_framePtr;
};
extern modbusMaster MBSerial;

#endif

