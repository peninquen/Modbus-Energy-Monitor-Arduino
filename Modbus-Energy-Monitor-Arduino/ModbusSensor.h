 /**********************************************************************
  ModbusSensor.h
  create ModbusSensor and ModbusMaster classes to process values from
  a Eastron SMD120 and family.

  version 0.5 BETA 4/01/2016

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
// Three-phase meters are 3 values, 6 registers and 12 bytes, plus 5 frame bytes, total 17
// You can even request 6 contiguous values, 12 registers, 24 bytes plus 5 frame bytes equals to 29 bytes
#define BUFFER_SIZE      32
#define TIMEOUT          110  // time to fail a request 
#define WAITING_INTERVAL 40   // time required by SDM120 to be prepared to receive a new request

// What happens when _status is diferent to MB_VALID_DATA?
// in case of offline device (MB_TIMEOUT) read value holds or changes to ...
#define CHANGE_TO_ZERO 0x00
#define CHANGE_TO_ONE  0x01
#define HOLD_VALUE     0xFF

// Function codes operative
#define READ_HOLDING_REGISTERS    0x03 // Reads the binary contents of holding registers (4X references) in the slave.
#define READ_INPUT_REGISTERS      0x04 // Reads the binary contents of input registers (3X references) in the slave. Not writable.
#define PRESET_MULTIPLE_REGISTERS 0x10 // Presets values into a sequence of holding registers (4X references).

#define MB_VALID_DATA     0x00
#define MB_INVALID_ID     0xE0
#define MB_INVALID_FC     0xE1
#define MB_TIMEOUT        0xE2
#define MB_INVALID_CRC    0xE3
#define MB_INVALID_BUFF   0xE4
#define MB_INVALID_ADR    0xE5
#define MB_INVALID_DATA   0xE6
#define MB_ILLEGAL_FC     0x01
#define MB_ILLEGAL_ADR    0x02
#define MB_ILLEGAL_DATA   0x03
#define MB_SLAVE_FAIL     0x04
#define MB_EXCEPTION      0x05

// forward definition
class modbusSensor;
uint16_t calculateCRC(uint8_t *array, uint8_t num);

//------------------------------------------------------------------------------
class modbusMaster {
  public:
    // constructor, implicit
    //    modbusMaster();

    // configure connection
    void config(HardwareSerial *hwSerial, uint8_t TxEnPin, uint16_t pollInterval);

    // Connect a modbusSensor to modbusMaster array of queries
    void connect(modbusSensor *mbSensor);

    // Disconnect a modbusSensor to modbusMaster array of queries
    void disconnect(modbusSensor *mbSensor);

    // begin communication using ModBus protocol over RS485
    void begin(uint16_t baudrate, uint8_t byteFormat);

    // end communication over serial port
    void end();

    // Finite State Machine core, process FSM and check if the array of sensors has been requested.
    // Non-blocking function, put the instrucction in a loop function to make the proccess work properly
    boolean available();

  protected:
    /*inline*/ void readBuffer(uint8_t frameSize);
    uint8_t  _state;                    // Modbus FSM state (SENDING, RECEIVING, STANDBY, WAINTING_NEXT_POLL)
    uint8_t  _TxEnablePin;              // pin to enable transmision in MAX485
    uint8_t  _buffer[BUFFER_SIZE];      // buffer to process rececived frame
    uint8_t  _totalSensors;             // constant, max number of sensors to poll
    uint16_t _pollInterval;             // constant, time between polling same data
    uint32_t _T2_5;                     // time between characters in a frame, in microseconds
    HardwareSerial  *_hwSerial;
    modbusSensor    *_mbSensorsPtr[MAX_SENSORS]; // array of modbusSensor's pointers
    //friend ModbusSensor;
} ;

//predefined instance to poll, collect and process values from modbus protocol

static modbusMaster MBSerial;

//------------------------------------------------------------------------------
class modbusSensor {
  public:
    // Constructor, implicit MBSerial, fc 0x4, register size 2, datatype float
    modbusSensor(uint8_t id, uint16_t adr, uint8_t hold);

    // Constructor, implicit MBSerial, fc 0x4,
    modbusSensor(uint8_t id, uint16_t adr, uint8_t hold, uint8_t sizeofValue);

    // Constructor,
    modbusSensor(uint8_t id, uint8_t fc, uint16_t adr, uint8_t hold, uint8_t sizeofValue);

    // Destructor
    ~modbusSensor() {
      delete[] _value;
      delete[] _frame;
      MBSerial.disconnect(this);
    }
    
    // read a float value from object buffer, non-blocking function
    float read();

    // read a value from object buffer, non-blocking function
    template< typename T > T &read(T &t) {
      uint8_t *ptr = (uint8_t *) &t;
      uint8_t *e = _value;
      for (int count = sizeof(T); count; --count, ++e) *ptr++ = *e;
      return t;
    }

    // Preset sensor value, fc 0x10, only holding registers defined with fc 0x03
    // complete funtion to make and send the frame and process response, check status
    template < typename T > void preset(const T &t) {
      uint8_t *ptr = (uint8_t *) &t + sizeof(T) - 1; // pointer to object last byte
      processPreset(ptr, sizeof(T));
    };

    void processPreset(const uint8_t *ptr, uint8_t objectSize);

    // get status of the value
    uint8_t getStatus() {
      return _status;
    };

    void sendFrame(HardwareSerial *hwSerial);

    void processBuffer(uint8_t *rxFrame, uint8_t rxFrameSize);

  protected:
    uint8_t *_value;      // pointer to an object value
    uint8_t *_frame;      // pointer to TX frame
    uint8_t  _frameSize;  // size of the TX frame
    uint8_t  _status;     // returned status
    uint8_t  _hold;       // offline value
    friend boolean modbusMaster::available(); //access _status variable
};


#endif

