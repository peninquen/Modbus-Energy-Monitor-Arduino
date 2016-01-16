/**********************************************************************
  ModbusSensor.h
  create ModbusSensor and ModbusMaster classes to process values from
  a Eastron SMD120 and family.

  version 0.5.5 BETA 16/01/2016

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

// Function codes operative
#define READ_HOLDING_REGISTERS    0x03 // Reads the binary content of holding registers (4X references) in the slave.
#define READ_INPUT_REGISTERS      0x04 // Reads the binary content of input registers (3X references) in the slave. Not writable.
#define PRESET_MULTIPLE_REGISTERS 0x10 // Presets values into a sequence of holding registers (4X references).

#define MB_VALID_DATA     0x00
#define MB_INVALID_ID     0xE0
#define MB_INVALID_FC     0xE1
#define MB_TIMEOUT        0xE2
#define MB_INVALID_CRC    0xE3
#define MB_INVALID_BUFF   0xE4
#define MB_INVALID_ADR    0xE5
#define MB_INVALID_DATA   0xE6
#define MB_MASTER_STOP    0xE7
#define MB_ILLEGAL_FC     0x01
#define MB_ILLEGAL_ADR    0x02
#define MB_ILLEGAL_DATA   0x03
#define MB_SLAVE_FAIL     0x04
#define MB_EXCEPTION      0x05

// forward definition
class modbusSensor;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
class modbusMaster {
  protected:
    HardwareSerial  *_hwSerial;
    modbusSensor    *_mbSensorsPtr[MAX_SENSORS]; // array of modbusSensor's pointers
    uint8_t  _TxEnablePin;              // pin to enable transmision in MAX485
    uint8_t  _totalSensors;             // constant, max number of sensors to poll
    uint8_t  _indexSensor;              // index number of sensors
    uint8_t  _rx_buffer[BUFFER_SIZE];   // buffer to process rececived frame
    uint8_t  _state;                    // Modbus FSM state (STOP, SEND, SENDING, RECEIVING, IDLE, WAITING_NEXT_POLL)
    uint32_t _T2_5;                     // time between characters in a frame, in microseconds
    uint32_t _timeoutMillis;            // time to check timeout interval
    uint32_t _waitingMillis;            // time to check waiting turnaround interval

    inline void sendFrame(uint8_t *frame, uint8_t frameSize);
    inline void readBuffer(uint8_t frameSize);

  public:
    //constructor
    modbusMaster() {
      _totalSensors = 0;
      for (uint8_t i = 0; i < MAX_SENSORS; i++)
        _mbSensorsPtr[i] = 0;
    };

    //constructor
    void config(HardwareSerial *mbSerial, uint8_t TxEnPin);

    // Connect a modbusSensor to modbusMaster array of queries
    void connect(modbusSensor *mbSensor);

    // Disconnect a modbusSensor to modbusMaster array of queries
    void disconnect(modbusSensor *mbSensor);

    // begin communication using ModBus protocol over RS485
    void begin(uint16_t baudrate, uint8_t byteFormat);

    // end communication over serial port
    void end();

    // Start process to send requests
    boolean sendRequest();

    // Finite State Machine core, process FSM
    // It returns 'true' when finish to request all the array of modbusSensors.
    // Non-blocking function, put the instrucction in a loop function to make the proccess work properly
    boolean available();
};

extern modbusMaster MBSerial;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
class modbusSensor {
  protected:
    uint8_t * _value;     // pointer to a dinamic allocated object, size inside _frame[5]
    uint8_t * _frame;     // pointer to a dinamic allocated array,
    uint8_t   _frameSize; // size of the _frame, 8 in read function, 9+sizeof(T) in preset function
    uint8_t   _status;    // register of the result of communication
    uint8_t   _hold;      // predefined behaiviour in case of timeout exception

    void processPreset(uint8_t *ptr, uint8_t objectSize);
    void processRead(uint8_t *ptr, uint8_t objectSize);
    void processBuffer(uint8_t *rxFrame, uint8_t rxFrameSize);

    //MBSerial.available need access to _frame, _frameSize and _status variables
    friend boolean modbusMaster::available();

  public:
    // Target constructor
    modbusSensor(uint8_t id, uint16_t adr, uint8_t hold, uint8_t sizeofValue, uint8_t fc);

    // delegating constructor
    modbusSensor(uint8_t id, uint16_t adr, uint8_t hold, uint8_t sizeofValue): modbusSensor(id, adr, hold, sizeofValue, READ_INPUT_REGISTERS) {};

    // delegating constructor
    modbusSensor(uint8_t id, uint16_t adr, uint8_t hold): modbusSensor(id, adr, hold, 4, READ_INPUT_REGISTERS) {};

    // Destructor
    ~modbusSensor() {
      delete[] _value;
      delete[] _frame;
      MBSerial.disconnect(this);
    }

    // Connect to MBSerial array of sensors
    void connect() {
      MBSerial.connect(this);
    };

    // Disconnect to MBSerial array of sensors
    void disconnect() {
      MBSerial.disconnect(this);
    };

    // Preset sensor value, fc 0x10, only holding registers defined with fc 0x03
    // complete funtion to make and send the frame and process response, check status
    template < typename T > void preset(const T &t) {
      processPreset((uint8_t *) &t, sizeof(T));
    };

    // read value in defined units
    float read();

    // read every struct value from object buffer, non-blocking function
    template< typename T > T &read(T &t) {
      processRead((uint8_t *) &t, sizeof(T));
      return t;
    }

    // print status message
    uint8_t printStatus();
};

#endif

