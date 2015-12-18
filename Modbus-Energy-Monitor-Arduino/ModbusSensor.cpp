/**********************************************************************
* ModbusSensor class
* A class to collect data from a Modbus energy monitor
*
* version 0.3 BETA 17/12/2015
*
* Author: Jaime García  @peninquen
* License: Apache License Version 2.0.
*
**********************************************************************/
//------------------------------------------------------------------------------

//#define MODBUS_SERIAL_OUTPUT  //VERBOSE

#ifdef MODBUS_SERIAL_OUTPUT
#define MODBUS_SERIAL_BEGIN(...) Serial.begin(__VA_ARGS__)
#define MODBUS_SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#define MODBUS_SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define MODBUS_SERIAL_BEGIN(...)
#define MODBUS_SERIAL_PRINT(...)
#define MODBUS_SERIAL_PRINTLN(...)
#endif



#include "ModbusSensor.h"

// Finite state machine status
#define STOP                0
#define SENDING             1
#define RECEIVING           2
#define WAITING_NEXT_POLL   3

#define READ_COIL_STATUS          0x01 // Reads the ON/OFF status of discrete outputs (0X references, coils) in the slave.
#define READ_INPUT_STATUS         0x02 // Reads the ON/OFF status of discrete inputs (1X references) in the slave.
#define READ_HOLDING_REGISTERS    0x03 // Reads the binary contents of holding registers (4X references) in the slave.
#define READ_INPUT_REGISTERS      0x04 // Reads the binary contents of input registers (3X references) in the slave. Not writable.
#define FORCE_MULTIPLE_COILS      0x0F // Forces each coil (0X reference) in a sequence of coils to either ON or OFF.
#define PRESET_MULTIPLE_REGISTERS 0x10 // Presets values into a sequence of holding registers (4X references).

#define MB_VALID_DATA     0x00
#define MB_INVALID_ID     0xE0
#define MB_INVALID_FC     0xE1
#define MB_TIMEOUT        0xE2
#define MB_INVALID_CRC    0xE3
#define MB_INVALID_BUFF   0xE4
#define MB_ILLEGAL_FC     0x01
#define MB_ILLEGAL_ADR    0x02
#define MB_ILLEGAL_DATA   0x03
#define MB_SLAVE_FAIL     0x04
#define MB_EXCEPTION      0x05

// when _status diferent to MB_VALID_DATA change to zero or hold value?
#define CHANGE_TO_ZERO 0x00
#define CHANGE_TO_ONE  0x01
#define HOLD_VALUE     0xFF

uint16_t calculateCRC(uint8_t *array, uint8_t num) {
  uint16_t crc, temp;
  crc = 0xFFFF;
  for (uint8_t i = 0; i < num; i++) {
    crc ^= array[i];
    for (uint8_t j = 8; j; j--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
        crc >>= 1;
    }
    // Reverse byte order.
    temp = crc >> 8;
    crc = (crc << 8) | temp;
    crc &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return crc;
  }
}

// Constructor
modbusSensor::modbusSensor(modbusMaster * mbm, uint8_t id, uint16_t adr, uint8_t hold) {
  _frame.id = id;
  _frame.fc = READ_INPUT_REGISTERS;
  _frame.address = adr;
  _frame.data = 2;
  _frame.crc = calculateCRC(_frame.array, 6);
  _status = MB_TIMEOUT;
  _hold = hold;
  _value = 0.0;
  (*mbm).connect(this);
}

// read value in defined units
float modbusSensor::read() {
  if(!_hold && _status == MB_TIMEOUT) return 0.0;
  else return _value;
  
}

// read value as a integer multiplied by factor
uint16_t modbusSensor::read(uint16_t factor) {
  if(!_hold && _status == MB_TIMEOUT) return 0;
  else return (uint16_t)(_value * factor);
}

// get status of the value
uint8_t modbusSensor::getStatus() {
  return _status;
}

// write sensor value
void modbusSensor::write(float value) {
  _value = value;
}

//  put new status
uint8_t modbusSensor::putStatus(uint8_t status) {
  _status = status;
  return _status;
}

// get pointer to _poll frame
pollFrame *modbusSensor::getFramePtr() {
  return &_frame;
}

//---------------------------------------------------------------------------------------//

//constructor
modbusMaster::modbusMaster(HardwareSerial * MBSerial, uint8_t TxEnPin) {
  _state = STOP;
  _TxEnablePin = TxEnPin;
  pinMode(_TxEnablePin, OUTPUT);
  _MBSerial = MBSerial;
  _totalSensors = 0;
  for (uint8_t i = 0; i < MAX_SENSORS; i++) {
    _mbSensorsPtr[i] = 0;
  }
}

// Connect modbusSensor to modbusMaster array of queries
boolean modbusMaster::connect(modbusSensor * mbs) {
  if (_totalSensors < MAX_SENSORS) {
    _mbSensorsPtr[_totalSensors] = mbs;
    _totalSensors++;
    return true;
  }
  else return false;
}

// begin comunication using ModBus protocol over RS485
void modbusMaster::begin(uint16_t baudrate, uint8_t byteFormat, uint16_t timeOut, uint16_t pollInterval) {
  _timeOut = timeOut;
  _pollInterval = pollInterval;
  if (baudrate > 19200)
    _T1_5 = 750;
  else
    _T1_5 = 16500000 / baudrate; // 1T * 1.5 = T1.5
  (*_MBSerial).begin(baudrate, byteFormat);
  _state = SENDING;
  digitalWrite(_TxEnablePin, LOW);
}

// process FSM and check if the array of sensors has been requested and processed
boolean modbusMaster::available() {
  static uint8_t  indexSensor = 0;                // index of arrry of sensors
  static uint32_t lastPollMillis = millis();      // time to check poll interval
  static uint32_t sendMillis = lastPollMillis;    // time to check timeout interval
  //  static uint32_t offlineMillis = lastPollMillis; // time to check offline interval
  static uint8_t  lastStatus = MB_TIMEOUT;        // ¿offline?

  switch (_state) {
    case SENDING:
      if (indexSensor < _totalSensors) {
        _mbSensorPtr = _mbSensorsPtr[indexSensor];
        _framePtr = (*_mbSensorPtr).getFramePtr();
        
        sendFrame();
        
        sendMillis = millis();
        _state = RECEIVING;
        return false;
      }
      else {
        indexSensor = 0;
        _state = WAITING_NEXT_POLL;
        return true;
      }
    case RECEIVING:
      if ((*_MBSerial).available()) {
        readBuffer();
        indexSensor++;
        _state = SENDING;
      }
      else if (millis() - sendMillis > _timeOut) {
        (*_mbSensorPtr).putStatus(MB_TIMEOUT);
        indexSensor++;
        _state = SENDING;
      }
      return false;
    case WAITING_NEXT_POLL:
      if ((millis() - lastPollMillis) > _pollInterval) {
        lastPollMillis = millis();
        _state = SENDING;
      }
      return false;
    case STOP: // do nothing
      return false;
  }
}

uint8_t modbusMaster::readBuffer() {
  uint8_t index;
  boolean ovfFlag = false;
  MODBUS_SERIAL_PRINT(millis());
  MODBUS_SERIAL_PRINT("  SLAVE:");
  while ((*_MBSerial).available()) {
    // The maximum number of bytes is limited to the serial buffer size
    // of BUFFER_SIZE. If more bytes is received than the BUFFER_SIZE the
    // overflow flag will be set and the serial buffer will be read until
    // all the data is cleared from the receive buffer, while the slave is
    // still responding.
    if (ovfFlag)
      (*_MBSerial).read();
    else {
      if (index == BUFFER_SIZE) ovfFlag = true;
      _buffer[index] = (*_MBSerial).read();
//#ifdef MODBUS_SERIAL_OUTPUT
      if (_buffer[index] < 0x10)
        Serial.print(F(" 0"));
      else
        Serial.print(F(" "));
      Serial.print(_buffer[index], HEX);
//#endif
      index++;
    }
    // This is not 100% correct but it will suffice.
    // worst case scenario is if more than one character time expires
    // while reading from the buffer then the buffer is most likely empty
    // If there are more bytes after such a delay it is not supposed to
    // be received and thus will force a frame_error.

    delayMicroseconds(_T1_5); // inter character time out
  }
  MODBUS_SERIAL_PRINTLN();

  // The minimum buffer size from a slave can be an exception response of
  // 5 bytes. If the buffer was partially filled set a frame_error.
  // The maximum number of bytes in a modbus packet is 256 bytes.
  // The serial buffer limits this to 128 bytes.
  if (index < 5 || ovfFlag)
    return ((*_mbSensorPtr).putStatus(MB_SLAVE_FAIL));

  if (_buffer[0] != (*_framePtr).id)
    return ((*_mbSensorPtr).putStatus(MB_INVALID_ID));

  if (calculateCRC(_buffer, index - 2) != (*_framePtr).crc)
    return ((*_mbSensorPtr).putStatus(MB_INVALID_CRC));

  if (_buffer[1] & 0x80 == 0x80)
    return ((*_mbSensorPtr).putStatus(MB_EXCEPTION));


  switch (_buffer[1]) {
    case READ_HOLDING_REGISTERS:
      if (_buffer[2] == 4) {
        float temp = (float)(_buffer[5] << 24 | _buffer[6] << 16 | _buffer[3] << 8 | _buffer[4]);
        (*_mbSensorPtr).write(temp);
        return ((*_mbSensorPtr).putStatus(MB_VALID_DATA));
      }
      else
        return ((*_mbSensorPtr).putStatus(MB_ILLEGAL_DATA));

    default:
      return ((*_mbSensorPtr).putStatus(MB_ILLEGAL_FC));
  }
}


void modbusMaster::sendFrame() {
  digitalWrite(_TxEnablePin, HIGH);
  MODBUS_SERIAL_PRINT(millis());
  MODBUS_SERIAL_PRINT(F(" MASTER:"));
  (*_MBSerial).write((*_framePtr).array, 8);
  for (uint8_t i; i < 8; i++) {
#ifdef MODBUS_SERIAL_OUTPUT
    if ((*_framePtr).array[i] < 0x10)
      Serial.print(F(" 0"));
    else
      Serial.print(F(" "));
    Serial.print((*_framePtr).array[i], HEX);
#endif
  }
//  (*_MBSerial).flush();
  MODBUS_SERIAL_PRINTLN();
  
  //  Cuánto tiempo mantenemos activo el pin de comuniciones?
  digitalWrite(_TxEnablePin, LOW);
}
