/**********************************************************************
  ModbusSensor.cpp
  create ModbusSensor and ModbusMaster classes to process values from
  a Eastron SMD120 and energy monitor family.

  version 0.1 ALPHA 14/12/2015

  Author: Jaime García  @peninquen
  License: Apache License Version 2.0.

**********************************************************************/
//------------------------------------------------------------------------------

#define MODBUS_SERIAL_OUTPUT  //VERBOSE

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
#define STOP      0
#define SENDING   1
#define RECEIVING 2
#define STANDBY   3

#define WAITING_INTERVAL 10

#define READ_COIL_STATUS          0x01 // Reads the ON/OFF status of discrete outputs (0X references, coils) in the slave.
#define READ_INPUT_STATUS         0x02 // Reads the ON/OFF status of discrete inputs (1X references) in the slave.
#define READ_HOLDING_REGISTERS    0x03 // Reads the binary contents of holding registers (4X references) in the slave.
#define READ_INPUT_REGISTERS      0x04 // Reads the binary contents of input registers (3X references) in the slave. Not writable.
#define FORCE_MULTIPLE_COILS      0x0F // Forces each coil (0X reference) in a sequence of coils to either ON or OFF.
#define PRESET_MULTIPLE_REGISTERS 0x10 // Presets values into a sequence of holding registers (4X references).

#define MB_VALID_DATA     0x00  // ok
#define MB_INVALID_ID     0xE0  // id received don't match
#define MB_INVALID_FC     0xE1  // function code don't match
#define MB_TIMEOUT        0xE2  // slave don't respond ¿maybe off?
#define MB_INVALID_CRC    0xE3  // calculated CRC don't match with recived CRC
#define MB_INVALID_BUFF   0xE4  // corrupted frame or overflow
#define MB_ILLEGAL_FC     0x01  // The function code is not supported by the product
#define MB_ILLEGAL_ADR    0x02  // Attempt to access an invalid address or an attempt to read or write part of a floating point value
#define MB_ILLEGAL_DATA   0x03  // Attempt to set a floating point variable to an invalid value
#define MB_SLAVE_FAIL     0x05  // An error occurred when the instrument attempted to store an update to it’s configuration

// What happens when _status is diferent to MB_VALID_DATA?
#define CHANGE_TO_ZERO 0x00
#define CHANGE_TO_ONE  0x01
#define HOLD_VALUE     0xFF

uint16_t calculateCRC(uint8_t *array, uint8_t num) {
  uint16_t temp, temp2, flag;
  temp = 0xFFFF;
  for (uint8_t i = 0; i < num; i++)
  {
    temp = temp ^ array[i];
    for (uint8_t j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp;
}

// Constructor
modbusSensor::modbusSensor(modbusMaster * mbm, uint8_t id, uint16_t adr, uint8_t hold) {
  _frame[0] = id;
  _frame[1] = READ_INPUT_REGISTERS;
  _frame[2] = adr >> 8;
  _frame[3] = adr & 0x00FF;
  _frame[4] = 0x00;
  _frame[5] = 0x02;
  uint16_t crc = calculateCRC(_frame, 6);
  _frame[6] = crc & 0x00FF;
  _frame[7] = crc >> 8;
  _status = MB_TIMEOUT;
  _hold = hold;
  _value.f = 0.0;
  (*mbm).connect(this);
}

// read value in defined units
float modbusSensor::read() {
  if (_status == MB_TIMEOUT)
    switch (_hold) {
      case CHANGE_TO_ZERO: return 0.0;
      case CHANGE_TO_ONE: return 1.0;
      case HOLD_VALUE: return _value.f;
    }
  return _value.f;
}

// read value as a integer multiplied by factor
uint16_t modbusSensor::read(uint16_t factor) {
  if (_status == MB_TIMEOUT)
    switch (_hold) {
      case CHANGE_TO_ZERO: return (uint16_t) 0;
      case CHANGE_TO_ONE: return (uint16_t) factor;
      case HOLD_VALUE: return (uint16_t)(_value.f * factor);
    }
  return (uint16_t)(_value.f * factor);
}

// get status of the value
uint8_t modbusSensor::getStatus() {
  return _status;
}

// write sensor value
void modbusSensor::write(float value) {
  _value.f = value;
}

//  put new status
uint8_t modbusSensor::putStatus(uint8_t status) {
  _status = status;
  return _status;
}

// get pointer to _poll frame
uint8_t *modbusSensor::getFramePtr() {
  return _frame;
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
  _pollInterval = pollInterval - 1; // reduce 1 to compensate proccess delays
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
  static uint8_t  indexSensor = 0;                 // index of arrry of sensors
  static uint32_t nowMillis = millis();
  static uint32_t lastPollMillis = nowMillis;      // time to check poll interval
  static uint32_t sendMillis = nowMillis;          // time to check timeout interval
  static uint32_t receiveMillis = 0;
  //  static uint32_t offlineMillis = nowMillis;   // time to check offline interval
//  static uint8_t  lastStatus = MB_TIMEOUT;         // ¿offline?

  switch (_state) {
    case SENDING:
      if (millis() - receiveMillis < WAITING_INTERVAL)
        return false;
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
        _state = STANDBY;
        return true;
      }
    case RECEIVING:
      if ((*_MBSerial).available()) {
        readBuffer();
        MODBUS_SERIAL_PRINTLN((*_mbSensorPtr).getStatus(), HEX);
        receiveMillis = millis();
        indexSensor++;
        _state = SENDING;
      }
      else if (millis() - sendMillis > _timeOut) {
        (*_mbSensorPtr).putStatus(MB_TIMEOUT);
        indexSensor++;
        _state = SENDING;
      }
      return false;
    case STANDBY:
      nowMillis = millis();
      if (nowMillis - lastPollMillis > _pollInterval) {
        lastPollMillis = nowMillis;
        _state = SENDING;
      }
      return false;
    case STOP: // do nothing
      return false;
  }
}

uint8_t modbusMaster::readBuffer() {
  uint8_t index = 0;
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
  MODBUS_SERIAL_PRINT(" ");
  MODBUS_SERIAL_PRINTLN(millis());

  // The minimum buffer size from a slave can be an exception response of 5 bytes.
  // If the buffer was partially filled set a frame_error.
  if (index < 5 || ovfFlag)
    return ((*_mbSensorPtr).putStatus(MB_SLAVE_FAIL));

  if (_buffer[0] != _framePtr[0])
    return ((*_mbSensorPtr).putStatus(MB_INVALID_ID));
  
  uint16_t crc = calculateCRC(_buffer, index - 2);
  if (_buffer[index-1] != crc >> 8 && _buffer[index-2] != crc & 0x00FF)
    return ((*_mbSensorPtr).putStatus(MB_INVALID_CRC));

  if (_buffer[1] & 0x80 == 0x80) {
    MODBUS_SERIAL_PRINTLN(_buffer[0], HEX);
    MODBUS_SERIAL_PRINTLN(_framePtr[0], HEX);
    return ((*_mbSensorPtr).putStatus(_buffer[2])); // see exception codes in define area
  }

  switch (_buffer[1]) {
    case READ_INPUT_REGISTERS:

      if (_buffer[2] == 4) {
        dataFloat temp;
        temp.arr[3] = _buffer[3];
        temp.arr[2] = _buffer[4];
        temp.arr[1] = _buffer[5];
        temp.arr[0] = _buffer[6];
        MODBUS_SERIAL_PRINTLN(temp.f);
        (*_mbSensorPtr).write(temp.f);
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
  for (uint8_t i; i < 8; i++) {
    (*_MBSerial).write(_framePtr[i]);
#ifdef MODBUS_SERIAL_OUTPUT
    if (_framePtr[i] < 0x10)
      Serial.print(F(" 0"));
    else
      Serial.print(F(" "));
    Serial.print(_framePtr[i], HEX);
#endif
  }
  (*_MBSerial).flush();
  MODBUS_SERIAL_PRINT("    ");
  MODBUS_SERIAL_PRINTLN(millis());
  // It may be necessary to add a another character delay T1_5 here to
  // avoid truncating the message on slow and long distance connections

  digitalWrite(_TxEnablePin, LOW);
}
