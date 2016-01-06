/******************************************************************************
  ModbusSensor class
  A class to collect data from a Modbus energy monitor model SDM120 and family

  version 0.5 BETA 4/01/2016

  Author: Jaime García  @peninquen
  License: Apache License Version 2.0.

*******************************************************************************/
//------------------------------------------------------------------------------

//#define MODBUS_SERIAL_OUTPUT  //Verbose MODBUS messages and timing

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
#define SEND                1
#define SENDING             2
#define RECEIVING           3
#define IDLE                4
#define WAITING_NEXT_POLL   5

/*#define READ_COIL_STATUS          0x01 // Reads the ON/OFF status of discrete outputs (0X references, coils) in the slave.
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
  #define MB_INVALID_ADR    0xE5
  #define MB_INVALID_DATA   0xE6
  #define MB_ILLEGAL_FC     0x01
  #define MB_ILLEGAL_ADR    0x02
  #define MB_ILLEGAL_DATA   0x03
  #define MB_SLAVE_FAIL     0x04
  #define MB_EXCEPTION      0x05

  // when _status is diferent to MB_VALID_DATA change it to zero or hold last valid value?
  #define CHANGE_TO_ZERO    0x00
  #define CHANGE_TO_ONE     0x01
  #define HOLD_VALUE        0xFF
*/

uint16_t calculateCRC(uint8_t *array, uint8_t num) {
  uint16_t temp, flag;
  temp = 0xFFFF;
  for (uint8_t i = 0; i < num; i++) {
    temp = temp ^ array[i];
    for (uint8_t j = 8; j; j--) {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag) temp ^= 0xA001;
    }
  }
  return temp;
}

//---------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------//

//configure object and connections
void modbusMaster::config(HardwareSerial * hwSerial, uint8_t TxEnPin, uint16_t pollInterval) {
  _state = STOP;
  _TxEnablePin = TxEnPin;
  pinMode(_TxEnablePin, OUTPUT);
  _hwSerial = hwSerial;
  _pollInterval = pollInterval - 1;
  _totalSensors = 0;
  for (uint8_t i = 0; i < MAX_SENSORS; i++)
    _mbSensorsPtr[i] = 0;
}

//------------------------------------------------------------------------------
// Connect a modbusSensor to the modbusMaster array of queries
void modbusMaster::connect(modbusSensor * mbSensor) {
  if (_totalSensors < MAX_SENSORS) {
    _mbSensorsPtr[_totalSensors] = mbSensor;
    _totalSensors++;
  }
}

//------------------------------------------------------------------------------
// Disconnect a modbusSensor to the modbusMaster array of queries
void modbusMaster::disconnect(modbusSensor * mbSensor) {
  uint8_t i = 0;
  while (i < _totalSensors) {
    if (_mbSensorsPtr[i] == mbSensor)  {
      while (i < _totalSensors - 1) {
        _mbSensorsPtr[i] = _mbSensorsPtr[i + 1]; //shift down pointers one position
      }
      _totalSensors--;  // reduce number of total sensors in the array
      _mbSensorsPtr[_totalSensors] = 0; // delete last position
    }
    i++;
  }
}

//------------------------------------------------------------------------------
// begin communication using ModBus protocol over RS485
void modbusMaster::begin(uint16_t baudrate, uint8_t byteFormat) {
  if (baudrate > 19200) _T2_5 = 1250;
  else                  _T2_5 = 27500000 / baudrate;
  // 2400 bauds --> 11458 us; 9600 bauds --> 2864 us

  (*_hwSerial).begin(baudrate, byteFormat);
  _state = SEND;
  digitalWrite(_TxEnablePin, LOW);
}

//------------------------------------------------------------------------------
// end communication
void modbusMaster::end() {
  _state = STOP;
  (*_hwSerial).end();
  digitalWrite(_TxEnablePin, LOW);
}

//------------------------------------------------------------------------------
// Finite State Machine core, return 'true' when all queries has been processed
boolean modbusMaster::available() {
  static uint8_t  indexSensor = 0;                // index of arrray of sensors
  static uint8_t  frameSize;                      // size of the answer frame
  static uint32_t tMicros;                        // time to check between characters in a frame
  static uint32_t nowMillis = millis();
  static uint32_t lastPollMillis = nowMillis;     // time to check poll interval
  static uint32_t sendMillis = nowMillis;         // time to check timeout interval
  static uint32_t receivedMillis = nowMillis;     // time to check waiting interval

  switch (_state) {
    //-----------------------------------------------------------------------------
    case SEND:

      if (indexSensor < _totalSensors) {
        // MAX485 Transmission mode
        digitalWrite(_TxEnablePin, HIGH);
        sendFrame((*_mbSensorsPtr[indexSensor])._frame, (*_mbSensorsPtr[indexSensor])._frameSize);
        //(*_mbSensorsPtr[indexSensor]).sendFrame(_hwSerial);

        _state = SENDING;
        return false;
      }
      else {
        indexSensor = 0;
        _state = WAITING_NEXT_POLL;
        // Only at this point available() returns true,
        return true;
      }

    //-----------------------------------------------------------------------------
    case SENDING:

      if ((*_hwSerial).availableForWrite() == SERIAL_TX_BUFFER_SIZE - 1) { //TX buffer empty
        // time to be sure last byte sended and required empty space
        delayMicroseconds(_T2_5);

        // clean RX buffer
        while ((*_hwSerial).available()) (*_hwSerial).read();

        // MAX485 Receiving mode
        digitalWrite(_TxEnablePin, LOW);
        _state = RECEIVING;

        //starts  slave timeOut
        sendMillis = millis();
        frameSize = 0;
      }
      return false;

    //-----------------------------------------------------------------------------
    case RECEIVING:

      if (!(*_hwSerial).available()) {
        if (millis() - sendMillis > TIMEOUT) {
          (*_mbSensorsPtr[indexSensor])._status = MB_TIMEOUT;
          indexSensor++;
          _state = SEND;
        }
        return false;
      }

      if ((*_hwSerial).available() > frameSize) {
        frameSize++;
        tMicros = micros();
      }
      else {
        if (micros() - tMicros > _T2_5) { // inter-character time exceeded
          readBuffer(frameSize);
          (*_mbSensorsPtr[indexSensor]).processBuffer(_buffer, frameSize);
          indexSensor++;
          receivedMillis = millis(); //starts waiting interval to next request
          _state = IDLE;
        }
      }
      return false;

    //-----------------------------------------------------------------------------
    case IDLE:
      if (millis() - receivedMillis > WAITING_INTERVAL)
        _state = SEND;
      return false;

    //-----------------------------------------------------------------------------
    case WAITING_NEXT_POLL:
      nowMillis = millis();
      if ((nowMillis - lastPollMillis) > _pollInterval) {
        lastPollMillis = nowMillis;
        _state = SEND;
      }
      return false;

    //-----------------------------------------------------------------------------
    case STOP:   // do nothing

      return false;

  }
}

//-----------------------------------------------------------------------------
//
inline void modbusMaster::sendFrame(uint8_t *frame, uint8_t frameSize) {
  MODBUS_SERIAL_PRINT(millis());
  MODBUS_SERIAL_PRINT(F(" MASTER:"));

  (*_hwSerial).write(frame, frameSize);

#ifdef MODBUS_SERIAL_OUTPUT
  for (uint8_t i = 0; i < frameSize; i++) {
    if (frame[i] < 0x10)
      Serial.print(F(" 0"));
    else
      Serial.print(F(" "));
    Serial.print(frame[i], HEX);
  }
#endif
  MODBUS_SERIAL_PRINT("    ");
  MODBUS_SERIAL_PRINTLN(millis());
}

//-----------------------------------------------------------------------------
//
inline void modbusMaster::readBuffer(uint8_t frameSize) {

  MODBUS_SERIAL_PRINT(millis());
  MODBUS_SERIAL_PRINT("  SLAVE:");
  (*_hwSerial).readBytes(_buffer, frameSize);
#ifdef MODBUS_SERIAL_OUTPUT
  for (uint8_t index = 0; index < frameSize; index++) {
    if (_buffer[index] < 0x10)
      Serial.print(F(" 0"));
    else
      Serial.print(F(" "));
    Serial.print(_buffer[index], HEX);
  }
#endif
  MODBUS_SERIAL_PRINT(" ");
  MODBUS_SERIAL_PRINTLN(millis());
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

// Constructor,
modbusSensor::modbusSensor(uint8_t id, uint16_t adr, uint8_t hold, uint8_t sizeofValue, uint8_t fc) {

  _value = new uint8_t[sizeofValue];
  uint8_t *ptr = _value;
  for (int count = sizeofValue; count; --count) *ptr++ = 0;
  switch (fc) {
    case PRESET_MULTIPLE_REGISTERS:
    case READ_HOLDING_REGISTERS:
      _frame = new uint8_t[9 + sizeofValue]; // reserve space for fc PRESET_MULTIPLE_REGISTERS
      _frame[1] = READ_HOLDING_REGISTERS;
      break;

    case READ_INPUT_REGISTERS:
      _frame = new uint8_t[8];
      _frame[1] = READ_INPUT_REGISTERS;
      break;

    default:
      // exit without connect to MBSerial
      _frame = 0;
      _frameSize = 0;
      _status = MB_INVALID_FC;
      return;
  }
  _frameSize = 8;
  _frame[0] = id;
  _frame[2] = adr >> 8;
  _frame[3] = adr & 0x00FF;
  _frame[4] = 0x00;         // Always, registers < 16
  _frame[5] = sizeofValue / 2;
  uint16_t crc = calculateCRC(_frame, 6);
  _frame[6] = crc & 0x00FF;
  _frame[7] = crc >> 8;

  _status = MB_TIMEOUT;
  _hold = hold;
  connect();
}

//-----------------------------------------------------------------------------
//Process RX buffer
void modbusSensor::processBuffer(uint8_t *rxFrame, uint8_t rxFrameSize) {

  // check minimum response frame size
  if (rxFrameSize < 5) {
    _status = MB_SLAVE_FAIL;
    return;
  }

  // check slave id
  if (rxFrame[0] != _frame[0]) {
    _status = MB_INVALID_ID;
    return;
  }

  // check CRC
  uint16_t crc = calculateCRC(rxFrame, rxFrameSize - 2);
  if (rxFrame[rxFrameSize - 1] != crc >> 8 && rxFrame[rxFrameSize - 2] != crc & 0x00FF) {
    _status = MB_INVALID_CRC;
    return;
  }

  // check exception error in function code
  if (rxFrame[1] & 0x80 == 0x80) {
    _status = rxFrame[2]; // see exception codes in define area
    return;
  }

  // check function code
  if (rxFrame[1] != _frame[1]) {
    _status = MB_INVALID_FC;
    return;
  }

  // READ frame, transfer modbus registers to _value
  switch (rxFrame[1]) {
    case READ_HOLDING_REGISTERS:
    case READ_INPUT_REGISTERS:
      // check byte count equals to registers request
      if (rxFrame[2] == _frame[2] * 2) {
        uint8_t *ptr = (uint8_t *)_value;
        ptr += rxFrame[2] - 1;            // object last byte
        uint8_t i = 3;
        for (int count = rxFrame[2]; count; --count, i++, ptr--) *ptr = rxFrame[i];
        _status = MB_VALID_DATA;
        return;
      }
      else {
        _status = MB_ILLEGAL_DATA;
        return;
      }

    // PRESET frame, return OK
    case PRESET_MULTIPLE_REGISTERS:
      _status = MB_VALID_DATA;
      return;

    default:
      _status = MB_INVALID_FC;
      return;
  }
}

//------------------------------------------------------------------------------
// Construct Tx frame, send it, receive response and process it, reconstruct READ frame
void modbusSensor::processPreset(const uint8_t *ptr, uint8_t objectSize) {

  if (_frame[2] != READ_HOLDING_REGISTERS) {
    _status = MB_INVALID_ADR;
    return;
  }

  if (objectSize != _frame[5] * 2) {
    _status = MB_INVALID_DATA;
    return;
  }

  // Construct PRESET frame
  _frame[2] = PRESET_MULTIPLE_REGISTERS;
  _frame[6] = objectSize;
  uint8_t i = 7;

  ptr += objectSize - 1; // pointer to object last byte
  for (int count = objectSize; count; count--, i++) _frame[i] = *ptr--;
  uint16_t crc = calculateCRC(_frame, i);
  _frame[i++] = crc & 0x00FF;
  _frame[i++] = crc >> 8;
  _frameSize = i;

  // Send the PRESET frame, receive response and process it
  while (!MBSerial.available()) {}

  // reconstruct READ frame
  _frame[2] = READ_HOLDING_REGISTERS;
  crc = calculateCRC(_frame, 6);
  _frame[6] = crc & 0x00FF;
  _frame[7] = crc >> 8;
  _frameSize = 8;
}

//------------------------------------------------------------------------------
// read value
float modbusSensor::read() {
  if (_status == MB_TIMEOUT)
    switch (_hold) {
      case CHANGE_TO_ZERO: return 0;
      case CHANGE_TO_ONE: return 1;
      case HOLD_VALUE:;
    }
  return (float) * _value;
}

//------------------------------------------------------------------------------
// process status and return the right answer
void modbusSensor::processRead(uint8_t *ptr, const uint8_t objectSize) {
  if (_status == MB_TIMEOUT)
    switch (_hold) {
      case CHANGE_TO_ZERO:
        for (int count = objectSize; count; --count) *ptr++ = 0;
        return;
      case CHANGE_TO_ONE:;
      case HOLD_VALUE:;
    }
  // copy _value on the returned struct
  const uint8_t *e = _value;
  for (int count = objectSize; count; --count, ++e) *ptr++ = *e;
}


//------------------------------------------------------------------------------
// print status message
uint8_t modbusSensor::printStatus() {
  switch (_status) {
    case MB_VALID_DATA:
      Serial.println(F("Transmision successful"));
      return _status;
    case MB_INVALID_ID:
      Serial.println(F("No valid Id"));
      return _status;
    case MB_INVALID_FC:
      Serial.println(F("No valid FC"));
      return _status;
    case MB_TIMEOUT:
      Serial.println(F("Time out"));
      return _status;
    case MB_INVALID_CRC:
      Serial.println(F("incorrect CRC"));
      return _status;
    case MB_INVALID_BUFF:
      Serial.println(F("No valid buffer"));
      return _status;
    case MB_INVALID_ADR:
      Serial.println(F("No valid address"));
      return _status;
    case MB_INVALID_DATA:
      Serial.println(F("No valid data"));
      return _status;
    case MB_ILLEGAL_FC:
      Serial.println(F("Exception: Illegal FC"));
      return _status;
    case MB_ILLEGAL_ADR:
      Serial.println(F("Exception: Illegal address"));
      return _status;
    case MB_ILLEGAL_DATA:
      Serial.println(F("Exception: Illegal data"));
      return _status;
    case MB_SLAVE_FAIL:
      Serial.println(F("Exception: Slave failed "));
      return _status;
    case MB_EXCEPTION:
      Serial.println(F("Exception"));
      return _status;
  }
}

