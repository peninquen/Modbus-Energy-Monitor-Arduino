<<<<<<< HEAD
#include "SimpleModbusMasterSDM120.h"
#include "HardwareSerial.h"

//#define SERIAL_OUTPUT 1 //VERBOSE

#if SERIAL_OUTPUT
#   define SERIAL_BEGIN(...) Serial.begin(__VA_ARGS__)
#   define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#   define SERIAL_PRINT(...) Serial.println(__VA_ARGS__)
#else
#   define SERIAL_BEGIN(...)
#   define SERIAL_PRINT(...)
#   define SERIAL_PRINTLN(...)
#endif

// state machine states
#define IDLE 1
#define WAITING_FOR_REPLY 2
#define WAITING_FOR_TURNAROUND 3

#define BUFFER_SIZE 64

unsigned char state;
unsigned char retry_count;
unsigned char TxEnablePin;

// frame[] is used to receive and transmit packages.
// The maximum number of bytes in a modbus packet is 256 bytes
// This is limited to the serial buffer of 64 bytes
unsigned char frame[BUFFER_SIZE];
unsigned char buffer;
unsigned int timeout; // timeout interval
unsigned int polling; // turnaround delay interval
unsigned int T1_5; // inter character time out in microseconds
unsigned long delayStart; // init variable for turnaround and timeout delay
unsigned int total_no_of_packets;
Packet* packetArray; // packet starting address
Packet* packet; // current packet
HardwareSerial* ModbusPort;

// function definitions
void idle();
void constructPacket();
unsigned char construct_F15();
unsigned char construct_F16();
void waiting_for_reply();
void processReply();
void waiting_for_turnaround();
void process_F1_F2();
void process_F4_SDM120();
void process_F3_F4();
void process_F15_F16();
void processError();
void processSuccess();
unsigned int calculateCRC(unsigned char bufferSize);
void sendPacket(unsigned char bufferSize);

// Modbus Master State Machine
void modbus_update()
{
  switch (state)
  {
    case IDLE:
      idle();
      break;
    case WAITING_FOR_REPLY:
      waiting_for_reply();
      break;
    case WAITING_FOR_TURNAROUND:
      waiting_for_turnaround();
      break;
  }
}

void idle()
{
  static unsigned int packet_index;

  unsigned int failed_connections = 0;

  unsigned char current_connection;

  do
  {
    if (packet_index == total_no_of_packets) // wrap around to the beginning
      packet_index = 0;

    // proceed to the next packet
    packet = &packetArray[packet_index];

    // get the current connection status
    current_connection = packet->connection;

    if (!current_connection)
    {
      // If all the connection attributes are false return
      // immediately to the main sketch
      if (++failed_connections == total_no_of_packets)
        return;
    }
    packet_index++;

    // if a packet has no connection get the next one
  } while (!current_connection);

  constructPacket();
}

void constructPacket()
{
  packet->requests++;
  frame[0] = packet->id;
  frame[1] = packet->function;
  frame[2] = packet->address >> 8; // address Hi
  frame[3] = packet->address & 0xFF; // address Lo
  // For functions 1 & 2 data is the number of points
  // For functions 3, 4 & 16 data is the number of registers
  // For function 15 data is the number of coils
  frame[4] = packet->data >> 8; // MSB
  frame[5] = packet->data & 0xFF; // LSB


  unsigned char frameSize;

  // construct the frame according to the modbus function
  if (packet->function == PRESET_MULTIPLE_REGISTERS)
    frameSize = construct_F16();
  else if (packet->function == FORCE_MULTIPLE_COILS)
    frameSize = construct_F15();
  else // else functions 1,2,3 & 4 is assumed. They all share the exact same request format.
    frameSize = 8; // the request is always 8 bytes in size for the above mentioned functions.

  unsigned int crc16 = calculateCRC(frameSize - 2);
  frame[frameSize - 2] = crc16 >> 8; // split cfrc into 2 bytes
  frame[frameSize - 1] = crc16 & 0xFF;
  sendPacket(frameSize);

  state = WAITING_FOR_REPLY; // state change

  // if broadcast is requested (id == 0) for function 15 or 16 then override
  // the previous state and force a success since the slave wont respond
  if (packet->id == 0)
    processSuccess();
}

unsigned char construct_F15()
{
  // function 15 coil information is packed LSB first until the first 16 bits are completed
  // It is received the same way..
  unsigned char no_of_registers = packet->data / 16;
  unsigned char no_of_bytes = no_of_registers * 2;

  // if the number of points dont fit in even 2byte amounts (one register) then use another register and pad
  if (packet->data % 16 > 0)
  {
    no_of_registers++;
    no_of_bytes++;
  }

  frame[6] = no_of_bytes;
  unsigned char bytes_processed = 0;
  unsigned char index = 7; // user data starts at index 7
  unsigned int temp;

  for (unsigned char i = 0; i < no_of_registers; i++)
  {
    temp = packet->register_array[i]; // get the data
    frame[index] = temp & 0xFF;
    bytes_processed++;

    if (bytes_processed < no_of_bytes)
    {
      frame[index + 1] = temp >> 8;
      bytes_processed++;
      index += 2;
    }
  }
  unsigned char frameSize = (9 + no_of_bytes); // first 7 bytes of the array + 2 bytes CRC + noOfBytes
  return frameSize;
}

unsigned char construct_F16()
{
  unsigned char no_of_bytes = packet->data * 2;

  // first 6 bytes of the array + no_of_bytes + 2 bytes CRC
  frame[6] = no_of_bytes; // number of bytes
  unsigned char index = 7; // user data starts at index 7
  unsigned char no_of_registers = packet->data;
  unsigned int temp;

  for (unsigned char i = no_of_registers; i ; i--)
  {
    temp = packet->register_array[i - 1]; // get the data
    frame[index] = temp >> 8;
    index++;
    frame[index] = temp & 0xFF;
    index++;
  }
  unsigned char frameSize = (9 + no_of_bytes); // first 7 bytes of the array + 2 bytes CRC + noOfBytes
  return frameSize;
}

void waiting_for_turnaround()
{
  if ((millis() - delayStart) > polling)
    state = IDLE;
}

// get the serial data from the buffer
void waiting_for_reply()
{
  if ((*ModbusPort).available()) // is there something to check?
  {
    unsigned char overflowFlag = 0;
    buffer = 0;
    Serial.print(" SLAVE:");
    while ((*ModbusPort).available())
    {
      // The maximum number of bytes is limited to the serial buffer size
      // of BUFFER_SIZE. If more bytes is received than the BUFFER_SIZE the
      // overflow flag will be set and the serial buffer will be read until
      // all the data is cleared from the receive buffer, while the slave is
      // still responding.
      if (overflowFlag)
        (*ModbusPort).read();
      else
      {
        if (buffer == BUFFER_SIZE)
          overflowFlag = 1;

        frame[buffer] = (*ModbusPort).read();
        if(frame[buffer] < 0x10) Serial.print(" 0");
        else Serial.print(" ");
        Serial.print(frame[buffer], HEX); 
        buffer++;
      }
      // This is not 100% correct but it will suffice.
      // worst case scenario is if more than one character time expires
      // while reading from the buffer then the buffer is most likely empty
      // If there are more bytes after such a delay it is not supposed to
      // be received and thus will force a frame_error.

      delayMicroseconds(T1_5); // inter character time out
    }
    Serial.println();
    // The minimum buffer size from a slave can be an exception response of
    // 5 bytes. If the buffer was partially filled set a frame_error.
    // The maximum number of bytes in a modbus packet is 256 bytes.
    // The serial buffer limits this to 128 bytes.

    if ((buffer < 5) || overflowFlag)
      processError();

    // Modbus over serial line datasheet states that if an unexpected slave
    // responded the master must do nothing and continue with the time out.
    // This seems silly cause if an incorrect slave responded you would want to
    // have a quick turnaround and poll the right one again. If an unexpected
    // slave responded it will most likely be a frame error in any event
    else if (frame[0] != packet->id) // check id returned
      processError();
    else
      processReply();
  }
  else if ((millis() - delayStart) > timeout) // check timeout
  {
    processError();
    state = IDLE; //state change, override processError() state
  }
}

void processReply()
{
  // combine the crc Low & High bytes
  unsigned int received_crc = ((frame[buffer - 2] << 8) | frame[buffer - 1]);
  unsigned int calculated_crc = calculateCRC(buffer - 2);

  if (calculated_crc == received_crc) // verify checksum
  {
    // To indicate an exception response a slave will 'OR'
    // the requested function with 0x80
    if ((frame[1] & 0x80) == 0x80) // extract 0x80
    {
      packet->exception_errors++;
      processError();
    }
    else
    {
      switch (frame[1]) // check function returned
      {
        case READ_COIL_STATUS:
        case READ_INPUT_STATUS:
          process_F1_F2();
          break;
        case READ_INPUT_REGISTERS:
        case READ_HOLDING_REGISTERS:
          if (packet -> is_SDM120 == true)
            process_F4_SDM120();
          else
            process_F3_F4();
          break;
        case FORCE_MULTIPLE_COILS:
        case PRESET_MULTIPLE_REGISTERS:
          process_F15_F16();
          break;
        default: // illegal function returned
          processError();
          break;
      }
    }
  }
  else // checksum failed
  {
    processError();
  }
}

void process_F1_F2()
{
  // packet->data for function 1 & 2 is actually the number of boolean points
  unsigned char no_of_registers = packet->data / 16;
  unsigned char number_of_bytes = no_of_registers * 2;

  // if the number of points dont fit in even 2byte amounts (one register) then use another register and pad
  if (packet->data % 16 > 0)
  {
    no_of_registers++;
    number_of_bytes++;
  }

  if (frame[2] == number_of_bytes) // check number of bytes returned
  {
    unsigned char bytes_processed = 0;
    unsigned char index = 3; // start at the 4th element in the frame and combine the Lo byte
    unsigned int temp;
    for (unsigned char i = 0; i < no_of_registers; i++)
    {
      temp = frame[index];
      bytes_processed++;
      if (bytes_processed < number_of_bytes)
      {
        temp = (frame[index + 1] << 8) | temp;
        bytes_processed++;
        index += 2;
      }
      packet->register_array[i] = temp;
    }
    processSuccess();
  }
  else // incorrect number of bytes returned
    processError();
}

void process_F4_SDM120()
{
  // Hacemos un swap LOW, HIGH (unsigned int) formato float Arduino.
  if (frame[2] == 4)
  {
    packet->register_array[1] = (frame[3] << 8) | frame[4];
    packet->register_array[0] = (frame[5] << 8) | frame[6];
    processSuccess();
  }
  else // incorrect number of bytes returned
    processError();
}


void process_F3_F4()
{
  // check number of bytes returned - unsigned int == 2 bytes
  // data for function 3 & 4 is the number of registers
  if (frame[2] == (packet->data * 2))
  {
    unsigned char index = 3;
    for (unsigned char i = packet->data; i; i--)
    {      
      // start at the 4th element in the frame and combine the Lo byte
      packet->register_array[i - 1] = (frame[index] << 8) | frame[index + 1];
      index += 2;
    }
    processSuccess();
  }
  else // incorrect number of bytes returned
    processError();
}

void process_F15_F16()
{
  // Functions 15 & 16 is just an echo of the query
  unsigned int recieved_address = ((frame[2] << 8) | frame[3]);
  unsigned int recieved_data = ((frame[4] << 8) | frame[5]);

  if ((recieved_address == packet->address) && (recieved_data == packet->data))
    processSuccess();
  else
    processError();
}

void processError()
{
  packet->retries++;
  packet->failed_requests++;

  // if the number of retries have reached the max number of retries
  // allowable, stop requesting the specific packet
  if (packet->retries == retry_count)
  {
    packet->connection = 0;
    packet->retries = 0;
  }
  state = WAITING_FOR_TURNAROUND;
  delayStart = millis(); // start the turnaround delay
}

void processSuccess()
{
  packet->successful_requests++; // transaction sent successfully
  packet->retries = 0; // if a request was successful reset the retry counter
  state = WAITING_FOR_TURNAROUND;
  delayStart = millis(); // start the turnaround delay
}

void modbus_configure(HardwareSerial* SerialPort,
                      long baud,
                      unsigned char byteFormat,
                      unsigned int _timeout,
                      unsigned int _polling,
                      unsigned char _retry_count,
                      unsigned char _TxEnablePin,
                      Packet* _packets,
                      unsigned int _total_no_of_packets)
{
  // Modbus states that a baud rate higher than 19200 must use a fixed 750 us
  // for inter character time out and 1.75 ms for a frame delay for baud rates
  // below 19200 the timing is more critical and has to be calculated.
  // E.g. 9600 baud in a 11 bit packet is 9600/11 = 872 characters per second
  // In milliseconds this will be 872 characters per 1000ms. So for 1 character
  // 1000ms/872 characters is 1.14583ms per character and finally modbus states
  // an inter-character must be 1.5T or 1.5 times longer than a character. Thus
  // 1.5T = 1.14583ms * 1.5 = 1.71875ms. A frame delay is 3.5T.
  // Thus the formula is T1.5(us) = (1000ms * 1000(us) * 1.5 * 11bits)/baud
  // 1000ms * 1000(us) * 1.5 * 11bits = 16500000 can be calculated as a constant

  if (baud > 19200)
    T1_5 = 750;
  else
    T1_5 = 16500000 / baud; // 1T * 1.5 = T1.5

  // initialize
  state = IDLE;
  timeout = _timeout;
  polling = _polling;
  retry_count = _retry_count;
  TxEnablePin = _TxEnablePin;
  total_no_of_packets = _total_no_of_packets;
  packetArray = _packets;

  ModbusPort = SerialPort;
  (*ModbusPort).begin(baud, byteFormat);

  pinMode(TxEnablePin, OUTPUT);
  digitalWrite(TxEnablePin, LOW);
=======
/*  SDM120C Modbus RTU Protocol con Arduino.
*  version 0.0.1 by @peninquen
*  based on version 0.0.4  by @cosmopaco.
*
*  Sketch para modificar los parametros de comunicación del dispositivo:
*  Poner el SDM120 en modo -SET- pulsando durante 3 segundos en el pulsador.
*  Introducir los nuevos datos por el terminal serial
*  En caso de introducir un valor no válido, no realiza la acción y salta al siguiente parámetro
*  Algunos parametros requieren reiniciar el SMD120 para tener efecto, eperar a completar el programa para reiniciar...
*  Materiales
*    Arduino Mega
*    Modulo RS485
*    Conexiones (Configuracion por defecto) Puerto serial 1
*    Arduino pin   Modulo RS485 pin
*    19            RO (receive out)
*    18            DI (data in)
*    17            DE/RE (data enable/receive enable).
*
*/
#include "SimpleModbusMasterSDM120.h"
# define SERIAL_OUTPUT 0 // verbose
// Direcciones registros de datos de configuración de lectura y escritura, valores tipo float.
// Utilizar funcion 03 para lectura, función 16 para escritura, 2 registros, número de bytes 4.

#define ID_ADR          0X0014    // meter id (1-247).
#define BAUD_ADR        0X001C    // Baud rate (0:2400 1:4800 2:9600 5:1200)
#define TURN_ADR        0XF900    // Tiempo entre pantallas (0-30s)
#define PULSE1_ADR      0XF910    // Pulsos/Kwh (0:1000, 1:100, 2:10 3:1 pulso/Kwh)
#define MODE_ADR        0XF920    // Modo medida energía (1-3)
#define PULSE1_MODE_ADR 0XF930    // Modo salida pulsos a led (0:imp+exp, 1:imp, 2:exp)
//

/*
Constants are provided for:
  Function 1  - READ_COIL_STATUS
  Function 2  - READ_INPUT_STATUS
  Function 3  - READ_HOLDING_REGISTERS
  Function 4  - READ_INPUT_REGISTERS
  Function 15 - FORCE_MULTIPLE_COILS
  Function 16 - PRESET_MULTIPLE_REGISTERS
   Valid modbus byte formats are:
    SERIAL_8N2: 1 start bit, 8 data bits, 2 stop bits
    SERIAL_8E1: 1 start bit, 8 data bits, 1 Even parity bit, 1 stop bit
    SERIAL_8O1: 1 start bit, 8 data bits, 1 Odd parity bit, 1 stop bit
*/
// Verificar que realmente tiene estos parametros antes de empezar.
// Pulsar el botón para pasar por las pantallas hasta las de configuración.
#define SDM120C_METER_NUMBER   1             // Id 001
#define SDM120C_BAUDRATE       2400          // b 2400
#define SDM120C_BYTEFORMAT     SERIAL_8N2    // Prty n
//

#define TIMEOUT 1000     // tiempo hasta dar por fallida la petición
#define POLLING 5000     // tiempo entre toma de datos, no se usa, solo una petición por parámetro.
#define RETRYCOUNT 10    // numero de reintentos fallidos hasta 'connection' false que desactiva el packet
// para volver poner 'connection' a true.
#define TXENPIN  17      // Pin cambio recibir/transmite para el driver RS485


// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
  PACKET1,
  TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Create a packetPointer to access each packet
// individually. This is not required you can access
// the array explicitly. E.g. packets[PACKET1].id = 2;
// This does become tedious though...
packetPointer parameterPacket = &packets[PACKET1];


// Union
union datas {
  byte  b[4];      // modo pruebas, en principio no se usa
  float F;
  unsigned int Array[2];
} parameter;
unsigned int newParameter;


void setup() {
  //Iniciamos puerto serial"0" Arduino Mega para entrada/salida de datos.
  Serial.begin(9600);
  delay(1000);
  Serial.println(F("Config SDM120-Modbus"));
  Serial.println(F("Long press button to enter -SET- mode"));

  // Iniciamos comunicación modbus SERIAL1 Arduino Mega.
  modbus_configure(&Serial1, SDM120C_BAUDRATE, SDM120C_BYTEFORMAT, TIMEOUT, POLLING, RETRYCOUNT, TXENPIN, packets, TOTAL_NO_OF_PACKETS);

  //Procesa el valor de registro id
  modbus_construct(parameterPacket, SDM120C_METER_NUMBER, READ_HOLDING_REGISTERS, ID_ADR, 2, parameter.Array);
  if (!processRequest()) return;
  Serial.print(parameter.Array[1], HEX); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("Meter Id: ")); Serial.println(parameter.F, 1);
  Serial.print(F("New Id: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 1 && newParameter <= 247) {
    parameter.F = (float)newParameter;
    Serial.println(parameter.F, 1);
    modbus_construct(parameterPacket, SDM120C_METER_NUMBER, PRESET_MULTIPLE_REGISTERS, ID_ADR, 2, parameter.Array);
    if (!processRequest()) return;
    Serial.println(F(" done"));
    
  }

  //Procesa el valor de registro baud rate
  modbus_construct(parameterPacket, SDM120C_METER_NUMBER, READ_HOLDING_REGISTERS, BAUD_ADR, 2, parameter.Array);
  if (!processRequest()) return;
  Serial.print(parameter.Array[1], HEX); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("Baud rate (0:2400 1:4800 2:9600 5:1200): ")); Serial.println(parameter.F, 1);
  Serial.print(F("New baud rate: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  switch (newParameter) {
    case 0:
    case 1:
    case 2:
    case 5:
      parameter.F = (float)newParameter;
      Serial.println(parameter.F, 1);
      modbus_construct(parameterPacket, SDM120C_METER_NUMBER, PRESET_MULTIPLE_REGISTERS, BAUD_ADR, 2, parameter.Array);
      if (!processRequest()) return;
      Serial.println(F(" done"));
  }

  //Procesa el valor de registro 'tiempo entre pantallas'
  modbus_construct(parameterPacket, SDM120C_METER_NUMBER, READ_HOLDING_REGISTERS, TURN_ADR, 2, parameter.Array);
  if (!processRequest()) return;
  Serial.print(parameter.Array[1], HEX); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("Time of display in turns (0 - 30 seconds): ")); Serial.print(parameter.Array[1], HEX); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("New time: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 0 && newParameter <= 30) {
    parameter.Array[0] = newParameter % 10 | (newParameter / 10 << 4); //convert to BCD
    Serial.println(parameter.Array[0], HEX);
//    modbus_construct(parameterPacket, SDM120C_METER_NUMBER, PRESET_MULTIPLE_REGISTERS, TURN_ADR, 2, parameter.Array);
//    if (!processRequest()) return;
    Serial.println(F(" done"));
  }

  //Procesa el valor de registro 'salida pulso 1'
  modbus_construct(parameterPacket, SDM120C_METER_NUMBER, READ_HOLDING_REGISTERS, PULSE1_ADR, 2, parameter.Array);
  if (!processRequest()) return;
  Serial.print(parameter.Array[1], HEX); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("Pulse 1 output (0:1000, 1:100, 2:10 3:1 imp/Kwh): ")); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("New pulse 1 output value: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 1 && newParameter <= 4) {
    parameter.Array[0] = newParameter;
    Serial.println(parameter.Array[0], HEX);
    modbus_construct(parameterPacket, SDM120C_METER_NUMBER, PRESET_MULTIPLE_REGISTERS, PULSE1_ADR, 2, parameter.Array);
    if (!processRequest()) return;
    Serial.println(F(" done"));
  }

  //Procesa el valor de registro 'modo salida pulso 1'
  modbus_construct(parameterPacket, SDM120C_METER_NUMBER, READ_HOLDING_REGISTERS, PULSE1_MODE_ADR, 2, parameter.Array);
  if (!processRequest()) return;
  Serial.print(parameter.Array[1], HEX); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("Pulse 1 output mode (0:imp+exp, 1:imp, 2:exp): ")); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("New pulse 1 output mode: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 1 && newParameter <= 2) {
    parameter.Array[0] = newParameter;
    Serial.println(parameter.Array[0], HEX);
    modbus_construct(parameterPacket, SDM120C_METER_NUMBER, PRESET_MULTIPLE_REGISTERS, PULSE1_MODE_ADR, 2, parameter.Array);
    if (!processRequest()) return;
    Serial.println(F(" done"));
  }

  //Procesa el valor de registro 'modo medida de energía'
  modbus_construct(parameterPacket, SDM120C_METER_NUMBER, READ_HOLDING_REGISTERS, MODE_ADR, 2, parameter.Array);
  if (!processRequest()) return;
  Serial.print(parameter.Array[1], HEX); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("Measurement mode (0:mode 1, 1:mode 2, 2:mode 3): ")); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("New pulse 1 output value: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 1 && newParameter <= 3) {
    parameter.Array[0] = newParameter;
    Serial.println(parameter.Array[0], HEX);
    modbus_construct(parameterPacket, SDM120C_METER_NUMBER, PRESET_MULTIPLE_REGISTERS, MODE_ADR, 2, parameter.Array);
    if (!processRequest()) return;
    Serial.println(F(" done"));
  }
>>>>>>> parent of 1d13894... Update configSDM120.ino

}

void modbus_construct_SDM120(Packet *_packet,
                             unsigned char id,
                             unsigned int address,
                             unsigned int* register_array)
{
  _packet->id = id;
  _packet->function = READ_INPUT_REGISTERS;
  _packet->address = address;
  _packet->data = 2; // En realidad no es necesario......
  _packet->register_array = register_array;
  _packet->connection = 1;
  _packet ->is_SDM120 = true;
}

void modbus_construct(Packet *_packet,
                      unsigned char id,
                      unsigned char function,
                      unsigned int address,
                      unsigned int data,
                      unsigned int* register_array)
{
  _packet->id = id;
  _packet->function = function;
  _packet->address = address;
  _packet->data = data;
  _packet->register_array = register_array;
  _packet->connection = 1;
  _packet ->is_SDM120 = false;
}

unsigned int calculateCRC(unsigned char bufferSize)
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order.
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp;
}

void sendPacket(unsigned char bufferSize)
{
  digitalWrite(TxEnablePin, HIGH);
  Serial.print("MASTER:");
  for (unsigned char i = 0; i < bufferSize; i++) {
    (*ModbusPort).write(frame[i]);
    if (frame[i] < 0x10) Serial.print(" 0");
    else Serial.print(" ");
    Serial.print(frame[i], HEX);
  }
  (*ModbusPort).flush();
  Serial.println();
  // It may be necessary to add a another character delay T1_5 here to
  // avoid truncating the message on slow and long distance connections

  digitalWrite(TxEnablePin, LOW);

  delayStart = millis(); // start the timeout delay
}
