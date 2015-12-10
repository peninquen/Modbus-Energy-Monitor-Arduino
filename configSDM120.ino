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
# define SERIAL_OUTPUT 1 // verbose
#include "SimpleModbusMasterSDM120.h"

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
unsigned int idNumber = 1;                   // Id 001
#define SDM120C_BAUDRATE       2400          // b 2400
#define SDM120C_BYTEFORMAT     SERIAL_8N2    // Prty n
//

#define TIMEOUT 1000     // tiempo hasta dar por fallida la petición
#define POLLING 5000     // tiempo entre toma de datos, no se usa, solo una petición por parámetro.
#define RETRYCOUNT 2    // numero de reintentos fallidos hasta 'connection' false que desactiva el packet
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
  Serial.println(F("Enter expected Id:"));
  while (!Serial.available()) {}
  idNumber = Serial.parseInt();
  Serial.println(F("Long press button to enter -SET- mode"));

  // Iniciamos comunicación modbus SERIAL1 Arduino Mega.
  modbus_configure(&Serial1, SDM120C_BAUDRATE, SDM120C_BYTEFORMAT, TIMEOUT, POLLING, RETRYCOUNT, TXENPIN, packets, TOTAL_NO_OF_PACKETS);

  //Procesa el valor de registro id
  modbus_construct(parameterPacket, idNumber, READ_HOLDING_REGISTERS, ID_ADR, 2, parameter.Array);
  if (!processRequest()) return;
  Serial.print(F("Meter Id: ")); Serial.println(parameter.F, 0);
  Serial.print(F("New Id: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 1 && newParameter <= 247) {
    parameter.F = (float)newParameter;
    Serial.println(parameter.F, 0);
    modbus_construct(parameterPacket, idNumber, PRESET_MULTIPLE_REGISTERS, ID_ADR, 2, parameter.Array);
    if (!processRequest()) return;
    Serial.println(F(" done"));
    idNumber = newParameter; // actuliza el id para las siguientes peticiones
  }
  else Serial.println(F("Skip"));
  
  //Procesa el valor de registro baud rate
  modbus_construct(parameterPacket, idNumber, READ_HOLDING_REGISTERS, BAUD_ADR, 2, parameter.Array);
  if (!processRequest()) return;
  Serial.print(F("Baud rate (0:2400 1:4800 2:9600 5:1200): ")); Serial.println(parameter.F, 0);
  Serial.print(F("New baud rate: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  switch (newParameter) {
    case 0:
    case 1:
    case 2:
    case 5:
      parameter.F = (float)newParameter;
      Serial.println(parameter.F, 0);
      modbus_construct(parameterPacket, idNumber, PRESET_MULTIPLE_REGISTERS, BAUD_ADR, 2, parameter.Array);
      if (!processRequest()) return;
      Serial.println(F(" done"));
      break;
    default: Serial.println(F("Skip"));
  }

  //Procesa el valor de registro 'tiempo entre pantallas'
  modbus_construct(parameterPacket, idNumber, READ_HOLDING_REGISTERS, TURN_ADR, 1, parameter.Array);
  if (!processRequest()) return;
  Serial.print(F("Time of display in turns (0 - 30 seconds): ")); Serial.print(parameter.Array[0]>>4); Serial.println(parameter.Array[0]%0x10);
  Serial.print(F("New time: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 0 && newParameter <= 30) {
    parameter.Array[0] = newParameter % 10 | (newParameter / 10 << 4); //convert to BCD
    Serial.println(newParameter);
    modbus_construct(parameterPacket, idNumber, PRESET_MULTIPLE_REGISTERS, TURN_ADR, 1, parameter.Array);
    if (!processRequest()) return;
    Serial.println(F(" done"));
  }
  else Serial.println(F("Skip"));

  //Procesa el valor de registro 'salida pulso 1'
  modbus_construct(parameterPacket, idNumber, READ_HOLDING_REGISTERS, PULSE1_ADR, 1, parameter.Array);
  if (!processRequest()) return;
  Serial.print(F("Pulse 1 output (0:1000, 1:100, 2:10 3:1 imp/Kwh): ")); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("New pulse 1 output value: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 0 && newParameter <= 3) {
    parameter.Array[0] = newParameter;
    Serial.println(parameter.Array[0], HEX);
    modbus_construct(parameterPacket, idNumber, PRESET_MULTIPLE_REGISTERS, PULSE1_ADR, 1, parameter.Array);
    if (!processRequest()) return;
    Serial.println(F(" done"));
  }
  else Serial.println(F("Skip"));

  //Procesa el valor de registro 'modo medida de energía'
  modbus_construct(parameterPacket, idNumber, READ_HOLDING_REGISTERS, MODE_ADR, 1, parameter.Array);
  if (!processRequest()) return;
  Serial.print(F("Measurement mode (0:mode 1, 1:mode 2, 2:mode 3): ")); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("New pulse 1 output value: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 0 && newParameter <= 2) {
    parameter.Array[0] = newParameter;
    Serial.println(parameter.Array[0], HEX);
    modbus_construct(parameterPacket, idNumber, PRESET_MULTIPLE_REGISTERS, MODE_ADR, 1, parameter.Array);
    if (!processRequest()) return;
    Serial.println(F(" done"));
  }
  else Serial.println(F("Skip"));

  //Procesa el valor de registro 'modo salida pulso 1'
  modbus_construct(parameterPacket, idNumber, READ_HOLDING_REGISTERS, PULSE1_MODE_ADR, 1, parameter.Array);
  if (!processRequest()) return;
  Serial.print(F("Pulse 1 output mode (0:imp+exp, 1:imp, 2:exp): ")); Serial.println(parameter.Array[0], HEX);
  Serial.print(F("New pulse 1 output mode: "));
  while (!Serial.available()) {}
  newParameter = Serial.parseInt();
  if (newParameter >= 0 && newParameter <= 2) {
    parameter.Array[0] = newParameter;
    Serial.println(parameter.Array[0], HEX);
    modbus_construct(parameterPacket, idNumber, PRESET_MULTIPLE_REGISTERS, PULSE1_MODE_ADR, 1, parameter.Array);
    if (!processRequest()) return;
    Serial.println(F(" done"));
  }
  else Serial.println(F("Skip"));

  // Salir del modo -SET- para que se hagan efectivos los cambios, después apagar y encender el SMD120
  Serial.println(F("Now long press for 3 seconds to exit the -SET- mode. Then restart the SMD120 to aply the changes."));
}

void loop() {} // no hace nada...

boolean  processRequest() {
  unsigned int successfulRequests = parameterPacket->successful_requests; // número actual de peticiones con éxito
  do {
    modbus_update(); //en marcha la FSM para procesar...
  } while ((successfulRequests == parameterPacket->successful_requests) && (parameterPacket->connection)); //repetir mientras no haya respuesta y esté activa la conexión
  if (parameterPacket->connection) { // si hay conexión es que ha sido éxito
    parameterPacket->connection = 0; // desactiva la conexión hasta construir un nuevo packet
    return true;
  }
  else {
    Serial.println(F("No connection"));
    return false;

  }
}
