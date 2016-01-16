#MANUAL DE USUARIO
##Alcance de la librería
Permitir la comunicación de un Arduino o ESP8266 en modo máster con dispositivos esclavos empleando un canal RS485 half-duplex.
Los esclavos son del tipo monitores de energía, en particular EASTRON modelos SDM120, SDM220, SDM320, SDM530M Y SDM630.
No utiliza interrupciones, excepto las empleadas por el puerto serie y la UART. El proceso de comunicación permite utilizar la librería con otros procesos en paralelo sin latencia.

Las funciones implementadas son:
 - 0x03 READ HOLDING REGISTERS
 - 0x04 READ INPUT REGISTERS
 - 0x10 PRESET MULTIPLES REGISTERS

##Conexiones.
(incluir esquema con MAX485 y MAX3485)

##Limitaciones.
- Las propias del MAX485, máximo 32 dispositivos esclavos.
- Se utiliza en exclusiva un puerto serie. El MEGA dispone de tres puertos serie. 
El UNO, Leonardo, etc, dispone de un único puerto serie conectado al puerto USB, por lo que no se pueden emplear simultáneamente. 
No se ha realizado la adaptación a SoftwareSerial, pero parece factible. 
- El ESP8266 dispone de dos puertos serie, uno de ellos ``Serial1`` solo de salida. (Ver apartado de ESP8266, pendiente).
- Se ha limitado el buffer RX a 32 bytes, lo que permite 6 registros de 4 bytes más 5 de la trama. En caso de requerir más modificar el archivo de cabecera.

No se ha implementado las funciones de lectura y escritura de 'coils'

##Archivos de la librería

La librería ModbusSensor se compone de los siguientes archivos:
- ``ModbusSensor.h`` archivo de cabecera con las definiciones de las clases,
- ``ModbusSensor.cpp`` archivo de métodos y rutinas,
- ``SDMdefines.h`` archivo de definiciones de los parámetros de los diferentes equipos modbus implementados. para los medidores de energía de la marca EASTRON, modelos SDM120, SDM220, SDM320, SDM530M Y SDM630.

##Objetos y métodos
``modbusMaster``

Para la clase ``modbusmaster`` se crea una única instancia, llamada ``MBSerial``. Se encarga de gestionar el canal serie al que está conectado un chip tipo MAX485.
Agrega una lista de punteros a objetos ``modbusSensor``, a los que llama consecutivamente en cada petición.

*Métodos:*

* ``MBSerial.config()`` configura los parámetros para realizar el proceso de peticiones
* ``MBserial.begin()`` inicia el canal serie.
* ``MBSerial.end()`` detiene el canal serie
* ``MBSerial.connect()`` conecta un objeto modbusSensor a la lista. Se realiza por defecto al construirse el objeto modbusSensor
* ``MBSerial.disconnect()`` desconecta un objeto modbusSensor de la lista. Se realiza por defecto al destruirse el objeto modbusSensor.
* ``MBSerial.sendRequest()`` inicia la lista de peticiones del bus, siempre que esté en estado ``IDLE``, es decir, haya terminado el último muestreo. Devuelve ``true`` si lo inicia con éxito, en caso contrario devuelve ``false``. En caso de no haber realizado antes  ``MBSerial.config()`` y ``MBserial.begin()`` el sketch puede quedarse en un bucle infinito.
* ``MBSerial.available()`` realiza el proceso de la Máquina de Estados Finitos, devuelve 'true' cuando se ha completado la lista de peticiones. 
En el resto de estados devuelve ``false``. Al no tener interrupción para activarse, se debe incluir dentro de un bucle.

Para procesos en bucle:
```
void loop() {
  ...
  MBSerial.sendRequest();
  if (MBSerial.available()) {
    energy = enrg.read();
  }
  ...
}
```
En caso de un proceso secuencial, en el que se requiera bloquear el proceso hasta obtener la respuesta:
```
  modbusSensor id(idNumber, DEVICE_ID, HOLD_VALUE, sizeof(float), READ_HOLDING_REGISTERS);
  while(!MBSerial.sendrequest()) {}
  while (!MBSerial.available()) {}
  Serial.print("Meter Id: ")); 
  Serial.println(id.read(), 0);
```

``modbusSensor``

El objeto ``modbusSensor`` está pensado para contener un valor o grupo de valores de un dispositivo esclavo modbus. En caso de varios valores deben almacenarse en registros consecutivos.
Dentro de cada instancia modbusSensor el valor o valores se almacena como una secuencia de bytes que refleja el contenido de los registros del esclavo.

Los dispositivos Eastron SDM almacenan los datos como valores ``float`` según IEEE 754, ocupan 4 bytes en 2 registros consecutivos. 
Otros valores están como valores BCD o hexadecimal. En todos ellos el orden de los bytes (*endianess*) es inverso al del Arduino. 
Por este motivo la transferencia de bytes se realiza en orden inverso. En caso de implementar un dispositivo con el mismo orden de bytes,
el usuario debera implementar su propia gestión de bytes para re-inverir el orden.

*Métodos:*

- Constructor, con los datos para construir la trama de petición. Solo funciones 0x03 y 0x04. Se conecta a la lista de ``MBSerial`` en orden de creación.
- Destructor, libera la memoria dinámica y desconecta el objeto de ``MBSerial``.
- ``connect()`` conecta el objeto a ``MBSerial``. Ya está incluido en el constructor, no es necesario llamarlo explícitamente.
- ``disconnect()`` desconecta el objeto de ``MBSerial``. Ya está incluido en el destructor, no es necesario llamarlo explícitamente
- ``preset()`` solo objetos con función 0x03. Recontruye la trama para enviar una petición 0x10 PRESET MULTIPLE REGISTERS con el valor del argumento. 
El tamaño del argumento debe coincidir con el de creación. no hace comprobación de coincidencia de tipos.
- ``read()`` lee el valor del registro. En caso de que la última petición no haya sido respondida (dispositivo sin corriente) devuelve el valor configurado (último leido, cero o uno). 
- ``getStatus()`` devuelve el status del objeto. La lista de posible valores se encuentra en el archivo de cabecera.
- ``printStatus()`` imprime un mensaje por el puerto *Serial* según el valor de *status*

**Sketchs de ejemplo**

*modbus-Energy-Monitor-Arduino.ino* Ejemplo para leer valores y parámetros en intervalos periódicos de tiempo, con salida por el puerto ``Serial``.

*modbus-Energy-max-min.ino* Ejemplo para leer valores en intervalos cortos, calcular máximos y mínimos, y en intervalo largo con salida por el puerto ``Serial``.

*SDM120configure.ino* Sketch para leer y cambiar los parámetros de un SDM120. Necesario para cambiar valores por defecto como *baud-rate*, o para poner más de uno dentro del mismo canal modbus.
