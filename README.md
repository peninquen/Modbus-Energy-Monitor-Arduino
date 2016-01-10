# Modbus-Energy-Monitor-Arduino
Comunicate with diferent energy monitors using modbus RTU protocol over RS485, mainly Eastron SDM120, SDM220, SDM530 and SDM630.
Implemented function codes 0x03, 0x04 and 0x10. No coils functions have been implemented.

Achieved milestones:
- Accesible to basic users. All modbus protocol issues are inside the classes. Simple messages to access data values and parameters.
- Poll design. Basic function repeat the array of queries to poll data values.
- Multi-thread, non-blocking behaviour. You can use it embebed with different process without delay.
- Escalable. Eastron energy monitors works with float values. modbusSensor object can do requests of single float values and contiguous float values, but also every struct of data that your slave's registers stores.
- Configurable. You can configure SDM's holding registers
- Exception error process. Adjust response in case of offline status due to power outage. Other errors are user configurable.

Next objetives:
- Define other energy monitors, like Circutor CVM-MINI
- Implement nodeMCU and other ESP8266 library version.
- Explore other slave modbus RTU equipment
- Create 'modbusSlave' object, use Arduino as a slave using Modbus RTU protocol. 
- Document library functions and example sketchs

Any comments, bugs and issues are wellcome.

Librería arduino para comunicar con el protocolo Modbus RTU sobre RS485 y gestionar datos de un medidor de energía, en principio de la marca EASTRON SDM120, SDM220, SDM530 y SDM630.
Se ha implementado la gestión de las funciones 0x03, 0x04 y 0x10 dl protocolo Moddbus. No se ha implementado las funciones de 'coils'.

Objetivos alcanzados:
- Accesible a usuarios con conocimientos básicos del protocolo Modbus. Toda la gestión del protocolo queda en el interior de las clases, empleado funciones sencillas de lectura y escritura para acceder a los datos y parámetros.
- Petición periódica de datos. Se repite la lista de peticiones para registrar los datos.
- Comportamiento sin bloqueo ni uso de 'delays', que permite ejecutar varios procesos conjuntos sin sufrir retrasos.
- Escalable. Los modelos de EASTRON trabajan con valores tipo 'float'. el objeto 'modbusSensor' puede hacer peticiones de un solo valor o varios valores contiguos, además de permitir al usuario crear estructuras de datos que almacene los registros del esclavo.
- Configurable. Dispone de las funciones y ejemplo de sketch de configuración de los parametros de un Eastron SDM.
- Gestión de errores. Ajusta la respuesta en caso de no responder por apagón. Otros errores configurables por el usuario.

Siguientes objetivos:
- Definir los registros y formato de otros monitores de energía, como el CIRCUTOR CVM-MINI.
- Implementar la librería sobre nodeMCU y otros ESP8266.
- Explorar otros equipos con protocolo Modbus RTU.
- Crear el objeto 'modbusSlave' y sus métodos, de forma que un arduino pueda operar como esclavo sobre Modbus RTU
- Documentar las funciones y los sketchs de ejemplo.

Cualquier comentarios, incidencias o sugerencias será  bienvenido para mejorar la librería.  

* Author: Jaime García  @peninquen
* License: Apache License Version 2.0.
