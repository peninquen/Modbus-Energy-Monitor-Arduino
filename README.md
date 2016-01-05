# Modbus-Energy-Monitor-Arduino
Comunicate with diferent energy monitors using modbus protocol over RS485, mainly Eastron SDM120, SDM220 and SDM630.
Implemented function codes 0x03, 0x04 and 0x10. No coils functions have been implemented.

Achieved milestones:
- Accesible to basic users. All modbus protocol issues are inside the classes. Simple messages to access data values and parameters.
- Poll design. Basic function repeat the array of queries to poll data values.
- Non-blocking behaviour. You can use it embebed with different process without delay.
- Escalable. Eastron energy monitors works with float values. modbusSensor object requests single float values and contiguous float values, but also every struct of data that your slave's registers stores.
- Configurable. You can configure SDM's holding registers
- Exception error process. Adjust response in case of offline status due to power outage. 

* Author: Jaime García  @peninquen
* License: Apache License Version 2.0.
