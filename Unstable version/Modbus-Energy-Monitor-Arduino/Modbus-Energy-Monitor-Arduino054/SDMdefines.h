/*
  Eastron address register values
*/

// Read values use function code 4 (0x04)
#define VOLTAGE   0x0000		//All Float
#define CURRENT   0x0006
#define POWER     0x000C
#define APOWER    0x0012
#define RAPOWER   0x0018
#define PFACTOR   0x001E
#define PANGLE    0x0024
#define FREQUENCY 0x0046
#define IAENERGY  0x0048
#define EAENERGY  0x004A
#define IRAENERGY 0x004C
#define ERAENERGY 0x004E
#define TAENERGY  0x0156
#define TRENERGY  0x0158

// Read/Write Configuration.
// Read function code 3  (0x03)
// Write function code 16 (0x10)
#define NPARSTOP  		  0x0012		// HEX, bit de parada, (0:none, 1:even, 2:odd)
#define DEVICE_ID 		  0x0014		// Float, identificador esclavo (1 - 247)
#define BAUD_RATE 		  0x001C		// Float, Baud rate (0:2400 1:4800 2:9600 5:1200)   
#define TIME_DISP_220   0xF500 		// BCD, Tiempo entre pantallas (0-30s) SDM220
#define TIME_DISP 		  0xF900		// BCD, Tiempo entre pantallas (0-30s) SDM120
#define PULSE1_KWH      0xF910    // Pulsos/Kwh (0:1000, 1:100, 2:10 3:1 pulso/Kwh)
#define TOT_MODE        0xF920    // HEX, modo medida energía (1-3)
#define PULSE1_MODE     0xF930    // Modo salida pulsos a led (0:imp+exp, 1:imp, 2:exp)

/*
   Maintaining old definitions.
*/


// Direcciones registros de datos solo lectura. Valores tipo float.
// Utilizar funcion 04 lectura, numero de bytes 4.

#define VOL_ADR VOLTAGE   // VOLTAJE.
#define CUR_ADR CURRENT   // CORRIENTE.
#define POW_ADR POWER     // POTENCIA ACTIVA. 
#define APO_ADR APOWER    // Potencia Aparente.
#define RPO_ADR RAPOWER   // Potencia Reactiva.
#define PFA_ADR PFACTOR   // Factor de potencia.
#define PAN_ADR PANGLE    // ANGULO PHI. desfase entre voltaje y corriente
#define FRE_ADR FREQUENCY // Frecuencia.
// REVISAR FALTAN PARAMENTROS O PARAMENTROS PARA SDM220

#define PEN_ADR IAENERGY  // ENERGIA IMPORTADA KWH
#define REN_ADR EAENERGY  // Energia exportada.
#define TEN_ADR TAENERGY  // Energia activa Total.
#define TRE_ADR TRENERGY  // Energia reactiva Total.

/* Default SDM configuration.
  #define SDM120C_METER_NUMBER   1
  #define SDM120C_BAUDRATE       2400
  #define SDM120C_BYTEFORMAT     SERIAL_8N2    //Prty n
*/

/*Modbus optimum settings
  #define TIMEOUT 1000
  #define POLLING 200  	// the scan rate
  #define RETRYCOUNT 10   // numero de reintentos, para volver set the "connection" variable to
*/

// Get Parameter
