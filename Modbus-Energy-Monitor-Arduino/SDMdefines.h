/*
 *Eastron addresses registers 
*/

// Read values use Funtion 4 (0x04)
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

// Read/Write Configuration. Funtion 16 0x10
#define NPARSTOP  		0x0012		//HEX
#define DEVICE_ID 		0x0014		//Float
#define BAUD_RATE 		0x001C		//Float
#define TIME_DISP_220 	0xF500 		//BCD
#define TIME_DISP 		0xF900		//BCD
#define TOT_MODE  		0xF920		//HEX	

/*
 * Maintaining old definitions. 
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
