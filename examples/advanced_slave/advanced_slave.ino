/**
 *  Modbus slave example 2:
 *  The purpose of this example is to link the Arduino digital and analog
 *	pins to an external device.
 *
 *  Recommended Modbus Master: QModbus
 *  http://qmodbus.sourceforge.net/
 *
 *  Editado al español por LuxARTS
 */

//Incluye la librería del protocolo Modbus
#include <ModbusRtu.h>
#define ID   1

//Crear instancia
Modbus slave(ID, Serial, 0); //ID del nodo. 0 para el master, 1-247 para esclavo
                        //Puerto serie (0 = TX: 1 - RX: 0)
                        //Protocolo serie. 0 para RS-232 + USB (default), cualquier pin mayor a 1 para RS-485
boolean led;
int8_t state = 0;
unsigned long tempus;

uint16_t au16data[9]; //La tabla de registros que se desea compartir por la red

/*********************************************************
 Configuración del programa
*********************************************************/
void setup() {
  io_setup(); //configura las entradas y salidas

  Serial.begin(19200); //Abre la comunicación como esclavo
  slave.start();
  tempus = millis() + 100; //Guarda el tiempo actual + 100ms
  digitalWrite(13, HIGH ); //Prende el led del pin 13 (el de la placa)
}

/*********************************************************
 Inicio del programa
*********************************************************/
void loop() {
  //Comprueba el buffer de entrada
  state = slave.poll( au16data, 9 ); //Parámetros: Tabla de registros para el intercambio de info
                                     //            Tamaño de la tabla de registros
                                     //Devuelve 0 si no hay pedido de datos
                                     //Devuelve 1 al 4 si hubo error de comunicación
                                     //Devuelve mas de 4 si se procesó correctamente el pedido

  if (state > 4) { //Si es mayor a 4 = el pedido fué correcto
    tempus = millis() + 50; //Tiempo actual + 50ms
    digitalWrite(13, HIGH);//Prende el led
  }
  if (millis() > tempus) digitalWrite(13, LOW );//Apaga el led 50ms después
  
  //Actualiza los pines de Arduino con la tabla de Modbus
  io_poll();
} 

/**
 * pin maping:
 * 2 - digital input
 * 3 - digital input
 * 4 - digital input
 * 5 - digital input
 * 6 - digital output
 * 7 - digital output
 * 8 - digital output
 * 9 - digital output
 * 10 - analog output
 * 11 - analog output
 * 14 - analog input
 * 15 - analog input
 *
 * pin 13 reservado para ver el estado de la comunicación
 */
void io_setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(6, LOW );
  digitalWrite(7, LOW );
  digitalWrite(8, LOW );
  digitalWrite(9, LOW );
  digitalWrite(13, HIGH ); //Led del pin 13 de la placa
  analogWrite(10, 0 ); //PWM 0%
  analogWrite(11, 0 ); //PWM 0%
}

/*********************************************************
Enlaza la tabla de registros con los pines
*********************************************************/
void io_poll() {
  // digital inputs -> au16data[0]
  // Lee las entradas digitales y las guarda en bits de la primera variable del vector
  // (es lo mismo que hacer una máscara)
  bitWrite( au16data[0], 0, digitalRead( 2 )); //Lee el pin 2 de Arduino y lo guarda en el bit 0 de la variable au16data[0] 
  bitWrite( au16data[0], 1, digitalRead( 3 ));
  bitWrite( au16data[0], 2, digitalRead( 4 ));
  bitWrite( au16data[0], 3, digitalRead( 5 ));

  // digital outputs -> au16data[1]
  // Lee los bits de la segunda variable y los pone en las salidas digitales
  digitalWrite( 6, bitRead( au16data[1], 0 )); //Lee el bit 0 de la variable au16data[1] y lo pone en el pin 6 de Arduino
  digitalWrite( 7, bitRead( au16data[1], 1 ));
  digitalWrite( 8, bitRead( au16data[1], 2 ));
  digitalWrite( 9, bitRead( au16data[1], 3 ));

  // Cambia el valor del PWM
  analogWrite( 10, au16data[2] ); //El valor de au16data[2] se escribe en la salida de PWM del pin 10 de Arduino. (siendo 0=0% y 255=100%)
  analogWrite( 11, au16data[3] );

  // Lee las entradas analógicas (ADC)
  au16data[4] = analogRead( 0 ); //El valor analógico leido en el pin A0 se guarda en au16data[4]. (siendo 0=0v y 1023=5v)
  au16data[5] = analogRead( 1 );

  // Diagnóstico de la comunicación (para debug)
  au16data[6] = slave.getInCnt();  //Devuelve cuantos mensajes se recibieron
  au16data[7] = slave.getOutCnt(); //Devuelve cuantos mensajes se transmitieron
  au16data[8] = slave.getErrCnt(); //Devuelve cuantos errores hubieron
}
