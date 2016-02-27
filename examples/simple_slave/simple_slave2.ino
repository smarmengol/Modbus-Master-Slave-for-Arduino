/**
 *  Modbus slave example 2:
 *  The purpose of this example is to link a data array
 *  from the Arduino to an external device.
 *
 *  Recommended Modbus Master: modpoll
 *  http://www.modbusdriver.com/modpoll.html
 */

#include <ModbusRtu.h>

// data array for modbus network sharing
uint16_t au16data[16] = {
  3, 1415, 9265, 4, 2, 7182, 28182, 8, 0, 0, 0, 0, 0, 0, 1, -1 };

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus slave(1,0,0); // this is slave @1 and RS-232 or USB-FTDI

void setup() {
  slave.begin( 19200, SERIAL_8E1 ); // 19200 baud, 8-bits, even, 1-bit stop
}

void loop() {
  slave.poll( au16data, 16 );
}
