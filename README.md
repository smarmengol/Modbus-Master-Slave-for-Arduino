README.txt

libmodbus is a library that provides a Serial Modbus implementation for Arduino.

A primary goal was to enable industrial communication for the Arduino in order to link it to industrial devices such as HMIs, CNCs, PLCs, temperature regulators or speed drives.

now you can use software serial with the update from Helium6072!

LIBRARY CONTENTS
=================================================================
LICENSE.txt			GNU Licence file
keywords.txt		Arduino IDE colouring syntax

/documentation
Library documentation generated with Doxygen.

/examples
Sample sketches to implement miscellaneous settings:

/examples/advanced_slave	Modbus slave node, which links Arduino pins to the Modbus port.
/examples/RS485_slave		Modbus slave adapted to the RS485 port
/examples/simple_master		Modbus master node with a single query
/examples/simple_slave		Modbus slave node with a link array
/examples/software_serial_simple_master		Modbus master node that works via software serial

INSTALLATION PROCEDURE
=================================================================
Refer to this documentation to Install this library:

http://arduino.cc/en/Guide/Libraries

Starting with version 1.0.5, you can install 3rd party libraries in the IDE.

Do not unzip the downloaded library, leave it as is.

In the Arduino IDE, navigate to Sketch > Import Library. At the top of the drop down list, select the option to "Add Library". 

You will be prompted to select this zipped library. 

Return to the Sketch > Import Library menu. You should now see the library at the bottom of the drop-down menu. It is ready to be used in your sketch. 

The zip file will have been expanded in the libraries folder in your Arduino sketches directory.

NB : the library will be available to use in sketches, but examples for the library will not be exposed in the File > Examples until after the IDE has restarted. 


KNOWN ISSUES
=================================================================
It is not compatible with ARDUINO LEONARDO and not tested under ARDUINO DUE and newer boards.

TODO List
=================================================================
Common to Master and Slave:

1) Implement other Serial settings: parity, stop bits, ...

2) End frame delay, also known as T35

3) Test it with several Arduino boards: UNO, Mega, etc..

4) Extend it to Leonardo

Master:

1) Function code 1 and 2 still not implemented

2) Function code 15 still not implement

3) Other codes under development

New features by Helium6072 29 July 2016
=================================================================
1) "port->flush();" changed into "while(port->read() >= 0);"

Since Serial.flush() (port->flush(); in ModbusRtu.h line 287, 337, & 827) no longer empties incoming buffer on 1.6 (Arduino.cc : flush() "Waits for the transmission of outgoing serial data to complete. Prior to Arduino 1.0, this instead removed any buffered incoming serial data.), use "while(port->read() >= 0);" instead.

2) software serial compatible

New constructor Modbus::Modbus(uint8_t u8id) and method void Modbus::begin(SoftwareSerial *sPort, long u32speed) that makes using software serial possible.
Check out sexample "software_serial_simple_master" and learn more!