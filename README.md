README.txt

libmodbus is a library that provides a Serial Modbus implementation for Arduino.

A primary goal was to enable industrial communication for the Arduino in order to link it to industrial devices such as HMIs, CNCs, PLCs, temperature regulators or speed drives.

Library contents:
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
