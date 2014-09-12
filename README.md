GP22
====

This is an Arduino C++ library for the Acam GP22 Time-to-Digital Converter (TDC), an integrated sub-nanosecond resolution timer chip. It will also work with the GP21.

This library includes
* Methods to initialize the SPI connection to the TDC and test communication
* Methods to read and write the GP22's registers
* Methods to calibrate the TDC (which cache register values to pause operation, calibrate, and resume arbitrarily).
* Enum definitions for each config register bit and opcode, for clearer code
* Debug methods to print register values to a serial port

The code meant to be used in conjunction with the [datasheet](http://www.acam.de/download-center/tdc/) as a starting point for your application, though it has not been tested extensively enough to act as a "black box" for the TDC. For example, the correct sequence of register reads and writes to initialize and calibrate the TDC can be tricky to work out from the translated datasheet, and it has already been done here.

## TDCtalk example
`examples/TDCtalk` includes application-specific code for using the GP22 for time-correlated single photon counting (TCSPC), and sending the data back to a PC application using a custom serial protocol. This is not a made-up instructional example, but somewhat ugly "real" application code. However, some parts of the code will hopefully be useful even for another application.

This example initializes the TDC and sets it up to work in "fast mode" (with EN_FAST_INIT enabled). It then attaches an interrupt to the pin connected to the GP22's `INT` pin. On each reading, the TDC triggers an interrupt, and the Arduino reads and saves the uncalibrated measurement to a buffer. Upon a request from the PC application, the Arduino sends the buffer contents to the PC. It can also calibrate the TDC at any time (momentarily ignoring new readings, then resuming). On an Arduino Uno, this can support a maximum event rate of about 5 kHz. The corresponding LabVIEW client application is included, but requires LabVIEW to run.


## Installation
To use with the Arduino IDE, place the GP22 library folder into the `libraries` subfolder of your Arduino sketches folder, and restart the IDE.
