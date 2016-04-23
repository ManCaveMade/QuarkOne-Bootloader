# QuarkOne - Bootloader
*By: ManCave Made (Mitchell A. Cox)*

A bootloader for the Atmel XMEGA 128A4U based on LUFA and xboot, with a serial pass-through mode for programming the ESP8266.

### Activating the Bootloader

To activate the bootloader, "double click" the reset button (or use the RST pin with a button) within 750 ms. The bootloader flashes the onboard LED with two fast flashes every second. 

### Usage

*You will have to install the .inf driver file before plugging the Quark One in (otherwise no drivers will be found).* See below for instructions.

The bootloader should show up on your PC as two separate CDC serial ports. One port is an AVR911 compatible bootloader for the XMEGA. You can use AVRDude to program the microcontroller via this port. If you can't tell which port this is, send a '?' via a serial console and it should echo a '?' back.

The other serial port is a transparent passthrough to the Quark One ESP8266 (ESP-01) module. You can flash the ESP-01 using esptool without any modifications. The Quark One bootloader passes the DTR and RTS signals through to the ESP-01 to allow esptool to control which mode the ESP is in. *Typically, the ESP passthrough is the first serial port.*

#### ESP8266 Arduino

The ESP passthrough has been tested with the Arduino IDE in 'Generic ESP8266 Module' mode. Simply make sure the correct serial port is selected and most of the examples should work! The WiFiScan and ESP8266HTTPUpdateServer seem to work nicely.

See the official ESP8266 Arduino Project for details and instructions: https://github.com/esp8266/Arduino

#### Xmegaduino

The XMEGA microcontroller on the Quark One should be compatible with the Xmegaduino project. Work is in progress to update and create the relevant board files to facilitate this process. 

See my forked repository for work so far: https://github.com/ManCaveMade/Xmegaduino


### USB Drivers
You may find that the USB device on Windows is unrecognised. On Windows, download the QuarkOneBootloader.inf file from this repository and in the file browser, right click it and "Install" the driver. This will enable Windows to recognise the USB IDs as CDC devices.

## License

MIT License. Copyright (c) Mitchell A. Cox (mitch[at]enox[.]co[.]za)
