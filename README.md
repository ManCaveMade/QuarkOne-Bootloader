# QuarkOne - Bootloader
## By: ManCave Made (Mitchell A. Cox)

A bootloader for the Atmel XMEGA 128A4U based on LUFA and xboot, with a serial pass-through mode for programming the ESP8266.

To activate the bootloader, "double click" the reset button (or use the RST pin with a button) within 750 ms. The bootloader flashes the onboard LED with two fast flashes every second. 

The bootloader should show up on your PC as two separate CDC serial ports. One port is an AVR911 compatible bootloader for the XMEGA. You can use AVRDude to program the microcontroller via this port. If you can't tell which port this is, send a '?' via a serial console and it should echo a '?' back.

The second serial port is a transparent passthrough to the Quark One ESP8266 (ESP-01) module. You can flash the ESP-01 using esptool without any modifications. The Quark One bootloader passes the DTR and RTS signals through to the ESP-01 to allow esptool to control which mode the ESP is in.

The ESP passthrough has been tested with the Arduino IDE in 'Generic ESP8266 Module' mode.

## License

MIT License. Copyright (c) Mitchell A. Cox (mitch[at]enox[.]co[.]za)
