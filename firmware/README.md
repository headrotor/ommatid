# ommatid -- CAD files

Firmware for the Ommatid spherical display/sensor array. 

* Here's an [Instructable about the enclosure and dome fabrication](http://www.instructables.com/id/Ommatid-Spherical-Display-constructing-the-enclosu/)
* Here's an [Instructable about the electronics and software](http://www.instructables.com/id/Ommatid-Spherical-Display-Electronics-Programming-/)

This is C code designed to be executed on the 19 triangular boards inside the Ommatid. Each board has an ATTiny microcontroller, four independent IR LEDs and corresponding IR photodarlingtons (to sense reflected IR), and four WS2812B RGB LEDs. Note the RGB LEDs are not controlled or even connected to,  the ATTtiny).

All 19 boards are connected to the same RS-485 serial line at 250000
baud. Boards communicate with the host using a "speak only when spoken
to" protocol to eliminate bus contention.  Each board has an address
and will respond only to serial commands where it has been
specifically addressed. Addresses are one-byte ASCII characters from
a-o inclusive.

Serial protocol documentation: forthcoming!
