# TeensyDMX

This is a library for receiving and transmitting DMX on the Teensy.

There is no support yet for processing RDM.

## Technical notes

### Simultaneous transmit and receive

The same serial port can't be used for simultaneous transmit and receive.
This is because the library uses the serial port hardware for data instead
of direct pin control. The break portion of a DMX frame needs to be
transmitted at a different baud rate than the slots (channels), and since
reception and transmission aren't necessarily synchronized, two different
serial ports must be used.

Use `TeensyDMXReceiver` to receive and `TeensyDMXSender` to transmit. 

### Transmission rate

From a UART perspective, there are two parts to a DMX frame:

1. Break, 83333 (1000000/12) baud, 8N1
2. 513 slots, 250000 baud, 8N2

The total frame time is:

> 10 bits * 12 us + 513 slots * 11 bits * 4us = 22692us
> This is about 44Hz.
