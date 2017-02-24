# TeensyDMX

This is a library for receiving DMX on the Teensy.

There is no support yet for sending DMX or for processing RDM.

## Technical notes

It is probably necessary to increase the serial buffer size to 513,
as follows. The example shows how to increase the size for Serial1.

```
#define SERIAL1_RX_BUFFER_SIZE 513
```

This is necessary in order to receive full-size DMX packets.
