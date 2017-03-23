# TeensyDMX

This is a library for receiving and transmitting DMX on Teensy 3.

There is no support yet for processing RDM.

## How to use

The classes you'll need are in the `qindesign::teensydmx` namespace:
`Receiver` and `Sender`. Complete examples of how to use each are in `Flasher`
and `Chaser`, respectively.

### DMX receive

First, create an object using one of the hardware serial ports:

```c++
namespace teensydmx = ::qindesign::teensydmx;

teensydmx::Receiver dmxRx{Serial1};
```

Before using the instance, start the serial port internals:

```c++
dmxRx.begin();
```

Using your own buffer whose length is at least 513 bytes, check for a packet
having arrived:

```c++
int read = dmxRx.readPacket(buf);
```

`read` will contain the size of the packet including the start code, or be
negative if no packet is available. The start code is a special value, usually
zero, that occupies the first byte of the packet. The maximum DMX packet size
is 513, but may be smaller, depending on the system.

Each call to `readPacket` is independent, meaning that if no packet has
arrived, subsequent calls will return -1.

### DMX transmit

First, create an object using one of the hardware serial ports:

```c++
namespace teensydmx = ::qindesign::teensydmx;

teensydmx::Sender dmxTx{Serial1};
```

Before using the instance, start the serial port internals:

```c++
dmxTx.begin();
```

Set one or more channel values using one of the `set` functions:

```c++
// Set channel 6 to 219
dmxTx.set(6, 219);
```

The other `set` function can set multiple channels at once. This is left as an
exercise to the reader.

## Technical notes

### Simultaneous transmit and receive

The same serial port can't be used for simultaneous transmit and receive.
This is because the library uses the serial port hardware for data instead
of direct pin control. The break portion of a DMX frame needs to be
transmitted at a different baud rate than the slots (channels), and since
reception and transmission aren't necessarily synchronized, two different
serial ports must be used.

Use `Receiver` to receive and `Sender` to transmit.

### Transmission rate

From a UART perspective, there are two parts to a DMX frame:

1. Break, 83333 (1000000/12) baud, 8N1
2. 513 slots, 250000 baud, 8N2

The total frame time is:

> 10 bits * 12 us + 513 slots * 11 bits * 4us = 22692us
> This is about 44Hz.

## Code style

Code style for this project follows the
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
