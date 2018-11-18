# TeensyDMX

This is a library for receiving and transmitting DMX on Teensy 3 and Teensy LC.
It follows the
[ANSI E1.11 DMX512-A specification](http://tsp.esta.org/tsp/documents/docs/ANSI-ESTA_E1-11_2008R2018.pdf).

There is no specific support yet for processing RDM or responding to other
alternate start codes.

## Features

Some notable features of this library:

1. Teensy's default serial buffer isn't used; the data goes directly to/from
   the DMX buffers from/to the UART ISRs. In other words, the library
   is asynchronous and runs independently; all you need to worry about is
   setting and getting channel data.
2. Simple API: After setup, there's only one read call (`readPacket`) and two
   forms of one write call (`set` for single and multiple channels).
3. The library properly handles DMX packets containing less than 513 slots.
4. The transmitter refresh rate can be changed to something less than
   "maximum rate".
5. The transmitter can be paused and resumed to allow for packets that must
   be adjacent to other packets. In other words, the asynchronous transmitter
   can be used synchronously. For example, System Information Packets (SIP)
   require this. See Annex D5 of ANSI E1.11.

## How to use

The classes you'll need are in the `qindesign::teensydmx` namespace:
`Receiver` and `Sender`. Complete examples of how to use each are in `Flasher`
and `Chaser`, respectively.

All class documentation can be found in `src/TeensyDMX.h`.

### DMX receive

First, create an object using one of the hardware serial ports, different from
any serial ports being used for transmission:

```c++
namespace teensydmx = ::qindesign::teensydmx;

teensydmx::Receiver dmxRx{Serial1};
```

Before using the instance, start the serial port internals:

```c++
dmxRx.begin();
```

Using your own buffer whose length is at least `len` bytes, check for a packet
having arrived, and if it has, copy `len` values starting from channel
`startChannel`:

```c++
// For this example, assume buf is at least len bytes long
int read = dmxRx.readPacket(buf, startChannel, len);
```

`read` will contain the number of bytes read, -1 if no packet is available, or
zero if no values were read.

Note that channel zero contains the start code, a special value, usually zero,
that occupies the first byte of the packet. The maximum DMX packet size is
513, but may be smaller, depending on the system.

Each call to `readPacket` is independent, meaning that if no packet has
arrived after a call to this function, subsequent calls will return -1.

### DMX transmit

First, create an object using one of the hardware serial ports, different from
any serial ports being used for receive:

```c++
namespace teensydmx = ::qindesign::teensydmx;

teensydmx::Sender dmxTx{Serial2};
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

#### Transmission rate

The transmission rate can be changed from a maximum of about 44Hz down to as
low as you wish. See the `setRefreshRate` and `getRefreshRate` in `Sender`.

Note that the rate won't be higher than the limits dictated by the protocol,
about 44Hz, no matter how high it's set. The default is, in fact, `INFINITY`.

#### Pausing and resuming transmit

`Sender` is an asynchronous packet transmitter; packets are always being sent.
To ensure that certain packets are adjacent to others, such as for System
Information Packets (SIP), the API provides a way to send packets synchronously.

The `pause()` function pauses packet transmission, the `resume()` function
resumes transmission, and `resumeFor(int)` resumes transmission for a specific
number of packets, after which transmission is paused again. `isTransmitting()`
indicates whether the transmitter is sending anything while paused---it always
returns `true` when not paused---and can be used to determine when it's safe
to start filling in packet data after a `resumeFor` call.

Let's say you want to send a SIP packet immediately after a regular packet.
The following code shows how to accomplish this:

```c++
dmxTx.pause();
// [Use one of the dmxTx.set functions to set regular packet data]
dmxTx.resumeFor(1);  // Send one regular packet
while (dmxTx.isTransmitting()) {  // Wait for this packet to be sent
  yield();
}
// [Fill in the SIP data]
dmxTx.resumeFor(1);  // Send the SIP data
while (dmxTx.isTransmitting()) {  // Wait for this packet to be sent
  yield();
}
// [Return to filling in the regular data, but fill in at least one packet]
dmxTx.resume();  // Resume transmitting regular data
```

Other functions of interest are `isPaused()` and `getResumedRemaining()`.
`isPaused` indicates whether the transmitter is paused. `getResumedRemaining`
returns the number of packets that will be sent before the transmitter is
paused again.

## Technical notes

### Simultaneous transmit and receive

The same serial port can't be used for simultaneous transmit and receive.
This is because the library uses the serial port hardware for data instead
of direct pin control. The break portion of a DMX frame needs to be
transmitted at a different baud rate than the slots (channels), and since
reception and transmission aren't necessarily synchronized, two different
serial ports must be used.

Use `qindesign::teensydmx::Receiver` to receive and
`qindesign::teensydmx::Sender` to transmit.

### Transmission rate

From a UART perspective, there are two parts to a DMX frame:

1. BREAK, 50000 baud, 8N1.
2. Up to 513 slots, 250000 baud, 8N2.

The total frame time is:

10 bits * 20 us + 513 slots * 11 bits * 4us = 22772us, or a rate of about
43.91Hz.

## Code style

Code style for this project mostly follows the
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

## References

Inspirations for this library:

1. Chris Staite's [TeensyDmx](https://github.com/chrisstaite/TeensyDmx)
   library, which is further based on
   Matthias Hertel's [DMXSerial2](https://github.com/mathertel/DmxSerial2),
   Ward's [DmxReceive](http://forum.pjrc.com/threads/19662-Arduinoesque-overriding-of-core-functionality?p=24993&viewfull=1#post24993),
   and
   Paul Stoffregen's [DmxSimple](https://github.com/PaulStoffregen/DmxSimple).
2. Claude Heintz's [LXTeensy3DMX_Library](https://github.com/claudeheintz/LXTeensy3DMX_Library).

---

Copyright (c) 2017-2018 Shawn Silverman
