# Readme for TeensyDMX v3.2.0

This is a library for receiving and transmitting DMX on Teensy 3 and Teensy LC.
It follows the
[ANSI E1.11 DMX512-A specification](http://tsp.esta.org/tsp/documents/docs/ANSI-ESTA_E1-11_2008R2018.pdf).

## Features

Some notable features of this library:

1. Teensy's default serial buffer isn't used; the data goes directly to/from
   the DMX buffers from/to the UART ISRs. In other words, the library is
   asynchronous and runs independently; all you need to worry about is setting
   and getting channel data.
2. Simple API: After setup, there's only two read calls (`readPacket` and `get`)
   and two forms of one write call (`set` for single and multiple channels).
3. The library properly handles DMX packets containing less than 513 slots.
4. The transmitter refresh rate can be changed to something less than
   "maximum rate".
5. The transmitter can be paused and resumed to allow for packets that must be
   adjacent to other packets. In other words, the asynchronous transmitter can
   be used synchronously. For example, System Information Packets (SIP) require
   this. See Annex D5 of ANSI E1.11.
6. The receiver checks for timeouts according to the DMX specification. It
   knows of the concept of being disconnected from a DMX transmitter when
   timeouts or bad BREAKs are encountered in the data stream.
7. Error counts are available in the receiver. These can be used to detect
   protocol problems, including timeouts, framing errors and bad BREAKs, and
   short packets (those less than 1196us).
8. The receiver can be used synchronously through the use of the `Responder`
   API. Alternate start codes can not only be handled, for example, for Text
   Packets or System Information Packets (SIP), but responses can be sent back
   to the transmitter, for example for RDM.

### Receiver timing limitations

There are limitations in the handling of received DMX frame timing. For BREAK
and Mark after Break (MAB) times, only the following cases are checked and not
accepted as a valid DMX frame start:

1. BREAK duration &lt; ~44us.
2. BREAK duration + MAB duration &lt; ~96us.
3. BREAK &lt; ~88us and MAB &ge; ~44us.

The following case is accepted as a valid frame start, even though it isn't
compliant with the DMX specification:

1. BREAK duration &gt; ~52us and MAB duration &lt; ~44us.

For example, if a BREAK comes in having a duration of 53us and then a MAB comes
in having a duration of 43us, their sum is 96us, and so the packet will be
accepted. More generally, this will allow BREAKs that are shorter than the
minimum required 88us if the MAB is shorter than 44us.

This limitation is solvable if code is added to watch for RX line changes, but
this likely won't happen until a future release.

## How to use

The classes you'll need are in the `qindesign::teensydmx` namespace: `Receiver`
and `Sender`. `Receiver` examples are in `BasicReceive` and `Flasher`, and
`Sender` examples are in `BasicSend` and `Chaser`. `Flasher` and `Chaser` are
more complete examples.

Other examples that show how to utilize synchronous transmission are in
`SIPSenderAsync` and `SIPSenderSync`.

Examples that show how to use a synchronous packet handler in a receiver are in
`SIPHandler` and `TextPacketHandler`.

All class documentation can be found in `src/TeensyDMX.h`.

### Synchronous vs. asynchronous operation

Both transmission and reception operate asynchronously. This means that
there's potentially a continuous stream of data being sent or received in
the background.

The transmitter will keep sending the same data until it's changed externally
using one of the `Sender::set` functions. Similarly, `Receiver::readPacket` and
`Receiver::get` will return only the latest data; if more than a small amount of
time elapses between calls, then the data may have been changed more than once.

Both `Sender` and `Receiver` have a mode where they can operate synchronously.
With `Sender`, the stream can be paused and resumed, and packets inserted at
appropriate spots. Similarly, `Receiver` can send received packets as they
arrive to start code-specific instances of `Responder`.

## DMX receive

### Code example

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

Each call to `readPacket` is independent, meaning that if no packet has arrived
after a call to this function, subsequent calls will return -1.

### Error counts and disconnection

The DMX receiver keeps track of three types of errors:

1. Packet timeouts.
2. Framing errors, including bad BREAKs.
3. Short packets, i.e. those packets that occupy less than 1196us.

The counts can be retrieved via `packetTimeoutCount()`, `framingErrorCount()`,
and `shortPacketCount()`.

An associated concept is _disconnection_. A receiver is considered _connected_
when it is receiving valid DMX packets. When any timeouts occur, or when invalid
BREAKs are detected, then the receiver considers itself _disconnected_. As soon
as valid packets reappear, the receiver is once again _connected_.

To determine whether a receiver is connected, call its `connected()` function.
A callback function, to be notified when this event happens, can also be set
using `onConnectChange`.

The following example sets the built-in LED according to the current connected
state. It uses a lambda expression, but a normal function could be used instead.

```c++
dmxRx.onConnectChange([](Receiver *r) {
  digitalWriteFast(LED_BUILTIN, r.connected() ? HIGH : LOW);
});
```

#### The truth about connection detection

In actual fact, the connection detection only works for _some_ of the cases
where timeouts occur or when bad BREAKs happen. The one thing it can't do is
detect when BREAK or IDLE conditions occur "permanently". Unless more data comes
in, no further UART interrupts will be generated, so it is up to the user's code
to detect this case.

The `Flasher` example contains an example of how to do this. In essence, the
`readPacket` function will return a positive value if the desired data is
available in the last packet received. A timer can be reset when valid data is
received, and then subsequent code can use the difference between the current
time and the timer value to determine if there's been a timeout.

An example that uses `elapsedMillis` to encapsulate the current time call:

```c++
constexpr uint32_t kTimeout = 1000;  // In milliseconds

elapsedMillis lastPacketTimer{0};
uint8_t buf[250];

void loop() {
  int read = dmxRx.readPacket(buf, 0, 217);
  if (read == 217) {  // Comparing to > 0 means that _some_ data was received
    // All the requested data was received
    // Do stuff with it
    lastPacketTimer = 0;
  }

  if (lastPacketTimer <= kTimeout) {
    // Do work
  } else {
    // No connection
  }
}
```

A second technique would be to use the value returned from
`lastPacketTimestamp()` as something that closely approximates the true latest
timestamp. For example:

```c++
constexpr uint32_t kTimeout = 1000;  // In milliseconds

void loop() {
  if (millis() - dmxRx.lastPacketTimestamp() > kTimeout) {
    // Respond to the timeout
  }
  // Do work
}
```

In actuality, a timer could be used to detect this condition, but it was chosen
to not do this because it would use up a timer, add one extra timeout-specifying
method, and the user code is probably good enough.

In summary, the _connected_ concept here has more to do with line noise and bad
timing than it does with a physical connection. Perhaps a future release will
rename this API concept or address it with the timer...

### Synchronous operation by using custom responders

There is the ability to notify specific instances of `Responder` when packets
having specific start codes arrive. To implement the simplest form, simply
extend `Responder`, override the protected `receivePacket` function, and attach
an instance to one or more start codes using `Receiver::setResponder`.
`receivePacket` will be called for each packet received that has one of the
desired start codes.

As well, by default, handlers will "eat" packets so that they aren't available
to callers to the `Receiver` API. To change this behaviour, override
`Responder::eatPacket()`.

For example, let's say you want to change the local display when a text packet
arrvies. The following partial code example shows how to do this.

```c++
class TextHandler : public teensydmx::Responder {
 public:
  static constexpr uint8_t kStartCode = 0x17;

  TextHandler() : teensydmx::Responder() {}

 protected:
  void receivePacket(const uint8_t *buf, int len) override {
    // The packet must contain at least 3 bytes (plus the start code)
    if (len < 4) {
      return;
    }
    uint8_t page = buf[1];
    uint8_t charsPerLine = buf[2];

    // Some checks should be made here for the data not ending in a
    // NUL character

    // Assume the existence of this function
    setText(page, charsPerLine,
            reinterpret_cast<const char *>(&buf[3]), len - 3);
  }
}

TextHandler textHandler;

void setup() {
  // ...
  dmxRx.setResponder(TextHandler::kStartCode, textHandler)
  // ...

  dmxRx.begin();
}
```

Responders can be added at any time.

Complete synchronous operation examples using SIP and text packets can be found
in `SIPHandler` and `TextPacketHandler`.

#### Responding

Protocols such as RDM need the ability, not only to process specific packets,
but to respond to them as well. Timing is important, so a `Responder`
implementation can also be notified of each byte as it arrives. The function of
interest is `processByte`.

As bytes are received, the implementation tracks some internal state. When it is
decided that a response is necessary, it returns a positive value indicating how
many bytes it placed into the output buffer, for transmitting back to the
transmitter. The `Responder` needs to implement `outputBufferSize()` in order
for any response to be sent. `processByte` will be passed a buffer at least as
large as the value returned from `outputBufferSize()`.

Some other functions that specify some timings should also be implemented.
Please consult the `Responder.h` documentation for more details.

Because all processing happens within an interrupt context, it should execute as
quickly as possible. Any long-running operations should be executed in the main
loop (or some other execution context). If the protocol allows for it, the
`Responder` can reply with a "not yet" response, and then return any queued
processing results when ready.

A more complete example is beyond the scope of this README.

## DMX transmit

### Code example

First, create an object using one of the hardware serial ports, different from
any serial ports being used for receive:

```c++
namespace teensydmx = ::qindesign::teensydmx;

teensydmx::Sender dmxTx{Serial2};
```

Before using the instance, optionally set up the packet size and refresh rate,
and then start the serial port internals to begin transmission:

```c++
// Optional: dmxTx.setPacketSize(100);
// Optional: dmxTx.setRefreshRate(40);
dmxTx.begin();
```

Set one or more channel values using one of the `set` functions:

```c++
// Set channel 6 to 219
dmxTx.set(6, 219);
```

The other `set` function can set multiple channels at once. This is left as an
exercise to the reader.

### Packet size

The packet size can be adjusted and retrieved via `setPacketSize` and
`packetSize()`. Smaller packets will naturally result in a higher
refresh rate.

This can be changed at any time.

### Transmission rate

The transmission rate can be changed from a maximum of about 44Hz down to as low
as you wish. See the `setRefreshRate` and `refreshRate()` in `Sender`.

Note that the rate won't be higher than the limits dictated by the protocol,
about 44Hz, no matter how high it's set. The default is, in fact, `INFINITY`.

This can be changed at any time.

### Synchronous operation by pausing and resuming

`Sender` is an asynchronous packet transmitter; packets are always being sent.
To ensure that certain packets are adjacent to others, such as for System
Information Packets (SIP), the API provides a way to send packets synchronously.

Firstly, the `pause()` function pauses packet transmission, the `resume()`
function resumes transmission, and `resumeFor(int)` resumes transmission for a
specific number of packets, after which transmission is paused again.

There are two ways to achieve synchronous operation. The first is with
`isTransmitting()`. It indicates whether the transmitter is sending anything
while paused---it always returns `true` when not paused---and can be used
to determine when it's safe to start filling in packet data after a `resumeFor`
or `pause()` call.

The second way is to provide a function to `onDoneTransmitting`. The function
will be called when the same conditions checked by `isTransmitting()` occur. It
will be called from inside an ISR, so take this into account.

It is important to note that when utilizing the pause feature, changing data via
the `set` functions should only be done while not transmitting. Pausing doesn't
immediately stop transmission; the pause happens after the current packet is
completely sent. Changing the data may affect this packet.

Let's say you want to send a SIP packet immediately after a regular packet. The
following code shows how to accomplish this using the polling approach:

```c++
// Before the code starts looping, pause the transmitter
dmxTx.pause();

// Loop starts here
fillRegularData();
dmxTx.resumeFor(1);  // Send one regular packet
while (dmxTx.isTransmitting()) {  // Wait for this packet to be sent
  yield();
}
fillSIPData();
dmxTx.resumeFor(1);  // Send the SIP data
while (dmxTx.isTransmitting()) {  // Wait for this packet to be sent
  yield();
}
```

Using the asynchronous notification approach requires keeping track of some
state, and is slightly more complex than the polling approach.

Other functions of interest are `isPaused()` and `resumedRemaining()`.
`isPaused()` indicates whether the transmitter is paused (but still potentially
transmitting). `resumedRemaining()` returns the number of packets that will be
sent before the transmitter is paused again.

Complete synchronous operation examples using SIP can be found in
`SIPSenderAsync` and `SIPSenderSync`. The first uses the asynchronous
notification approach and the second uses the polling approach.

## Technical notes

### Simultaneous transmit and receive

The same serial port can't be used for simultaneous transmit and receive. This
is because the library uses the serial port hardware for data instead of direct
pin control. The break portion of a DMX frame needs to be transmitted at a
different baud rate than the slots (channels), and since reception and
transmission aren't necessarily synchronized, two different serial ports must
be used.

Use `qindesign::teensydmx::Receiver` to receive and
`qindesign::teensydmx::Sender` to transmit.

### Transmission rate

From a UART perspective, there are two parts to a DMX frame:

1. BREAK, 50000 baud, 8N1.
2. Up to 513 slots, 250000 baud, 8N2.

The total frame time is approximately:

10 bits * 20 us + 513 slots * 11 bits * 4us = 22772us, or a rate of about
43.91Hz.

The total frame time may be a little longer due to switching baud rates
internally and the existence of some interrupt and code execution latency.

### Transmit/receive enable pins

Some setups may require that an external part be enabled when transmitting or
receiving. For example, an RS485 transceiver may require enabling or disabling
specific buffers. That may be accomplished by using one of the GPIO pins. Please
be sure the logic levels are compatible.

### Thread safety

This code is not thread-safe and should be handled appropriately if utilized in
a concurrent context.

### Dynamic memory allocation failures

The `Receiver::setResponder` function dynamically allocates memory. On small
systems, this may fail. The caller can check for this condition by examining
`errno` for `ENOMEM`. If this occurs, then the function will return `nullptr`,
but otherwise fails silently. Additionally, all responders are wiped out,
including any previously-set responders.

### Hardware connection

DMX uses RS-485 differential signalling. This means that a transceiver is needed
between the Teensy and the DMX lines. See
[DMX512](https://en.wikipedia.org/wiki/DMX512) for DMX connector pin guidance.

1. Basically, DMX uses A, B, and ground, and the Teensy serial ports use RX, TX,
   and ground. After choosing a serial port on the Teensy, connect TX to the
   _Driver Input_ and RX to the _Receiver Output_ of the transceiver.
2. Since RS-485 is half-duplex, the transceiver provides a way, via either one
   or two pins, to select between receiving and transmitting modes. For the
   two-pin case, these likely can be merged because they're logically opposite,
   for example, HIGH to enable the driver and LOW to enable the receiver. In
   both cases, choose one of the Teensy's digital output pins to act as the
   transmit-or-receive selector.
3. Last, connect the ground of the Teensy to the ground of the transceiver.

It is beyond the scope of this document to describe how to accommodate
transmission line effects of long lines.

## Code style

Code style for this project mostly follows the
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

Other conventions are adopted from Bjarne Stroustrup's and Herb Sutter's
[C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md).

## References

Inspirations for this library:

1. Chris Staite's [TeensyDmx](https://github.com/chrisstaite/TeensyDmx)
   library, which is further based on
   Matthias Hertel's [DMXSerial2](https://github.com/mathertel/DmxSerial2),
   Ward's
   [DmxReceive](http://forum.pjrc.com/threads/19662-Arduinoesque-overriding-of-core-functionality?p=24993&viewfull=1#post24993),
   and
   Paul Stoffregen's [DmxSimple](https://github.com/PaulStoffregen/DmxSimple).
2. Claude Heintz's
   [LXTeensy3DMX_Library](https://github.com/claudeheintz/LXTeensy3DMX_Library).

---

Copyright (c) 2017-2019 Shawn Silverman
