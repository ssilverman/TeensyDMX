<a href="https://www.buymeacoffee.com/ssilverman" title="Donate to this project using Buy Me A Coffee"><img src="https://img.shields.io/badge/buy%20me%20a%20coffee-donate-orange.svg" alt="Buy Me A Coffee donate button"></a>

# Readme for TeensyDMX v4.2.0

This is a full-featured library for receiving and transmitting DMX on Teensy 3,
Teensy LC, and Teensy 4. It follows the
[ANSI E1.11 DMX512-A specification](https://tsp.esta.org/tsp/documents/published_docs.php).

## Table of contents

1. [Features](#features)
   1. [Receiver timing limitations and RX line monitoring](#receiver-timing-limitations-and-rx-line-monitoring)
2. [The TODO list](#the-todo-list)
3. [How to use](#how-to-use)
   1. [Examples](#examples)
   2. [Synchronous vs. asynchronous operation](#synchronous-vs-asynchronous-operation)
4. [DMX receive](#dmx-receive)
   1. [Code example](#code-example)
   2. [Retrieving 16-bit values](#retrieving-16-bit-values)
   3. [Error counts and disconnection](#error-counts-and-disconnection)
      1. [The truth about connection detection](#the-truth-about-connection-detection)
      2. [Keeping short packets](#keeping-short-packets)
   4. [Packet statistics](#packet-statistics)
   5. [Error statistics](#error-statistics)
   6. [Synchronous operation by using custom responders](#synchronous-operation-by-using-custom-responders)
      1. [Responding](#responding)
5. [DMX transmit](#dmx-transmit)
   1. [Code example](#code-example-1)
   2. [Packet size](#packet-size)
   3. [Transmission rate](#transmission-rate)
   4. [Synchronous operation by pausing and resuming](#synchronous-operation-by-pausing-and-resuming)
   5. [Choosing BREAK and MAB times](#choosing-break-and-mab-times)
      1. [Specific BREAK/MAB times](#specific-breakmab-times)
         1. [A note on BREAK timing](#a-note-on-break-timing)
         2. [A note on MAB timing](#a-note-on-mab-timing)
      2. [BREAK/MAB times using serial parameters](#breakmab-times-using-serial-parameters)
   6. [Error handling in the API](#error-handling-in-the-api)
6. [Technical notes](#technical-notes)
   1. [Simultaneous transmit and receive](#simultaneous-transmit-and-receive)
   2. [Transmission rate](#transmission-rate)
   3. [Transmit/receive enable pins](#transmitreceive-enable-pins)
   4. [Thread safety](#thread-safety)
   5. [Dynamic memory allocation failures](#dynamic-memory-allocation-failures)
   6. [Hardware connection](#hardware-connection)
   7. [`Receiver` and driving the TX pin](#receiver-and-driving-the-tx-pin)
7. [Code style](#code-style)
8. [References](#references)
9. [Acknowledgements](#acknowledgements)

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
6. The transmitter BREAK and MAB times can be specified (where the actual MAB
   time is something slightly larger than requested).
7. The receiver checks for timeouts according to the DMX specification. It
   knows of the concept of being disconnected from a DMX transmitter when
   timeouts or bad BREAKs are encountered in the data stream.
8. Packet and error statistics are available. These can be used to detect
   protocol problems, including timeouts, framing errors and bad BREAKs, short
   packets (those less than 1196us), and long packets (those that exceed
   513 bytes).
9. The receiver can be used synchronously through the use of the `Responder`
   API. Alternate start codes can not only be handled, for example, for Text
   Packets or System Information Packets (SIP), but responses can be sent back
   to the transmitter, for example for RDM.
10. Functions for handling 16-bit data.

### Receiver timing limitations and RX line monitoring

When the RX line is not being monitored, there are limitations in the handling
of received DMX frame timing. For BREAK and Mark after BREAK (MAB) times, only
the following cases are checked and not accepted as a valid DMX frame start:

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

This limitation does not exist if the RX line is monitored. To monitor the line,
connect it to a digital I/O-capable pin and call `setRXWatchPin` with the pin
number. The pin cannot be the same as the RX pin.

## The TODO list

These are either in the works or ideas for subsequent versions:

1. Asynchronous responder data. Currently, the data is sent synchronously inside
   the UART ISR where responders process the packet.
2. Better MAB transmit timing, perhaps by somehow synchronizing with the baud
   rate clock.

## How to use

The classes you'll need are in the `qindesign::teensydmx` namespace: `Receiver`
and `Sender`.

All class documentation can be found in `src/TeensyDMX.h`.

### Examples

`Receiver` examples:
* `BasicReceive`: A basic receive example
* `Flasher`: Change the flash speed of the board LED based on DMX input

`Sender` examples:
* `BasicSend`: A basic send example
* `Chaser`: Chases values across all channels
* `SendADC`: Sends the value from an ADC over a DMX channel
* `SendTestPackets`: Sends test packets (start code 55h)

Examples that show how to utilize synchronous and asynchronous transmission:
* `SIPSenderAsync`: Sends SIP packets asynchronously
* `SIPSenderSync`: Sends SIP packets synchronously

Examples that show how to use a synchronous packet handler in a receiver:
* `SIPHandler`: Understands System Information Packets (SIP) (start code CFh)
* `TextPacketHandler`: Understands text packets (start codes 17h and 90h)

A more complex example showing how to behave as a DMX USB Pro Widget is
in `USBProWidget`.

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

### Retrieving 16-bit values

Retrieving 16-bit values is easy with one available 16-bit function:
1. `get16Bit` for single words.

This works the same as the 8-bit `get` function, but uses the `uint16_t`
type instead.

### Error counts and disconnection

The DMX receiver keeps track of three types of errors:

1. Packet timeouts.
2. Framing errors, including bad BREAKs.
3. Short packets, i.e. those packets that occupy less than 1196us.

The counts can be retrieved via `errorStats()`. This returns an `ErrorStats`
object containing each of the error counts. These metrics are reset to zero when
the receiver is started or restarted.

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

In actual fact, the connection detection only works for most of the cases where
timeouts occur or when bad BREAKs happen. The one thing it can't do is detect
when a BREAK condition occurs permanently, and, if none of the periodic
interrupt timers (PIT) are available, when an IDLE condition occurs permanently.
Unless more data comes in, no further UART interrupts will be generated, so it
is up to the user's code to detect this case.

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

  if (lastPacketTimer < kTimeout) {
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
  if (millis() - dmxRx.lastPacketTimestamp() >= kTimeout) {
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

#### Keeping short packets

_Short packets_ are nothing more than valid-looking packets that span a duration
less than 1196us. The default is to discard them because they may indicate
line noise.

It is possible to keep the data from these packets by enabling the "Keep Short
Packets" feature using the `setKeepShortPackets` function. If these are kept
then the `PacketStats::isShort` variable will indicate whether the associated
packet is a _short packet_.

Recall that the `readPacket` function can atomically retrieve packet statistics
associated with the packet data. If `packetStats()` is used instead, then
there's no guarantee that the values will be associated with the most recent or
next packet data retrieval calls.

Note that there is also an `isKeepShortPackets` function that can be polled for
the current state of this feature.

### Packet statistics

Packet statistics are tracked and the latest can be retrieved from a
`PacketStats` object returned by `packetStats()`. Included are these variables:

1. `size`: The latest received packet size.
2. `isShort`: Whether the last packet was a _short packet_, a packet having a
   duration less than 1196us. Keeping these packets can be enabled with the
   `setKeepShortPackets` function.
3. `timestamp`: The timestamp of the last packet received, in milliseconds. This
   is set to the time the packet was recognized as a packet and not the end of
   the last stop bit.
4. `breakPlusMABTime`: The sum of the BREAK and MAB times, in microseconds. It's
   not possible to determine where the BREAK ends and the MAB starts without
   using another pin to watch the RX line. To set up a connected pin, see
   `setRXWatchPin` and the section above on
   [RX line monitoring](#receiver-timing-limitations-and-rx-line-monitoring).
5. `breakToBreakTime`: The latest measured BREAK-to-BREAK time, in microseconds.
   Note that this is not collected at the same time as the other variables and
   only represents the last known duration. This will be out of sync with the
   rest of the values in the presence of packet errors.
6. `frameTimestamp`: The BREAK start timestamp, in microseconds.
7. `packetTime`: The duration of the last packet, in microseconds, measured from
   BREAK start to the end of the last slot.
8. `breakTime`: The packet's BREAK time, set if RX line monitoring is enabled.
9. `mabTime`: The packet's MAB time, set if RX line monitoring is enabled.

If the RX line is not being monitored, then the BREAK and MAB times will be set
to zero.

There is also an optional parameter in `readPacket`, a `PacketStats*`, that
enables retrieval of this data atomically with the packet data.

These metrics are reset to zero when the receiver is started or restarted.

### Error statistics

Error statistics are tracked and the latest can be retrieved from an
`ErrorStats` object returned by `errorStats()`. Included are these counts:

1. `packetTimeoutCount`: Packet timeouts.
2. `framingErrorCount`: Framing error count, including BREAKs that were
   too short.
3. `shortPacketCount`: Packets that were too short.
4. `longPacketCount`: Packets that were too long.

### Synchronous operation by using custom responders

There is the ability to notify specific instances of `Responder` when packets
having specific start codes arrive. To implement the simplest form, simply
extend `Responder`, override the `receivePacket` function, and attach an
instance to one or more start codes using `Receiver::setResponder`.
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
  dmxRx.setResponder(TextHandler::kStartCode, &textHandler)
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

### Setting 16-bit values

Setting 16-bit values is easy with two available 16-bit functions:
1. `set16Bit` for single words, and
2. `set16Bit` for an array of words.

These work the same as the 8-bit `set` functions, but use the `uint16_t`
type instead.

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

### Choosing BREAK and MAB times

The BREAK and MAB times can be specified in two ways:
1. By specifying specific times using the `setBreakTime` and `setMABTime`
   functions, or
2. By specifying serial port parameters and relying on multiples of bit times,
   using the `setBreakSerialParams` function.

The default times are set, in both cases, to approximately 180us for the BREAK
and approximately 20us for the MAB.

The mode is switched between these two options using the
`setBreakUseTimerNotSerial` function. The default is to use
serial parameters.

#### Specific BREAK/MAB times

This is the first way to generate these times.

The BREAK will be transmitted with a duration reasonably close to the specified
value, but the actual MAB time may be larger than requested. This has to do with
how the UARTs on the chip work.

This feature uses one of the _PIT_ timers via the `IntervalTimer` API, but if
none are available, then the transmitter will fall back on using the baud rate
generator with the specified serial port parameters.

##### A note on BREAK timing

The BREAK timing is pretty accurate, but there's some inaccuracy due to the
default `IntervalTimer` API. It doesn't provide a way to execute an action (in
this case, starting a BREAK) just before the timer starts. Instead, the timer
will have already started and some time elapsed before the BREAK can start.

Efforts have been made to make the BREAK time be at least the amount requested,
but it likely won't be exactly the requested duration.

##### A note on MAB timing

The MAB time may be longer than requested. The reason is that it's not possible
to immediately start sending a character using the UART on the Teensy chips. It
seems that the best resolution one can get is "somewhere within one or two
bit times".

To illustrate:
BREAK ->(immediate) MAB ->(not immediate) First packet character

For example, if a timer aims for a 23us MAB, and then a character is sent to the
UART after the timer expires, then that character might start after 25us. If the
timer is adjusted so that it expires 2us earlier, then the character might still
appear too late, still after 25us, for example. A further adjustment of 2us, for
a timer duration of 19us, might result in the character appearing after 21us,
earlier than requested.

When a character starts, it appears that the UART performs its functions
asynchronously, and so it is not possible to achieve exact timing. The code
adjusts the MAB time under the covers to achieve times that are as close to the
requested time as possible without going under, but it is often not possible to
be more precise than "within one or two bit times", depending on the processor.

#### BREAK/MAB times using serial parameters

This is the second way to generate these times.

The times are restricted to having a BREAK:MAB ratio of 9:2, 10:2, 9:1, 10:1,
9:3, 8:2, or 11:1. These correspond to the UART formats, 8N2, 8E2, 8N1, 8E1,
8O2, 7O1, and 9E1. For additional information on this subject, see the
[BREAK Timing in DMX512-A](extras/break-timing.md) note.

The BREAK time will be fairly accurate, but the MAB time will be a little longer
than expected, by up to at least several bit times.

This is a less flexible way to specify the BREAK and MAB times. The baud rate is
changed just for these and then changed back to 250kbaud (8N2) for the packet
data. The act of changing the baud rate can introduce a delay, somewhere up to
one full character.

This mode is also used as a fallback if the system doesn't have the
timers available.

### Error handling in the API

Several `Sender` functions that return a `bool` indicate whether an operation
was successful. Prior versions of the library did nothing with, or silently
ignored, bad arguments.

The status-returning functions are as follows:
1. `setPacketSize`,
2. Both `set` functions,
3. Both `set16Bit` functions,
4. `setRefreshRate`, and
5. Both `resumeFor` functions.

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

### `Receiver` and driving the TX pin

By default, when a `Receiver` is used, the TX pin for its serial port is
enabled. This means that the line is driven. This default state was chosen
because custom responders may wish to transmit information. Additionally, the
caller does not need to remember to enable the transmitter when adding a
custom responder.

If it is known that there are no responders or that no responders will send
data, then the TX pin can be disabled so that the UART hardware isn't driving
the line. The `Receiver::setTXEnabled` function can be called either during
operation or outside a `begin` and `end` pair.

Be aware that if the receiver is currently in operation, enabling the
transmitter will cause an 11-bit idle character to be queued for output and the
line to remain high after that.

## Code style

Code style for this project mostly follows the
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

Other conventions are adopted from Bjarne Stroustrup's and Herb Sutter's
[C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md).

## References

1. [BREAK Timing in DMX512-A](extras/break-timing.md)

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

## Acknowledgements

1. It's one thing to have chip documentation, but it's an entirely new beast
   when there exists code that already implements all the hard parts. Thank you
   to Paul Stoffregen for all the Teensy source code. This helped immensely when
   developing this library.
   See: https://github.com/PaulStoffregen/cores

---

Copyright (c) 2017-2020 Shawn Silverman
