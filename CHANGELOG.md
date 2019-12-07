# Changelog for the TeensyDMX Library

This document details the changes between each release.

## [4.0.0-alpha.4]

### Added
* BREAK and MAB times can be determined separately in the receiver, in addition
  to their sum, if the RX pin is monitored by connecting it to a digital
  I/O-capable pin and calling the new `Receiver::setRXWatchPin` function. This
  removes the limitations the receiver has when checking for packets having
  invalid timing, but only when using this feature.
* The BREAK and MAB times in the transmitter can now be specified more
  accurately via new `Sender::setBreakTime` and `Sender::setMABTime` functions.
  The actual times will be close but the MAB may be a little longer
  than specified.

### Fixed
* MAB and BREAK-plus-MAB time calculations have been fixed when bytes are
  received all at once from a FIFO.
* Data is now discarded at all places a framing error is encountered.

## [4.0.0-alpha.3]

### Added
* Added a way to set and retrieve 16-bit values using new `Receiver::get16Bit`
  and `Sender::set16Bit` functions.
* Added the ability to enable and disable the transmitter in the receiver via
  a new `Receiver::setTXEnabled` function. This is useful when it is known that
  the receiver will be receive-only and it is not desired to drive the TX line.
* New `Receiver::packetStats()` and `Receiver::errorStats()` functions with
  associated `Receiver::PacketStats` and `Receiver::ErrorStats` classes for
  examining the latest packet statistics and error counts.

### Changed
* In the sender, changed how the baud rate is set when switching between the
  BREAK baud rate and slots baud rate. The UART/LPUART parameters are set
  directly instead of calling `begin` on the serial port object. This avoids
  setting values where we don't need to and has the effect of shaving some
  microseconds off the Mark after BREAK (MAB) so that it is closer to the
  desired duration.
* `Receiver::begin()` now resets all the packet statistics.
* Error counts and packet statistics functions have been replaced with functions
  to retrieve `Receiver::ErrorStats` and `Receiver::PacketStats`, respectively:
  `Receiver::errorStats()` and `Receiver::packetStats()`.
* `Receiver::readPacket` has an additional but optional parameter that provides
  a place to store the packet statistics, a `PacketStats*`, allowing atomic
  retrieval with the packet data.

### Removed
* `Receiver::packetTimeoutCount()`, `Receiver::framingErrorCount()`, and
  `Receiver::shortPacketCount()` were replaced by `Receiver::errorStats()` and
  its associated `Receiver::ErrorStats` class.

## [4.0.0-alpha.2]

### Fixed
* This release should _really_ fix the test-first-stop-bit problem. It turns out
  that Teensy 3.5, 3.6, and LC all support a 2-stop-bit mode, and so it is never
  appropriate to test bit R8 for these platforms. Bit R8 is now only tested as
  the first stop bit if the board is not one of these types.

## [4.0.0-alpha.1]

### Fixed
* Teensy LC wasn't able to receive DMX data because the framing check that
  examined the first stop bit in each byte was always returning false. For some
  reason, the R8 bit in UARTx_C3, which functions as the first stop bit, is
  always zero. The check was changed for the Teensy LC to always return true.

## [4.0.0-alpha]

### Added
* Added a note to the `BasicSend` example that the transmit-enable pin may not
  be needed. The pin's existence was causing some confusion.

### Changed
* The `main.cpp` example program now uses a smart pointer for the current
  sketch, `currSketch`.
* Responders are now passed as smart pointers instead of raw pointers to
  `Receiver::setResponder`.
* Prep-work for Teensy 4 support. The library compiles but does not yet work on
  a Teensy 4.

### Fixed
* When a frame error was being recorded due to the first stop bit not being
  high, the input UART buffer was not being flushed. The buffer is now flushed
  when this happens. There were reports of DMX receive freezing the device; this
  should fix the problem.

## [3.2.2]

This version is in a branch called `fix-dmx-receive-in-v3.2.0`.

### Fixed
* This release should _really_ fix the test-first-stop-bit problem. It turns out
  that Teensy 3.5, 3.6, and LC all support a 2-stop-bit mode, and so it is never
  appropriate to test bit R8 for these platforms. Bit R8 is now only tested as
  the first stop bit if the board is not one of these types.

## [3.2.1]

This version is in a branch called `fix-dmx-receive-in-v3.2.0`.

### Fixed
* When a frame error was being recorded due to the first stop bit not being
  high, the input UART buffer was not being flushed. The buffer is now flushed
  when this happens. There were reports of DMX receive freezing the device; this
  should fix the problem. [The fix does not work for Teensy LC.]
* Teensy LC wasn't able to receive DMX data because the framing check that
  examined the first stop bit in each byte was always returning false. For some
  reason, the R8 bit in UARTx_C3, which functions as the first stop bit, is
  always zero. The check was changed for the Teensy LC to always return true.

## [3.2.0]

### Added
* New `Responder::preDataDelay()` method that returns the time in microseconds
  to wait after the pre-BREAK or no-pre-BREAK delay times (`preBreakDelay()` or
  `preNoBreakDelay()`), and after the transmitter is turned on.

### Changed
* If a `Responder` is being used, `Receiver` now delays for `preNoBreakDelay()`
  _before_ enabling the transmitter. After delaying for `preBreakDelay()` or
  `preNoBreakDelay()`, the transmitter is enabled, and then `Receiver` delays
  for `preDataDelay()` microseconds.
* Improved framing error detection: the case where there's a short BREAK
  followed by a long MAB is now detected.

### Fixed
* `Receiver` now checks that the first stop bit of a received byte is logic 1.
  On the Kinetis chips, the first stop bit of two stop bits is implemented as
  the 9th bit of the 9-bit mode.

## [3.1.1]

### Added
* New section in the README that describes how to connect DMX via a transceiver
  to a Teensy. See the new "Hardware connection" section under
  "Technical notes".

### Changed
* Updated library description.

### Fixed
* Added some missing literals to `keywords.txt`.

## [3.1.0]

### Added
* New `Sender::clear()` method that sets all the channels to zero.
* New `Sender::breakTime()` and `Sender::mabTime()` methods that return the
  BREAK and MARK after BREAK (MAB) times in microseconds, respectively.
* Added a section to the README that describes when connection detection doesn't
  work and what to do about it.

### Changed
* Made the constructors `explicit` because they're single-argument.
* There's now less protected data in the base `TeensyDMX` class.
* Made `Sender::set` behave atomically.
* Class documentation updates.
* The packet count is now reset to zero when the receiver or transmitter
  is either started for the first time or restarted.

### Fixed
* Interrupts are now correctly not enabled if an instance of `Sender` or
  `Receiver` is stopped, i.e. never started or had its `end()` method called.
* `Receiver::setConnected` wasn't actually changing the internal connected
  state. The state is now updated correctly so that `Receiver::connected()`
  returns the correct value.

## [3.0.0]

### Added
* Two new examples, `BasicSend` and `BasicReceive`.

### Changed
* Updated `keywords.txt`.
* The internal array that keeps track of responders in `Receiver` is now
  dynamically allocated; this saves about 1k of memory.
* Changed `Receiver::addResponder` to `Receiver::setResponder`; also changed
  its behaviour so that setting a responder to `nullptr` removes any responder
  for the given start code.
* Changed the behaviour of `Sender::pause` so that the caller must wait until
  transmission is complete before setting any values with one of the `set`
  functions. Instances of `Sender` are smaller by about 0.5k because there's no
  need to hold an internal "paused" array.

### Fixed
* It was technically possible to overwrite the contents of a packet in the
  process of being sent if pause->set->resume was done quickly. Requiring any
  `set` call to wait until transmission is complete removes having to manage
  the concurrency.
* On small systems such as the Teensy LC, dynamic memory allocation in
  `Receiver::setResponder` may fail. This was fixed and the documentation in
  `TeensyDMX.h` and the README was updated. The caller can check the `ENOMEM`
  condition in `errno` to detect this condition.
* The behaviour of `Receiver::get` was changed so that it always returns the
  last received value for a channel. It is no longer affected by a previous call
  to `Receiver::readPacket` for the same received packet, where the packet was
  reset to empty. This change makes the function behave closer to expected.

## [3.0.0-beta]

### Added
* New document describing BREAK timing in relation to serial ports.

### Changed
* Unified the RX status and error ISRs because framing errors and data should
  have the same priority.
* Refactored all the synchronous UART TX routines in `Receiver` to common
  \#defines.
* Minor documentation updates.
* `setRXNotTX` concept renamed to `setTXNotRX`, in `Receiver`.
* Small updates to the examples.
* Some concurrency improvements.

### Fixed
* ISR-triggering status bits are now cleared properly for Teensy LC.

## [3.0.0-alpha]

### Added
* Short packet detection. These are packets that are smaller than 1196us.
* Ability to get the current transmitted packet size with
  `Sender::packetSize()`.
* The concept of being _connected_ in `Receiver`. There is a polling function,
  `connected()`, and also a way to register a callback for when the state
  changes, via `onConnectChange`.
* Associated with the concept of being _connected_ in the receiver, and in
  addition to the improved BREAK plus MAB timing checks, `Receiver` now also
  checks for inter-slot and Mark before Break (MBB) IDLE times that are too
  long. Detecting these conditions will register as a change to the
  _connected_ state.
* Synchronous operation in `Receiver`. Using the new `Responder` class, it's
  now possible to intercept, and even respond to, each packet as it comes in.
* Two examples that use the new `Responder` feature: `SIPHandler`
  and `TextPacketHandler`.

### Changed
* Renamed:
  * `Sender::getRefreshRate()` to `Sender::refreshRate()`.
  * `Sender::getResumedRemaining()` to `Sender::resumedRemaining()`.
* The licence changed from BSD-3-Clause to BSD-3-Clause-Clear.

### Fixed
* Documentation fixes.
* Improved packet timeout detection. It now detects when the BREAK plus MAB
  duration is too short, but not when they are individually too short.
* Added a define around the contents of `main.cpp` so that the project can
  be used in an Arduino environment. This prevents the `setup()` and `loop()`
  definitions in `main.cpp` from interfering with the Arduino program.
* `Receiver::get` now respects the current received packet size and behaves
  more similarly to `Receiver::readPacket`.

## [2.5.0]

### Added
* The sender can now be paused and resumed. This allows implementations to
  send packets that must be adjacent to other packets, for example,
  System Information Packets (SIP) (see Annex D5 of ANSI E1.11). Essentially,
  the effect is being able to use the asynchronous transmitter synchronously.

  New `Sender` functions include:
  * `pause()`
  * `isPaused()`
  * `resume()`
  * `resumeFor(int)`
  * `getResumedRemaining()`
  * `isTransmitting()`
  * `onDoneTransmitting`
* Two examples that show how to send SIP packets using synchronous
  transmission: `SIPSenderAsync` and `SIPSenderSync`.

### Changed
* The receiver no longer keeps packet data if it's followed by a framing error
  having non-zero data. Framing errors are used to detect BREAKs and must
  consist of all zeros in order for it to be considered a valid BREAK. This
  choice was made because the condition may indicate corrupt data.

  See: [BREAK timing at the receiver](http://www.rdmprotocol.org/forums/showthread.php?t=1292)
* Reduced the amount of duplicated code in the UART transmit and receive ISRs
  via macros.
* Internally, when `Sender` needs to disable interrupts, only the UART
  interrupts are disabled for the required duration. This change is similar
  to the change made in the previous release for `Receiver::readPacket`.
* Updated `keywords.txt`.
* Now allowing packet sizes less than 25 (i.e. less than 24 channels) in
  `Sender::setPacketSize`.

### Fixed
* UARTs 2 and above (Serial3 and above) were not correctly detecting BREAKs
  because the framing error detection appeared to be overidden by the other
  receive routines. This was fixed by increasing the priority of the framing
  error interrupts to one greater than the priority of the status interrupts.
* Sending via UARTs having FIFOs now works correctly with transmission-complete
  timing. `Sender::completePacket()` is now called at the correct place.

## [2.4.0]

### Added
* A basic _main_ program so that it's easy to compile the project.
* Support for Teensy 3.6's LPUART0 (Serial6).
* Support for Teensy LC.
* The ability to change the transmit refresh rate, via
  `Sender::setRefreshRate`.

### Changed
* Reading from a packet via `Receiver::readPacket` no longer disables
  all interrupts. Only the serial interrupts are disabled.
* The `TeensyDMX` constructor and destructor are now `protected`.
* BREAK/Mark-After-Break timing changed from 108us/12us to 180us/20us. This
  more closely matches the "typical" DMX BREAK timing specified in the
  DMX specification (ANSI E1.11).
* Some function and README documentation updates.
* Moved the `kMaxDMXPacketTime` constant into `Receiver`.

### Fixed
* Added some missing disable-interrupt cases to `Sender::end()`.

## [2.3.2]

### Changed
* Chaser.ino and Flasher.ino examples now properly wait for the serial monitor
  at program start. The baud was also changed to 115200.
* Changed print interval in Flasher.ino to 1000ms.
* Slightly optimized the Flasher.ino wave calculation.

## [2.3.1]

### Added
* Added a CHANGELOG.

### Changed
* Updated the keywords.txt file.

## [2.3.0]

### Fixed
* Better handling for packet boundaries and timeouts.

## [2.2.0]

### Added
* Support for serial ports 4-6.

## [2.1.0]

### Added
* A `lastPacketTimestamp()` function to retrieve the timestamp of the
  most recent packet received.

## [2.0.0]

Initial public release.

---

Copyright (c) 2017-2019 Shawn Silverman
