# Changelog

This document details the changes between each release.

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
  transmission: `SIPSenderASync` and `SIPSenderSync`.

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

Copyright (c) 2017-2018 Shawn Silverman
