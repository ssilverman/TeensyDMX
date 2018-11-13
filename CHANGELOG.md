# Changelog

This document details the changes between each release.

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
