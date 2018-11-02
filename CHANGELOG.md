# Changelog

This document details the changes between each release.

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
