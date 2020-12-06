/*
 * Demonstration program that implements a USB Pro widget.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2019-2020 Shawn Silverman
 */

// C++ includes
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>

#include <TeensyDMX.h>
#include "ReceiveHandler.h"

namespace teensydmx = ::qindesign::teensydmx;

// ---------------------------------------------------------------------------
//  Types
// ---------------------------------------------------------------------------

// Defines the parse states.
enum class ParseStates {
  kStart,
  kLabel,
  kLenLSB,
  kLenMSB,
  kData,
  kEnd,
};

enum class DMXStates {
  kRx,
  kTx,
};

// Labels for different message types.
enum class Labels : uint8_t {  // Fixed type to avoid undefined behaviour when
                               // casting to Labels from a uint8_t
  kGetParams          = 3,
  kSetParams          = 4,
  kReceivedDMX        = 5,
  kSendDMX            = 6,
  kReceiveDMXOnChange = 8,
  kReceivedDMXChange  = 9,
  kGetSerial          = 10,

  // https://wiki.openlighting.org/index.php/USB_Protocol_Extensions
  kDeviceManufacturer = 77,
  kDeviceName         = 78,
};

// Holds a received message.
struct Message {
  Labels label;
  uint16_t dataLen;  // The declared data length
  uint16_t dataEnd;  // The actual end of the data, may be less than dataLen
  uint8_t data[600];
};

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

// Read timeout.
constexpr uint32_t kReadTimeout = 500;

constexpr uint8_t kStartByte = 0x7e;
constexpr uint8_t kEndByte   = 0xe7;

constexpr uint32_t kRxTimeout = 1000;  // In milliseconds

// ---------------------------------------------------------------------------
//  User-settable parameters
// ---------------------------------------------------------------------------

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Pin for enabling or disabling the transmitter.
constexpr uint8_t kTxPin = 17;

// TX pin states
constexpr uint8_t kTxEnable = HIGH;
constexpr uint8_t kTxDisable = LOW;

constexpr uint16_t kFirmwareVersion = 0x0100;  // MSB=1 means supports all
                                               // except RDM (is this true)?
constexpr uint8_t kSerialNumber[4]{0xff, 0xff, 0xff, 0xff};  // LSB first

// ESTA manufacturer ID
// https://tsp.esta.org/tsp/working_groups/CP/mfctrIDs.php
constexpr bool kHasManufacturerID = true;
constexpr uint16_t kManufacturerID = 0x7ff0;
constexpr char kManufacturerName[] = "Manufacturer Nom";

constexpr bool kHasDeviceID = true;
constexpr uint16_t kDeviceID = 0x0001;
constexpr char kDeviceName[] = "USB Pro Widget (TeensyDMX demo)";

// DMX serial port
HardwareSerial &kDMXSerial = Serial3;

// ---------------------------------------------------------------------------
//  Program variables and main functions
// ---------------------------------------------------------------------------

// Data input stream.
Stream &stream = Serial;

// Parsing and response
ParseStates parseState = ParseStates::kStart;
elapsedMillis lastReadTimer{0};
Message msg;
uint8_t msgBuf[5 + 1 + 513]{0};  // Largest possible message has a
                                 // complete DMX packet plus 1

// DMX
teensydmx::Sender dmxTx{kDMXSerial};
teensydmx::Receiver dmxRx{kDMXSerial};
auto receiveHandler = std::make_shared<ReceiveHandler>(stream);

DMXStates dmxState = DMXStates::kRx;

// Main program setup.
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for serial port initialization
  }

  // Set up the pins
  pinMode(kLEDPin, OUTPUT);
  digitalWriteFast(kLEDPin, LOW);
  pinMode(kTxPin, OUTPUT);

  // Set up DMX
  dmxRx.setResponder(0, receiveHandler);
  dmxTx.setBreakUseTimerNotSerial(true);
  if (dmxState != DMXStates::kTx) {
    digitalWriteFast(kTxPin, kTxDisable);
    dmxRx.begin();
  } else {
    digitalWriteFast(kTxPin, kTxEnable);
    dmxTx.begin();
  }
}

// Main program loop.
void loop() {
  processStreamIn();

  static elapsedMillis blinkTimer{0};
  static bool ledState = false;

  // LED blinking

  if (dmxState != DMXStates::kTx) {
    bool timedOut = millis() - dmxRx.lastPacketTimestamp() > kRxTimeout;
    if (timedOut) {
      if (ledState) {
        digitalWriteFast(LED_BUILTIN, LOW);
        ledState = false;
      }
      return;
    }
  }

  uint32_t rate;
  if (dmxState == DMXStates::kTx) {
    // 2Hz blink
    rate = 2;
  } else {
    // 8Hz blink
    rate = 8;
  }

  if (blinkTimer >= 1000 / rate / 2) {
    blinkTimer = 0;
    ledState = !ledState;
    digitalWriteFast(LED_BUILTIN, ledState ? HIGH : LOW);
  }
}

// Sends a DMX message to the host.
// TODO: Execute on the main thread.
void sendDMX(const uint8_t *buf, int len) {
  // It's possible for buf and msgBuf to be the same, so use memmove
  memmove(&msgBuf[5], buf, len);
  msgBuf[0] = kStartByte;
  msgBuf[1] = static_cast<uint8_t>(Labels::kReceivedDMX);
  msgBuf[2] = static_cast<uint8_t>(len + 1);
  msgBuf[3] = static_cast<uint8_t>(static_cast<uint16_t>(len + 1) >> 8);
  msgBuf[4] = 0;  // No queue overflow nor overrun
  msgBuf[5 + len] = kEndByte;
  stream.write(msgBuf, 5 + len + 1);
  stream.flush();
}

// Sends a DMX change message to the host.
// The 'block' is which block of 8 bytes.
// TODO: Execute on the main thread.
void sendDMXChange(int block, const uint8_t changeBits[5],
                   const uint8_t *data, int dataLen) {
  msgBuf[0] = kStartByte;
  msgBuf[1] = static_cast<uint8_t>(Labels::kReceivedDMXChange);
  uint16_t len = static_cast<uint16_t>(1 + 5 + dataLen);
  msgBuf[2] = static_cast<uint8_t>(len);
  msgBuf[3] = static_cast<uint8_t>(len >> 8);
  msgBuf[4] = static_cast<uint8_t>(block);
  memcpy(&msgBuf[5], changeBits, 5);
  memcpy(&msgBuf[10], data, dataLen);
  msgBuf[10 + dataLen] = kEndByte;
  stream.write(msgBuf, 10 + dataLen + 1);
  stream.flush();
}

// Handles a received message.
void handleMessage(const Message &msg) {
  switch (msg.label) {
    case Labels::kGetParams: {
      if (msg.dataEnd != 2) {
        // Bad message
        break;
      }
      // Ignore the user configuration size and just respond
      msgBuf[0] = kStartByte;
      msgBuf[1] = static_cast<uint8_t>(msg.label);
      msgBuf[2] = 5;
      msgBuf[3] = 0;
      msgBuf[4] = static_cast<uint8_t>(kFirmwareVersion);
      msgBuf[5] = static_cast<uint8_t>(kFirmwareVersion >> 8);
      uint32_t t = (dmxTx.breakTime()*100 + 1066)/1067;  // Ceiling
      if (t < 9) {
        t = 9;
      } else if (t > 127) {
        t = 127;
      }
      msgBuf[6] = static_cast<uint8_t>(t);
      t = (dmxTx.mabTime()*100 + 1066)/1067;  // Ceiling
      if (t < 1) {
        t = 1;
      } else if (t > 127) {
        t = 127;
      }
      msgBuf[7] = static_cast<uint8_t>(t);
      float refreshRate = dmxTx.refreshRate();
      if (refreshRate > 40.0f) {
        refreshRate = 40.0f;  // Why doesn't the spec allow zero?
      } else if (refreshRate < 1.0f) {
        refreshRate = 1.0f;
      }
      msgBuf[8] = static_cast<uint8_t>(refreshRate);
      msgBuf[9] = kEndByte;
      stream.write(msgBuf, 10);
      stream.flush();
      break;
    }

    case Labels::kSetParams: {
      if (msg.dataEnd < 5) {
        // Bad message
        break;
      }
      // Ignore user configuration size

      // BREAK and MAB times
      uint32_t val = msg.data[2];
      if (9 <= val && val <= 127) {
        dmxTx.setBreakTime((val * 1067) / 100);
      }
      val = msg.data[3];
      if (1 <= val && val < 127) {
        dmxTx.setMABTime((val * 1067) / 100);
      }

      val = msg.data[4];
      if (val == 0) {
        dmxTx.setRefreshRate(std::numeric_limits<float>::infinity());
      } else if (1 <= val && val <= 40) {
        dmxTx.setRefreshRate(val);
      }

      break;
    }

    case Labels::kSendDMX: {
      if (msg.dataEnd < 1 || 513 < msg.dataEnd) {
        // Bad message
        break;
      }
      if (dmxState == DMXStates::kRx) {
        dmxRx.end();
      }

      int len = msg.dataEnd;
      // Make changing the packet size atomic with setting the new data
      if (len != dmxTx.packetSize()) {
        dmxTx.pause();
        while (dmxTx.isTransmitting()) {
          yield();
        }
        dmxTx.setPacketSize(len);
        dmxTx.set(0, msg.data, len);
        dmxTx.resume();
      } else {
        dmxTx.set(0, msg.data, len);
      }

      if (dmxState != DMXStates::kTx) {
        digitalWriteFast(kTxPin, kTxEnable);
        dmxTx.begin();
        dmxState = DMXStates::kTx;
      }
      break;
    }

    case Labels::kReceiveDMXOnChange: {
      if (msg.dataEnd != 1 || msg.data[0] > 1) {  // Note: Not a minimum of 25
        // Bad message
        break;
      }
      if (dmxState == DMXStates::kTx) {
        dmxTx.end();
      }

      receiveHandler->setSendOnChangeOnly(msg.data[0] == 1);

      if (dmxState != DMXStates::kRx) {
        digitalWriteFast(kTxPin, kTxDisable);
        dmxRx.begin();
        dmxState = DMXStates::kRx;
      }
      break;
    }

    case Labels::kGetSerial: {
      if (msg.dataEnd != 0) {
        // Bad message
        break;
      }
      msgBuf[0] = kStartByte;
      msgBuf[1] = static_cast<uint8_t>(msg.label);
      msgBuf[2] = 4;
      msgBuf[3] = 0;
      memcpy(&msgBuf[4], kSerialNumber, 4);
      msgBuf[8] = kEndByte;
      stream.write(msgBuf, 9);
      stream.flush();
      break;
    }

    case Labels::kDeviceManufacturer: {
      if (msg.dataEnd != 0) {
        // Bad message
        break;
      }
      if (!kHasManufacturerID) {
        break;
      }
      msgBuf[0] = kStartByte;
      msgBuf[1] = static_cast<uint8_t>(msg.label);
      size_t nameLen = strlen(kManufacturerName);
      if (nameLen > 32) {
        nameLen = 32;
      }
      msgBuf[2] = static_cast<uint8_t>(nameLen + 2);
      msgBuf[3] = static_cast<uint8_t>(static_cast<uint16_t>(nameLen + 2) >> 8);
      msgBuf[4] = static_cast<uint8_t>(kManufacturerID);
      msgBuf[5] = static_cast<uint8_t>(kManufacturerID >> 8);
      memcpy(&msgBuf[6], kManufacturerName, nameLen);
      msgBuf[6 + nameLen] = kEndByte;
      stream.write(msgBuf, 6 + nameLen + 1);
      stream.flush();
      break;
    }

    case Labels::kDeviceName: {
      if (msg.dataEnd != 0) {
        // Bad message
        break;
      }
      if (!kHasDeviceID) {
        break;
      }
      msgBuf[0] = kStartByte;
      msgBuf[1] = static_cast<uint8_t>(msg.label);
      size_t nameLen = strlen(kDeviceName);
      if (nameLen > 32) {
        nameLen = 32;
      }
      msgBuf[2] = static_cast<uint8_t>(nameLen + 2);
      msgBuf[3] = static_cast<uint8_t>(static_cast<uint16_t>(nameLen + 2) >> 8);
      msgBuf[4] = static_cast<uint8_t>(kDeviceID);
      msgBuf[5] = static_cast<uint8_t>(kDeviceID >> 8);
      memcpy(&msgBuf[6], kDeviceName, nameLen);
      msgBuf[6 + nameLen] = kEndByte;
      stream.write(msgBuf, 6 + nameLen + 1);
      stream.flush();
      break;
    }

    default:
      break;
  }
}

// Parses protocol data from the input stream.
void processStreamIn() {
  while (stream.available() > 0) {
    int b = stream.read();
    if (b < 0) {
      break;
    }
    lastReadTimer = 0;

    switch (parseState) {
      case ParseStates::kStart:
        if (b == kStartByte) {
          parseState = ParseStates::kLabel;
        }
        break;

      case ParseStates::kLabel:
        msg.label = static_cast<Labels>(b);
        parseState = ParseStates::kLenLSB;
        break;

      case ParseStates::kLenLSB:
        msg.dataLen = static_cast<uint8_t>(b);
        parseState = ParseStates::kLenMSB;
        break;

      case ParseStates::kLenMSB:
        msg.dataLen |= uint16_t{static_cast<uint8_t>(b)} << 8;
        msg.dataEnd = 0;
        if (msg.dataLen > 0) {
          parseState = ParseStates::kData;
        } else {
          parseState = ParseStates::kEnd;
        }
        break;

      case ParseStates::kData:
        if (msg.dataEnd < sizeof(msg.data)) {
          msg.data[msg.dataEnd++] = static_cast<uint8_t>(b);
        }
        msg.dataLen--;
        if (msg.dataLen == 0) {
          parseState = ParseStates::kEnd;
        }
        break;

      case ParseStates::kEnd:
        if (b == kEndByte) {
          handleMessage(msg);
        }
        parseState = ParseStates::kStart;
        break;

      default:
        parseState = ParseStates::kStart;
        break;
    }
  }

  if (parseState != ParseStates::kStart && lastReadTimer > kReadTimeout) {
    parseState = ParseStates::kStart;
  }
}
