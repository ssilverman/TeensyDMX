/*
 * Demonstration program that implements a USB Pro widget.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2019-2020 Shawn Silverman
 */

// C++ includes
#include <algorithm>
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

// Possible states of the DMX line.
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

// Error types passed to the handleError function, for received messages.
enum class Errors {
  kBadLength,
  kBadValue,
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

constexpr uint8_t kStartByte = 0x7E;
constexpr uint8_t kEndByte   = 0xE7;

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

// Firmware version.
//
// This example is v1.44.
constexpr uint16_t kFirmwareVersion = 0x012C;  // MSB=1 means supports all
                                               // except RDM?

// Serial number, MSB first.
constexpr uint8_t kSerialNumber[4]{0x01, 0x02, 0x03, 0x04};

// ESTA manufacturer ID
// https://tsp.esta.org/tsp/working_groups/CP/mfctrIDs.php
constexpr bool kHasManufacturerID = true;
constexpr uint16_t kManufacturerID = 0x7FF0;  // Prototype use
constexpr char kManufacturerName[] = "Manufacturer Nom";

// Device ID and name
constexpr bool kHasDeviceID = true;
constexpr uint16_t kDeviceID = 0x0001;
constexpr char kDeviceName[] = "USB Pro Widget (TeensyDMX demo)";

// DMX serial port.
HardwareSerial &kDMXSerial = Serial3;

// ---------------------------------------------------------------------------
//  Program variables and main functions
// ---------------------------------------------------------------------------

// Data input stream.
Stream &stream = Serial;

// Parsing and response
ParseStates parseState = ParseStates::kStart;
elapsedMillis lastReadTimer{0};

// Received message.
Message recvMsg;

// Message buffer for receiving DMX data. This will be copied into the send
// message buffer in the main loop.
//
// These are marked volatile because they're accessed asynchronously from an
// interrupt and we don't want the compiler to optimize anything improperly.
volatile int recvDMXMsgLen;
volatile uint8_t recvDMXMsgBuf[5 + 1 + 513]{0};  // Largest possible message has
                                                 // a complete DMX packet plus 1

// Send message buffer.
uint8_t msgBuf[5 + 1 + 513]{0};  // Largest possible message has
                                 // a complete DMX packet plus 1

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
  // Handle any received DMX data in addition to processing the serial stream
  processReceivedData();
  processStreamIn();

  static elapsedMillis blinkTimer{0};
  static bool ledState = false;

  // LED blinking

  if (dmxState == DMXStates::kRx) {
    bool timedOut = millis() - dmxRx.lastPacketTimestamp() > kRxTimeout;
    if (timedOut) {
      if (ledState) {
        digitalWriteFast(LED_BUILTIN, LOW);
        ledState = false;
      }
      return;
    }
  }

  // Flash the light at a rate depending on the current state
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

// Handles an error with a received message.
// Implement this to perform some custom behaviour.
//
// For example, you could play a beep or light an LED.
void handleError(Labels msgLabel, Errors err) {
  // Implement me
}

// Processes any asynchronously received DMX data and sends it to the host.
void processReceivedData() {
  int recvLen = 0;

  __disable_irq();
  if (recvDMXMsgLen > 0) {
    std::copy_n(recvDMXMsgBuf, recvDMXMsgLen, msgBuf);
    recvLen = recvDMXMsgLen;
    recvDMXMsgLen = 0;
  }
  __enable_irq();

  if (recvLen > 0) {
    stream.write(msgBuf, recvLen);
  }
}

// Sends a DMX message to the host. The data is sent in the main loop.
//
// This is called from the DMX receive handler.
void sendDMXToHost(const uint8_t *buf, int len) {
  std::copy_n(buf, len, &recvDMXMsgBuf[5]);
  recvDMXMsgBuf[0] = kStartByte;
  recvDMXMsgBuf[1] = static_cast<uint8_t>(Labels::kReceivedDMX);
  recvDMXMsgBuf[2] = static_cast<uint8_t>(len + 1);
  recvDMXMsgBuf[3] = static_cast<uint8_t>(static_cast<uint16_t>(len + 1) >> 8);
  recvDMXMsgBuf[4] = 0;  // No queue overflow nor overrun
  recvDMXMsgBuf[5 + len] = kEndByte;

  recvDMXMsgLen = 5 + len + 1;
}

// Sends a DMX change message to the host. The data is sent in the main loop.
//
// The 'block' is which block of 8 bytes.
//
// This is called from the DMX receive handler.
void sendDMXChangeToHost(int block, const uint8_t changeBits[5],
                         const uint8_t *data, int dataLen) {
  recvDMXMsgBuf[0] = kStartByte;
  recvDMXMsgBuf[1] = static_cast<uint8_t>(Labels::kReceivedDMXChange);
  uint16_t len = static_cast<uint16_t>(1 + 5 + dataLen);
  recvDMXMsgBuf[2] = static_cast<uint8_t>(len);
  recvDMXMsgBuf[3] = static_cast<uint8_t>(len >> 8);
  recvDMXMsgBuf[4] = static_cast<uint8_t>(block);
  recvDMXMsgBuf[10 + dataLen] = kEndByte;

  std::copy_n(changeBits, 5, &recvDMXMsgBuf[5]);
  std::copy_n(data, dataLen, &recvDMXMsgBuf[10]);

  recvDMXMsgLen = 10 + dataLen + 1;
}

// Handles a received message from the host.
void handleMessage(const Message &msg) {
  switch (msg.label) {
    case Labels::kGetParams: {
      if (msg.dataEnd != 2) {
        handleError(msg.label, Errors::kBadLength);
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
        refreshRate = 0.0f;  // Why doesn't the spec allow zero? Send it anyway
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
        handleError(msg.label, Errors::kBadLength);
        break;
      }
      // Ignore user configuration size

      if ((msg.data[2] < 9 || 127 < msg.data[2]) ||
          (msg.data[3] < 1 || 127 < msg.data[3]) ||
          (msg.data[4] > 40)) {
        handleError(msg.label, Errors::kBadValue);
        break;
      }

      // BREAK and MAB times
      uint32_t val = msg.data[2];
      dmxTx.setBreakTime((val * 1067) / 100);  // Floor
      val = msg.data[3];
      dmxTx.setMABTime((val * 1067) / 100);  // Floor

      val = msg.data[4];
      if (val == 0) {
        dmxTx.setRefreshRate(std::numeric_limits<float>::infinity());
      } else {
        dmxTx.setRefreshRate(val);
      }

      break;
    }

    case Labels::kSendDMX: {
      if (msg.dataEnd < 1 || 513 < msg.dataEnd) {
        handleError(msg.label, Errors::kBadLength);
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
      if (msg.dataEnd != 1) {
        handleError(msg.label, Errors::kBadLength);
        break;
      }
      if (msg.data[0] > 1) {
        handleError(msg.label, Errors::kBadValue);
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
        handleError(msg.label, Errors::kBadLength);
        break;
      }
      msgBuf[0] = kStartByte;
      msgBuf[1] = static_cast<uint8_t>(msg.label);
      msgBuf[2] = 4;
      msgBuf[3] = 0;
      msgBuf[4] = kSerialNumber[3];
      msgBuf[5] = kSerialNumber[2];
      msgBuf[6] = kSerialNumber[1];
      msgBuf[7] = kSerialNumber[0];
      msgBuf[8] = kEndByte;
      stream.write(msgBuf, 9);
      stream.flush();
      break;
    }

    case Labels::kDeviceManufacturer: {
      if (msg.dataEnd != 0) {
        handleError(msg.label, Errors::kBadLength);
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
        handleError(msg.label, Errors::kBadLength);
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

// Parses protocol data from the input stream from the host.
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
        recvMsg.label = static_cast<Labels>(b);
        parseState = ParseStates::kLenLSB;
        break;

      case ParseStates::kLenLSB:
        recvMsg.dataLen = static_cast<uint8_t>(b);
        parseState = ParseStates::kLenMSB;
        break;

      case ParseStates::kLenMSB:
        recvMsg.dataLen |= uint16_t{static_cast<uint8_t>(b)} << 8;
        recvMsg.dataEnd = 0;
        if (recvMsg.dataLen > 0) {
          parseState = ParseStates::kData;
        } else {
          parseState = ParseStates::kEnd;
        }
        break;

      case ParseStates::kData:
        if (recvMsg.dataEnd < sizeof(recvMsg.data)) {
          recvMsg.data[recvMsg.dataEnd++] = static_cast<uint8_t>(b);
        }
        recvMsg.dataLen--;
        if (recvMsg.dataLen == 0) {
          parseState = ParseStates::kEnd;
        }
        break;

      case ParseStates::kEnd:
        if (b == kEndByte) {
          handleMessage(recvMsg);
        }
        parseState = ParseStates::kStart;
        break;

      default:
        parseState = ParseStates::kStart;
        break;
    }
  }  // While there's data

  if (parseState != ParseStates::kStart && lastReadTimer > kReadTimeout) {
    parseState = ParseStates::kStart;
  }
}
