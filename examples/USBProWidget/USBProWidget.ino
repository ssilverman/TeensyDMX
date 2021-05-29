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
enum class Labels : uint8_t {  // Fixed type to avoid undefined
                               // behaviour when casting to Labels
                               // from a uint8_t
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

// Error types passed to the handleError function,
// for received messages.
enum class Errors {
  kBadLength,
  kBadValue,
};

// Holds a received message.
struct Message {
  Labels label;
  uint16_t dataLen;  // The declared data length
  uint16_t dataEnd;  // The actual end of the data, may be < dataLen
  uint8_t data[600];
};

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------

// Read timeout.
constexpr uint32_t kReadTimeout = 500;

// Message constants
constexpr uint8_t kStartByte = 0x7E;
constexpr uint8_t kEndByte   = 0xE7;

constexpr uint32_t kRxTimeout = 1000;  // In milliseconds

// Start codes
constexpr uint8_t SC_NULL = 0x00;

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
constexpr uint16_t kFirmwareVersion = 0x012C;

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

// Buffer for receiving DMX data. This will be processed in
// the main loop.
//
// These are marked volatile because they're accessed asynchronously
// from an interrupt and we don't want the compiler to optimize
// anything improperly.
volatile int recvDMXLen = 0;
volatile uint8_t recvDMXBuf[513]{0};

// Send message buffer.
uint8_t msgBuf[5 + 1 + 513]{0};  // Largest possible message has
                                 // a complete DMX packet plus 1

// DMX
teensydmx::Sender dmxTx{kDMXSerial};
teensydmx::Receiver dmxRx{kDMXSerial};
ReceiveHandler receiveHandler{};
DMXStates dmxState = DMXStates::kRx;

// Track DMX packet changes
bool sendOnChangeOnly = false;
uint8_t lastDMXBuf[513]{0};
int lastDMXLen = 0;
uint8_t changeMsg[5 + 46];  // Contents of a "change of state" message

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
  dmxRx.setResponder(SC_NULL, &receiveHandler);
  dmxTx.setBreakUseTimerNotSerial(true);
  if (dmxState == DMXStates::kTx) {
    startTx();
  } else {
    startRx();
  }
}

// Main program loop.
void loop() {
  // Handle any received DMX data in addition to processing
  // the serial stream
  processReceivedData();
  processStreamIn();

  static elapsedMillis blinkTimer{0};
  static bool ledState = false;

  // LED blinking

  if (dmxState != DMXStates::kTx) {
    bool timedOut = millis() - dmxRx.lastPacketTimestamp() >= kRxTimeout;
    if (timedOut) {
      if (ledState) {
        digitalWriteFast(kLEDPin, LOW);
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
    digitalWriteFast(kLEDPin, ledState ? HIGH : LOW);
  }
}

// Handles an error with a received message.
// Implement this to perform some custom behaviour.
//
// For example, you could play a beep or light an LED.
void handleError(Labels msgLabel, Errors err) {
  // Implement me
}

// Starts the receiver.
void startRx() {
  digitalWriteFast(kTxPin, kTxDisable);
  dmxRx.begin();
}

// Starts the transmitter.
void startTx() {
  digitalWriteFast(kTxPin, kTxEnable);
  dmxTx.begin();
}

// ---------------------------------------------------------------------------
//  Input processing functions
// ---------------------------------------------------------------------------

// Processes any asynchronously received DMX data and sends it to
// the host.
void processReceivedData() {
  int len = 0;

  // Copy the DMX packet into the message buffer
  __disable_irq();
  if (recvDMXLen > 0) {
    std::copy_n(recvDMXBuf, recvDMXLen, &msgBuf[5]);
    len = recvDMXLen;
    recvDMXLen = 0;
  }
  __enable_irq();

  if (len <= 0) {
    return;
  }
  if (len > 513) {  // Trim the length just in case
    len = 513;
  }

  uint8_t startCode = msgBuf[5];
  if (startCode != SC_NULL || !sendOnChangeOnly) {
    msgBuf[0] = kStartByte;
    msgBuf[1] = static_cast<uint8_t>(Labels::kReceivedDMX);
    msgBuf[2] = static_cast<uint8_t>(len + 1);
    msgBuf[3] = static_cast<uint8_t>(static_cast<uint16_t>(len + 1) >> 8);
    msgBuf[4] = 0;  // No queue overflow nor overrun
    msgBuf[5 + len] = kEndByte;
    stream.write(msgBuf, 5 + 1 + len);

    if (startCode == SC_NULL) {
      std::copy_n(&msgBuf[5], len, lastDMXBuf);
    }

    return;
  }

  // Process the change
  if (len == lastDMXLen &&
      std::equal(&msgBuf[5], &msgBuf[5 + len], lastDMXBuf)) {
    return;
  }

  // If the length doesn't match, assume any additional bytes
  // are reset
  if (len > lastDMXLen) {
    std::fill_n(&lastDMXBuf[lastDMXLen], len - lastDMXLen, 0);
  }
  lastDMXLen = len;

  changeMsg[0] = kStartByte;
  changeMsg[1] = static_cast<uint8_t>(Labels::kReceivedDMXChange);

  for (int i = 0; i < len; i++) {
    if (msgBuf[i + 5] == lastDMXBuf[i]) {
      i++;
      continue;
    }

    // A mismatch, look at the next (up to) 40 bytes,
    // starting at the block-of-8 start
    int block = i / 8;
    i = block * 8;  // Reset 'i' to the start of this block
    changeMsg[4] = block;
    int changeLen = 10;
    for (int j = 0; j < 40 && i < len; j++) {
      if (msgBuf[i + 5] != lastDMXBuf[i]) {
        changeMsg[5 + j/8] |= 1 << (j % 8);
        changeMsg[changeLen++] = msgBuf[i + 5];
      }
      i++;
    }
    changeMsg[2] = changeLen - 4;
    changeMsg[3] = static_cast<uint16_t>(changeLen - 4) >> 8;
    changeMsg[changeLen] = kEndByte;
    stream.write(changeMsg, changeLen + 1);
  }

  std::copy_n(&msgBuf[5], len, lastDMXBuf);
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

  if (parseState != ParseStates::kStart && lastReadTimer >= kReadTimeout) {
    parseState = ParseStates::kStart;
  }
}

// ---------------------------------------------------------------------------
//  Callback functions
// ---------------------------------------------------------------------------

// Sends a DMX message to the host. The data is sent in the main loop.
//
// This is called from the DMX receive handler.
void sendDMXToHost(const uint8_t *buf, int len) {
  std::copy_n(buf, len, recvDMXBuf);
  recvDMXLen = len;
}

// ---------------------------------------------------------------------------
//  Message handling
// ---------------------------------------------------------------------------

// Handles a received message from the host.
void handleMessage(const Message &msg) {
  // Most commands reset the device to input
  bool resetToInput = true;

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

      // Ceiling calculations for the BREAK and MAB times

      uint32_t t = (dmxTx.breakTime()*100 + 1066)/1067;
      if (t < 1) {  // Really should be 9, according to the spec,
                    // but the library allows smaller
        t = 1;
      } else if (t > 127) {
        t = 127;
      }
      msgBuf[6] = static_cast<uint8_t>(t);

      t = (dmxTx.mabTime()*100 + 1066)/1067;
      if (t < 1) {
        t = 1;
      } else if (t > 127) {
        t = 127;
      }
      msgBuf[7] = static_cast<uint8_t>(t);

      float refreshRate = dmxTx.refreshRate();
      if (refreshRate > 40.0f) {
        refreshRate = 0.0f;  // Why doesn't the spec allow zero?
                             // Send it anyway
      } else if (refreshRate < 1.0f) {
        refreshRate = 1.0f;
      }
      msgBuf[8] = static_cast<uint8_t>(refreshRate);

      msgBuf[9] = kEndByte;
      stream.write(msgBuf, 10);
      stream.flush();

      resetToInput = false;

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
      if (dmxState != DMXStates::kTx) {
        dmxRx.end();
      }

      int len = msg.dataEnd;
      // Make changing the packet size atomic with setting
      // the new data
      if (len != dmxTx.packetSize()) {
        __disable_irq();
        dmxTx.setPacketSize(len);
        dmxTx.set(0, msg.data, len);
        __enable_irq();
      } else {
        dmxTx.set(0, msg.data, len);
      }

      if (dmxState != DMXStates::kTx) {
        startTx();
        dmxState = DMXStates::kTx;
      }

      resetToInput = false;

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

      sendOnChangeOnly = (msg.data[0] != 0);

      // Reset everything to zero, per the spec
      std::fill_n(lastDMXBuf, 513, 0);
      lastDMXLen = 0;

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
      std::copy_n(kManufacturerName, nameLen, &msgBuf[6]);
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
      std::copy_n(kDeviceName, nameLen, &msgBuf[6]);
      msgBuf[6 + nameLen] = kEndByte;
      stream.write(msgBuf, 6 + nameLen + 1);
      stream.flush();
      break;
    }

    default:
      break;
  }

  // Potentially reset the device to input
  if (resetToInput && dmxState == DMXStates::kTx) {
    dmxTx.end();
    startRx();
    dmxState = DMXStates::kRx;
  }
}
