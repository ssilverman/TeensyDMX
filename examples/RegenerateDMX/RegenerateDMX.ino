/*
 * Regenerates DMX from Serial1 onto Serial2 using new
 * timing parameters.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2021 Shawn Silverman
 */

// C++ includes
#include <algorithm>

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// Timeout after which it's considered that DMX is no longer
// being received.
constexpr uint32_t kDMXTimeout = 1000;  // 1s

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Creates the DMX receiver on Serial1.
teensydmx::Receiver dmxRx{Serial1};

// Creates the DMX sender on Serial2.
teensydmx::Sender dmxTx{Serial2};

// Buffer used for reading the DMX data.
uint8_t buf[teensydmx::kMaxDMXPacketSize];

// Keeps track of when the last frame was received.
elapsedMillis lastRxTimer;

// Indicates whether the receiver is connected.
bool connected = false;

// Main program setup.
void setup() {
  // Serial initialization, for printing things (optional)
  // Serial.begin(115200);
  // while (!Serial && millis() < 4000) {
  //   // Wait for initialization to complete or a time limit
  // }
  // Serial.println("Starting DMXRegenerator.");

  // Turn on the LED, for indicating activity
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, HIGH);

  // Set up any pins
  pinMode(kLEDPin, OUTPUT);

  // Configure the output
  // Note: Due to how the UART works, the transmitted timings will be
  //       close to the requested values but not necessarily exact.
  dmxTx.setBreakTime(242);
  dmxTx.setMABTime(100);
  dmxTx.setBreakUseTimerNotSerial(true);
      // Use a timer for the BREAK and MAB; using the serial approach
      // uses different settings and is more restrictive
      // See: extras/break-timing.md
  dmxTx.setInterSlotTime(40);
  dmxTx.setMBBTime(12);  // Depending on which Teensy is used, the MBB
                         // may be much larger (Teensy LC) or just
                         // a little bit larger.

  // Start the receiver
  // Use data presence to enable or disable the transmitter
  dmxRx.begin();
  lastRxTimer = kDMXTimeout;
}

// Changes the receiver connection state. This affects the LED
// and transmitter.
void setConnected(bool flag) {
  if (connected == flag) {
    return;
  }
  connected = flag;

  if (flag) {
    digitalWriteFast(kLEDPin, HIGH);
    dmxTx.begin();
  } else {
    digitalWriteFast(kLEDPin, LOW);
    dmxTx.end();
  }
}

// Main program loop.
void loop() {
  int read = dmxRx.readPacket(buf, 0, teensydmx::kMaxDMXPacketSize);
  if (read > 0) {
    lastRxTimer = 0;

    // Fill un-received values with zero and set the output
    std::fill_n(&buf[read], teensydmx::kMaxDMXPacketSize - read, 0);
    dmxTx.set(0, buf, teensydmx::kMaxDMXPacketSize);

    setConnected(true);
  } else {
    // Check connected state to avoid constantly polling the timer
    if (connected && lastRxTimer >= kDMXTimeout) {
      setConnected(false);
    }
  }
}
