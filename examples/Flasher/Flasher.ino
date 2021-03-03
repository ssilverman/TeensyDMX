/*
 * Demonstrates changing the flashing speed of the board LED via
 * the value of a DMX channel.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2017-2020 Shawn Silverman
 */

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// DMX channel from which to read the flash speed.
constexpr int kChannel = 1;

// Timeout after which it's considered that DMX is no longer sending.
constexpr unsigned long kDMXTimeout = 1000;  // 1s

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Interval between printing received values.
constexpr unsigned long kPrintInterval = 1000;  // 1s

// Constants for flashing rate
constexpr int32_t kPeriodMax = 1000;  // 1s
constexpr int32_t kPeriodMin = 30;    // 30ms

// Create the DMX receiver on Serial1.
teensydmx::Receiver dmxRx{Serial1};

// Buffer used for reading the DMX data.
uint8_t buf[1];

// The last value received on kChannel.
uint8_t lastValue;

// Keeps track of when the last frame was received.
elapsedMillis lastFrameTimer;

// Flashing state
int32_t period = kPeriodMax;
int64_t phi = 0;

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }
  Serial.println("Starting Flasher.");

  // Set up any pins
  pinMode(kLEDPin, OUTPUT);
  digitalWriteFast(kLEDPin, HIGH);  // Start with the LED on

  dmxRx.begin();
  lastFrameTimer = kDMXTimeout;
}

void loop() {
  int read = dmxRx.readPacket(buf, kChannel, 1);
  int64_t t = millis();
  if (read > 0) {
    // We've read everything we want to
    lastValue = buf[0];
    lastFrameTimer = 0;

    // Print the data every so often
    static elapsedMillis p = kPrintInterval;
    if (p >= kPrintInterval) {
      Serial.printf("%d: %d\n", dmxRx.packetCount(), lastValue);
      p = 0;
    }

    // Use a wave equation to make the speed-ups and slow-downs
    // smoother using the offset, phi
    int32_t newPeriod = map(lastValue, 0, 255, kPeriodMax, kPeriodMin);
    if (newPeriod != period) {
      phi = (t*(period - newPeriod) + newPeriod*phi)/period;
      period = newPeriod;
    }
  }

  if (lastFrameTimer < kDMXTimeout) {
    int32_t v = (t - phi)%period;
    if (v < period/2) {
      digitalWriteFast(kLEDPin, HIGH);
    } else {
      digitalWriteFast(kLEDPin, LOW);
    }
  } else {
    digitalWriteFast(kLEDPin, LOW);
  }
}
