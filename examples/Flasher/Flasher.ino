/*
 * Demonstrates changing the flashing speed of the board LED via
 * the value of a DMX channel.
 *
 * (c) 2017-2019 Shawn Silverman
 */

#include <TeensyDMX.h>

// DMX channel from which to read the flash speed.
constexpr int kChannel = 1;

// Timeout after which it's considered that DMX is no longer sending.
constexpr unsigned long kDMXTimeout = 1000;

// The pin for which to flash the LED.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Interval between printing received values.
constexpr unsigned long kPrintInterval = 1000;

namespace teensydmx = ::qindesign::teensydmx;

// Create the DMX receiver on Serial1.
teensydmx::Receiver dmxRx{Serial1};

// Keeps track of when the last frame was received.
elapsedMillis lastFrameTime;

// The last value sent to kChannel.
uint8_t lastValue;

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }
  Serial.println("Starting.");

  // Set up any pins
  pinMode(kLEDPin, OUTPUT);

  dmxRx.begin();
  lastFrameTime = kDMXTimeout;
}

void loop() {
  static uint8_t buf[1];
  int read = dmxRx.readPacket(buf, kChannel, 1);
  if (read > 0) {
    lastValue = buf[0];
    lastFrameTime = 0;

    // Print the data every so often
    static elapsedMillis p = kPrintInterval;
    if (p >= kPrintInterval) {
      Serial.printf("%d: %d\n", dmxRx.packetCount(), lastValue);
      p = 0;
    }
  }

  if (lastFrameTime < kDMXTimeout) {
    // Use a wave equation to make the speed-ups and slow-downs smoother
    // using the offset, phi
    static int32_t period = 1000;
    static int64_t phi = 0 * period;
    int32_t newPeriod = map(lastValue, 0, 255, 1000, 30);  // T range is 1s to 30ms
    int64_t t = millis();
    if (newPeriod != period) {
      phi = (t*(period - newPeriod) + newPeriod*phi)/period;
      period = newPeriod;
    }
    int32_t v = -(phi - t)%period;

    if (v < period/2) {
      digitalWriteFast(kLEDPin, HIGH);
    } else {
      digitalWriteFast(kLEDPin, LOW);
    }
  } else {
    digitalWriteFast(kLEDPin, LOW);
  }
}
