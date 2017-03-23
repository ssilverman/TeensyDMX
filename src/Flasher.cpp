/*
 * Demonstrates changing the flashing speed of the board LED via
 * the value of a DMX channel.
 *
 * (c) 2017 Shawn Silverman
 */

#include "TeensyDMX.h"

// DMX channel from which to read the flash speed.
constexpr int kChannel = 1;

// Timeout after which it's considered that DMX is no longer sending.
constexpr unsigned long kDMXTimeout = 1000;

// The pin for which to flash the LED.
constexpr uint8_t kLEDPin = 13;

// Interval between printing received values.
constexpr unsigned long kPrintInterval = 500;

namespace teensydmx = ::qindesign::teensydmx;

// Create the DMX receiver on Serial1.
teensydmx::Receiver dmxRx{Serial1};

// Keeps track of when the last frame was received.
elapsedMillis lastFrameTime;

// The last value sent to kChannel.
uint8_t lastValue;

void setup() {
  Serial.begin(9600);
  delay(2000);  // Instead of while (!Serial), doesn't seem to work on Teensy
  Serial.println("Starting.");

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
      Serial.printf("%d:", dmxRx.packetCount());
      for (int i = 0; i < 20; i++) {
        Serial.printf(" %d:%d", i, buf[kChannel + i]);
      }
      Serial.println();
      p = 0;
    }
  }

  if (lastFrameTime < kDMXTimeout) {
    // Use a wave equation to make the speed-ups and slow-downs smoother
    // using the offset, phi
    static int period = 1000;
    static long phi = 0 * period;
    int newPeriod = map(lastValue, 0, 255, 1000, 30);  // T range is 1s to 30ms
    long t = static_cast<long>(millis());
    phi = (t*period - newPeriod*(t - phi))/period;
    period = newPeriod;
    int v = static_cast<int>(-(phi - t)%period);

    if (v < period/2) {
      digitalWrite(kLEDPin, HIGH);
    } else {
      digitalWrite(kLEDPin, LOW);
    }
  } else {
    digitalWrite(kLEDPin, LOW);
  }
}
