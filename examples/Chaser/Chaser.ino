/*
 * Demonstrates outputting DMX by chasing values across
 * all the channels.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2017-2020 Shawn Silverman
 */

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// Interval between channel value changes. A value of 50ms was
// chosen so that all we cycle through all 512 channels every
// 512 * 0.05s = 25.6 seconds.
constexpr unsigned long kChaseInterval = 50;  // 50ms

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Pin for enabling or disabling the transmitter.
// This may not be needed for your hardware.
constexpr uint8_t kTXPin = 17;

// Create the DMX transmitter on Serial1.
teensydmx::Sender dmxTx{Serial1};

// The current channel outputting a value.
int channel;

// Elapsed time since the last chase.
elapsedMillis sinceLastChase;

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }
  Serial.println("Starting Chaser.");

  // Set up any pins
  pinMode(kLEDPin, OUTPUT);
  digitalWriteFast(kLEDPin, LOW);

  // Set the pin that enables the transmitter; may not be needed
  pinMode(kTXPin, OUTPUT);
  digitalWriteFast(kTXPin, HIGH);

  dmxTx.begin();

  // Initialize the state to the end conditions
  channel = 512;
  sinceLastChase = kChaseInterval;
}

void loop() {
  if (sinceLastChase < kChaseInterval) {
    return;
  }
  sinceLastChase = 0;

  // Set the current channel to zero, advance it, and then set
  // the new channel to 255
  dmxTx.set(channel, 0);
  channel++;
  if (channel > 512) {
    channel = 1;
  }
  dmxTx.set(channel, 255);

  // Blink the LED
  digitalWriteFast(kLEDPin, HIGH);
  delay(kChaseInterval / 2);
  digitalWriteFast(kLEDPin, LOW);
}
