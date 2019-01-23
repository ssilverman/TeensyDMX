// This file is part of the TeensyDMX library.
// (c) 2018-2019 Shawn Silverman

// Basic main program, so it's easy to compile the project.

// Define MAIN_TEST_PROGRAM to use this test program.
#ifdef MAIN_TEST_PROGRAM

#include <Arduino.h>

// Project includes
#include "TeensyDMX.h"

namespace teensydmx = ::qindesign::teensydmx;

// The pin for which to flash the LED.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// The pin to use for enabling the transmitter.
constexpr uint8_t kTXPin = 17;

// Create the DMX transmitter on Serial1.
teensydmx::Sender dmxTx{Serial1};

int channel = 0;

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }

  // Set up any pins
  pinMode(kLEDPin, OUTPUT);
  digitalWriteFast(kLEDPin, LOW);
  pinMode(kTXPin, OUTPUT);
  digitalWriteFast(kTXPin, HIGH);

  Serial.println("Hello, DMX World!");

  // dmxTx.setPacketSize(25);
  // dmxTx.setRefreshRate(40);
  dmxTx.begin();
  channel = 512;
}

void loop() {
  digitalWriteFast(kLEDPin, HIGH);
  dmxTx.set(channel++, 0);
  if (channel > 512) {
    channel = 1;
  }
  dmxTx.set(channel, 255);
  delay(20);
  digitalWriteFast(kLEDPin, LOW);

  delay(980);
}

#endif  // MAIN_TEST_PROGRAM
