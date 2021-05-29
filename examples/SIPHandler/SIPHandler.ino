/*
 * SIPHandler shows how to implement a Responder to make use of
 * System Information Packets (SIP).
 *
 * This example is part of the TeensyDMX library.
 * (c) 2018-2019 Shawn Silverman
 */

// C++ includes
#include <cstdint>

#include <TeensyDMX.h>
#include "SIPHandler.h"

namespace teensydmx = ::qindesign::teensydmx;

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Create the DMX receiver on Serial1.
teensydmx::Receiver dmxRx{Serial1};

// Holds a SIPHandler instance.
SIPHandler sipHandler{};

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }
  Serial.println("Starting SIPHandler.");

  // Set up any pins
  pinMode(kLEDPin, OUTPUT);
  digitalWriteFast(kLEDPin, HIGH);  // Start with the LED on

  // Set up the responder
  dmxRx.setResponder(0, &sipHandler);
  dmxRx.setResponder(SIPHandler::startCode(), &sipHandler);

  dmxRx.begin();
}

void loop() {
}
