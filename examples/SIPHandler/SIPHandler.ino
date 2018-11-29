/*
 * SIPHandler shows how to implement a Responder to make use of
 * System Information Packets (SIP).
 *
 * (c) 2018 Shawn Silverman
 */

#include <TeensyDMX.h>
#include "SIPHandler.h"

namespace teensydmx = ::qindesign::teensydmx;

// Create the DMX receiver on Serial1.
teensydmx::Receiver dmxRx{Serial1};

// Holds a SIPHandler instance.
SIPHandler sipHandler;

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }
  Serial.println("Starting.");

  // Set up the responder
  dmxRx.addResponder(0, &sipHandler);
  dmxRx.addResponder(SIPHandler::startCode(), &sipHandler);

  dmxRx.begin();
}

void loop() {
}
