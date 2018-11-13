// Basic main program, so it's easy to compile the project.

#include <Arduino.h>

// Project includes
#include "TeensyDMX.h"

namespace teensydmx = ::qindesign::teensydmx;

// Create the DMX transmitter on Serial1.
teensydmx::Sender dmxTx{Serial1};

void setup() {
  // Initialize the serial port and wait for initialization to complete
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
  }
  Serial.println("Hello, DMX World!");

  // dmxTx.setPacketSize(25);
  // dmxTx.setRefreshRate(40);
  dmxTx.begin();
}

void loop() {
}
